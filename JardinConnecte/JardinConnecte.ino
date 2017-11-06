// #include <Event.h>
//#include <PCF8574Timer.h>
//#include <PCint.h>
//#include <psl_130907.h>
#include <SimpleDHT.h>
#include <PCF8574.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
//#include "nRF24L01.h"
#include "RF24.h"
/*************************************************************************/
/*            P I N O U T     O N    A R D U I N O                       */
/*************************************************************************/
#define INTERRUPT_PIN     3
#define TEMP_HUM_PIN      4
#define MOTOR_PWM_PIN     2
#define HC_SR04_1_ECHO    7
#define HC_SR04_1_TRIG    8 
#define RF_CE             9
#define RF_CSN            10

#define DEBUG
/*************************************************************************/
/*            P I N O U T     O N    E X P A N D E R                     */
/*************************************************************************/
#define RELAI_GPIO_EX_0 0        // GPIO for PUMP On/Off
#define MOTOR_IN1_GPIO_EX_1 1    // GPIO for Motor Rotation sens
#define MOTOR_IN2_GPIO_EX_2 2    // GPIO for Motor Rotation sens
#define WATER_OFF_INT_PIN 3      // GPIO for Interruption related to Water On/Off
#define GPIO_EX_4 4
#define GPIO_EX_5 5
#define GPIO_EX_6 6
#define GPIO_EX_7 7
#define GPIO_EX_8 8

/*************************************************************************/
/*                      C O N S T A N T S                                */
/*************************************************************************/
#define LCD_I2C_ADDRESS         (0x3F)
#define IO_EXPANDER_I2C_ADDRESS (0x38)
#define MAX_LENGTH              10
#define  SOUND_SPEED            (340.0 / 1000)   /* Vitesse du son dans l'air en mm/us */
#define MEASURE_TIMEOUT         (25000UL) // Constantes pour le timeout ==> 25ms = ~8m à 340m/s
#define WATERTANK_THRESHOLD     (1500) // Seuil pour la reserve d'eau ( vide ou non ) exprimé en millimètres
#define TEMP_THRESHOLD_LOW      4      // Seuil pour la temperature (gel ou pas) ==> 4°C 

#define ACTION_SLEEP_NO_ACTION   (1<<0)
#define ACTION_SWITCH_ON_PUMP    (1<<1)
#define ACTION_SWITCH_OFF_PUMP   (1<<2)
#define ACTION_CHECK_TEMPERATURE (1<<3)
#define ACTION_CHECK_WATER_POOL  (1<<4)
#define ACTION_SWITCH_ON_MOTOR   (1<<5)
#define ACTION_SWITCH_OFF_MOTOR  (1<<6)

#define PUMPONTIME 5000    // 5 seconds
#define PUMPOFFTIME 30000  // 30 seconds
#define MOTORONTIME 10000  // 10 seconds
#define MOTOROFFTIME 60000 // 1 minutes
#define TXINFOTIME   30000 // 30 seonds 
/*************************************************************************/
/*             G L O B A L     V A R I A B L E S                         */
/*************************************************************************/

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS,16,2); // set the LCD I2C address for a 16 chars and 2 line display
PCF8574           expander;
SimpleDHT11       dht11;
int               count = 0;
RF24              radio(RF_CE, RF_CSN); // Set up nRF24L01 radio on SPI bus plus pins 9 & 10
volatile int      ActionsBitmap = ACTION_SLEEP_NO_ACTION;
// RF Topology
// Radio pipe addresses for the 2 nodes to communicate  { Tx Address, Rx Address }
const uint64_t pipes[2] = { 0xF0F0F0F0F3LL, 0xF0F0F0F0D2LL };

unsigned long PumpOffTime;
unsigned long MotorOnTime;
unsigned long MotorOffTime;
unsigned long TxInfoTime;

byte Temperature = 0;
byte Humidity = 0;
int  WaterHeight = 0;
int  FoodHeight = 0;
  
/*************************************************************************/
/*                 M A C R O S                                           */
/*************************************************************************/
#define SwitchOnPump()       (expander.digitalWrite(RELAI_GPIO_EX_0,HIGH))
#define SwitchOffPump()      (expander.digitalWrite(RELAI_GPIO_EX_0,LOW))

#define SwitchOnMotor()  expander.digitalWrite(MOTOR_IN1_GPIO_EX_1, HIGH);  // Clockwise 
#define SwitchOffMotor()  expander.digitalWrite(MOTOR_IN1_GPIO_EX_1, LOW);  // Clockwise 

#define SwitchOnMotor_ActionSet()         (ActionsBitmap |= ACTION_SWITCH_ON_MOTOR)
#define SwitchOnMotor_ActionClear()       (ActionsBitmap &= (~ACTION_SWITCH_ON_MOTOR))
#define Is_SwitchOnMotor_ActionPending()  ((ActionsBitmap & ACTION_SWITCH_ON_MOTOR) == ACTION_SWITCH_ON_MOTOR )

#define SwitchOffMotor_ActionSet()         (ActionsBitmap |= ACTION_SWITCH_OFF_MOTOR)
#define SwitchOffMotor_ActionClear()       (ActionsBitmap &= (~ACTION_SWITCH_OFF_MOTOR))
#define Is_SwitchOffMotor_ActionPending()  ((ActionsBitmap & ACTION_SWITCH_OFF_MOTOR) == ACTION_SWITCH_OFF_MOTOR)

#define SwitchOnPump_ActionSet()         (ActionsBitmap |= ACTION_SWITCH_ON_PUMP)
#define SwitchOnPump_ActionClear()       (ActionsBitmap &= (~ACTION_SWITCH_ON_PUMP))
#define Is_SwitchOnPump_ActionPending()  ((ActionsBitmap & ACTION_SWITCH_ON_PUMP) == ACTION_SWITCH_ON_PUMP)

#define SwitchOffPump_ActionSet()         (ActionsBitmap |= ACTION_SWITCH_OFF_PUMP)
#define SwitchOffPump_ActionClear()       (ActionsBitmap &= (~ACTION_SWITCH_OFF_PUMP))
#define Is_SwitchOffPump_ActionPending()  ((ActionsBitmap & ACTION_SWITCH_OFF_PUMP) == ACTION_SWITCH_OFF_PUMP )
/*************************************************************************/
/*             L O C A L     F U N C T I O N S                           */
/*************************************************************************/

/** This function will be called each time the state of a pin of the PCF8574 change */
void ISRgateway() {
  expander.checkForInterrupt();
}

boolean IsWaterTankFull(void) {
  if (expander.digitalRead(WATER_OFF_INT_PIN) == HIGH)
    return true;
  return false;
}

/* This routines handle interrupt linked to empty tank
   It turns on the pump
*/
void WaterOff_InterruptHandler(void) {
  SwitchOnPump_ActionSet();
}

boolean rf_tx(float meas) {

  static char tx_data[40];
  int  count = 0;
  static char rx_data[MAX_LENGTH] ;

  // First, stop listening so we can talk.
  radio.stopListening();
  // build message to send
  count = sprintf(tx_data,"la distance est %d mm",int(meas));
  radio.write( tx_data, count );

  // Now, continue listening
  radio.startListening();

  // Wait here until we get a response, or timeout
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! radio.available() && ! timeout )
    if (millis() - started_waiting_at > 5000 )
      timeout = true;

  // Describe the results
  if ( timeout )
  {
    //Serial.println(F("Failed, response timed out."));
    lcd.setCursor(9,0);
    lcd.print("KO!");
    return false;
  }
  else
  {
    // Grab the response, compare, and send to debugging spew
    uint8_t len = radio.getDynamicPayloadSize();

    // If a corrupt dynamic payload is received, it will be flushed
    if (!len) {
      //Serial.println(F("Corrupted data."));
      lcd.setCursor(9,0);
      lcd.print("Bad");
      return false;
    }

    radio.read( rx_data, len );

    // Put a zero at the end for easy printing
    rx_data[len] = 0;

    // Spew it
    //Serial.print(F("Got response size="));
    //Serial.print(len);
    //Serial.print(F(" value="));
    //Serial.println(rx_data);
    lcd.setCursor(9,0);
    lcd.print("OK");
    return true;
  }
}

void setup() {
  // put your setup code here, to run once:
  // initialize the lcd 
  lcd.init();                      
  lcd.backlight();
  lcd.clear();
  lcd.print("Initialisation...");

  // Start GPIO Expander
  expander.begin(IO_EXPANDER_I2C_ADDRESS);

  /* Enable PCF8574 interrupts, use pin D8 as "INT" pin and ISRgateway() as callback function 
     Here the pin in the interrupt Pin on Arduino ( use cable btw Arduino Pin & PCF8574 INT pin (8)*/
  expander.enableInterrupt(INTERRUPT_PIN, ISRgateway);
  expander.pinMode(RELAI_GPIO_EX_0, OUTPUT);
  expander.digitalWrite(RELAI_GPIO_EX_0, LOW);  // Pump Off 

  // Configure Motor Control Driver pins
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  expander.pinMode(MOTOR_IN1_GPIO_EX_1, OUTPUT);
  expander.pinMode(MOTOR_IN2_GPIO_EX_2, OUTPUT);

  // Configure Speed = 200/255 
  analogWrite(MOTOR_PWM_PIN, 200);
  expander.digitalWrite(MOTOR_IN1_GPIO_EX_1, LOW);  // Motor Off
  expander.digitalWrite(MOTOR_IN2_GPIO_EX_2, LOW);
  
  /* Attach a software interrupt on pin 3 of the PCF8574 */
  expander.pinMode(WATER_OFF_INT_PIN, INPUT_PULLUP);
  expander.attachInterrupt(WATER_OFF_INT_PIN, WaterOff_InterruptHandler, FALLING);
  

  
   /* Initialise les broches du capteur HC-SR04 */
  pinMode(HC_SR04_1_TRIG, OUTPUT);
  digitalWrite(HC_SR04_1_TRIG, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(HC_SR04_1_ECHO, INPUT);

  // Setup and configure rf radio
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.enableDynamicPayloads(); // enable dynamic payloads
  radio.setRetries(5, 15); // optionally, increase the delay between retries & # of retries
  // Open pipes to other nodes for communication
  radio.openWritingPipe(pipes[0]); // Tx Pipe  
  radio.openReadingPipe(1, pipes[1]); // Rx Pipe

  ComputeTemperature(&Temperature, &Humidity);
  delay(100);
  WaterHeight = ComputeWaterHeight();
  delay(500);

  lcd.setCursor(0,1);
  lcd.print("OK") ;
    delay(1500);
    
#ifdef DEBUG
  // For test purpose, initialize timings
  MotorOnTime = millis() ;
  SwitchOnMotor_ActionSet();
  TxInfoTime  = MotorOnTime;
  PumpOffTime = 0UL;

  lcd.clear();
  lcd.print("M+");
  lcd.print(MotorOnTime);
  lcd.print("P-");
  lcd.print(PumpOffTime);
#endif
  
  lcd.setCursor(0,1);
  lcd.print("T=");
  lcd.print(Temperature);
  lcd.print(", D=");
  lcd.print(WaterHeight);
  delay(1000);
}

/* This routine uses HC-SR04_1 to measure height of water in the tank
   */
int ComputeWaterHeight(void) {
  long measure ;

  /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(HC_SR04_1_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_SR04_1_TRIG, LOW);
  
  /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  measure = pulseIn(HC_SR04_1_ECHO, HIGH, MEASURE_TIMEOUT);
  
  /* 3. Calcul la distance à partir du temps mesuré */
  return ( (int )(measure / 2.0 * SOUND_SPEED));
}

/*  This routine uses DHT11 or DHT22 to measure temperature & humidity
 */
int ComputeTemperature(byte *temp, byte *hum) {
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(TEMP_HUM_PIN, temp, hum, NULL)) != SimpleDHTErrSuccess) {
    return -1;
  }
  return 0;
}

boolean TimeToAction( unsigned long action_t , unsigned long t ) {
  // Check elapsed time 
  // roll over is done automatically by manipulating unsigned long
  if ( t >= action_t)
    return true;
  return false;
}

int HandleRegularActions(unsigned long t){
  if (((ActionsBitmap & ACTION_SWITCH_OFF_PUMP) == ACTION_SWITCH_OFF_PUMP) && ( TimeToAction(PumpOffTime, t) == true)) {
    // Check if enough water ?
    if ( IsWaterTankFull() == true ) {
      // Switch Off Water Pump
      SwitchOffPump();
      SwitchOffPump_ActionClear();
    }
    else {
      // restart 
      // Compute Elapsed Time => T+60s
      PumpOffTime = t + PUMPONTIME;
    }
  }

  if (((ActionsBitmap & ACTION_SWITCH_ON_MOTOR) == ACTION_SWITCH_ON_MOTOR) && (TimeToAction(MotorOnTime, t) == true)) {
    // Switch On Motor
    SwitchOnMotor();
    SwitchOnMotor_ActionClear();

    // Compute MotorOffTime
    MotorOffTime = t + MOTORONTIME ;
    SwitchOffMotor_ActionSet();
  }

  if (((ActionsBitmap & ACTION_SWITCH_OFF_MOTOR) == ACTION_SWITCH_OFF_MOTOR) && (TimeToAction(MotorOffTime, t) == true)) {
    // Switch Off Motor
    SwitchOffMotor();
    SwitchOffMotor_ActionClear();

    // Compute MotorOnTime
    MotorOnTime = t + MOTOROFFTIME ;
    SwitchOnMotor_ActionSet();
  }

  if (TimeToAction(TxInfoTime, t) == true) {
    // Send measure through RF link
    rf_tx(0.5);
    // TxInfoSend_ActionClear();

    // Compute MotorOffTime
    TxInfoTime = t + TXINFOTIME ;
  }
}

void loop() {
  unsigned long current_t = millis();
  
  // put your main code here, to run repeatedly:

  /* Main applications consists in 2 main parts 
   * 1. Handle actions based on time, regular events (e.g. check temperature, ... ) 
   * 2. Handle acions based on interrupt (asynchronous) example water tank empty ... 
   */
  HandleRegularActions(current_t); 

  // Handle asynchronous actions, 
//  while (ActionsBitmap != ACTION_SLEEP_NO_ACTION) {
    if (Is_SwitchOnPump_ActionPending()) {
      // clear bitmap, mentioning action being processed
      SwitchOnPump_ActionClear();

      // Verify if temperature is above threshold and if there
      // is enough water in pool 
      if (ComputeTemperature(&Temperature, &Humidity) == 0) {

        WaterHeight = ComputeWaterHeight();
 
        if ((Temperature < TEMP_THRESHOLD_LOW) || 
            (WaterHeight > WATERTANK_THRESHOLD)) {
              // Alerte 
        }
        else {
          // Compute Elapsed Time => T+60s
          SwitchOnPump();
          PumpOffTime = current_t + PUMPONTIME ;
          SwitchOffPump_ActionSet();
        }
      }
      else {
        // Alerte : bad temperature sensor
      }  
    }

#if 0
    if (Is_SwitchOffPump_ActionPending()) {
      SwitchOffPump();
      SwitchOffPump_ActionClear();
    }

    if (Is_SwitchOnMotor_ActionPending()) {
      SwitchOnMotor();
      SwitchOnMotor_ActionClear();
    }

    if (Is_SwitchOffMotor_ActionPending()) {
      SwitchOffMotor();
      SwitchOffMotor_ActionClear();
    }
#endif
 // }
#ifdef DEBUG
  lcd.clear();
  lcd.print(">");
  lcd.print(current_t);
  lcd.print("B=");
  lcd.print(ActionsBitmap);
#endif
  
  lcd.setCursor(0,1);
  
#ifdef DEBUG
  //lcd.clear();
  lcd.print("M+");
  lcd.print(MotorOnTime);
  lcd.print("M-");
  lcd.print(MotorOffTime);
#else
  lcd.print("T=");
  lcd.print(Temperature);
  lcd.print(", D=");
  lcd.print(WaterHeight);
#endif
  delay(1000);

}
