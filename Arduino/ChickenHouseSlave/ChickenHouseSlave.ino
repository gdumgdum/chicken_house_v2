// Board: "Arduino Nano"
// Processor: "ATmega328P"
// Programmer: "AVRISP mkII" (not sure this is necessary)
// Install OneWire library through the Library Manager
// Install NewRemoteSwitch from https://github.com/1technophile/NewRemoteSwitch/tree/master

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ArduinoJson.h>
#include <ChickenHouseCommon.h>
// To read data from Temperature Sensor
#include <OneWire.h>
#include <printf.h>

#define THN132N 1

#define INTERRUPT 1

// MOTOR
const byte MOTOR_SPEED = 3;
const byte MOTOR_IN1   = 4;
const byte MOTOR_IN2   = 5;

// Wiring:
// Pin 5 to IN2 on the L293D driver
// Pin 4 to IN1 on the L293D driver
// Pin 3 to EN1 on the L293D driver (PWM)

// SWITCHES
const byte DOOR_CLOSED = 9; 
const byte DOOR_OPENED = 10;

// LEDS
const byte LED_CLOSE = A0;
const byte LED_OPEN = A1;

// 1-WIRE PIN (temperature sensor)
const byte BROCHE_ONEWIRE = 6;

bool closingFailed = false;
bool openingFailed = false;
unsigned long lastMotorStart = 0;
const byte motorMaxSpeed = 255; // 100% of 5V => 5V
const byte motorMinSpeed = 125; // 50% of 5V => 2.5V
const byte motorSpeedStep = 10;
byte currentMotorSpeed = 0;

const byte CONF_RF_CHANNEL = A6;
const byte CONF_RF_DATARATE_BIT1 = A5;
const byte CONF_RF_DATARATE_BIT0 = A4;
const byte CONF_RF_POWER_BIT1 = A3;
const byte CONF_RF_POWER_BIT0 = A2;

// Variable used to handle the 1-Wire bus
OneWire ds(BROCHE_ONEWIRE);

// Codes returned by function getTemperature()
enum DS18B20_RCODES {
  READ_OK,
  NO_SENSOR_FOUND,
  INVALID_ADDRESS,
  INVALID_SENSOR
};
  
byte receptionBuffer[32];
ChickenHouseStatus chickenHouseStatus;
ChickenHouseCommand* chickenHouseCommand = (ChickenHouseCommand*)receptionBuffer;
#define pinCE   7             // On associe la broche "CE" du NRF24L01 à la sortie digitale D7 de l'arduino
#define pinCSN  8            // On associe la broche "CSN" du NRF24L01 à la sortie digitale D8 de l'arduino
RF24 radio(pinCE, pinCSN, 1000000);    // Instanciation du NRF24L01
const byte pipeW[6] = PIPE_SLAVE_TO_MASTER; 
const byte pipeR[6] = PIPE_MASTER_TO_SLAVE;
uint8_t   rf_channel = 111;
uint8_t   rf_power = RF24_PA_MIN;
rf24_datarate_e   rf_datarate = RF24_2MBPS;
#ifdef INTERRUPT
#define PIN_RADIO_IRQ 2
volatile bool got_interrupt = false;   // used to signal processing of interrupt
#endif

// variables used to determine if a received RF command should be processed
enum Command lastCommand = Command::OPEN_DOOR;
unsigned long lastCommandTime = 0;
#define MIN_DELAY_BETWEEN_COMMANDS  2000

// Function called when the Arduino is powered on
void setup() {
  // Configure Serial speed at 115200 bauds
  Serial.begin(115200);
  printf_begin();

  pinMode(CONF_RF_POWER_BIT0, INPUT_PULLUP);
  pinMode(CONF_RF_POWER_BIT1, INPUT_PULLUP);
  pinMode(CONF_RF_DATARATE_BIT0, INPUT_PULLUP);
  pinMode(CONF_RF_DATARATE_BIT1, INPUT_PULLUP);

  if (digitalRead(CONF_RF_POWER_BIT1) == LOW) {
    if (digitalRead(CONF_RF_POWER_BIT0) == LOW)
      rf_power = RF24_PA_MAX;
    else
      rf_power = RF24_PA_HIGH;
  }
  else {
    if (digitalRead(CONF_RF_POWER_BIT0) == LOW)
      rf_power = RF24_PA_LOW;
    else
      rf_power = RF24_PA_MIN;
  }

  if (digitalRead(CONF_RF_DATARATE_BIT1) == LOW) {
    rf_datarate = RF24_2MBPS;
  }
  else {
    if (digitalRead(CONF_RF_DATARATE_BIT0) == LOW)
      rf_datarate = RF24_1MBPS;
    else
      rf_datarate = RF24_250KBPS;
  }

  if (analogRead(CONF_RF_CHANNEL) < 512)
    rf_channel = 124;

  rf_power = RF24_PA_HIGH;        // TO REMOVE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  rf_channel = 111;              // TO REMOVE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  rf_datarate = RF24_1MBPS;      // TO REMOVE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  chickenHouseStatus.version_a = VERSION_A;
  chickenHouseStatus.version_b = VERSION_B;
  chickenHouseStatus.version_c = VERSION_C;
  
  pinMode(LED_CLOSE, OUTPUT);
  pinMode(LED_OPEN, OUTPUT);
  digitalWrite(LED_OPEN, HIGH);
  digitalWrite(LED_CLOSE, HIGH);
 
  Serial.print("ChickenHouseV2 Slave module v");
  Serial.print(VERSION_A);
  Serial.print(".");
  Serial.print(VERSION_B);
  Serial.print(".");
  Serial.println(VERSION_C);

  Serial.print("RF channel: ");
  Serial.println(rf_channel);
  
  Serial.print("RF data rate: ");
  switch (rf_datarate) {
    case RF24_250KBPS:
      Serial.println("250Kbps");
      break;
    case RF24_1MBPS:
      Serial.println("1Mbps");
      break;
    case RF24_2MBPS:
      Serial.println("2Mbps");
      break;
    default:
      Serial.println("ERROR");
      break;
  }
  
  Serial.print("RF Power Amplifier: ");
  switch (rf_power) {
    case RF24_PA_MIN:
      Serial.println("Min");
      break;
    case RF24_PA_LOW:
      Serial.println("Low");
      break;
    case RF24_PA_HIGH:
      Serial.println("High");
      break;
    case RF24_PA_MAX:
      Serial.println("Max");
      break;
    default:
      Serial.println("ERROR");
      break;
  }
  
  bool b = radio.begin();                      // Initialisation du module NRF24

  #ifdef INTERRUPT
  pinMode(PIN_RADIO_IRQ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RADIO_IRQ), radioInterrupt, FALLING);
  #endif

  radio.setChannel(rf_channel); 		// en remplaçant « x » par une valeur comprise entre 0 et 125
  radio.setAutoAck(true);
  radio.setPALevel(rf_power);	// en remplaçant « xxx » par RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, ou RF24_PA_MAX
  radio.setDataRate(rf_datarate);	// en remplaçant « xxx » par RF24_250KBPS, RF24_1MBPS, ou encore, RF24_2MBPS
  radio.openWritingPipe(pipeW);      // Ouverture du "tunnel1" en ÉCRITURE (émission radio)
  radio.openReadingPipe(1, pipeR);   // Ouverture du "tunnel2" en LECTURE (réception radio)

  #ifdef INTERRUPT
  // let IRQ pin only trigger on "data ready" event in RX mode
  radio.maskIRQ(1, 1, 0);  // args = "data_sent", "data_fail", "data_ready"
  #endif

  radio.startListening();		// permet de pouvoir utiliser la fonction « read » par la suite
  radio.printPrettyDetails();
  Serial.println(b);

  pinMode(MOTOR_SPEED, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  analogWrite(MOTOR_SPEED, 0);
  makeMotorLoose();
  
  pinMode(DOOR_OPENED, INPUT_PULLUP);
  pinMode(DOOR_CLOSED, INPUT_PULLUP);

  //Serial.flush();

  digitalWrite(LED_OPEN, LOW);
  digitalWrite(LED_CLOSE, LOW);

  Serial.println("Chicken House is ready.");
  startOpeningDoor();
}


void loop() {
  switch(chickenHouseStatus.stateMachine) {
  case StateMachine::CLOSING:
    Closing();
    break;
  case StateMachine::OPENING:
    Opening();
    break;
  case StateMachine::CLOSED:
    Closed();
    break;
  case StateMachine::OPENED:
    Opened();
    break;
  default:
    break;
  }

#ifdef INTERRUPT
  if (got_interrupt) {
    handleInterrupt();

    delay(100);             // let ACK payload finish transmitting
    radio.stopListening();  // also discards unused ACK payloads
#endif

  while (radio.available()) {
    uint8_t size = radio.getPayloadSize();

    Serial.print(size);
    Serial.print(":");
    radio.read(receptionBuffer, size);
    for (int i = 0; i < size; i++) {
      Serial.print(receptionBuffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  
    chickenHouseStatus.secret = ComputeSecret(chickenHouseCommand->secret);
    Serial.print("Secret: ");
    Serial.print(chickenHouseCommand->secret, HEX);
    Serial.print(" > ");
    Serial.println(chickenHouseStatus.secret, HEX);
    
    unsigned long currentTime = millis();
    if ((currentTime - lastCommandTime >= MIN_DELAY_BETWEEN_COMMANDS)
    || (chickenHouseCommand->command != lastCommand)) {

      lastCommandTime = currentTime;
      lastCommand = chickenHouseCommand->command;

      switch (chickenHouseCommand->command) {
        case Command::OPEN_DOOR:
          startOpeningDoor();
        break;
        case Command::CLOSE_DOOR:
          startClosingDoor();
        break;
        case Command::GET_STATUS:
          Serial.println("GetStatus received");
        break;
        default:
          Serial.print("Received unknown command ");
          Serial.println(chickenHouseCommand->command);
        break;
      }

      // Read temperature from sensor
      byte tempResult = getTemperature(&chickenHouseStatus.temperature, true);
      if (tempResult != READ_OK) {
        chickenHouseStatus.temperature = 0.0;
        Serial.print("getTemperature() returned error ");
        Serial.println(tempResult);
      }

#ifndef INTERRUPT
      radio.stopListening();
#endif
      // delay(100);
      bool ok = radio.write(&chickenHouseStatus, sizeof(ChickenHouseStatus));

      radio.startListening();		// permet de pouvoir utiliser la fonction « read » par la suite
      delay(100);

      if (ok) {
        Serial.println("Status sent to master module");
      }
      else {
        Serial.println("Failed to send status to master module");
        lastCommandTime = 0;
      }
    }
  }
#ifdef INTERRUPT
  }
#endif
  delay (5); // This helps keeping CPU low
}

// Called when state machine is CLOSING
void Closing() {
  unsigned long currentTime = millis();
  
  // Make the Close led blink
  digitalWrite(LED_OPEN, LOW);
  digitalWrite(LED_CLOSE, ((currentTime & 0x0100) == 0) ? LOW : HIGH);

  // If motor is running for too long (15 seconds), or if door
  // arrives at the closed position, move state machine to CLOSED
  closingFailed = ((currentTime - lastMotorStart) >= 15000);
  if (closingFailed){
    Serial.println("Failed detecting door closure during 15 seconds, considering it closed.");  
  }
  
  if (closingFailed
  || (digitalRead(DOOR_CLOSED) == LOW)) {
    makeMotorLoose();
    
    digitalWrite(LED_OPEN, LOW);
    digitalWrite(LED_CLOSE, HIGH);
    
    chickenHouseStatus.stateMachine = StateMachine::CLOSED;

    Serial.println("Door closed.");
    //Serial.flush();
  }
  else {
    IncreaseMotorSpeed();
  }
}

// Called when state machine is CLOSED
void Closed() {
  if ((closingFailed == false)
  && (digitalRead(DOOR_CLOSED) == HIGH)) {
    startClosingDoor();
  }
}

// Called when state machine is OPENING
void Opening() {
  unsigned long currentTime = millis();
  
  // Make the Open led blink
  digitalWrite(LED_CLOSE, LOW);
  digitalWrite(LED_OPEN, ((currentTime & 0x0100) == 0) ? LOW : HIGH);

  // If motor is running for too long (15 seconds), or if door
  // arrives at the opened position, move state machine to OPENED
  openingFailed = ((currentTime - lastMotorStart) >= 15000);
  if (openingFailed){
    Serial.println("Failed detecting door opening during 15 seconds, considering it opened.");  
  }
  
  if (openingFailed
  || (digitalRead(DOOR_OPENED) == LOW)) {
    makeMotorLoose();
    
    digitalWrite(LED_CLOSE, LOW);
    digitalWrite(LED_OPEN, HIGH);
    
    chickenHouseStatus.stateMachine = StateMachine::OPENED;

    Serial.println("Door opened.");
    //Serial.flush();
  }
  else {
    IncreaseMotorSpeed();
  }
}

// Called when state machine is OPENED
void Opened() {
  if ((openingFailed == false)
  && (digitalRead(DOOR_OPENED) == HIGH)) {
    startOpeningDoor();
  }
}

void makeMotorLoose()
{
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void IncreaseMotorSpeed() {
  if (currentMotorSpeed <= (motorMaxSpeed - motorSpeedStep)) {
    currentMotorSpeed += motorSpeedStep;

    analogWrite(MOTOR_SPEED, currentMotorSpeed);
  }
}

void startClosingDoor()
{
  Serial.println("Closing door.");
  //Serial.flush();

  chickenHouseStatus.stateMachine = StateMachine::CLOSING;
  currentMotorSpeed = motorMinSpeed;
  analogWrite(MOTOR_SPEED, currentMotorSpeed);
  
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);

  lastMotorStart = millis();
  closingFailed = false;
}

void startOpeningDoor()
{
  Serial.println("Opening door.");
  //Serial.flush();

  chickenHouseStatus.stateMachine = StateMachine::OPENING;
  currentMotorSpeed = motorMinSpeed;
  analogWrite(MOTOR_SPEED, currentMotorSpeed);
  
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);

  lastMotorStart = millis();
  openingFailed = false;
}

/**
 * \brief    Read temperature from a DS18B20 sensor
 * \details  assumes that the DS18B20 sensor is alone on the wire,
 * \         thus is the first device to be detected.
 */
byte getTemperature(float *temperature, byte reset_search) {
  byte data[9], addr[8];
  // data[] : Data read from scratchpad
  // addr[] : Address of the detected 1-Wire device
  
  // Reset the 1-Wire bus if requested (required for the first device reading)
  if (reset_search) {
    ds.reset_search();
  }
 
  // Search for the next available 1-Wire device
  if (!ds.search(addr)) {
    // No device found
    return NO_SENSOR_FOUND;
  }
  
  // Check that the address is valid
  if (OneWire::crc8(addr, 7) != addr[7]) {
    // Wrong checksum, the address is then invalid
    return INVALID_ADDRESS;
  }
 
  // Check that the detected device is a DS18B20
  if (addr[0] != 0x28) {
    // Wrong device type
    return INVALID_SENSOR;
  }
 
  // Reset the 1-Wire bus and select the sensor
  ds.reset();
  ds.select(addr);
  
  // Trigger a temperature reading and wait for completion
  ds.write(0x44, 1);
  delay(800);
  
  // Reset the 1-Wire but, select the sensor and read the scratchpad
  ds.reset();
  ds.select(addr);
  ds.write(0xBE);
 
  // Scratchpad reading
  for (byte i = 0; i < 9; i++) {
    data[i] = ds.read();
  }
  
  // Compute the temperature in degrees Celsius
  *temperature = (int16_t) ((data[1] << 8) | data[0]) * 0.0625; 
  
  // No error
  return READ_OK;
}

#ifdef INTERRUPT
void radioInterrupt() {
    // Ask the radio what caused the interrupt.  This also resets the IRQ pin on the
    // radio so a new interrupt can be triggered.
  got_interrupt = true;  // forward event handling back to main loop()
}

void handleInterrupt() {
  delayMicroseconds(250);
  bool tx_ds, tx_df, rx_dr;                 // declare variables for IRQ masks
  radio.whatHappened(tx_ds, tx_df, rx_dr);  // get values for IRQ masks
  // whatHappened() clears the IRQ masks also. This is required for
  // continued TX operations when a transmission fails.
  // clearing the IRQ masks resets the IRQ pin to its inactive state (HIGH)

  // Serial.print(F("\tdata_sent: "));
  // Serial.print(tx_ds);  // print "data sent" mask state
  // Serial.print(F(", data_fail: "));
  // Serial.print(tx_df);  // print "data fail" mask state
  // Serial.print(F(", data_ready: "));
  // Serial.println(rx_dr);  // print "data ready" mask state

  got_interrupt = false;   // reset this flag to prevent calling this function from loop()
}
#endif
