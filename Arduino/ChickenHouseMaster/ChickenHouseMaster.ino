#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ArduinoJson.h>
#include <ChickenHouseCommon.h>

byte receptionBuffer[32];
ChickenHouseStatus* chickenHouseStatus = (ChickenHouseStatus*)receptionBuffer;
ChickenHouseCommand chickenHouseCommand;
#define pinCE   7             // On associe la broche "CE" du NRF24L01 à la sortie digitale D7 de l'arduino
#define pinCSN  8             // On associe la broche "CSN" du NRF24L01 à la sortie digitale D8 de l'arduino
RF24 radio(pinCE, pinCSN, 1000000);    // Instanciation du NRF24L01
const byte pipeW[6] = PIPE_MASTER_TO_SLAVE; 
const byte pipeR[6] = PIPE_SLAVE_TO_MASTER; 
uint16_t      secret = 0;

#define CMD_SET_PA_LEVEL  "SetPALevel "
#define CMD_SET_DATA_RATE "SetDataRate "
#define CMD_SET_CHANNEL   "SetChannel "

#define TIMEOUT_MS  2000

char master_version[12];

void setup() {
  randomSeed(analogRead(A7));
  secret = random(0xFFFF);

  sprintf(master_version, "%u.%u.%u", 
    VERSION_A, 
    VERSION_B, 
    VERSION_C);

  Serial.begin(115200);
  
  while (!radio.begin()) {
    Serial.println("RF module not responding");
    delay(1000);
  }

  radio.setChannel(111); 		// en remplaçant « x » par une valeur comprise entre 0 et 125
  radio.setAutoAck(true);
  radio.setPALevel(RF24_PA_MIN);	// en remplaçant « xxx » par RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, ou RF24_PA_MAX
  radio.setDataRate(RF24_2MBPS);	// en remplaçant « xxx » par RF24_250KBPS, RF24_1MBPS, ou encore, RF24_2MBPS
  radio.openWritingPipe(pipeW);      // Ouverture du "tunnel1" en ÉCRITURE (émission radio)
  radio.openReadingPipe(1, pipeR);
  radio.startListening();		// permet de pouvoir utiliser la fonction « read » par la suite
}

void loop() {
  // Consume any unsollicited incoming message, do nothing with them for this version
  while (radio.available()) {
    uint8_t size = radio.getPayloadSize();
    radio.read(receptionBuffer, size);
    Serial.print(size);
    Serial.println(" bytes received and discarded");
  }

  // treat any command comming from the serial port (Home Assistant for instance)
  while (Serial.available() > 0) {
    String command = Serial.readString();  //read until timeout
    command.trim();                        // remove any \r \n whitespace at the end of the String

    bool ok = false;
    JsonDocument jsonBuffer;
    if (command == "CloseDoor") {
      chickenHouseCommand.command = Command::CLOSE_DOOR;
      ok = SendCommandOverRF(chickenHouseCommand, TIMEOUT_MS, jsonBuffer);
    }
    else if (command == "OpenDoor") {
      chickenHouseCommand.command = Command::OPEN_DOOR;
      ok = SendCommandOverRF(chickenHouseCommand, TIMEOUT_MS, jsonBuffer);
    }
    else if (command == "GetStatus") {
      jsonBuffer["master_version"] = master_version;
      chickenHouseCommand.command = Command::GET_STATUS;
      ok = SendCommandOverRF(chickenHouseCommand, TIMEOUT_MS, jsonBuffer);
    }
    else if (command.startsWith(CMD_SET_PA_LEVEL)) {
      radio.stopListening();
      String value = command.substring(strlen(CMD_SET_PA_LEVEL));
      if (value == "MIN") {
        radio.setPALevel(RF24_PA_MIN);
        ok = true;
      }
      else if (value == "LOW") {
        radio.setPALevel(RF24_PA_LOW);
        ok = true;
      }
      else if (value == "HIGH") {
        radio.setPALevel(RF24_PA_HIGH);
        ok = true;
      }
      else if (value == "MAX") {
        radio.setPALevel(RF24_PA_MAX);
        ok = true;
      }
      radio.startListening();
    }
    else if (command.startsWith(CMD_SET_DATA_RATE)) {
      radio.stopListening();
      String value = command.substring(strlen(CMD_SET_DATA_RATE));
      if (value == "250KBPS") {
        radio.setDataRate(RF24_250KBPS);
        ok = true;
      }
      else if (value == "1MBPS") {
        radio.setPALevel(RF24_1MBPS);
        ok = true;
      }
      else if (value == "2MBPS") {
        radio.setPALevel(RF24_2MBPS);
        ok = true;
      }
      radio.startListening();
    }
    else if (command.startsWith(CMD_SET_CHANNEL)) {
      radio.stopListening();
      String value = command.substring(strlen(CMD_SET_CHANNEL));
      uint8_t channel = value.toInt();
      radio.setChannel(channel);
      radio.startListening();
      ok = true;
    }
    else {
      String msg = "Unknown command [";
      msg += command;
      msg += "]";
      jsonBuffer["reason"] = msg;
    }

    jsonBuffer["result"] = ok ? "OK" : "ERROR";
    serializeJson(jsonBuffer, Serial);
    Serial.println();
    Serial.flush();
  }

  delay(5);
}

bool SendCommandOverRF(ChickenHouseCommand& command, const unsigned long timeout_ms, JsonDocument& jsonDoc) {
  command.secret = secret;
  uint16_t expected_secret = ComputeSecret(secret);

  radio.stopListening();
  delay(50);
  bool ok = radio.write(&command, sizeof(ChickenHouseCommand));
  radio.startListening();		// permet de pouvoir utiliser la fonction « read » par la suite

  if (!ok) {
    jsonDoc["reason"] = "Unable to send command to the slave module";
    return false;
  }
  delay(50);

  unsigned long starttime_ms = millis();

  while ((millis() - starttime_ms) < timeout_ms) {
    while (radio.available()) {
      uint8_t size = radio.getPayloadSize();
      radio.read(receptionBuffer, size);

      if (chickenHouseStatus->secret != expected_secret) {
        jsonDoc["reason"] = "Wrong secret received from slave module";
        return false;
      }

      secret = expected_secret + 1;

      char version[12];
      sprintf(version, "%u.%u.%u", 
        chickenHouseStatus->version_a, 
        chickenHouseStatus->version_b, 
        chickenHouseStatus->version_c);
      jsonDoc["slave_version"] = version;
      jsonDoc["temperature"] = chickenHouseStatus->temperature;
      switch (chickenHouseStatus->stateMachine) {
        case StateMachine::CLOSED:
        jsonDoc["door_state"] = "CLOSED";
        break;
        case StateMachine::OPENING:
        jsonDoc["door_state"] = "OPENING";
        break;
        case StateMachine::OPENED:
        jsonDoc["door_state"] = "OPENED";
        break;
        case StateMachine::CLOSING:
        jsonDoc["door_state"] = "CLOSING";
        break;
        default:
        jsonDoc["door_state"] = "ERROR";
        break;
      }

      return true;
    }

    delay(100);
  }

  jsonDoc["reason"] = "Unable to receive the slave module answer";
  return false;
}