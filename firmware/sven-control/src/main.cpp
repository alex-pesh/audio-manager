#define DEBUG_MODE  1 
#define SERIAL_MODE ~DEBUG_MODE 

#include <Arduino.h>
#include <Wire.h>

#include <avr/io.h>

#include <stdio.h>
#include "i2cmaster.h"
#include "PT2313.h"
#include "buffers.cpp"


#if DEBUG_MODE
  #include "avr8-stub.h"
  #include "app_api.h" // only needed with flash breakpoints
#endif  // DEBUG_MODE

#define ADDR_SLAVE   0x44       // Self I2C slave address
#define ADDR_TARGET  0x88       // Target I2C device address
#define PT2313_ADDR  ADDR_TARGET // Redefined value of PT2313.h
#define SDA_PIN      4
#define SCL_PIN      5
#define BUFF_SIZE    4


unsigned char startResult;
char i2cBuff[BUFF_SIZE];
volatile int rxLength = 0;
CyclicBuff serialBuff(4);


PT2313 audioChip;
int volume = 33, 
    bass = 5, 
    treble = 5, 
    balance = 0,
    gain = 3;
bool muted = false;
bool loudness = true;
    

void printBuff() {
#if SERIAL_MODE
  for (uint8_t i = 0; i < rxLength; i++) {
    Serial.print(i2cBuff[i], HEX);
    Serial.print("  ");
  }

  Serial.println();
  Serial.flush();
  
#endif  // SERIAL_MODE
}

void printVal(const char label[], int value) {
#if SERIAL_MODE

    Serial.print(label);
    Serial.print(" ");
    Serial.print(value, DEC);  
    Serial.print("(0x");
    Serial.print(value, HEX);
    Serial.print(")");
    Serial.println();
    Serial.flush();
#endif  // SERIAL_MODE
}


void printVal(const char label[], const char *value) {
#if SERIAL_MODE

    Serial.print(label);
    Serial.print(" ");
    for (uint8_t i = 0; i < sizeof(value); i++) {
      Serial.print((uint8_t) value[i], DEC);  
      Serial.print("(0x");
      Serial.print((uint8_t) value[i], HEX);
      Serial.print(")");
      Serial.print("  ");
    }

    Serial.println();
    Serial.flush();

#endif  // SERIAL_MODE
}



void wireReceiveHandler(int numBytes) {
  rxLength = min(numBytes, BUFF_SIZE);
  while (Wire.available()) {
    Wire.readBytes(i2cBuff, rxLength);
  }
}


void setup() {
  
#if DEBUG_MODE
  debug_init();
#endif  // DEBUG_MODE

#if SERIAL_MODE
  Serial.begin(9600);
  Serial.setTimeout(0);
#endif  // SERIAL_MODE

  Wire.begin(ADDR_SLAVE);
  Wire.onReceive(wireReceiveHandler);

  i2c_init();

  audioChip.initialize(PT2313_ADDR, 0, false); //source 1, mute off
  audioChip.source(0); //select your source 0...3
  audioChip.volume(volume); //Vol 0...62 : 63=muted
  audioChip.bass(bass); //bass -7...+7
  audioChip.treble(treble); //treble -7...+7
  audioChip.balance(balance); //-31...+31

//  audioChip.gain(gain); //gain 0...11.27 db
//  audioChip.loudness(loudness); //true or false  

}



void processSerial() {
  #if SERIAL_MODE

  int serialData = serialBuff.getInt();
  if (serialData <= 0) {
    return;
  }

  char cmd = serialData >> 16;
  int val = (serialData & ~0xFFFF0000);

  // Serial.print("Received bytes: ");
  // Serial.print(readLength);
  // Serial.println();

  printVal("Data: ", serialData);
  printVal("Command: ", cmd);
  printVal("Value: ", val);


  switch(cmd) {
    case 1:
      volume = audioChip.volume(val);
      printVal("Volume set: ", volume);
    break;
    
    case 2: 
      treble = audioChip.treble(val);
      printVal("Treble set: ", treble);
    break;

    case 3: 
      bass = audioChip.bass(val);
      printVal("Bass set: ", bass);
    break;

    case 4: 
      balance = audioChip.balance(val);
      printVal("Balance set: ", balance);
    break;

/*
    case 5: 
      loudness = audioChip.loudness(!loudness);
      Serial.print("loudness: ");
      Serial.println(loudness);
    break;
*/   
    case 6: 
      muted = audioChip.mute(val == 1);
      printVal("Mited: ", muted);
    break;

    default:
      printVal("Unknown command: ", cmd);
  }

#endif  // SERIAL_MODE
}

void loop() {

  while (rxLength > 0) {

    startResult = i2c_start(ADDR_TARGET);
    if (!startResult) {
      printVal("Master data: ", i2cBuff);

      for (uint8_t i = 0; i < rxLength; i++) {
        printVal("Writing data to slave: ", i2cBuff);
        unsigned char result = i2c_write(i2cBuff[i]);
        if (result) {
          printVal("Failed to write data: ", i2cBuff);
        }
      }
    } else {
      // Serial.println("failed to issue start condition, possibly no device found");
    }

    i2c_stop();

    rxLength = 0;
  }

  delay(10);

  processSerial();

  delay(10);

	// Wire.beginTransmission(ADDR_SLAVE);
	// Wire.write(0);
	// Wire.endTransmission();
}





void printBytes(unsigned n) {
    unsigned char bytes[4];
    bytes[0] = (n >> 24) & 0xFF;
    bytes[1] = (n >> 16) & 0xFF;
    bytes[2] = (n >> 8) & 0xFF;
    bytes[3] = n & 0xFF;

    Serial.print(bytes[0]);
    Serial.print(bytes[1]);
    Serial.print(bytes[2]);
    Serial.print(bytes[3]);
}


void serialEvent() {
#if SERIAL_MODE

  char buf[4];
  int available = 0;
  size_t readLength = 0;

  while ((available = Serial.available()) > 0) {
      serialBuff.put(Serial.read());
      // readLength = Serial.readBytes(buf, available);
  }


/*
  if (readLength <= 0) return;

  char cmd = buf[1];
  int val = 0 | (buf[2] << 8 | buf[3]);

  // cmd = Serial.read();
  // val = Serial.parseInt();

  Serial.print("Received bytes: ");
  Serial.print(readLength);
  Serial.println();

  printVal("Data: ", buf);
  printVal("Command: ", cmd);
  printVal("Value: ", val);

  // Serial.println();
  // Serial.print(buf);

  // Serial.print("; Command: ");
  // Serial.print(cmd);
  // Serial.print("; Vaule: ");
  // Serial.print(val);
  // Serial.println();

  switch(cmd) {
    case 1:
      volume = audioChip.volume(val);
      printVal("Volume set: ", volume);
    break;
    
    case 2: 
      treble = audioChip.treble(val);
      printVal("Treble set: ", treble);
    break;

    case 3: 
      bass = audioChip.bass(val);
      printVal("Bass set: ", bass);
    break;

    case 4: 
      balance = audioChip.balance(val);
      printVal("Balance set: ", balance);
    break;

    case 6: 
      muted = audioChip.mute(val == 1);
      printVal("Mited: ", muted);
    break;

    default:
      printVal("Unknown command: ", cmd);
  }
*/

#endif  // SERIAL_MODE
}


