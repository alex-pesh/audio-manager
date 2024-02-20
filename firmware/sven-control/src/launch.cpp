#define DEBUG_MODE  0
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
volatile boolean serialReady = false;
CyclicBuff serialBuff(64);


PT2313 audioChip;
int16_t volume = 33,
    bass = 5,
    treble = 5,
    balance = 0,
    gain = 3;
bool muted = false;
bool loudness = false;

enum CMD {
    SET_VOLUME = 1,
    SET_TREBLE,
    SET_BASS,
    SET_BALANCE,
    SET_LOUDNESS,
    SET_MUTE
};


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


void printVal(const char label[], int16_t value) {
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


void printVal(const char label[], const char *value, uint16_t offset, uint16_t size) {
#if SERIAL_MODE

    Serial.print(label);
    Serial.print(" ");
    for (uint16_t i = offset; i < offset + size; i++) {
      Serial.print((int8_t) value[i], DEC);
      Serial.print("(0x");
      Serial.print((int8_t) value[i], HEX);
      Serial.print(")");
      Serial.print("  ");
    }

    Serial.println();
    Serial.flush();

#endif  // SERIAL_MODE
}


void printBytes(unsigned n) {
#if SERIAL_MODE

    unsigned char bytes[4];
    bytes[0] = (n >> 24) & 0xFF;
    bytes[1] = (n >> 16) & 0xFF;
    bytes[2] = (n >> 8) & 0xFF;
    bytes[3] = n & 0xFF;

    Serial.print(bytes[0]);
    Serial.print(bytes[1]);
    Serial.print(bytes[2]);
    Serial.print(bytes[3]);

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

    delay(2000);

    Serial.println("Initialized");

}


void processSerial() {
#if SERIAL_MODE

    while (serialReady) {
        size_t cmdSize = 4;
        char buff[cmdSize];

        while (Serial.available() > 0) {
            if (Serial.readBytes(buff, cmdSize) < cmdSize) {
                serialReady = false;
                break;
            }
            uint8_t cmd = *((uint8_t *) buff);
            int8_t val = *((int8_t *) (buff + 2));

            printVal("Data: ", buff, 0, 4);
            printVal("Command: ", cmd);
            printVal("Value: ", val);

            switch (cmd) {
                case SET_VOLUME:
                    volume = audioChip.volume(val);
                    printVal("Volume set: ", volume);
                    break;

                case SET_TREBLE:
                    treble = audioChip.treble(val);
                    printVal("Treble set: ", treble);
                    break;

                case SET_BASS:
                    bass = audioChip.bass(val);
                    printVal("Bass set: ", bass);
                    break;

                case SET_BALANCE:
                    balance = audioChip.balance(val);
                    printVal("Balance set: ", balance);
                    break;

                case SET_LOUDNESS:
                  loudness = audioChip.loudness(val == 1);
                  Serial.print("Loudness set: ");
                  Serial.println(loudness);
                break;

                case SET_MUTE:
                    muted = audioChip.mute(val == 1);
                    printVal("Muted: ", muted);
                    break;

                default:
                    printVal("Unknown command: ", cmd);
            }
        }

        serialReady = false;
    }

#endif  // SERIAL_MODE
}


void loop() {

  while (rxLength > 0) {

    startResult = i2c_start(ADDR_TARGET);
    if (!startResult) {
      printVal("Master data: ", i2cBuff, 0, BUFF_SIZE);

      for (uint8_t i = 0; i < rxLength; i++) {
        printVal("Writing data to slave: ", i2cBuff, 0, BUFF_SIZE);
        unsigned char result = i2c_write(i2cBuff[i]);
        if (result) {
          printVal("Failed to write data: ", i2cBuff, 0, BUFF_SIZE);
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


void serialEvent() {
#if SERIAL_MODE

  serialReady = true;

    int available = Serial.available();
    printVal("Serial event. Available: ", available);
/*



  while ((available = Serial.available()) > 0) {
    // int readVal = Serial.read();
    // serialBuff.putInt(readVal);
    // int readVal = Serial.readBytes(serialBuff.buff, 4);

    char c = Serial.read();
    serialBuff.putChar(c);

    printVal("Read value to buff: ", c);
    printVal("WriteIdx: ", serialBuff.writeIdx);
    printVal("ReadIdx: ", serialBuff.readIdx);
  }

  printVal("Value in buff by readIdx: ", serialBuff.buff, serialBuff.readIdx, 4);
  // printVal("Read int: ", serialBuff.getInt());
*/

#endif  // SERIAL_MODE
}


