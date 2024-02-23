#define DEBUG_MODE  0
#define SERIAL_MODE ~DEBUG_MODE

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#include <avr/io.h>

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
#define I2C_BUFF_SIZE    4
#define SERIAL_BUFF_SIZE    4

#define IDX_VOLUME      0
#define IDX_TREBLE      1
#define IDX_BASS        2
#define IDX_BALANCE     3
#define IDX_MUTED       4
#define IDX_LOUDNESS    5

unsigned char startResult;
char i2cBuff[I2C_BUFF_SIZE];
char serialBuff[SERIAL_BUFF_SIZE];

volatile int rxLength = 0;
volatile boolean serialReady = false;


PT2313 audioChip;
int8_t values[6];
volatile bool saved = false;

enum CMD {
    SET_VOLUME = 1,
    SET_TREBLE,
    SET_BASS,
    SET_BALANCE,
    SET_MUTE,
    SET_LOUDNESS,
    CUSTOM = 100
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
  rxLength = min(numBytes, I2C_BUFF_SIZE);
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

    delay(1000);

    i2c_init();

    for (unsigned int i = 0; i < sizeof (values); i++) {
        values[i] = (int8_t) EEPROM.read(i);
    }

    audioChip.initialize(PT2313_ADDR, 0, false);    //source 1, mute off
    audioChip.source(0);                            //select your source 0...3
    audioChip.volume(values[IDX_VOLUME]);           //Vol 0...62 : 63=muted
    audioChip.treble(values[IDX_TREBLE]);           //treble -7...+7
    audioChip.bass(values[IDX_BASS]);               //bass -7...+7
    audioChip.balance(values[IDX_BALANCE]);         //-31...+31
    audioChip.mute(values[IDX_MUTED]);
    audioChip.loudness(values[IDX_LOUDNESS]);
//  audioChip.gain(gain); //gain 0...11.27 db
//  audioChip.loudness(loudness); //true or false  

    printVal("Initialized: ", (char*) values, 0, sizeof (values));

}


void processSerial() {
#if SERIAL_MODE

    while (serialReady) {

        while (Serial.available() > 0) {
            if (Serial.readBytes(serialBuff, SERIAL_BUFF_SIZE) < SERIAL_BUFF_SIZE) {
                serialReady = false;
                break;
            }
            uint8_t cmd = *((uint8_t *) serialBuff);
            int8_t val = *((int8_t *) (serialBuff + 2));

            printVal("Data: ", serialBuff, 0, 4);
            printVal("Command: ", cmd);
            printVal("Value: ", val);

            switch (cmd) {
                case SET_VOLUME:
                    values[IDX_VOLUME] = audioChip.volume(val);
                    printVal("Volume set: ", values[IDX_VOLUME]);
                    break;

                case SET_TREBLE:
                    values[IDX_TREBLE] = audioChip.treble(val);
                    printVal("Treble set: ", values[IDX_TREBLE]);
                    break;

                case SET_BASS:
                    values[IDX_BASS] = audioChip.bass(val);
                    printVal("Bass set: ", values[IDX_BASS]);
                    break;

                case SET_BALANCE:
                    values[IDX_BALANCE] = audioChip.balance(val);
                    printVal("Balance set: ", values[IDX_BALANCE]);
                    break;

                case SET_MUTE:
                    values[IDX_MUTED] = audioChip.mute(val == 1);
                    printVal("Muted: ", values[IDX_MUTED]);
                    break;

                case SET_LOUDNESS:
                    values[IDX_LOUDNESS] = audioChip.loudness(val == 1);
                  printVal("Loudness set: ", values[IDX_LOUDNESS]);
                break;

                case CUSTOM:
                    if (val == 0) {
                        for (unsigned int i = 0; i < sizeof (values); i++) {
                            values[i] = (int8_t) EEPROM.read(i);
                        }
                        printVal("Read from eeprom: ", (char*) values, 0, sizeof (values));
                    } else {
                        EEPROM.put(0, values);
                        printVal("Written to eeprom: ", (char*) values, 0, sizeof (values));
                    }

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
            printVal("Master data: ", i2cBuff, 0, I2C_BUFF_SIZE);

            for (uint8_t i = 0; i < rxLength; i++) {
                printVal("Writing data to slave: ", i2cBuff, 0, I2C_BUFF_SIZE);
                unsigned char result = i2c_write(i2cBuff[i]);
                if (result) {
                    printVal("Failed to write data: ", i2cBuff, 0, I2C_BUFF_SIZE);
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

    if (!saved) {
        int analogIn = analogRead(A7);
        if (analogIn > 200 && analogIn < 810) {
            EEPROM.put(0, values);
            saved = true;
            printVal("Saved to eeprom: ", (char*) values, 0, sizeof (values));
            printVal("Voltage: ", analogIn);
        }
    }

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


