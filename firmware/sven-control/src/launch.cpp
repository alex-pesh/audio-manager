#define DEBUG_MODE  0
#define SERIAL_MODE ~DEBUG_MODE

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#include <avr/io.h>

#include "i2cmaster.h"
#include "PT2313.h"
#include "buffers.cpp"
#include "exchange.h"

#include "EncButton.h"
#include "TimerOne.h"


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

#define PIN_INTERRUPT 2

const int ledPin = 12;

unsigned char startResult;
char i2cBuff[I2C_BUFF_SIZE];
char serialBuff[SERIAL_BUFF_SIZE];

volatile int rxLength = 0;
volatile boolean serialReady = false;


PT2313 audioChip;
volatile bool saved = false;


struct Encoders {
    Encoder volume;
    Encoder treble;
    Encoder bass;

    void enableISR(const bool value) {
        volume.setEncISR(value);
        treble.setEncISR(value);
        bass.setEncISR(value);
    }

} encoders;


void encodersTimerISR() {
    encoders.volume.tickISR();
    encoders.treble.tickISR();
    encoders.bass.tickISR();
};


struct Values {
    int8_t volume = 33;
    int8_t treble = 5;
    int8_t bass = 5;
    int8_t balance = 0;
    uint8_t mute_loud = 0;

    void muted(bool value) {
        value ? bitSet(mute_loud, 0) : bitClear(mute_loud, 0);
    }

    int8_t muted() const {
        return bitRead(mute_loud, 0);
    }

    void loudnessOn(bool value) {
        value ? bitSet(mute_loud, 1) : bitClear(mute_loud, 1);
    }

    uint8_t loudnessOn() const {
        return bitRead(mute_loud, 1);
    }

} values;


void saveValues() {
    EEPROM.put(0, values);
}



#define \
printDecimal(value) \
    Serial.print((value), DEC); \
    Serial.print("(0x");        \
    Serial.print((value), HEX); \
    Serial.print(")");          \
    Serial.print("  ");


void printValues(const char label[]) {
#if SERIAL_MODE

    Serial.print(label);
    Serial.print(" ");
    printDecimal(values.volume)
    printDecimal(values.treble)
    printDecimal(values.bass)
    printDecimal(values.balance)
    printDecimal(values.mute_loud)
    Serial.println();
    Serial.flush();

#endif  // SERIAL_MODE
}



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
    printDecimal(value);
    Serial.println();
    Serial.flush();

#endif  // SERIAL_MODE
}


void sendValues() {
#if SERIAL_MODE

    Serial.write(CMD::SYNC);
    Serial.write(values.volume);
    Serial.write(values.treble);
    Serial.write(values.bass);
    Serial.write(values.balance);
    Serial.write(values.mute_loud);
    Serial.flush();

#endif  // SERIAL_MODE
}


void printVal(const char label[], const char *value, uint16_t offset, uint16_t size) {
#if SERIAL_MODE

    Serial.print(label);
    Serial.print(" ");
    for (uint16_t i = offset; i < offset + size; i++) {
        printDecimal((int8_t) value[i]);
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
/*

    Serial.print(bytes[0]);
    Serial.print(bytes[1]);
    Serial.print(bytes[2]);
    Serial.print(bytes[3]);
*/
#endif  // SERIAL_MODE
}



#define CMD_DELAY 150

void restoreValues() {

    values.volume = (int8_t) EEPROM.read(0);
    values.treble = (int8_t) EEPROM.read(1);
    values.bass = (int8_t) EEPROM.read(2);
    values.balance = (int8_t) EEPROM.read(3);
    values.mute_loud = (int8_t) EEPROM.read(4);


    audioChip.source(0);
    delay(CMD_DELAY);

    // // values.volume = 
    audioChip.volume(values.volume);
    delay(CMD_DELAY);
  
    values.treble = 
    audioChip.treble(values.treble);
    delay(CMD_DELAY);

    // values.bass = 
    audioChip.bass(values.bass);
    delay(CMD_DELAY);

    // values.balance = 
    audioChip.balance(values.balance);
    delay(CMD_DELAY);

    audioChip.mute(values.muted());
    delay(CMD_DELAY);
    
    // audioChip.loudness(values.loudnessOn());
    // delay(CMD_DELAY);

//  audioChip.gain(gain); //gain 0...11.27 db
   
    Serial.println("-- Setting encoders...");

    encoders.volume.counter = values.volume;
    encoders.treble.counter = values.treble;
    encoders.bass.counter = values.bass;
}


void wireReceiveHandler(int numBytes) {
  rxLength = min(numBytes, I2C_BUFF_SIZE);
  while (Wire.available()) {
    Wire.readBytes(i2cBuff, rxLength);
  }
}


int ledState = LOW;
unsigned long previousMillis = 0;
long interval = 200;

void blink() {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);

/*
    unsigned int cnt = 3;
    while (cnt-- > 0) {
        unsigned long currentMillis = millis();

        if(currentMillis - previousMillis > interval) {
            previousMillis = currentMillis;

            if (ledState == LOW) {
                ledState = HIGH;
            } else {
                ledState = LOW;
            }

            digitalWrite(ledPin, ledState);
        }
    }
*/
}

void powerInterrupt() {
    blink();
}


// Encoder enc(6, 7, INPUT_PULLUP);
// void encIS() {
//     enc.tickISR();
// }



void setup() {

#if DEBUG_MODE
    debug_init();
#endif  // DEBUG_MODE

#if SERIAL_MODE
    Serial.begin(9600);
    Serial.setTimeout(1000);
#endif  // SERIAL_MODE

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(ledPin, OUTPUT);

/*
    pinMode(PIN_INTERRUPT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), powerInterrupt, FALLING);
*/

/* 
    Serial.println("Init wire...");
    Wire.begin(ADDR_SLAVE);
    Wire.onReceive(wireReceiveHandler);
 */

    Serial.println("Init encoders...");
    encoders = {
        .volume = Encoder(6, 7, INPUT_PULLUP),
        .treble = Encoder(8, 9, INPUT_PULLUP),
        .bass = Encoder(10, 11, INPUT_PULLUP),
    };
    encoders.enableISR(false);

    Serial.println("Init timer...");
    Timer1.initialize(1000);    // установка таймера на каждые 1000 микросекунд (= 1 мс)
    Timer1.attachInterrupt(encodersTimerISR);


    Serial.println("Waitig for remote device init...");

    delay(1000); // wait for PT2313 I2C remote device to be initialized

    // Serial.println("Init i2c...");
    // i2c_init();


    Serial.println("Init audio...");
    audioChip.initialize(ADDR_TARGET, 0, false);
    
    Serial.println("Init values...");
    restoreValues();

    printValues("Initialized with values: ");

    digitalWrite(ledPin, LOW);

}

/* 
void processI2CBuff() {

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
}
 */

void processSerialBuff() {
//#if SERIAL_MODE

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
                    values.volume = audioChip.volume(val);
                    printVal("Volume set: ", values.volume);
                    break;

                case SET_TREBLE:
                    values.treble = audioChip.treble(val);
                    printVal("Treble set: ", values.treble);
                    break;

                case SET_BASS:
                    values.bass = audioChip.bass(val);
                    printVal("Bass set: ", values.bass);
                    break;

                case SET_BALANCE:
                    values.balance = audioChip.balance(val);
                    printVal("Balance set: ", values.balance);
                    break;

                case SET_MUTE:
                    values.muted(audioChip.mute(val == 1));
                    printVal("Muted: ", values.muted());
                    break;

                case SET_LOUDNESS:
                    values.loudnessOn(audioChip.loudness(val == 1));
                    printVal("Loudness set: ", values.loudnessOn());
                break;

                case SYNC:
//                    sendValues();
                    printValues("Current values: ");

                    break;

                case CUSTOM:
                    if (val == 0) {
                        restoreValues();
                        printValues("Read from eeprom: ");
//                        DataPacket packet = DataPacket::asMessage("Read from eeprom");
//                        Serial.print(packet);
                    } else {
                        saveValues();
                        printValues("Written to eeprom: ");
                    }

                    break;

                default:
                    printVal("Unknown command: ", cmd);
            }
        }

        serialReady = false;
    }

//#endif  // SERIAL_MODE
}



void processEncoders() {

    if (encoders.volume.tick()) {
        values.volume = encoders.volume.counter =
        audioChip.volume(encoders.volume.counter);

        printVal("Volume set: ", values.volume);
    }
    
    if (encoders.treble.tick()) {
        values.treble = encoders.treble.counter =
        audioChip.treble(encoders.treble.counter);

        printVal("Treble set: ", values.treble);
    }

    if (encoders.bass.tick()) {
        values.bass = encoders.bass.counter =
        audioChip.bass(encoders.bass.counter);

        printVal("Bass set: ", values.bass);
    }
}



void loop() {

    processEncoders();
    delay(10);

    processSerialBuff();
    delay(10);


    // processI2CBuff();
    // delay(10);

    if (!saved) {
        int analogIn = analogRead(A7);
        if (analogIn > 512) { // 2.5V
            saveValues();
            saved = true;
/* 
            if (ledState == LOW) {
                ledState = HIGH;
            } else {
                ledState = LOW;
            }
            digitalWrite(ledPin, ledState);
            
            delay (200);
             */

            printValues("Saved to eeprom: ");
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


