#define DEBUG_MODE  0
#define SERIAL_MODE ~DEBUG_MODE

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/io.h>

#include "i2cmaster.h"
#include "PT2313.h"
#include "EncButton.h"
#include "TimerOne.h"
// #include <IRremote.h>

#include "exchange.h"


#if DEBUG_MODE
  #include "avr8-stub.h"
  #include "app_api.h" // only needed with flash breakpoints
#endif  // DEBUG_MODE

#define SERIAL_RX_BUFFER_SIZE 256
#define ADDR_SLAVE   0x44       // Self I2C slave address
#define ADDR_TARGET  0x88       // Target I2C device address
#define PT2313_ADDR  ADDR_TARGET // Redefined value of PT2313.h
#define SDA_PIN      4
#define SCL_PIN      5
#define I2C_BUFF_SIZE    4
#define SERIAL_BUFF_SIZE    4

#define PIN_INTERRUPT 2
#define PIN_IR_RECEIVE 12

#define EXT_POWER_THRESHOLD 512  // ~2.5V


unsigned char startResult;
char i2cBuff[I2C_BUFF_SIZE];
char serialBuff[SERIAL_BUFF_SIZE];

volatile int rxLength = 0;
volatile boolean serialReady = false;
volatile int8_t deviceState = LOW;


PT2313 audioChip;
volatile bool 
    saved = false,
    changed = false;


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



#define \
printDecimal(value) \
    Serial.print((value), DEC); \
    Serial.print("(0x");        \
    Serial.print((value), HEX); \
    Serial.print(")");          \
    Serial.print("  ");


void printValues(const char label[], const Values &values) {
#if SERIAL_MODE

    Serial.write(CMD::CUSTOM);
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

  Serial.write(CMD::CUSTOM);

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

    Serial.write(CMD::CUSTOM);
    Serial.print(label);
    Serial.print(" ");
    printDecimal(value);
    Serial.println();
    Serial.flush();

#endif  // SERIAL_MODE
}


void sendValues(Values const &values) {
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


void sendValueCmd(CMD cmd, const int8_t &value) {
#if SERIAL_MODE

    Serial.write(cmd);
    Serial.write(value);
    Serial.flush();

#endif  // SERIAL_MODE    
}


void printVal(const char label[], const char *value, uint16_t offset, uint16_t size) {
#if SERIAL_MODE

    Serial.write(CMD::CUSTOM);
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



void saveValues(Values &values) {
    EEPROM.put(0, values);
    saved = true;
    changed = false;
}

void restoreValues(Values &values) {
    values.volume = (int8_t) EEPROM.read(0);
    values.treble = (int8_t) EEPROM.read(1);
    values.bass = (int8_t) EEPROM.read(2);
    values.balance = (int8_t) EEPROM.read(3);
    values.mute_loud = (int8_t) EEPROM.read(4);
}


#define CMD_DELAY 150

void restoreValues() {

    restoreValues(values);

    audioChip.source(0);
    delay(CMD_DELAY);

    values.volume = 
    audioChip.volume(values.volume);
    delay(CMD_DELAY);
  
    values.treble = 
    audioChip.treble(values.treble);
    delay(CMD_DELAY);

    values.bass = 
    audioChip.bass(values.bass);
    delay(CMD_DELAY);

    values.balance = 
    audioChip.balance(values.balance);
    delay(CMD_DELAY);

    audioChip.mute(values.muted());
    delay(CMD_DELAY);

    // audioChip.loudness(values.loudnessOn());
    // audioChip.gain(gain); //gain 0...11.27 db

    encoders.volume.counter = values.volume;
    encoders.treble.counter = values.treble;
    encoders.bass.counter = values.bass;

    changed = false;
}


void wireReceiveHandler(int numBytes) {
  rxLength = min(numBytes, I2C_BUFF_SIZE);
  while (Wire.available()) {
    Wire.readBytes(i2cBuff, rxLength);
  }
}



/* 
int ledState = LOW;
unsigned long previousMillis = 0;
long interval = 200;

void blink(int count) {
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

            digitalWrite(LED_BUILTIN, ledState);
        }
    }
}
*/


void blink(int count) {
    while (count > 0) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
        count--;
    }
}


void powerInterrupt() {
    deviceState = LOW;
}

boolean isDevicePowerDown() {
    int analogIn = analogRead(A7);
    return analogIn >= EXT_POWER_THRESHOLD;
}




void setup() {

#if DEBUG_MODE
    debug_init();
#endif  // DEBUG_MODE

#if SERIAL_MODE
    Serial.begin(9600, SERIAL_8N2);
    // Serial.setTimeout(0);
#endif  // SERIAL_MODE

    pinMode(PIN_INTERRUPT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), powerInterrupt, HIGH);


/* 
    Serial.println("Init wire...");
    Wire.begin(ADDR_SLAVE);
    Wire.onReceive(wireReceiveHandler);
 */

    // pinMode(LED_BUILTIN, OUTPUT);
    // digitalWrite(LED_BUILTIN, LOW);


    // Serial.println("Init IrReceiver...");
    // IrReceiver.begin(PIN_IR_RECEIVE, false);

    // Serial.println("Init encoders...");
    encoders = {
        .volume = Encoder(6, 7, INPUT_PULLUP),
        .treble = Encoder(8, 9, INPUT_PULLUP),
        .bass = Encoder(10, 11, INPUT_PULLUP),
    };
    encoders.enableISR(false);


    // Serial.println("Init timer...");
    Timer1.initialize(1000);    // установка таймера на каждые 1000 микросекунд (= 1 мс)
    Timer1.attachInterrupt(encodersTimerISR);

    // Serial.println("Waitig for remote device to init...");
    delay(1000); // wait for PT2313 I2C remote device to be initialized


    // Serial.println("Init i2c...");
    // i2c_init();

    // Serial.println("Init audio...");
    audioChip.initialize(ADDR_TARGET, 0, false);
    

    // Serial.println("Restoring values...");
    restoreValues();

    deviceState = HIGH;

    sendValues(values);
    printValues("Initialized with values: ", values);

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

            // printVal("Data: ", serialBuff, 0, 4);
            // printVal("Command: ", cmd);
            // printVal("Value: ", val);

            switch (cmd) {
                case SET_VOLUME:
                    values.volume = encoders.volume.counter =
                    audioChip.volume(val);
                    changed = true;
                    printVal("Volume set: ", values.volume);
                    break;

                case SET_TREBLE:
                    values.treble = encoders.treble.counter =
                    audioChip.treble(val);
                    changed = true;
                    printVal("Treble set: ", values.treble);
                    break;

                case SET_BASS:
                    values.bass = encoders.bass.counter =
                    audioChip.bass(val);
                    changed = true;
                    printVal("Bass set: ", values.bass);
                    break;

                case SET_BALANCE:
                    values.balance = audioChip.balance(val);
                    changed = true;
                    printVal("Balance set: ", values.balance);
                    break;

                case SET_MUTE:
                    values.muted(audioChip.mute(val == 1));
                    changed = true;
                    printVal("Muted: ", values.muted());
                    break;

                case SET_LOUDNESS:
                    values.loudnessOn(audioChip.loudness(val == 1));
                    changed = true;
                    printVal("Loudness set: ", values.loudnessOn());
                    break;

                case SYNC:
                    switch (val) {
                        case 0:
                            sendValues(values);
                            break;

                        case 1: {
                            Values values;
                            restoreValues(values);
                            // sendValues(values);
                            printValues("Read from eeprom: ", values);
                            // DataPacket packet = DataPacket::asMessage("Read from eeprom");
                            break;
                        }

                        case 2:
                            saveValues(values);
                            changed = false;
                            printValues("Written to eeprom: ", values);
                            break;
                    }

                    break;

                case CUSTOM:
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
        encoders.volume.counter = values.volume = 
        audioChip.volume(encoders.volume.counter);
        changed = true;

        sendValueCmd(CMD::SET_VOLUME, values.volume);
        // printVal("Volume set: ", values.volume);
    }
    
    if (encoders.treble.tick()) {
        encoders.treble.counter = values.treble = 
        audioChip.treble(encoders.treble.counter);
        changed = true;

        sendValueCmd(CMD::SET_TREBLE, values.treble);
        // printVal("Treble set: ", values.treble);
    }

    if (encoders.bass.tick()) {
        encoders.bass.counter = values.bass = 
        audioChip.bass(encoders.bass.counter);
        changed = true;

        sendValueCmd(CMD::SET_BASS, values.bass);
        // printVal("Bass set: ", values.bass);
    }
}


void processIR() {
/* 
if (IrReceiver.decode()) {
        Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
        IrReceiver.printIRResultShort(&Serial);                       // Print complete received data in one line
        IrReceiver.printIRSendUsage(&Serial);                         // Print the statement required to send this data

        IrReceiver.resume(); // Enable receiving of the next value
    }
 */
}


void loop() {

    if (deviceState == LOW) {
        if (changed && !saved) {
            saveValues(values);
            saved = true;
            changed = false;

            sendValueCmd(CMD::DISCONNECT, 1);
        }

        sendValueCmd(CMD::DISCONNECT, 0);
    }

    processEncoders();
    delay(1);

    processSerialBuff();
    delay(1);

    processIR();
    delay(1);

    // processI2CBuff();
    // delay(10);


}



void serialEvent() {
  serialReady = true;
}

