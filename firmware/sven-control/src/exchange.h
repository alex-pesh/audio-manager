//
// Created by alexander on 06.03.24.
//

#ifndef SVEN_CONTROL_EXCHANGE_H
#define SVEN_CONTROL_EXCHANGE_H

#include <stdint.h>

enum CMD : uint8_t {
    SET_VOLUME = 1,
    SET_TREBLE,
    SET_BASS,
    SET_BALANCE,
    SET_MUTE,
    SET_LOUDNESS,
    SET_GAIN,
    SET_SOURCE,
    SYNC = 100,
    DISCONNECT,
    CUSTOM = 200
};

enum DataType : uint8_t {
    COMMAND = 1,
    MESSAGE = 2
};

struct DataHead : Printable {
    DataType type;
    uint8_t length;

    size_t printTo(Print &p) const override {
        size_t count = 0;
        count += p.write(type);
        count += p.write(length);

        return count;
    }
};

struct DataPacket : Printable {
    DataHead head;
    char* payload;

    size_t printTo(Print &p) const override {
        size_t count = 0;
        count += head.printTo(p);
        if (head.length > 0 && payload) {
            count += p.write((uint8_t *) payload, head.length);
        }

        return count;
    }

    static DataPacket asMessage(char* msg) {
        DataPacket packet;
        packet.head.type = MESSAGE;
        packet.head.length = strlen(msg);
        packet.payload = msg;

        return packet;
    }
};


struct Values {
    int8_t volume = 33;
    int8_t treble = 5;
    int8_t bass = 5;
    int8_t balance = 0;
    int8_t source = 2;
    uint8_t mute_loud = 0;
    uint8_t gain = 0;
};

#endif //SVEN_CONTROL_EXCHANGE_H
