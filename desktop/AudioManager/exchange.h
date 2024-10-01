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
    SYNC = 100,
    DISCONNECT,
    CUSTOM = 200
};

enum DataType : uint8_t {
    COMMAND = 1,
    MESSAGE = 2
};

struct DataHead {
    DataType type;
    uint8_t length;
};

struct DataPacket {
    DataHead head;
    char* payload;
};

struct Values {
    int8_t volume = 33;
    int8_t treble = 5;
    int8_t bass = 5;
    int8_t balance = 0;
    uint8_t mute_loud = 0;
};

#endif //SVEN_CONTROL_EXCHANGE_H
