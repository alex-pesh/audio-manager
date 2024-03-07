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

#endif //SVEN_CONTROL_EXCHANGE_H
