#include <stdint.h>
#include <string.h>

//#include <cstdint>
//#include <cstring>

#define int_size sizeof (uint32_t)

class CyclicBuff {

private:


public:
    char* buff;
    size_t buffSize = 0;
    size_t readIdx = 0;
    size_t writeIdx = 0;

    /**
     * @param size Buffer size in bytes
    */
    explicit CyclicBuff(size_t size) {
        buffSize = size + 1;
        buff = new char[buffSize];
    }

    ~CyclicBuff() {
        delete [] buff;
    }

    uint8_t putChar(char value) {
        size_t nextIdx = (writeIdx + 1) % buffSize;

        if (nextIdx == readIdx) {
            return 0;
        }

        buff[writeIdx] = value;
        writeIdx = nextIdx;

        return 1;
    }


    uint8_t getChar(char *value) {
        if (readIdx == writeIdx) {
            return 0;
        }

        *value = buff[readIdx];
        readIdx = (readIdx + 1) % buffSize;

        return 1;
    }


    uint8_t putInt(uint32_t value) {
        size_t nextIdx = (writeIdx + int_size) % buffSize;

        if (nextIdx == readIdx) {
            return 0;
        }

//        buff[writeIdx] = (char) (value >> 24);
//        buff[writeIdx + 1] = (char) (value >> 16);
//        buff[writeIdx + 2] = (char) (value >> 8);
//        buff[writeIdx + 3] = (char) value;

        memcpy(buff+writeIdx, &value, sizeof(value));

        writeIdx = nextIdx;

        return 1;        
    }


    uint32_t getInt() {
        size_t nextReadIdx = readIdx + int_size;
        if (nextReadIdx > writeIdx) { // don't perform read until the next 4 bites is written
            return 0;
        }

        uint32_t value = 0;

//        for (unsigned n = readIdx; n < nextReadIdx; n++) {
//            value = (value << 8) + buff[n];
//        }

        value = *(uint32_t *) (buff); // dereferences 4 bytes of data at the specified buff address

        readIdx = (nextReadIdx) % buffSize;

        return value;
    }
    
};