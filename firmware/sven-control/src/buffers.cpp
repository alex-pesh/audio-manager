#include <stdint.h>


class CyclicBuff {

private:


public:
    char* buff;
    int buffSize = 0;
    int readIdx = 0;
    int writeIdx = 0;

    /**
     * @param size Buffer size in bytes
    */
    explicit CyclicBuff(int size) {
        buffSize = size + 1;
        buff = new char[buffSize];
    }

    ~CyclicBuff() {
        delete [] buff;
    }

    int putChar(char value) {
        int nextIdx = (writeIdx + 1) % buffSize;

        if (nextIdx == readIdx) {
            return 0;
        }

        buff[writeIdx] = value;
        writeIdx = nextIdx;

        return 1;
    }


    int getChar(char *value) {
        if (readIdx == writeIdx) {
            return 0;
        }

        *value = buff[readIdx];
        readIdx = (readIdx + 1) % buffSize;

        return 1;
    }


    int putInt(int value) {
        int nextIdx = (writeIdx + 4) % buffSize;

        if (nextIdx == readIdx) {
            return 0;
        }

        buff[writeIdx] = (char) (value >> 24);
        buff[writeIdx + 1] = (char) (value >> 16);
        buff[writeIdx + 2] = (char) (value >> 8);
        buff[writeIdx + 3] = (char) value;

        writeIdx = nextIdx;

        return 1;        
    }


    uint32_t getInt() {
        unsigned nextReadIdx = readIdx + 4;
        if (nextReadIdx > writeIdx) { // don't perform read until the next 4 bites is written
            return 0;
        }

        uint32_t value = 0;


        for (unsigned n = readIdx; n < nextReadIdx; n++) {
            value = (value << 8) + buff[n];
        }

        readIdx = (nextReadIdx) % buffSize;

        return value;
    }
    
};