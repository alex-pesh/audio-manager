
class CyclicBuff {

private:
    char* buff;
    int buffSize = 0;
    int readIdx = 0;
    int writeIdx = 0;

public:
    explicit CyclicBuff(int size) {
        this->buffSize = size+1;
        buff = new char[size+1];
    }

    ~CyclicBuff() {
        delete [] buff;
    }

    int put(char value) {
        int nextIdx = (writeIdx + 1) % buffSize;

        if (nextIdx == readIdx) {
            return 0;
        }

        buff[writeIdx] = value;
        writeIdx = nextIdx;

        return 1;
    }

    int get(char *value) {
        if (readIdx == writeIdx) {
            return 0;
        }

        *value = buff[readIdx];
        readIdx = (readIdx + 1) % buffSize;

        return 1;
    }

    int getInt() {
        if (readIdx == writeIdx) {
            return 0;
        }

        int value = 0;
        value =
                (buff[0] << 24) |
                (buff[1] << 16) |
                (buff[2] << 8) |
                (buff[3]);

        readIdx = (readIdx + 4) % buffSize;

        return value;
    }
};