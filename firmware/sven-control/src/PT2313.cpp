#include "PT2313.h"
#include "i2cmaster.h"


unsigned char _addr = PT2313_ADDR;
int8_t balanceVal = 0;


void PT2313::initialize(unsigned char addr, byte src, bool muted) {
	_addr = addr;

	i2c_init();

	if (muted) {
		volume(63);
	} else {
	// 		volume(PT2313_DEFVOL);
	}

// 	audioSwitch_reg = 0x5C;
// 	source(src);
// 	balance(0);
// 	bass(0);
// 	treble(0);
}


int8_t PT2313::source(int8_t val) {
    val = val % 3; //range 0-2
	switch(val){
	case 0:
		bitClear(audioSwitch_reg,0);
		bitClear(audioSwitch_reg,1);
	break;
	case 1:
		bitSet(audioSwitch_reg,0);
		bitClear(audioSwitch_reg,1);
	break;
	case 2:
		bitClear(audioSwitch_reg,0);
		bitSet(audioSwitch_reg,1);
	break;
	}
	writeByte(audioSwitch_reg);

	return val;
}


int8_t PT2313::volume(int8_t val) {
	val = boundary(val,0,0x3F);
	writeByte(PT2313_VOL_REG|(0x3F - val));

	return val;
}

int8_t PT2313::bass(int8_t val){
    int8_t temp = eqSet(val);
	writeByte(PT2313_BASS_REG|temp);

	return temp;
}

int8_t PT2313::treble(int8_t val){
    int8_t temp = eqSet(val);
	writeByte(PT2313_TREBLE_REG|temp);

	return temp;
}

int8_t PT2313::balance(int8_t val) {
	val = boundary(val, -31, 31);
	if (val == 0) {
		writeByte(PT2313_L_ATT_REG|0x00);
		writeByte(PT2313_R_ATT_REG|0x00);
		writeByte(PT2313_L1_ATT_REG|0x00);
		writeByte(PT2313_R1_ATT_REG|0x00);
	} else {
		if (val < 0) {
			writeByte(PT2313_L_ATT_REG|0x00);
			writeByte(PT2313_R_ATT_REG|((byte)abs(val)));
			writeByte(PT2313_L1_ATT_REG|0x00);
			writeByte(PT2313_R1_ATT_REG|((byte)abs(val)));
		} else {
			writeByte(PT2313_L_ATT_REG|((byte)abs(val)));
			writeByte(PT2313_R_ATT_REG|0x00);
			writeByte(PT2313_L1_ATT_REG|((byte)abs(val)));
			writeByte(PT2313_R1_ATT_REG|0x00);
		}
	}

	return balanceVal = val;
}

bool PT2313::mute(bool value) {
	if (value) {
		writeByte(PT2313_L_ATT_REG|PT2313_MUTE_VAL);
		writeByte(PT2313_R_ATT_REG|PT2313_MUTE_VAL);
		writeByte(PT2313_L1_ATT_REG|PT2313_MUTE_VAL);
		writeByte(PT2313_R1_ATT_REG|PT2313_MUTE_VAL);	
	} else {
		balance(balanceVal);
	}

	return value;
}

byte PT2313::gain(byte val) {//range 0-3
	switch(val) {
	case 0://0db
		bitSet(audioSwitch_reg,3);
		bitSet(audioSwitch_reg,4);
	break;
	case 1://+3,75db
		bitClear(audioSwitch_reg,3);
		bitSet(audioSwitch_reg,4);
	break;
	case 2://+7.5db
		bitSet(audioSwitch_reg,3);
		bitClear(audioSwitch_reg,4);
	break;
	case 3://+11.25db
		bitClear(audioSwitch_reg,3);
		bitClear(audioSwitch_reg,4);
	break;
	}

	writeByte(audioSwitch_reg);

	return val;
}

bool PT2313::loudness(bool val) {
	if (val){
		bitClear(audioSwitch_reg,2);
	} else {
		bitSet(audioSwitch_reg,2);
	}
	writeByte(audioSwitch_reg);

	return val;
}


int8_t PT2313::eqSet(int8_t val) {
    int8_t temp;
	val = (int8_t) boundary(val, -7, 7);
	if (val < 0) {
		temp = 7 - abs(val);
	} else {
		temp = 15 - val;
	}
	return temp;
}

int8_t PT2313::boundary(int8_t val, int8_t min, int8_t max){
	if (val < min) val = min;
	if (val > max) val = max;
	return val;
}

void PT2313::writeByte(unsigned char val) {
    unsigned char ret = i2c_start(_addr);

    if (!ret) {
    	i2c_write(val);
    } else {
		// Serial.print("failed to connect to device ");
		// Serial.println(_addr, HEX);
	}

    i2c_stop();
}
