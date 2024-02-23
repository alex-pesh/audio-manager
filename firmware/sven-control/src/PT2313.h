/*
                               _                
 ___  _   _  _ __ ___    ___  | |_  ___   _   _ 
/ __|| | | || '_ ` _ \  / _ \ | __|/ _ \ | | | |
\__ \| |_| || | | | | || (_) || |_| (_) || |_| |
|___/ \__,_||_| |_| |_| \___/  \__|\___/  \__, |
                                          |___/ 
PT2313 Library
----------------------------------------
A C library for Princeton Technology Corp PT2313 L or E Audio Chip (I2C)
version 1.0
coded by Max MC Costa
--------------------------------------------------------
Library works with most arduino compatible processors and teensy3

*/
#ifndef PT2313_h
#define PT2313_h

#include "Arduino.h"

#define PT2313_ADDR            0x44
#define PT2313_DEFVOL          35

#define PT2313_MUTE_VAL		  0x1F //00011111

#define PT2313_VOL_REG        0x00 //00000000

#define PT2313_L_ATT_REG      0xC0 //11000000
#define PT2313_R_ATT_REG      0xE0 //11100000

#define PT2313_L1_ATT_REG     0x80 //10000000
#define PT2313_R1_ATT_REG     0xA0 //10100000

#define PT2313_BASS_REG       0x60 //01100000
#define PT2313_TREBLE_REG     0x70 //01110000

class PT2313 {

public:
    void    initialize(unsigned char addr=PT2313_ADDR, byte source=0, bool muted=true);
    int8_t	source(int8_t val);
    int8_t  volume(int8_t val);
    int8_t  bass(int8_t val);
    int8_t  treble(int8_t val);
    int8_t  balance	(int8_t val);
	bool    mute(bool value);
    bool    loudness(bool val);
    byte    gain(byte val);
private:
    int8_t  eqSet(int8_t val);
    int8_t  boundary(int8_t val, int8_t min, int8_t max);
	void    writeByte(unsigned char val);
	uint8_t	audioSwitch_reg;
};

#endif
