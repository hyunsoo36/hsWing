#ifndef _SRF08_H__
#define _SRF08_H__

#define SRF08_ADDRESS						0x70			// Address of the SRF08
#define SRF08_CMD								0x00		// Command byte, values of 0 being sent with write have to be masked as a byte to stop them being misinterpreted as NULL this is a bug with arduino 1.0
#define SRF08_ANALOGUE_GAIN				0x01			// Range register
#define SRF08_RANGE						0x02			// Range register
#define SRF08_RANGE_INCHES				0x50
#define SRF08_RANGE_CENTI				0x51
#define SRF08_MAX_RANGE_4M				0x5D
#define SRF08_MAX_RANGE_3M				0x44
#define SRF08_MAX_RANGE_2M				0x2D
#define SRF08_MAX_ANALOGUE_GAIN_94		0x00
#define SRF08_MAX_ANALOGUE_GAIN_97		0x01
#define SRF08_MAX_ANALOGUE_GAIN_100		0x02
#define SRF08_MAX_ANALOGUE_GAIN_128		0x09
#define SRF08_MAX_ANALOGUE_GAIN_199		0x12
#define SRF08_MAX_ANALOGUE_GAIN_1025	0x1F
#define SRG08_RANGE_DATA_START_ADDRESS	0x02			// Byte for start of ranging data

#define HS_SRF08_SAMPLING_TIME 0.05//.035


#include "I2Cdev.h"


class SRF08 {

public:

	void initialize();
	void sendRangeCommand();
	int recieveRange();


};



#endif