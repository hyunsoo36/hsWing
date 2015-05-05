
// ************************************************ //
//			Disigned by hs. In RACS Lab.			//
// ************************************************ //

// ������� ��ſ� ���̴� ��ɾ� ��Ŷ�� 0x02�� �����ϰ� 0x2C, 0x03���� ������.

// HsCommunicator Ŭ������ ���۹��� �����͸� �����۸� ���� ��󳻴� ������ �����Ѵ�.

#include "d:\Program Files\arduino-1.0.3_test\hardware\arduino\cores\arduino\HardwareSerial.h"

#ifdef HS_RINGBUFFER
#define packetLength  3
#define ringLength  ((3+3)*2)
#define PACKET_START_BYTE 0x02
#define PACKET_END_BYTE1 0x2C
#define PACKET_END_BYTE2 0x03
#define HS_BLUETOOTH_DT 0.1
#endif

#define HS_BUFFER_LENGTH  12

#define HS_PACKET_LENGTH 18
#define HS_PACKET_HEADER1 0xEF
#define HS_PACKET_HEADER2 0xFE
#define HS_PACKET_TAIL 0xFF
#define HS_SERIAL_DELAY 0.05


//byte dataBuffer[packetLength];


class HsCommunicator {
	
private:
	unsigned char buffer[HS_BUFFER_LENGTH];	// for recv
	uint8_t packet[HS_PACKET_LENGTH];			// for send
	uint8_t data[HS_PACKET_LENGTH-4];			// consider headers, tails, length byte

#ifdef HS_RINGBUFFER
	unsigned char ringBuffer[ringLength];
	//byte dataBuffer[packetLength];
	char ringPtr;
	char dataPtr;
#endif
	

public:
	HsCommunicator();
	~HsCommunicator();

	void initialize();
	void destroy();


	
	int serialFromPi(double *roll, double *pitch, double *yaw, double *alt);

	int serialToPi(double roll, double pitch, double yaw, double alt, double ax, double ay, double az);
	int makePacket(uint8_t* data, int len);


#ifdef HS_RINGBUFFER
	int blueTooth(unsigned char* buffer);
#endif
	// getter, setter method

};