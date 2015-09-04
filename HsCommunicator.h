
// ************************************************ //
//			Disigned by hs. In RACS Lab.			//
// ************************************************ //

// 통신에 쓰이는 명령어 패킷은 0x02로 시작하고 0x2C, 0x03으로 끝난다.
// 전송받은 데이터를 링버퍼를 통해 골라내는 역할을 수행한다.
// 전송할 데이터를 패킷으로 만들어 전송하는 역할을 수행한다.
// 굳이 해놓는 용어정리 (반대쪽 단말도 같은 방식으로 적용되어 있음)
// BUFFER : 데이터를 수신할 때 사용되는 변수명에 이용
// PACKET : 데이터를 송신할 때 사용되는 변수명에 이용

//#include "d:\Program Files\arduino-1.0.3_test\hardware\arduino\cores\arduino\HardwareSerial.h"
#include "HardwareSerial.h"
//#include "Overmind.h"

#ifdef HS_RINGBUFFER
#define packetLength  3
#define ringLength  ((3+3)*2)
#define PACKET_START_BYTE 0x02
#define PACKET_END_BYTE1 0x2C
#define PACKET_END_BYTE2 0x03
#define HS_BLUETOOTH_DT 0.1
#endif

#define HS_BUFFER_LENGTH  8

#define HS_PACKET_LENGTH 24
#define HS_PACKET_HEADER1 0xEF
#define HS_PACKET_HEADER2 0xFE
#define HS_PACKET_TAIL 0xFF

#ifdef WIRELESS_DEBUGGING
#define HS_SERIAL_DELAY 0.020
#else
#define HS_SERIAL_DELAY 0.020
#endif


//byte dataBuffer[packetLength];


class HsCommunicator {
	
private:
	unsigned char buffer[HS_BUFFER_LENGTH];	// for recv
	uint8_t packet[HS_PACKET_LENGTH];			// for send
	uint8_t data[HS_PACKET_LENGTH-4];			// consider headers, tails, length byte
	int safe_cnt;
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


	
	int serialFromPi(int *state, double *roll, double *pitch, double *yaw, double *alt);

	int serialToPi(double roll, double pitch, double yaw, double alt
			, double ax, double ay, double az, double gx, double gy, double gz);
	int makePacket(uint8_t* data, int len);


#ifdef HS_RINGBUFFER
	int blueTooth(unsigned char* buffer);
#endif
	// getter, setter method

};