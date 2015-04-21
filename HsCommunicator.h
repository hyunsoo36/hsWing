
// ************************************************ //
//			Disigned by hs. In RACS Lab.			//
// ************************************************ //

// 블루투스 통신에 쓰이는 명령어 패킷은 0x02로 시작하고 0x2C, 0x03으로 끝난다.

// HsCommunicator 클래스는 전송받은 데이터를 링버퍼를 통해 골라내는 역할을 수행한다.
#define packetLength  3
#define ringLength  ((3+3)*2)
#define PACKET_START_BYTE 0x02
#define PACKET_END_BYTE1 0x2C
#define PACKET_END_BYTE2 0x03
#define HS_BLUETOOTH_DT 0.1



class HsCommunicator {
	
private:

	unsigned char ringBuffer[ringLength];
	//byte dataBuffer[packetLength];
	char ringPtr;
	char dataPtr;

	

public:
	HsCommunicator();
	~HsCommunicator();

	void initialize();
	void destroy();


	

	int blueTooth(unsigned char* buffer);

	// getter, setter method

};