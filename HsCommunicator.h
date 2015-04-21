
// ************************************************ //
//			Disigned by hs. In RACS Lab.			//
// ************************************************ //

// ������� ��ſ� ���̴� ��ɾ� ��Ŷ�� 0x02�� �����ϰ� 0x2C, 0x03���� ������.

// HsCommunicator Ŭ������ ���۹��� �����͸� �����۸� ���� ��󳻴� ������ �����Ѵ�.
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