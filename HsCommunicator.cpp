#include "HsCommunicator.h"
#include "D:\Program Files\arduino-1.0.3\hardware\arduino\cores\arduino\arduino.h"


int HsCommunicator::blueTooth(unsigned char* buffer) {
	
	// Store to ringBuffer
	int tmp = Serial1.read();
	
	while( tmp != -1 ) {
		ringBuffer[ringPtr] = tmp;
		ringPtr = ringPtr+1 >= ringLength ? 0 : ringPtr+1;

		tmp = Serial1.read();
	}
	//링버퍼에서 가장 최근 data 찾기
	int tmpCnt = 0;
	while( tmpCnt++ < ringLength ) {
		if( ringBuffer[dataPtr] == PACKET_START_BYTE ) { 
			int modPtr = (dataPtr+packetLength+1) % ringLength;
			if( ringBuffer[modPtr] == PACKET_END_BYTE1 && ringBuffer[(modPtr+1)%ringLength] == PACKET_END_BYTE2 ) {
				//찾은 위치에서 data 뽑기
				int tmpPtr;
				dataPtr = dataPtr+1 >= ringLength ? 0 : dataPtr+1; //a제외
				for( int i=0; i<packetLength; i++ ) {
					tmpPtr = (dataPtr+i) % ringLength;
					buffer[i] = ringBuffer[tmpPtr];
				}

				// init ring
				for( int i=0; i<ringLength; i++ ) {
					ringBuffer[i] = 0;
				}
				break;
			}else {
				dataPtr = dataPtr+1 >= ringLength ? 0 : dataPtr+1;
			}
		}else {
			dataPtr = dataPtr+1 >= ringLength ? 0 : dataPtr+1;
		}

	}
	return 1;
	

		

}

void HsCommunicator::initialize() {
	ringPtr = 0;
	dataPtr = 0;

}


void HsCommunicator::destroy()  {


}

HsCommunicator::HsCommunicator() {

}

HsCommunicator::~HsCommunicator() {

}

