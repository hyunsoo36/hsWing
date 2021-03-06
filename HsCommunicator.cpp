#include "HsCommunicator.h"



int HsCommunicator::serialFromPi(int *state, double *roll_sp, double *pitch_sp, double *yaw_sp, double *alt_sp) {
	int serial_len = Serial1.available();
	char serial_buf[HS_BUFFER_LENGTH];
	int flag = 0;
	
	safe_cnt++;

	if( serial_len > 0 && serial_len <= HS_BUFFER_LENGTH) {
		Serial1.readBytes(serial_buf, serial_len);		// 읽어들이기
		memcpy( &buffer[0], &buffer[serial_len], HS_BUFFER_LENGTH-serial_len );
		memcpy( &buffer[HS_BUFFER_LENGTH-serial_len], &serial_buf[0], serial_len );
		
		if( buffer[0]==HS_PACKET_HEADER1 && buffer[1]==HS_PACKET_HEADER2 && buffer[HS_BUFFER_LENGTH-1]==HS_PACKET_TAIL) {
			//Serial.println(*state);
			safe_cnt = 0;
			*state = (int)((signed char)buffer[2]);
			*roll_sp =	(double)((signed char)buffer[3] / 2.0);
			*pitch_sp = (double)((signed char)buffer[4] / 2.0);
			*yaw_sp =	(double)((signed char)buffer[5] / 2.0);
			*alt_sp =	(double)(signed char)(buffer[6]);
			//*roll =	(double)(((short)buffer[3] << 8) | ((short)buffer[4] << 0));
			

			flag = 1;
			
		}

		//for(int i=0; i<HS_BUFFER_LENGTH; i++) {
		//	Serial.print((unsigned char)serial_buf[i]);
		//	Serial.print("  ");
		//}
		//Serial.println();
		

	}else if( serial_len > HS_BUFFER_LENGTH ) {
		char tmpBuf[1024];
		Serial1.readBytes(tmpBuf, serial_len);
		//Serial.print(serial_len);
		//Serial.println(" : buffer is flushed.");
	}

	return flag;
}
int HsCommunicator::serialToPi(double roll, double pitch, double yaw, double alt
							   , double ax, double ay, double az, double gx, double gy, double gz) {
	
	data[0] = (uint8_t)(( ((short)(roll*10.0)) & 0xFF00 ) >> 8);
	data[1] = (uint8_t)(( ((short)(roll*10.0)) & 0x00FF ) >> 0);
	data[2] = (uint8_t)(( ((short)(pitch*10.0)) & 0xFF00 ) >> 8);
	data[3] = (uint8_t)(( ((short)(pitch*10.0)) & 0x00FF ) >> 0);
	data[4] = (uint8_t)(( ((short)(yaw*10.0)) & 0xFF00 ) >> 8);
	data[5] = (uint8_t)(( ((short)(yaw*10.0)) & 0x00FF ) >> 0);
	data[6] = (uint8_t)(( ((short)(alt*10.0)) & 0xFF00 ) >> 8);
	data[7] = (uint8_t)(( ((short)(alt*10.0)) & 0x00FF ) >> 0);
	data[8] = (uint8_t)(( ((short)(ax*100.0)) & 0xFF00 ) >> 8);
	data[9] = (uint8_t)(( ((short)(ax*100.0)) & 0x00FF ) >> 0);
	data[10] = (uint8_t)(( ((short)(ay*100.0)) & 0xFF00 ) >> 8);
	data[11] = (uint8_t)(( ((short)(ay*100.0)) & 0x00FF ) >> 0);
	data[12] = (uint8_t)(( ((short)(az*100.0)) & 0xFF00 ) >> 8);
	data[13] = (uint8_t)(( ((short)(az*100.0)) & 0x00FF ) >> 0);
	data[14] = (uint8_t)(( ((short)(gx*100.0)) & 0xFF00 ) >> 8);
	data[15] = (uint8_t)(( ((short)(gx*100.0)) & 0x00FF ) >> 0);
	data[16] = (uint8_t)(( ((short)(gy*100.0)) & 0xFF00 ) >> 8);
	data[17] = (uint8_t)(( ((short)(gy*100.0)) & 0x00FF ) >> 0);
	data[18] = (uint8_t)(( ((short)(gz*100.0)) & 0xFF00 ) >> 8);
	data[19] = (uint8_t)(( ((short)(gz*100.0)) & 0x00FF ) >> 0);
	
	makePacket(data, HS_PACKET_LENGTH-4);

	Serial1.write(packet, HS_PACKET_LENGTH);
	
	//for(int i=0; i<8; i++) {
	//	Serial.print((unsigned char)packet[i]);
	//	Serial.print("  ");
	//}
	//Serial.println();


}

int HsCommunicator::makePacket(uint8_t* data, int len) {
	packet[0] = HS_PACKET_HEADER1;
	packet[1] = HS_PACKET_HEADER2;
	packet[2] = len;
	memcpy( &packet[3], &data[0], len );
	packet[3+len] = HS_PACKET_TAIL;


}


#ifdef HS_RINGBUFFER
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
#endif

void HsCommunicator::initialize() {

	safe_cnt = 0;

#ifdef HS_RINGBUFFER
	ringPtr = 0;
	dataPtr = 0;
#endif

}


void HsCommunicator::destroy()  {


}

HsCommunicator::HsCommunicator() {

}

HsCommunicator::~HsCommunicator() {

}

