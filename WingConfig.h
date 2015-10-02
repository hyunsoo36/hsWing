#ifndef _WINGCONFIG_H__
#define _WINGCONFIG_H__



//#define WIRELESS_DEBUGGING


#define ALTITUDE_CONTROL							// 고도제어 on/off	
#define SERIAL_COMUNICATION

#define TIMED_MULTIPLEX_CONTROL						// 각도-각속도 이중 PID제어와 각도 PID제어 비교할 수 있도록 함
#define TIMED_MULTIPLEX_CONTROL_DELAY_COUNT 4		// 각도 제어 1회 당 각속도 제어 횟수

#define ROTOR_ENABLE								// 모터 작동 on/off
#define MAX_ANGLE		40							// 최대 기체 각도	// 이 각도를 넘으면 착륙시킴

//#define FC_RELEASE									// 무선 비행시 필요없는 기능 on/off		// PC연결 및 디버깅할 땐 주석처리해야 함.

#endif