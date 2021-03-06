// HsWing Filght Control System - Arduino Source File
// Copyright (c) 2015 HyunSoo Kim

// Changelog
//		2015-07-13 - added D controller in angular rate controller
//		2015-05-13 - added serial communication module (to rasb Pi)
//		2015-04-13 - changed control timming (angular : angular speed = 1 : 4)
//		2015-04-03 - added and changed sensors(inertial, sonar)
//		2015-04-01 - integrated librarys, sources into project
//		2015-03-12 - changed attitude controller (motived by AR.Drone)
//		2014-12-03 - added attitude & altitude controller (for paper)
//		2014-12-01 - changed frame with prop guard (motived by ESContest)
//		2014-08-08 - initial version make (using F450 Frame.. he was broken)


// I'm doing branch test of git haha
// second test haha

#include "I2Cdev.h"
#include "MPU6050.h"
#include "BMA150.h"
#include "srf08.h"
#include "WingConfig.h"
//#include <EEPROM.h> 

//#include "DebugUtils.h"
//#include "CommunicationUtils.h"

#include <Wire.h>
//#include <SPI.h>

#include "HsCommunicator.h"
#include "HsFilter.h"
#include "HsFlightController.h"



double tmp_yaw = 0;
int VTOL_state = 0;	// State management variable
int VTOL_isReady = 0;
long loop_cnt=0;
boolean safe_mode = false;

double roll_setPoint = 0, pitch_setPoint = 0, yaw_setpoint = 0, alt_setpoint = 80;
double roll_sp_lpf = roll_setPoint, pitch_sp_lpf = pitch_setPoint, yaw_sp_lpf = yaw_setpoint, alt_sp_lpf = alt_setpoint;

//double roll_offset = 0.0, pitch_offset = 0.0;
double roll_offset = 2.55, pitch_offset = 2.75;

//double p_gain = 1.2, i_gain = 0.21, d_gain = 0.0008;

// timming variable
long start_time = 0, end_time = 0, loop_time = 0;
double elapsed_time = 0;
double dt = 0;
double sum_dt=0;
double dt_4angular = 0, cnt_4angular = 0;
unsigned long test_time1, test_time2;

// PWM variable
int pwm = 0, pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;

double roll = 0, pitch = 0, yaw = 0;


// bias calibration variable
double gx_bias = 0, gy_bias = 0, gz_bias = 0;
double ax_bias = 0, ay_bias = 0, az_bias = 0;


// LPF Filter variable
double alt_lpf = 0;

// ultrasonic sensor variable
int ultra = 0, prev_ultra = 0;;
#ifdef ALTITUDE_CONTROL

SRF08 srf08;


double ultra_filtered = 0;
double n_timming_ultra = 0;
bool b_alt_ctrl_flag = false;
int n_alt_turn_cnt = 0, n_alt_turn_length = 7;

#endif

// communication variable
#ifdef SERIAL_COMUNICATION

double communi_dt = 0;
unsigned char dataBuffer[HS_BUFFER_LENGTH] = {0, };

#endif

int upDownIntegral = 0, leftRightIntegral = 0;

// FreeIMU object, variable
//FreeIMU imu = FreeIMU();
MPU6050 mpu6050 = MPU6050();
int16_t gx, gy, gz;

BMA150 bma150;
int16_t bma150_x, bma150_y, bma150_z;


HsFilter hsFilter = HsFilter();
HsFlightController hsFC = HsFlightController();
HsCommunicator hsCommuni = HsCommunicator();

// sampling time demultiply variable
int sampling_cnt = 0;


int nHsWingMode = 0;
double pwm_smooth = 0;


int buzzer_hz = 0;
double last_pitch;

void setup() {

	Serial.begin(115200);
#ifdef SERIAL_COMUNICATION
	Serial1.begin(115200);	// to Pi
#endif

	Wire.begin();

	Serial.println("Initialize Sensors...");
	
	////////////////////////////////////////////////////////////////
	//					SENSOR INITIALIZE
	mpu6050.initialize();
	mpu6050.setI2CMasterModeEnabled(0);
	mpu6050.setI2CBypassEnabled(1);
	mpu6050.setDLPFMode(MPU6050_DLPF_BW_5);
	delay(5);

	bma150.initialize();

#ifdef ALTITUDE_CONTROL
	srf08.initialize();
#endif


	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(11, OUTPUT);
	pinMode(12, OUTPUT);

	pinMode(13, OUTPUT);   

	// Buzzer
	pinMode(4, OUTPUT);
	

	////////////////////////////////////////////////////////////////
	//					BIAS MEASUREMENT
	Serial.println("Bias Measurment Step. Please Don't Move..");
	for(int i=1; i<=1000; i++) {
		mpu6050.getRotation(&gx, &gy, &gz);
		//Serial.println(gx);
		bma150.getAcceleration(&bma150_x, &bma150_y, &bma150_z);

		gx_bias += gx/1000.0;
		gy_bias += gy/1000.0;
		gz_bias += gz/1000.0;

		ax_bias += bma150_x/1000.0;
		ay_bias += bma150_y/1000.0;
		az_bias += (bma150_z-256)/1000.0;

	}


	//hsFilter.initialize(bma150_x-ax_bias, bma150_y-ay_bias, bma150_z-az_bias); 
	hsFilter.initialize(bma150_x, bma150_y, bma150_z); 

	hsFC.initialize();
	hsCommuni.initialize();

	Serial.println("Waiting Rasb Pi...");

	// 패킷이 한번이라도 전송될 때 까지 대기
	//while( hsCommuni.serialFromPi(&roll_setPoint, &pitch_setPoint, &yaw_setpoint, &alt_setpoint) == 0 );

	////////////////////////////////////////////////////////////////
	//						INITIALIZE ESC
	

	Serial.print("Starting ESC...");
	analogWrite(2, (int)90);
	analogWrite(3, (int)90);
	analogWrite(11, (int)90);
	analogWrite(12, (int)90);
	Serial.print("4..");
	digitalWrite(13, LOW); 
	delay(100);
	digitalWrite(13, HIGH);
	Serial.print("3..");
	delay(100);
	digitalWrite(13, LOW);
	Serial.print("2..");
	delay(100);
	digitalWrite(13, HIGH);
	Serial.print("1..");
	delay(100);
	digitalWrite(13, LOW);

	
	start_time = end_time = micros();


	//FlexiTimer2::set(100, communication); // MsTimer2 style is also supported
	//FlexiTimer2::start();

	Serial.println("OK!! Let's go!!");
#ifdef ALTITUDE_CONTROL
	ultra = 5;	// Minimum of SRF08 is about 5cm.
	alt_lpf = ultra;
#endif
	pwm = 90;
	pwm_smooth = pwm;



}


void loop() {

	// Timming
	end_time = micros();
	loop_time = end_time - start_time;
	start_time = micros();
	//if( loop_time < 5000 ) {		
	//	delayMicroseconds(5000 - loop_time);
	//	loop_time = 5000;
	//}
	dt = loop_time / 1000000.0;
	elapsed_time += dt;

#ifndef FC_RELEASE
	if( Serial.available() ) {

		char cmd = Serial.read();

		if( cmd == 'l' ) {
			pwm = 90;
		}else if( cmd == 'p' ) {
			pwm = 0;
		}else if( cmd == 'a' ) {
			pwm +=1;
		}else if( cmd == 'z' ) {
			pwm -=1;
		}else if( cmd == 'n' ) {
			roll_setPoint = 10;
		}else if( cmd == 'm' ) {
			roll_setPoint = 0;
		}else if( cmd == 's' ) {	// 이륙모드
			VTOL_state = 2;
		}else if( cmd == 'x' ) {	// 착륙모드
			VTOL_state = 1;
		}else if( cmd == 'b' ) {
			hsFC.yaw_err_integral = (hsFC.i_wind_up_limit_yaw*3);
			tmp_yaw += 5;
		}else if( cmd == 'v' ) {
			hsFC.yaw_err_integral = -(hsFC.i_wind_up_limit_yaw*3);
			tmp_yaw -= 5;
		}
	}
#endif
	
#ifdef SERIAL_COMUNICATION
	/////////////////////////////////////////////////////////////////////////////////////////////// Communication Part
	
	communi_dt += dt;
	if( communi_dt >= HS_SERIAL_DELAY ) {
		communi_dt = 0.0;
		double tmp_alt = 0;
		double tmp_yaw = 0;
		hsCommuni.serialFromPi(&VTOL_state, &roll_setPoint, &pitch_setPoint, &yaw_setpoint, &tmp_alt);

		//Serial.print(VTOL_state);
		//Serial.print("\t");
		//Serial.print(roll_setPoint);
		//Serial.print("\t");
		//Serial.print(pitch_setPoint);
		//Serial.print("\t");
		//Serial.print(yaw_setpoint);
		//Serial.print("\t");
		//Serial.print(alt_setpoint);
		//Serial.println();

		
		//hsCommuni.serialToPi(hsFC.m_roll, hsFC.roll_pid, hsFC.roll_ar_p, hsFC.gx_mean, roll_setPoint, hsFilter.m_ax, hsFilter.m_ay);
		hsCommuni.serialToPi(hsFilter.m_roll, hsFilter.m_pitch, hsFilter.m_yaw, ultra
			, hsFilter.m_ax, hsFilter.m_ay, hsFilter.m_az, hsFilter.m_gx, hsFilter.m_gy, hsFilter.m_gz);
		//hsCommuni.serialToPi(hsFC.m_roll, hsFC.m_pitch, hsFC.m_yaw, ultra, hsFilter.m_gx, hsFilter.m_gy, hsFilter.m_gz);
	}

#endif


	if(VTOL_state == 2) {	// 이륙 모드
		VTOL_isReady = 1;
#ifdef ALTITUDE_CONTROL
		if( pwm < 180 && elapsed_time > 5.0) {
			if( pwm < 140 ) {
				pwm_smooth += 0.1;
			}else {
				pwm_smooth += 0.05;
			}
			pwm = (int)pwm_smooth;
		}

		if( ultra < 20 ) {
			hsFC.roll_err_integral = 0.0f;
			hsFC.pitch_err_integral = 0.0f;
		}

#endif
	}else if(VTOL_state == 1) {		// 착륙 모드
		// Landing Process
		if( VTOL_isReady == 1) {
			//pwm  = 175;
			if( alt_setpoint > -10 ) {
				alt_setpoint -= 0.15;
			}

			if( ultra < 8 ) {		// 거의 바닥에 내려왔을 때
				VTOL_isReady = 0;
				pwm = 90;
				pwm_smooth = 90;
				analogWrite(2, (int)pwm);
				analogWrite(3, (int)pwm);
				analogWrite(11, (int)pwm);
				analogWrite(12, (int)pwm);
				alt_setpoint = 80;
			
			}
		}
		


	}
//	Read raw accel/gyro measurements from device
	//mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	mpu6050.getRotation(&gx, &gy, &gz);
	if( gx==0 && gy==0 && gz==0 ) {
		mpu6050.getRotation(&gx, &gy, &gz);
	}

	bma150.getAcceleration(&bma150_x, &bma150_y, &bma150_z);

//	Apply Calibration
	gx -= gx_bias;
	gy -= gy_bias;
	gz -= gz_bias;

	//bma150_x -= (int)ax_bias;
	//bma150_y -= (int)ay_bias;
	//bma150_z -= (int)az_bias;

	sampling_cnt++;

#ifdef ALTITUDE_CONTROL
/////////////////////////////////////////////////////////////////////////////////////////////// UltraSonic Sensor Part
	if( n_timming_ultra == 0.0 ) {
		srf08.sendRangeCommand();
	}

	n_timming_ultra += dt;
	//n_alt_turn_cnt ++;

	if( n_timming_ultra >= HS_SRF08_SAMPLING_TIME ) {

		n_timming_ultra = 0.0;
		
		prev_ultra = ultra;
		
		ultra = srf08.recieveRange();
		
		if( (ultra - prev_ultra > 20) || (ultra - prev_ultra < -20) ) {
			ultra = prev_ultra;
		}
		//n_alt_turn_length = n_alt_turn_cnt;
		//n_alt_turn_cnt = 0;
		b_alt_ctrl_flag = true;

		//Serial.print(hsFilter.yaw_gyro);
		//Serial.print("\t");
		//Serial.print(ultra);
		//Serial.print("\t");
		//Serial.print(tmp_yaw);
		//Serial.print("\t");
		//Serial.print(hsFC.yaw_rate_pid);
		//Serial.print("\t");
		//Serial.println();
		
	}

	if (b_alt_ctrl_flag == true) {
		hsFilter.generalLowPassFilter(&alt_lpf, ultra, 0.5, 0.5);
		hsFC.altitudeControl(alt_setpoint, alt_lpf);
		b_alt_ctrl_flag = false;
		hsFC.isFirst = false;
	}else {
		hsFC.altitudeControl_onlyD();
	}
	
/////////////////////////////////////////////////////////////////////////////////////////////// UltraSonic Sensor Part
#endif


/////////////////////////////////////////////////////////////////////////////////////////////// Filtering Part

	hsFilter.setRawData(bma150_x, bma150_y, bma150_z, gx, gy, gz);
	hsFilter.setDt(dt);
	hsFilter.integralYaw();

#ifdef TIMED_MULTIPLEX_CONTROL 

	if(	cnt_4angular >= TIMED_MULTIPLEX_CONTROL_DELAY_COUNT ) {
		hsFilter.setDt(dt_4angular);
		hsFilter.complementaryFilter();

		// 속도추정 테스트용 // 비전센서와 함께 구현해야함
		//hsFilter.accelerometerHSR();
		//hsFilter.estimateVelbyAccel();
		//hsFilter.estimateVelbyGyro();
		
	}

#else
	hsFilter.setDt(dt);
	hsFilter.complementaryFilter();
#endif

	
	

/////////////////////////////////////////////////////////////////////////////////////////////// Filtering Part

	// S A F E   P R O C E S S 
	if( abs(hsFilter.m_roll) > MAX_ANGLE || abs(hsFilter.m_pitch) > MAX_ANGLE ) {
		VTOL_state = 1;		// 착륙모드
	}


//test_time1 = micros();
//Serial.print( (test_time1 - start_time) / 1000.0 );
//Serial.print( "\t" );

/////////////////////////////////////////////////////////////////////////////////////////////// Controller Part
	hsFC.setDt(dt);
	hsFC.setSmoothValue(hsFilter.m_gx, hsFilter.m_gy, hsFilter.m_gz);

	
#ifdef TIMED_MULTIPLEX_CONTROL 
	if(	cnt_4angular >= TIMED_MULTIPLEX_CONTROL_DELAY_COUNT ) {
		hsFC.setEulerAngleAndRate(hsFilter.m_roll - roll_offset, hsFilter.m_pitch - pitch_offset, hsFilter.yaw_gyro_dot);
		roll_sp_lpf = roll_sp_lpf * 0.5 + roll_setPoint * 0.5;
		pitch_sp_lpf = pitch_sp_lpf * 0.5 + pitch_setPoint * 0.5;
		hsFC.setSetPoint( roll_sp_lpf, pitch_sp_lpf );
		hsFC.AngularControl(dt_4angular);
	}

	hsFC.AngularRateControl();
#else
	hsFC.setSetPoint(roll_setPoint, pitch_setPoint);
	hsFC.setEulerAngle(hsFilter.m_roll, hsFilter.m_pitch, hsFilter.m_yaw);
	hsFC.AngularControl(dt);
	hsFC.AngularRateControl();
#endif


	hsFC.setPwm(pwm);
	hsFC.generatePWM(&pwm1, &pwm2, &pwm3, &pwm4);

	

/////////////////////////////////////////////////////////////////////////////////////////////// Controller Part

//test_time2 = micros();
//Serial.print( (test_time2 - test_time1) / 1000.0 );
//Serial.println();	


#ifdef ROTOR_ENABLE
	if( VTOL_isReady == 1 ) {
		analogWrite(2, (int)pwm1);
		analogWrite(3, (int)pwm2);
		analogWrite(11, (int)pwm3);
		analogWrite(12, (int)pwm4);਍ऀ紀ഀ਀⌀攀氀猀攀ഀ਀ऀ愀渀愀氀漀最圀爀椀琀攀⠀㈀Ⰰ ⠀椀渀琀⤀㤀　⤀㬀ഀ਀ऀ愀渀愀氀漀最圀爀椀琀攀⠀㌀Ⰰ ⠀椀渀琀⤀㤀　⤀㬀ഀ਀ऀ愀渀愀氀漀最圀爀椀琀攀⠀㄀㄀Ⰰ ⠀椀渀琀⤀㤀　⤀㬀ഀ਀ऀ愀渀愀氀漀最圀爀椀琀攀⠀㄀㈀Ⰰ ⠀椀渀琀⤀㤀　⤀㬀ഀ਀⌀攀渀搀椀昀ഀ਀ഀ਀ऀ⼀⼀吀䌀䌀刀㄀䈀 㴀 㐀㬀ഀ਀ऀ⼀⼀吀䌀䌀刀㌀䈀 㴀 㐀㬀ഀ਀ഀ਀ഀ਀ऀ⼀⼀ 瀀爀椀渀琀 瀀愀爀琀ഀ਀ഀ਀ऀഀ਀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀瘀攀氀开砀愀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀瘀攀氀开礀愀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀瘀攀氀开砀最⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀瘀攀氀开礀最⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀氀渀⠀∀尀琀∀⤀㬀ഀ਀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀爀漀氀氀开猀瀀开氀瀀昀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀爀漀氀氀开猀攀琀倀漀椀渀琀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀爀漀氀氀开愀挀挀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀爀漀氀氀开最礀爀漀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀洀开爀漀氀氀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀瀀椀琀挀栀开愀挀挀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀瀀椀琀挀栀开最礀爀漀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀洀开瀀椀琀挀栀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀氀渀⠀⤀㬀ഀ਀ഀ਀ഀ਀ऀഀ਀ഀ਀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀洀开爀漀氀氀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀最砀开洀攀愀渀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀爀漀氀氀开爀愀琀攀开瀀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀爀漀氀氀开爀愀琀攀开搀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀氀渀⠀⤀㬀ഀ਀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀爀漀氀氀开瀀椀搀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀爀漀氀氀开瀀椀搀开焀开愀瘀最⤀㬀ഀ਀ऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀氀渀⠀⤀㬀ഀ਀ऀഀ਀ഀ਀⌀椀昀搀攀昀 吀䤀䴀䔀䐀开䴀唀䰀吀䤀倀䰀䔀堀开䌀伀一吀刀伀䰀 ഀ਀ऀ椀昀⠀ऀ挀渀琀开㐀愀渀最甀氀愀爀 㸀㴀 吀䤀䴀䔀䐀开䴀唀䰀吀䤀倀䰀䔀堀开䌀伀一吀刀伀䰀开䐀䔀䰀䄀夀开䌀伀唀一吀 ⤀ 笀ഀ਀ഀ਀ഀ਀ऀऀ搀琀开㐀愀渀最甀氀愀爀 㴀 　㬀ഀ਀ऀऀ挀渀琀开㐀愀渀最甀氀愀爀 㴀 　㬀ഀ਀ഀ਀ऀ紀ഀ਀ऀ搀琀开㐀愀渀最甀氀愀爀 ⬀㴀 搀琀㬀ഀ਀ऀ挀渀琀开㐀愀渀最甀氀愀爀 ⬀⬀㬀ഀ਀⌀攀渀搀椀昀ഀ਀ഀ਀ऀഀ਀ഀ਀ഀ਀ऀ椀昀⠀ 氀漀漀瀀开挀渀琀 㸀㴀 ㄀　 ⤀ 笀ഀ਀ऀऀഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀椀氀琀攀爀⸀礀愀眀开最礀爀漀开搀漀琀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀礀愀眀开爀愀琀攀开瀀椀搀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀礀愀眀开戀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀洀开爀漀氀氀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀爀漀氀氀开瀀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀爀漀氀氀开瀀椀搀开焀开愀瘀最⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀爀漀氀氀开爀愀琀攀开瀀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀最砀开洀攀愀渀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀爀漀氀氀开猀攀琀倀漀椀渀琀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀爀漀氀氀开爀愀琀攀开搀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀栀猀䘀䌀⸀爀漀氀氀开椀⤀㬀ഀ਀ऀऀ⼀⼀⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀氀渀⠀⤀㬀ഀ਀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀∀尀琀∀⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀⠀搀琀⨀㄀　　　⤀㬀ഀ਀ऀऀ⼀⼀匀攀爀椀愀氀⸀瀀爀椀渀琀氀渀⠀搀琀开㐀愀渀最甀氀愀爀⨀㄀　　　⤀㬀ഀ਀ഀ਀ഀ਀ऀऀ氀漀漀瀀开挀渀琀 㴀 　㬀ഀ਀ऀऀ椀昀⠀ 爀漀氀氀开猀攀琀倀漀椀渀琀㴀㴀　 ☀☀ 瀀椀琀挀栀开猀攀琀倀漀椀渀琀㴀㴀　 ⤀ 笀ഀ਀ऀऀऀ猀愀昀攀开洀漀搀攀 㴀 ℀猀愀昀攀开洀漀搀攀㬀ഀ਀ऀऀ紀攀氀猀攀 笀ഀ਀ऀऀऀ猀愀昀攀开洀漀搀攀 㴀 ㄀㬀ഀ਀ऀऀ紀ഀ਀ऀऀ搀椀最椀琀愀氀圀爀椀琀攀⠀㄀㌀Ⰰ 猀愀昀攀开洀漀搀攀⤀㬀 ഀ਀ऀ紀ഀ਀ഀ਀ऀഀ਀ऀ愀渀愀氀漀最圀爀椀琀攀⠀㐀Ⰰ ㄀　⤀㬀ഀ਀ऀ氀漀漀瀀开挀渀琀⬀⬀㬀ഀ਀ഀ਀ഀ਀ऀഀ਀紀ഀ਀ഀ਀ഀ਀ഀ਀瘀漀椀搀 椀渀椀琀开琀椀洀攀爀⠀⤀ 笀ഀ਀ऀ⼀⼀吀䌀䌀刀㄀䈀㴀　砀　䄀㬀ഀ਀ഀ਀紀ഀ਀瘀漀椀搀 琀椀洀攀匀琀愀洀瀀⠀⤀ 笀ഀ਀ഀ਀紀ഀ਀\00
