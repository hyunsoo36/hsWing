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




int VTOL_state = 0;	// State management variable
long loop_cnt=0;
boolean safe_mode = false;

double roll_setPoint = 0, pitch_setPoint = 0, yaw_setpoint = 0, alt_setpoint = 80;
//double roll_offset = -9, pitch_offset = -3;
double roll_offset = 7.5, pitch_offset = 7;

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


void setup() {

	Serial.begin(115200);
#ifdef SERIAL_COMUNICATION
	Serial1.begin(115200);	// to Pi
#endif

	Wire.begin();

	Serial.println("Initialize Sensors...");
	
	// Sensor Initialize
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
	//
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


	hsFilter.initialize(bma150_x-ax_bias, bma150_y-ay_bias, bma150_z-az_bias); 
	//hsFilter.initialize(bma150_x, bma150_y, bma150_z); 

	hsFC.initialize();
	hsCommuni.initialize();

	Serial.println("Waiting Rasb Pi...");

	// 패킷이 한번이라도 전송될 때 까지 대기
	//while( hsCommuni.serialFromPi(&roll_setPoint, &pitch_setPoint, &yaw_setpoint, &alt_setpoint) == 0 );

	////////////////////////////////////////////////////////////////
	//						INITIALIZE ESC
	//
	

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
		}else if( cmd == 's' ) {
			VTOL_state = 2;
		}


	}
	

#ifdef SERIAL_COMUNICATION
	/////////////////////////////////////////////////////////////////////////////////////////////// Communication Part
	
	communi_dt += dt;
	if( communi_dt >= HS_SERIAL_DELAY ) {
		communi_dt = 0.0;
		double tmp_alt = 0;
		double tmp_yaw = 0;
		hsCommuni.serialFromPi(&VTOL_state, &roll_setPoint, &pitch_setPoint, &yaw_setpoint, &alt_setpoint);

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
		hsCommuni.serialToPi(hsFC.m_roll, hsFC.m_pitch, hsFC.m_yaw, ultra, hsFilter.m_ax, hsFilter.m_ay, hsFilter.m_az);
		//hsCommuni.serialToPi(hsFC.m_roll, hsFC.m_pitch, hsFC.m_yaw, ultra, hsFilter.m_gx, hsFilter.m_gy, hsFilter.m_gz);
	}

		// 무선통신용 setPoint 조절 코드 (통신 클래스로 옮겨야 함!!)
	//if(abs(((double)dataBuffer[0]-64.0) / 10.0) <= 10.0 && abs(((double)dataBuffer[1]-64.0) / 10.0) <= 10.0) {
 //		roll_setPoint = -((double)dataBuffer[0]-64.0) / 10.0;
 //		pitch_setPoint = -((double)dataBuffer[1]-64.0) / 10.0;
 //	}

	//pwm = dataBuffer[2]*2;    // 매우 안좋음.. 통신 두절 시 이상한 값 들어감

	// 무선통신용 setPoint 조절 코드 (통신 클래스로 옮겨야 함!!)
	//hsFC.setPidGain(
	//	(p_gain+((double)dataBuffer[5]-64)*0.01),
	//	(i_gain),
	//	(d_gain + ((double)dataBuffer[6]-64)*0.0001));

	//hsFC.setPidGain(
	//	(p_gain),
	//	(i_gain),
	//	(d_gain));
/////////////////////////////////////////////////////////////////////////////////////////////// Communication Part
#endif


	if(VTOL_state == 2) {

#ifdef ALTITUDE_CONTROL
		if( pwm < 180 && elapsed_time > 10.0) {
			pwm_smooth += 0.05;
			pwm = (int)pwm_smooth;
		}
#endif

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
			hsFilter.accelerometerHSR();
			hsFilter.estimateVelbyAccel();
			hsFilter.estimateVelbyGyro();
		}

	#else
		hsFilter.setDt(dt);
		hsFilter.complementaryFilter();
	#endif

	

	


	#ifdef ALTITUDE_CONTROL

		//if (b_alt_ctrl_flag == true) {
		//	ultra_filtered = hsFilter.alt_filter((int)ultra, alt_lpf);
		//	b_alt_ctrl_flag = false;
		//}

	

		//if (b_alt_ctrl_flag == true) {
		//	ultra_filtered = hsFilter.alt_filter((int)ultra, n_alt_turn_length);
		//	
		//}

	#endif

	/////////////////////////////////////////////////////////////////////////////////////////////// Filtering Part

	//test_time1 = micros();
	//Serial.print( (test_time1 - start_time) / 1000.0 );
	//Serial.print( "\t" );

	/////////////////////////////////////////////////////////////////////////////////////////////// Controller Part
		hsFC.setDt(dt);
		hsFC.setSmoothValue(hsFilter.m_gx, hsFilter.m_gy, hsFilter.m_gz);

	
	#ifdef TIMED_MULTIPLEX_CONTROL 
		if(	cnt_4angular >= TIMED_MULTIPLEX_CONTROL_DELAY_COUNT ) {
			hsFC.setSetPoint( roll_setPoint+roll_offset, pitch_setPoint+pitch_offset );
			hsFC.setEulerAngle(hsFilter.m_roll, hsFilter.m_pitch, hsFilter.m_yaw);
			hsFC.AngularControl(dt_4angular);
		}

		hsFC.AngularRateControl();
	#else
		hsFC.setSetPoint(roll_setPoint, pitch_setPoint);
		hsFC.setEulerAngle(hsFilter.m_roll, hsFilter.m_pitch, hsFilter.m_yaw);
		hsFC.AngularControl(dt);
		hsFC.AngularRateControl();
	#endif

	#ifdef ALTITUDE_CONTROL

		if (b_alt_ctrl_flag == true) {
			hsFilter.generalLowPassFilter(&alt_lpf, ultra, 0.5, 0.5);
			hsFC.altitudeControl(80, alt_lpf);
			b_alt_ctrl_flag = false;
			hsFC.isFirst = false;
		}else {
			hsFC.altitudeControl_onlyD();
		}
	

	#endif
		hsFC.calcForce();
		hsFC.setPwm(pwm);
		hsFC.generatePWM(&pwm1, &pwm2, &pwm3, &pwm4);

	

	/////////////////////////////////////////////////////////////////////////////////////////////// Controller Part

	//test_time2 = micros();
	//Serial.print( (test_time2 - test_time1) / 1000.0 );
	//Serial.println();	


	#if 0
		analogWrite(2, (int)pwm1);
		analogWrite(3, (int)pwm2);
		analogWrite(11, (int)pwm3);
		analogWrite(12, (int)pwm4);
	#else
		analogWrite(2, (int)90);
		analogWrite(3, (int)90);
		analogWrite(11, (int)90);
		analogWrite(12, (int)90);
	#endif

		//TCCR1B = 4;
		//TCCR3B = 4;
		/////////////////////////////////////////////////////////////////////////////////// 통신 안정되게 구현하고 실험하자!! 또 부셔먹지 말고..
		/////////////////////////////////////////////////////////////////////////////////// 아니면 줄이나 안전장치 연결을 하든가...
		/////////////////////////////////////////////////////////////////////////////////// 야외 실험시 실내에서 충분히 테스트 후 나가자

		// print part


		//Serial.print(hsFilter.vel_xa);
		//Serial.print("\t");
		//Serial.print(hsFilter.vel_ya);
		//Serial.print("\t");
		//Serial.print(hsFilter.vel_xg);
		//Serial.print("\t");
		//Serial.print(hsFilter.vel_yg);
		//Serial.println("\t");

		//Serial.print(hsFilter.m_roll);
		//Serial.print("\t");
		//Serial.print(hsFilter.m_pitch);
		//Serial.println("\t");

		//Serial.print(hsFilter.roll_acc);
		//Serial.print("\t");
		//Serial.print(hsFilter.roll_gyro);
		//Serial.print("\t");
		//Serial.print(hsFilter.m_roll);
		//Serial.print("\t");
		//Serial.print(hsFilter.pitch_acc);
		//Serial.print("\t");
		//Serial.print(hsFilter.pitch_gyro);
		//Serial.print("\t");
		//Serial.print(hsFilter.m_pitch);
		//Serial.print("\t");
		//
		//Serial.println();


		//Serial.print(hsFC.m_roll);
		//Serial.print("\t");
		//Serial.print(hsFC.roll_p);
		//Serial.print("\t");
		//Serial.print(hsFC.roll_pid_q_avg);
		//Serial.print("\t");
		//Serial.print(hsFC.roll_rate_p);
		//Serial.print("\t");
		//Serial.print(hsFC.gx_mean);
		//Serial.print("\t");
		//Serial.print(roll_setPoint);
		//Serial.print("\t");
		//Serial.print(hsFC.roll_rate_d);
		//Serial.println();


		//Serial.print(dt*1000);
		//Serial.println();

		//Serial.print(hsFilter.zeroG_ax);
		//Serial.print("\t");
		//Serial.print(hsFilter.zeroG_ay);
		//Serial.print("\t");
		////Serial.print(hsFilter.zeroG_az);
		////Serial.print("\t");
		//Serial.print(hsFilter.vel_x);
		//Serial.print("\t");
		//Serial.print(hsFilter.vel_y);
		//Serial.print("\t");
		////Serial.print(hsFilter.vel_z);
		////Serial.println();

		//Serial.print(hsFC.m_roll);
		//Serial.print("\t");
		//Serial.print(hsFC.gx_mean);
		//Serial.print("\t");
		//Serial.print(hsFC.roll_rate_p);
		//Serial.print("\t");
		//Serial.print(hsFC.roll_rate_d);
		//Serial.println();

		
	

	#ifdef TIMED_MULTIPLEX_CONTROL 
		if(	cnt_4angular >= TIMED_MULTIPLEX_CONTROL_DELAY_COUNT ) {


			dt_4angular = 0;
			cnt_4angular = 0;

		}
		dt_4angular += dt;
		cnt_4angular ++;
	#endif



		if( loop_cnt >= 200 ) {
		
			
			//Serial.print(hsFilter.m_roll);
			//Serial.print("\t");
			//Serial.print(hsFilter.m_pitch);
			//Serial.print("\t");
			//Serial.println(ultra);

			loop_cnt = 0;
			safe_mode = !safe_mode;
			digitalWrite(13, safe_mode); 
		}

	
		analogWrite(4, 10);
		loop_cnt++;



	}else if(VTOL_state == 1) {
		if( pwm > 90 ) {
			pwm_smooth -= 0.02;
			pwm = (int)pwm_smooth;
		}

		analogWrite(2, (int)pwm);
		analogWrite(3, (int)pwm);
		analogWrite(11, (int)pwm);
		analogWrite(12, (int)pwm);

	}

	
}



void init_timer() {
	//TCCR1B=0x0A;

}
void timeStamp() {

}
