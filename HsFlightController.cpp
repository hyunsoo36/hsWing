#include "HsFlightController.h"


void HsFlightController::attitudeControl() {



}
void HsFlightController::AngularControl(double dt) {
	roll_err = roll_setPoint - m_roll;
	pitch_err = pitch_setPoint - m_pitch;
	//yaw_err = yaw_setPoint - m_yaw;

	// P control
	roll_p = roll_err * p_gain;
	pitch_p = pitch_err * p_gain;
	//yaw_p = yaw_err * p_gain_yaw;

	// I control
	roll_err_integral += roll_err * dt;
	roll_i = roll_err_integral * i_gain;
	pitch_err_integral += pitch_err * dt;
	pitch_i = pitch_err_integral * i_gain;
	

	// integral wind_up 
	if(roll_i > i_wind_up_limit) { 
		roll_i = i_wind_up_limit;
	}
	else if(roll_i < 0-i_wind_up_limit) {
		roll_i = 0-i_wind_up_limit;
	}
	if(pitch_i > i_wind_up_limit) { 
		pitch_i = i_wind_up_limit;
	}
	else if(pitch_i < 0-i_wind_up_limit) {
		pitch_i = 0-i_wind_up_limit;
	}
	

	// D control
	roll_d = gx_mean * d_gain / dt;	
	pitch_d = -gy_mean * d_gain / dt;
	//yaw_d = gz_lpf * d_gain_yaw / dt;

	roll_pid = roll_p + roll_i - roll_d;
	pitch_pid = pitch_p  + pitch_i - pitch_d;
	//yaw_pid = yaw_p + yaw_i + yaw_d;

	

	//tmp_pid = roll_p + roll_i + roll_d;
}
void HsFlightController::AngularRateControl() {
	
	// queue shift
	memcpy( &roll_pid_q[0], &roll_pid_q[1], sizeof(double)*(TIMED_MULTIPLEX_CONTROL_DELAY_COUNT-1));
	memcpy( &pitch_pid_q[0], &pitch_pid_q[1], sizeof(double)*(TIMED_MULTIPLEX_CONTROL_DELAY_COUNT-1));
	roll_pid_q[TIMED_MULTIPLEX_CONTROL_DELAY_COUNT-1] = roll_pid;
	pitch_pid_q[TIMED_MULTIPLEX_CONTROL_DELAY_COUNT-1] = pitch_pid;
	roll_pid_q_avg = 0;	pitch_pid_q_avg = 0;
	for(int i=0; i<TIMED_MULTIPLEX_CONTROL_DELAY_COUNT; i++) {
		roll_pid_q_avg += roll_pid_q[i];
		pitch_pid_q_avg += pitch_pid_q[i];
	}
	roll_pid_q_avg /= (double)TIMED_MULTIPLEX_CONTROL_DELAY_COUNT;
	pitch_pid_q_avg /= (double)TIMED_MULTIPLEX_CONTROL_DELAY_COUNT;

	//
	//roll_rate_err = roll_pid - gx_mean;
	//pitch_rate_err = pitch_pid - gy_mean;

	// 각속도 P 제어		// 부호 주의할 것!
	roll_rate_p = (roll_pid - gx_mean) * p_gain_rate;
	pitch_rate_p = (pitch_pid - gy_mean) * p_gain_rate;

	roll_rate_err = roll_pid_q_avg - gx_mean;
	pitch_rate_err = pitch_pid_q_avg - gy_mean;

	yaw_rate_err = yaw_setPoint - m_yaw_rate;
	yaw_rate_p = yaw_rate_err * p_gain_yaw;

	// 각속도 I 제어	// Only for yaw control
	yaw_err_integral += yaw_rate_err * m_dt;
	
	//if(yaw_err_integral > i_wind_up_limit_yaw) { 
	//	yaw_err_integral = i_wind_up_limit_yaw;
	//}else if(yaw_err_integral < 0-i_wind_up_limit_yaw) {
	//	yaw_err_integral = 0-i_wind_up_limit_yaw;
	//}
	yaw_rate_i = yaw_err_integral * i_gain_yaw;

	// 각속도 D제어
	roll_rate_d = (roll_rate_err - roll_rate_err_last) * d_gain_rate / m_dt;
	pitch_rate_d = (pitch_rate_err - pitch_rate_err_last) * d_gain_rate / m_dt;

	roll_rate_err_last = roll_rate_err;
	pitch_rate_err_last = pitch_rate_err;

	roll_rate_pid = roll_rate_p + roll_rate_d;
	pitch_rate_pid = pitch_rate_p + pitch_rate_d;

	yaw_rate_pid = yaw_rate_p + yaw_rate_i;


}


void HsFlightController::generatePWM(int* pwm1, int* pwm2, int* pwm3, int* pwm4) {
	
#if 1
	// yaw 제어에 의한 고도변화를 최소화하기 위한 알고리즘
	basePWM = m_pwm + alt_pid;
	yaw_b = sqrt( 2*(basePWM)*(basePWM) - (basePWM-abs(yaw_rate_pid))*(basePWM-abs(yaw_rate_pid)) ) - basePWM;
	if( yaw_rate_pid >= 0 ) {
		// x콥터 방식
		*pwm1 = round( (m_pwm) + alt_pid + pitch_rate_pid + roll_rate_pid + yaw_b);
		*pwm2 = round( (m_pwm) + alt_pid - pitch_rate_pid + roll_rate_pid - yaw_rate_pid );
		*pwm3 = round( (m_pwm) + alt_pid - pitch_rate_pid - roll_rate_pid + yaw_b );
		*pwm4 = round( (m_pwm) + alt_pid + pitch_rate_pid - roll_rate_pid - yaw_rate_pid );
	}else {
		// x콥터 방식
		*pwm1 = round( (m_pwm) + alt_pid + pitch_rate_pid + roll_rate_pid - yaw_b);
		*pwm2 = round( (m_pwm) + alt_pid - pitch_rate_pid + roll_rate_pid - yaw_rate_pid );
		*pwm3 = round( (m_pwm) + alt_pid - pitch_rate_pid - roll_rate_pid - yaw_b );
		*pwm4 = round( (m_pwm) + alt_pid + pitch_rate_pid - roll_rate_pid - yaw_rate_pid );
	}
#else
	*pwm1 = round( (m_pwm) + alt_pid + pitch_rate_pid + roll_rate_pid + yaw_rate_pid);
	*pwm2 = round( (m_pwm) + alt_pid - pitch_rate_pid + roll_rate_pid - yaw_rate_pid );
	*pwm3 = round( (m_pwm) + alt_pid - pitch_rate_pid - roll_rate_pid + yaw_rate_pid );
	*pwm4 = round( (m_pwm) + alt_pid + pitch_rate_pid - roll_rate_pid - yaw_rate_pid );

#endif

	*pwm1 = *pwm1 < 0 ? 0 : (*pwm1 > 254 ? 254 : *pwm1);
	*pwm2 = *pwm2 < 0 ? 0 : (*pwm2 > 254 ? 254 : *pwm2);
	*pwm3 = *pwm3 < 0 ? 0 : (*pwm3 > 254 ? 254 : *pwm3);
	*pwm4 = *pwm4 < 0 ? 0 : (*pwm4 > 254 ? 254 : *pwm4);

}

void HsFlightController::altitudeControl(double setpoint, double alt_lpf) {
	alt_err[1] = alt_err[0];
	alt_err[0] = setpoint - alt_lpf;

	// P control
	alt_p = alt_err[0] * p_gain_alt;

	// I control
	alt_err_integral += alt_err[0] * HS_SRF08_SAMPLING_TIME;
	alt_i = alt_err_integral * i_gain_alt;
	
	// integral wind up
	if(alt_i > i_wind_up_limit_alt) { 
		alt_err_integral = i_wind_up_limit_alt / i_gain_alt;
	}
	else if(alt_i < 0-i_wind_up_limit_alt) {
		alt_err_integral = -i_wind_up_limit_alt / i_gain_alt;
	}

	// D control
	if( ! isFirst ) {
		alt_d = ((alt_err[0] - alt_err[1]) * d_gain_alt) / HS_SRF08_SAMPLING_TIME;
		alt_d_lpf = alt_d_lpf * 0.9 + alt_d * 0.1;
	}
	


	alt_pid = alt_p + alt_i + alt_d_lpf;
	//Serial.print(alt_p);
	//Serial.print("\t");
	//Serial.print(alt_i);
	//Serial.print("\t");
	//Serial.print(alt_d_lpf);
	//Serial.print("\t");
	//Serial.print(alt_pid);
	//Serial.println("\t");

	
}
void HsFlightController::altitudeControl_onlyD() {
	if( ! isFirst ) {
		alt_d_lpf = alt_d_lpf * 0.9 + alt_d * 0.1;
	}
	alt_pid = alt_p + alt_i + alt_d_lpf;
}

HsFlightController::HsFlightController() {
	initialize();

}

HsFlightController::~HsFlightController() {


}



void HsFlightController::initialize() {

#if 1

	p_gain = 5.0;
	i_gain = 0.0;//7.5;
	d_gain = 0.0;

	p_gain_rate = 0.08;//0.18;
	d_gain_rate = 0.004;//0.006;


	p_gain_yaw = 0.22;//0.38;
	i_gain_yaw = 0.80;
	d_gain_yaw = 0;//0.0017;

	
	p_gain_alt = 0.17;
	i_gain_alt = 0.01;//0.08;
	d_gain_alt = 0.17;


	//p_gain = 5.0;//2.0;//2.2;
	//i_gain = 0;//0.3;
	//d_gain = 0;//0.015;

	//p_gain_rate = 0.05;//0.15;//0.17;//0.16;
	//d_gain_rate = 0.006;

	//p_gain_yaw = 0.18;//0.38;
	//i_gain_yaw = 0;//0.10;
	//d_gain_yaw = 0.0018;

	//
	//p_gain_alt = 0.21;
	//i_gain_alt = 0;//0.08;
	//d_gain_alt = 0.17;
	
#else

	p_gain = 0;
	i_gain = 0;
	d_gain = 0;

	p_gain_rate = 0;
	d_gain_rate = 0;
	
	p_gain_yaw = 0;
	i_gain_yaw = 0;
	d_gain_yaw = 0;

	p_gain_alt = 0;
	i_gain_alt = 0;
	d_gain_alt = 0;	

#endif

	roll_p=0, roll_i=0, roll_d=0;
	pitch_p=0, pitch_i=0, pitch_d=0;
	yaw_p=0, yaw_i=0, yaw_d=0;

	roll_rate_p = 0, pitch_rate_p = 0;

	roll_setPoint = 0.0;
	pitch_setPoint = 0.0;
	yaw_setPoint = 0.0;
	roll_err = 0;
	pitch_err = 0;
	yaw_err = 0;
	roll_err_integral = 0; pitch_err_integral = 0; yaw_err_integral = 0;
	i_wind_up_limit = 20; i_wind_up_limit_yaw = 12.5;
	
	f_right = 0; f_left = 0; f_front = 0; f_back = 0;

	alt_setPoint = 0;
	alt_p = 0, alt_i = 0, alt_d = 0;
	alt_err[0] = 0;	alt_err[1] = 0;
	alt_err_integral = 0;
	i_wind_up_limit_alt = 5;
	alt_d_lpf = 0;
	alt_pid = 0;

	ctl_gain = 1.00;

	isFirst = true;

	tmp_pid = 0;

	roll_rate_p = 0;	roll_rate_d = 0;	roll_rate_pid = 0; 
	pitch_rate_p = 0;	pitch_rate_d = 0;	pitch_rate_pid = 0; 

}

void HsFlightController::destroy() {

}
void HsFlightController::setEulerAngleAndRate(double roll, double pitch, double yaw_rate) {
	m_roll = roll;
	m_pitch = pitch;
	m_yaw_rate = yaw_rate;
}
void HsFlightController::setSmoothValue(double xValue, double yValue, double zValue) {
	gx_mean = xValue;
	gy_mean = yValue;
	gz_lpf = zValue;

}
void HsFlightController::setSetPoint(double r, double p) {
	roll_setPoint = r;
	pitch_setPoint = p;


}