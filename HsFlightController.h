
// ************************************************ //
//			Disigned by hs. In RACS Lab.			//
// ************************************************ //

 
// HsFlightController 클래스는 중첩 제어기를 사용한다.
// 현재 자세각을 입력받으면 목표 자세각으로 가기 위한 모터 출력치를 출력한다.


#include "srf08.h"
#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

class HsFlightController {

public:
	double m_roll, m_pitch, m_yaw;

	double p_gain, i_gain, d_gain;
	double p_gain_rate, d_gain_rate;
	double roll_p, roll_i, roll_d, roll_pid;
	double pitch_p, pitch_i, pitch_d, pitch_pid;
	double yaw_p, yaw_i, yaw_d, yaw_pid;
	double roll_rate_err, pitch_rate_err;
	double roll_rate_err_last, pitch_rate_err_last;
	double roll_rate_p, roll_rate_d, roll_rate_pid;
	double pitch_rate_p, pitch_rate_d, pitch_rate_pid;

	double m_dt;
	double m_pwm;

	double gx_mean, gy_mean;
	double gz_lpf;

	double roll_setPoint;
	double pitch_setPoint;
	double yaw_setPoint;
	double roll_err, pitch_err, yaw_err;
	double roll_err_integral, pitch_err_integral, yaw_err_integral;
	double i_wind_up_limit, i_wind_up_limit_yaw;
	double p_gain_yaw, i_gain_yaw, d_gain_yaw;

	double alt_setPoint;
	double p_gain_alt, i_gain_alt, d_gain_alt;
	double alt_p, alt_i, alt_d;
	double alt_err[2];
	double alt_err_integral;
	double i_wind_up_limit_alt;
	double alt_d_lpf;
	double alt_pid;

	bool isFirst;

	double f_right, f_left, f_front, f_back;

	unsigned char* dataBuffer;

	double ctl_gain;

	double tmp_pid;

public:
	HsFlightController();
	~HsFlightController();

	void initialize();
	void destroy();

	void attitudeControl();
	void calcForce();
	void generatePWM(int* pwm1, int* pwm2, int* pwm3, int* pwm4);
	void altitudeControl(double setpoint, double alt_lpf);
	void altitudeControl_onlyD();

	// 중첩 제어
	void AngularControl(double dt);
	void AngularRateControl();

	// getter, setter method
	void setDt(double dt) { m_dt = dt; }	// 필수
	void setEulerAngle(double roll, double pitch, double yaw);	// 필수
	void setSmoothValue(double xValue, double yValue, double zValue);	// 필수
	void setPwm(double pwm) { m_pwm = pwm; }	// 필수
	void setSetPoint(double roll, double pitch);	// 필수
	void setDataBuffer(unsigned char* buf) { dataBuffer = buf; }
	void setPidGain(double p, double i, double d) {
		p_gain = p; i_gain = i; d_gain = d;
	}
};