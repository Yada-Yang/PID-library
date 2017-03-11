#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>

class PID
{
public:
	PID(double, double, double);

	bool compute();

	void setMode(bool mode); // * sets PID to either Manual (0) or Auto (non-0)

	void setOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
	                                      //   it's likely the user will want to change this depending on
	                                      //   the application

	void setTunings(double, double, double);

	void setSampleTime(uint32_t); // * sets the frequency, in Milliseconds, with which
	                              //   the PID calculation is performed.  default is 100

	//Display functions ****************
	double getKp();
	double getKi();
	double getKd();
	bool   getMode();

	volatile double input;
	volatile double output;
	volatile double setpoint;

private:
	void initialize();
	double disp_kp;
	double disp_ki;
	double disp_kd;

	double _kp;
	double _ki;
	double _kd;

	double ITerm;
	double lastInput;

	uint32_t sampleTime;
	double outMin;
	double outMax;
	bool inAuto;
};
#endif
