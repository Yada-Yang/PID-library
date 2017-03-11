#include "PID.h"

PID::PID(double kp, double ki, double kd)
{
	output   = 0;
	input    = 0;
	setpoint = 0;

	ITerm    = 0;
	lastInput= 0;

	sampleTime = 100;
	inAuto = true;

	PID::setOutputLimits(0, 255);

	PID::setTunings(kp, ki, kd);
}

bool PID::compute()
{
	if (!inAuto)
		return false;
	/*Compute all the working error variables*/
	double error = setpoint - input;
	ITerm += (_ki * error);
	if (ITerm > outMax)      ITerm = outMax;
	else if (ITerm < outMin) ITerm = outMin;
	double dInput = (input - lastInput);

	/*Compute PID output*/
	output = _kp*error + ITerm - _kd*dInput;

	if (output > outMax)      output = outMax;
	else if (output < outMin) output = outMin;

	/*Remember some variables for next time*/
	lastInput = input;
	return true;
}


/* setTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::setTunings(double kp, double ki, double kd)
{
	if ( (kp * ki < 0) || (kp * kd < 0) || (ki * kd < 0) )
		return;

	disp_kp = kp;
	disp_ki = ki;
	disp_kd = kd;

	_kp = kp;
	_ki = ki * sampleTime;
	_kd = kd / sampleTime;
}

/* setSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::setSampleTime(uint32_t newSampleTime)
{

	double ratio  = (double)newSampleTime / (double)sampleTime;
	_ki *= ratio;
	_kd /= ratio;
	sampleTime = newSampleTime;

}

/* setOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::setOutputLimits(double min, double max)
{
	if (min >= max)
		return;

	outMin = min;
	outMax = max;

	if (inAuto) {
		if (output > outMax)      output = outMax;
		else if (output < outMin) output = outMin;

		if (ITerm > outMax)       ITerm = outMax;
		else if (ITerm < outMin)  ITerm = outMin;
	}

}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::setMode(bool mode)
{
	if (mode && !inAuto) {
	/*we just went from manual to auto*/
        PID::initialize();
	}
	inAuto = mode;
}

/* initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::initialize()
{
   ITerm = output;
   lastInput = input;
   if (ITerm > outMax)      ITerm = outMax;
   else if (ITerm < outMin) ITerm = outMin;
}


/* Status Funcions*************************************************************
 * Just because you set the kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::getKp(){ return  disp_kp;}
double PID::getKi(){ return  disp_ki;}
double PID::getKd(){ return  disp_kd;}
bool   PID::getMode(){ return inAuto;}
