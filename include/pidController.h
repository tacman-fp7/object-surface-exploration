#pragma once
#include <cstddef>
#include <time.h>

typedef struct pidParameters pidParams_t;

struct pidParameters
{
	double Kp;
	double Ki;
	double Kd;
	double outMax;
	double outMin;
};

class PidController
{

public:
	PidController();
	void setKp(double Kp);
	void setKi(double Ki);
	void setKd(double Kd);
	void setOutMax(double outMax);
	void setOutMin(double outMin);
	virtual void setSetpoint(double setpoint);
	
	double getSetpoint();
    void setParameters(pidParams_t params);
	virtual void setRampSetpoint(double setpoint);


    virtual double update(double curval);
//{return pidUpdate(curval);}
	
    //double update(double curVal, double velocity);
    //double pidUpdate(double curVal);

private:
	inline double P(double error);
	inline double I(double error);
	inline double D(double error);
	

protected:
	double _integral;
	double _setpoint;
	double _rampSetpoint;
	void rampSetpoint();
	double _rampValue;

private:
	double _Kp;
	double _Ki;
	double _Kd;
	double _outMax;
	double _outMin;
	clock_t _clkTicks;
	time_t _timer;
	double _tSample;
	
	
};

