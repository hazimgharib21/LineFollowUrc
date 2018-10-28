#include <Wire.h>
#include <ZumoShield.h>

/* Zumo Setup */
ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

/* Line sensor variables */
unsigned int sensors[6];
double position;

/* PID variables */
unsigned long lastTime = 0;
double Input, Output, Setpoint = 2500;
double errSum, lastErr, lastInput, ITerm;
double kp, ki, kd;
double SampleTime = 10;// 10ms
double outMin, outMax;
bool inAuto = false;

#define MANUAL 0
#define AUTOMATIC 1

#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;

double P = 1;
double I = 0;
double D = 0;
int baseSpeed = 400;

void setup()
{
	// Initialize the reflectance sensors module
	reflectanceSensors.init();

	sensorCalibration();
	SetControllerDirection(REVERSE);
	SetMode(AUTOMATIC);
	SetTunings(P, I, D);
	SetOutputLimits(-400, 400);
}

void loop()
{
	ReadInput();
	ComputePID();
}

void ComputePID()
{
	if (!inAuto) return;
	/* How long since we last calculated */
	unsigned long now = millis();
	double timeChange = (double)(now - lastTime);

	/* If not time to compute, ignore calculation */
	if (timeChange <= SampleTime) { return; }


	/* Change the setpoint according to last error
	   This ensure the robot turns correctly to find the line
	*/
	if (Input == 0)
	{
		if (lastErr < 0) { Setpoint = -2500; }
		else { Setpoint = 2500; }
	}
	else { Setpoint = 2500; }

	/* Compute all the working error variables */
	double error = Setpoint - Input;
	//Serial.println(error);
	ITerm += (ki * error);
	if (ITerm > outMax) { ITerm = outMax; }
	else if (ITerm < outMin) { ITerm = outMin; }
	double dInput = (Input - lastInput);


	/* Compute PID Output */
	Output = (kp * error) + ITerm - (kd * dInput);

	MotorMove(Output);
	/* Remember some variables for next time */
	lastInput = Input;
	lastErr = error;
	lastTime = now;
}

void SetTunings(double Kp, double Ki, double Kd)
{
	if (Kp < 0 || Ki < 0 || Kd < 0) return;

	double SampleTimeInSec = ((double)SampleTime) / 1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;

	if (controllerDirection == REVERSE)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

/* re-tweaked ki and kd in case the sample time change during operation */
void SetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		double ratio = (double)NewSampleTime / (double)SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned long)NewSampleTime;
	}
}

void SetOutputLimits(double Min, double Max)
{
	if (Min > Max) return;
	outMin = Min;
	outMax = Max;

	if (Output > outMax) { Output = outMax; }
	else if (Output < outMin) { Output = outMin; }

	if (ITerm > outMax) { ITerm = outMax; }
	else if (ITerm < outMin) { ITerm = outMin; }
}

void SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto && !inAuto)
	{	/* went from manual to auto */
		Initialize();
	}
	inAuto = newAuto;
}

void Initialize()
{
	lastInput = Input;
	ITerm = Output;

	if (ITerm > outMax) { ITerm = outMax; }
	else if (ITerm < outMin) { ITerm = outMin; }
}

void SetControllerDirection(int Direction)
{
	controllerDirection = Direction;
}

void ReadInput()
{
	Input = reflectanceSensors.readLine(sensors);
}

void sensorCalibration()
{
	// Wait for the user button to be pressed and released
	//button.waitForButton();

	// Turn on LED to indicate we are in calibration mode
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	// Wait 1 second and then begin automatic sensor calibration
	// by rotating in place to sweep the sensors over the line
	delay(1000);
	int i;
	for (i = 0; i < 80; i++)
	{
		if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
			motors.setSpeeds(-200, 200);
		else
			motors.setSpeeds(200, -200);
		reflectanceSensors.calibrate();

		// Since our counter runs to 80, the total delay will be
		// 80*20 = 1600 ms.
		delay(20);
	}
	motors.setSpeeds(0, 0);

	// Turn off LED to indicate we are through with calibration
	digitalWrite(13, LOW);

	// Wait for the user button to be pressed and released
	button.waitForButton();

}

void MotorMove(int speed)
{
	int pwmRight, pwmLeft;

	if (speed < 0)
	{
		pwmRight = baseSpeed - abs(speed);
		pwmLeft = baseSpeed + abs(speed);
	}
	else
	{
		pwmRight = baseSpeed + speed;
		pwmLeft = baseSpeed - speed;
	}

	if (pwmRight > outMax) { pwmRight = outMax; }
	else if (pwmRight < outMin) { pwmRight = outMin; }
	if (pwmLeft > outMax) { pwmLeft = outMax; }
	else if (pwmLeft < outMin) { pwmLeft = outMin; }

	//motors.setSpeeds(pwmLeft, pwmRight);
}