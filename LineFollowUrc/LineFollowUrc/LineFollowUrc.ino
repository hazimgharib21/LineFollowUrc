#define arraySize(x) ((sizeof(x))/(sizeof(x[0])))

/* Motor variables*/
const uint8_t motorRightA = 7;
const uint8_t motorRightB = 8;
const uint8_t motorRightPWM = 10;
const uint8_t motorLeftA = 5;
const uint8_t motorLeftB = 6;
const uint8_t motorLeftPWM = 9;


uint8_t outputPin[] = {
  motorRightA,
  motorRightB,
  motorRightPWM,
  motorLeftA,
  motorLeftB,
  motorLeftPWM,
};

/* Line sensor variables */
uint8_t lineSensor[5] = { 2, 3, 4, 11, 12 };
int sensorReading[5] = { 0 };
double avgSensor;

/* PID variables */
unsigned long lastTime = 0;
double Input, Output, Setpoint = 3.0;
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

double P = 50;
double I = 0;
double D = 0;
int baseSpeed = 100;

void setup()
{
	Serial.begin(9600);

	for (uint8_t i = 0; i < arraySize(lineSensor); i++)
	{
		pinMode(lineSensor[i], INPUT);
	}
	for (uint8_t i = 0; i < arraySize(outputPin); i++) {
		pinMode(outputPin[i], OUTPUT);
	}

	SetMode(AUTOMATIC);
	SetTunings(P, I, D);
	SetOutputLimits(-255, 255);
}

void loop()
{
	readLineSensor();
	ComputeInput();
	ComputePID();
}

void readLineSensor()
{
	for (uint8_t i = 0; i < arraySize(lineSensor); i++)
	{
		sensorReading[i] = digitalRead(lineSensor[i]);
		//Serial.print(sensorReading[i]);
	}
	//Serial.println();
}

void ComputeInput()
{
	int activeSensor = 0;
	double totalSensor = 0;

	/* Calculate the cumulative average */
	for (uint8_t i = 0; i < arraySize(sensorReading); i++) {
		if (sensorReading[i] == 1) { activeSensor += 1; }
		totalSensor += (double)sensorReading[i] * (i + 1);
	}

	/* We don't want error from division by 0 */
	if (activeSensor == 0) { Input = 0; }
	else
	{
		/* Compute the cumulative average */
		Input = totalSensor / activeSensor;
	}
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
		if (lastErr < 0) { Setpoint = -3.0; }
		else { Setpoint = 3.0; }
	}
	else { Setpoint = 3.0; }

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

	if(controllerDirection == REVERSE)
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

	motorRight(pwmRight);
	motorLeft(pwmLeft);
}

void motorLeft(int motorSpeed) 
{

	if (motorSpeed > 0)
	{
		digitalWrite(motorLeftA, HIGH);
		digitalWrite(motorLeftB, LOW);
		analogWrite(motorLeftPWM, motorSpeed);
	}
	else
	{
		digitalWrite(motorLeftA, LOW);
		digitalWrite(motorLeftB, HIGH);
		analogWrite(motorLeftPWM, abs(motorSpeed));
	}
}

void motorRight(int motorSpeed)
{

	if (motorSpeed > 0)
	{
		digitalWrite(motorRightA, LOW);
		digitalWrite(motorRightB, HIGH);
		analogWrite(motorRightPWM, motorSpeed);
	}
	else
	{
		digitalWrite(motorRightA, HIGH);
		digitalWrite(motorRightB, LOW);
		analogWrite(motorRightPWM, abs(motorSpeed));
	}
}