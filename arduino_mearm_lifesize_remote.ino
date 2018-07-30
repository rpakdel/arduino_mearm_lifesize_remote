#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "IRLifeSize.h"

#define IRPIN 6
IRLifeSize irlifesize(IRPIN);

Adafruit_PWMServoDriver pwm;

// servo index

#define BASE_SERVO 0  // middle, turn left/right
#define LEFT_SERVO 1  // left, extend/constract arm
#define RIGHT_SERVO 2 // right forward/backward arm
#define CLAW_SERVO 3  // claw open/close

// PWM controller pins

uint8_t pwmPins[4] = { 0, 15, 1, 3 };

#define TOWERPRO_MG90S_FREQ 50 // from manual

#define CLAW_MIN_PULSE 300 // open
#define CLAW_MAX_PULSE 550 // closed

#define BASE_MIN_PULSE 120 // right dir
#define BASE_MAX_PULSE 440 // left dir

#define LEFT_MIN_PULSE 200 // arm contracted
#define LEFT_MAX_PULSE 350 // arm extended

#define RIGHT_MIN_PULSE 250 // arm contracted
#define RIGHT_MAX_PULSE 450 // arm extended


#define CLAW_OPEN_ANGLE 160 // not used
#define CLAW_CLOSE_ANGLE 0 // not used

#define BASE_LEFT_ANGLE 10 // not used
#define BASE_RIGHT_ANGLE 80 // not used


// actual min/max [135, 585] based on BASE servo
// try 150, 530 to make base centered
int servo_min_pulse[4] = { BASE_MIN_PULSE, LEFT_MIN_PULSE, RIGHT_MIN_PULSE, CLAW_MIN_PULSE };
int servo_max_pulse[4] = { BASE_MAX_PULSE, LEFT_MAX_PULSE, RIGHT_MAX_PULSE, CLAW_MAX_PULSE };

uint16_t currentPulseValues[4] = { 0, 0, 0, 0 };
uint16_t prevPulseValues[4] = { 0, 0, 0, 0 };

float valueScales[4] = { 1.0f, 1.0f, 1.0f, 0.5f };

#define SPEED_FAST 18.0
#define SPEED_NORMAL 10.0
#define SPEED_SLOW 5.0

float turbo = SPEED_NORMAL;

void resetAllValues()
{
    Serial.println("Resetting values");
    currentPulseValues[LEFT_SERVO] = (servo_min_pulse[LEFT_SERVO] + servo_max_pulse[LEFT_SERVO]) / 2.0f;
    currentPulseValues[RIGHT_SERVO] = (servo_min_pulse[RIGHT_SERVO] + servo_max_pulse[RIGHT_SERVO]) / 2.0f;
    currentPulseValues[BASE_SERVO] = (servo_min_pulse[BASE_SERVO] + servo_max_pulse[BASE_SERVO]) / 2.0f;
    currentPulseValues[CLAW_SERVO] = (servo_min_pulse[CLAW_SERVO] + servo_max_pulse[CLAW_SERVO]) / 2.0f;
}

void setupIR()
{
    irlifesize.enable();
}

void setup()
{
    Serial.println("Init");
    Serial.begin(9600);
    pwm.begin();
    pwm.setPWMFreq(TOWERPRO_MG90S_FREQ);
    resetAllValues();
    setupIR();
}

void setPulseLength(uint8_t servonum, uint16_t pulselen)
{
    pwm.setPWM(pwmPins[servonum], 0, pulselen);
}

void servo_setMin(uint8_t servonum)
{
    setPulseLength(servonum, servo_min_pulse[servonum]);
}

void servo_setMax(uint8_t servonum)
{
    setPulseLength(servonum, servo_max_pulse[servonum]);
}

void servo_setMid(uint8_t servonum)
{
    setPulseLength(servonum, (servo_min_pulse[servonum] + servo_max_pulse[servonum]) / 2);
}

void servo_sweep_up(uint8_t servonum)
{
    for (uint16_t pulselen = servo_min_pulse[servonum]; pulselen <= servo_max_pulse[servonum]; pulselen++)
    {
        setPulseLength(servonum, pulselen);
    }
}

void servo_sweep_down(uint8_t servonum)
{
    for (uint16_t pulselen = servo_max_pulse[servonum]; pulselen >= servo_min_pulse[servonum]; pulselen--)
    {
        setPulseLength(servonum, pulselen);
    }
}

void getServoValues(Button button, int8_t values[4])
{
    // reset the arm
    if (button == Red)
    {
        resetAllValues();
    }

    if (button == Yellow)
    {
        turbo = SPEED_SLOW;
    }

    if (button == Blue)
    {
        turbo = SPEED_NORMAL;
    }

    if (button == Green)
    {
        turbo = SPEED_FAST;
    }

    int8_t rightServo = 0;
    if (button == Up)
    {
        rightServo = 1;
    }

    if (button == Down)
    {
        rightServo = -1;
    }

    int8_t baseServo = 0;
    if (button == Left)
    {
        baseServo = -1;
    }

    if (button == Right)
    {
        baseServo = 1;
    }

    int8_t leftServo = 0;
    if (button == ZoomIn)
    {
        leftServo = 1;
    }

    if (button == ZoomOut)
    {
        leftServo = -1;
    }

    int8_t clawServo = 0.0;
    if (button == VolUp)
    {
        clawServo = -1;
    }

    if (button == VolDown)
    {
        clawServo = 1;
    }
    
    values[0] = baseServo;
    values[1] = leftServo;
    values[2] = rightServo;
    values[3] = clawServo;
}

char* getServoName(uint8_t servo)
{
    switch (servo)
    {
    case 0:
        return "Base";

    case 1:
        return "Left";

    case 2:
        return "Right";

    case 3:
        return "Claw";
    }

    return "unknown";
}



void setAllPulseLengths(uint16_t pulseValues[4])
{
    for (uint8_t servo = 0; servo < 4; ++servo)
    {
        if (prevPulseValues[servo] != pulseValues[servo])
        {
            //Serial.print(getServoName(servo));
            //Serial.print(F(" "));
            //Serial.println(pulseValues[servo]);
            prevPulseValues[servo] = pulseValues[servo];
            setPulseLength(servo, pulseValues[servo]);
        }
    }
}

void applyToCurrentPulseValue(uint8_t servo, int8_t value)
{
    if (value == 0)
    {
        return;
    }

    //Serial.print("Apply ");
    //Serial.print(getServoName(servo));
    //Serial.print("-");
    //Serial.println(value);

    float currentValue = currentPulseValues[servo];

    currentValue += value * turbo / valueScales[servo];
    // clip to min max
    if (currentValue >= servo_max_pulse[servo])
    {
        currentValue = servo_max_pulse[servo];
    }

    if (currentValue <= servo_min_pulse[servo])
    {
        currentValue = servo_min_pulse[servo];
    }

    currentPulseValues[servo] = currentValue;
}

bool isScriptButton(Button button)
{
    switch (button)
    {
    case One:
        return true;
    }

    return false;
}



void playScript(Button button)
{
    applyButtonToArm(Left);
    applyButtonToArm(Left);
    applyButtonToArm(Up);
    applyButtonToArm(Up);
    applyButtonToArm(VolUp);
    applyButtonToArm(Right);
}

void applyButtonToArm(Button button)
{
    // manual button
    // apply button if no button is pressed
    int8_t servoValues[4] = { 0, 0, 0, 0 };
    getServoValues(button, servoValues);

    applyToCurrentPulseValue(BASE_SERVO, servoValues[0]);
    applyToCurrentPulseValue(LEFT_SERVO, servoValues[1]);
    applyToCurrentPulseValue(RIGHT_SERVO, servoValues[2]);
    applyToCurrentPulseValue(CLAW_SERVO, servoValues[3]);

    setAllPulseLengths(currentPulseValues);
}

void loop()
{
    Button button = irlifesize.checkForButton();

    if (isScriptButton(button))
    {
        playScript(button);
    }
    else
    {
        applyButtonToArm(button);
    }
}
