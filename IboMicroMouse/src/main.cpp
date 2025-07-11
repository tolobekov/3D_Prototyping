#include <Arduino.h>
#include <black_box.h>
#include <FS_MX1508.h>
#include <micromouse.h>
#include <PID_v1.h>

unsigned long previous;
bool pin_state=true;
uint16_t sl;
uint16_t sfl;
uint16_t sr;
uint16_t sfr;
MX1508 motor_left(MM::PINS::MOTOR_1_A, MM::PINS::MOTOR_1_B);
MX1508 motor_right(MM::PINS::MOTOR_2_B, MM::PINS::MOTOR_2_A);

double setpoint=0.0;
double input;
double output;
 double Kp=0.027;
 double Ki=0.0;
 double Kd=0.0;
 PID steering_pid(&input, &output, &setpoint, Kp,Ki,Kd, DIRECT);

void setup() {
    steering_pid.SetMode(AUTOMATIC);
    steering_pid.SetSampleTime(10);
    steering_pid.SetOutputLimits(-50,50);
pinMode(1, OUTPUT);
pinMode(2, OUTPUT);
pinMode(MM::PINS::MOTOR_DRV_EN, OUTPUT);
digitalWrite(MM::PINS::MOTOR_DRV_EN, HIGH);
pinMode(MM::PINS::IR_LED_1, OUTPUT);
pinMode(MM::PINS::IR_LED_2, OUTPUT);
pinMode(MM::PINS::IR_LED_3, OUTPUT);
pinMode(MM::PINS::IR_LED_4, OUTPUT);
digitalWrite(MM::PINS::IR_LED_1, HIGH);
digitalWrite(MM::PINS::IR_LED_2, HIGH);
digitalWrite(MM::PINS::IR_LED_3, HIGH);
digitalWrite(MM::PINS::IR_LED_4, HIGH);

Serial.begin();
}

void loop() {
 sfl= analogRead(MM::PINS::IR_1);
sl= analogRead(MM::PINS::IR_2);
sr=analogRead(MM::PINS::IR_3);
sfr= analogRead(MM::PINS::IR_4);
Serial.printf("sl: %d, sfl: %d, sr: %d, sfr: %d\n", sl,sfl,sr,sfr);   

input = sl-sr;
steering_pid.Compute();
int baseSpeed=25;

int adjustSpeed = baseSpeed;

if (sfl<50 && sfr<50){
    adjustSpeed=baseSpeed+30;
}
else adjustSpeed=baseSpeed-4;
motor_left.motorGoP(adjustSpeed-output);
motor_right.motorGoP(adjustSpeed+output);
if (sfl>780 && sfr >780 && sl> 780 && sr>780){
    motor_left.motorBrake();
    motor_right.motorBrake();
    while(true);
}

}
