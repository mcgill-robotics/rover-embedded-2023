#ifdef SCIENCE //LEAVE THIS AT THE TOP OF THIS FILE
#include "science.h"

float scienceTargets[3];
float scienceFeedback[5];

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);
IntervalTimer stepperTimer, brakeTimer;

int leadScrewState = 0, brakeTimeDone = 1;
float leadScrewLastSpeed = 0;

enum Limits {
  off = 0,
  up = 1,
  down = -1
};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void stepperISR(){
  stepper.run();
}

void brakeCB(){
  brakeTimeDone = 1;
  brakeTimer.end();
}

void readAllMoistures(){
  scienceFeedback[0] = analogRead(MOIST_1) * (89.189189189/4095.0);
  scienceFeedback[1] = analogRead(MOIST_2) * (89.189189189/4095.0);
  scienceFeedback[2] = analogRead(MOIST_3) * (89.189189189/4095.0);
  scienceFeedback[3] = analogRead(MOIST_4) * (89.189189189/4095.0);
  scienceFeedback[4] = stepper.currentPosition() / GEAR_RATIO;
}

void writeAllMotors(){
  stepper.move(scienceTargets[0] * GEAR_RATIO);
  move(MOT1_DIR, MOT1_DIR_2, MOT1_PWM, scienceTargets[1]);
  float motorSpeed = scienceTargets[2];
  if((leadScrewState == Limits::down && motorSpeed < 0) || (leadScrewState == Limits::up && motorSpeed > 0)){
    motorSpeed = 0;
  }
  if((sgn(motorSpeed) != sgn(leadScrewLastSpeed)) || (leadScrewLastSpeed != 0 && motorSpeed == 0)){
    motorSpeed = 0;
    brake();
    brakeTimer.begin(brakeCB, 100000);
    brakeTimeDone = 0;
  }
  if(brakeTimeDone){
    move(MOT2_DIR, MOT2_DIR_2, MOT2_PWM, motorSpeed);
  }else{
    move(MOT2_DIR, MOT2_DIR_2, MOT2_PWM, 0);
  }
  leadScrewLastSpeed = motorSpeed;
}

void init_motors(){
  pinMode(MOT1_DIR, OUTPUT); 
  pinMode(MOT1_DIR_2, OUTPUT); 
  pinMode(MOT1_PWM, OUTPUT);

  pinMode(MOT2_DIR, OUTPUT);
  pinMode(MOT2_DIR_2, OUTPUT);
  pinMode(MOT2_PWM, OUTPUT);

}
void brake(){
  digitalWrite(MOT1_DIR, LOW); 
  digitalWrite(MOT1_DIR_2, LOW);
}

void forward(int dir1, int dir2){
  digitalWrite(dir1, HIGH); 
  digitalWrite(dir2, LOW); 
}

void reverse(int dir1, int dir2){
  digitalWrite(dir1, LOW); 
  digitalWrite(dir2, HIGH); 
}

void move(int dir1, int dir2, int pwmpin, int speed){
  (speed > 0) ? forward(dir1, dir2) : reverse(dir1, dir2); 
  analogWrite(pwmpin, speed*2.55);
}

void science_setup () {
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);

  stepper.setMaxSpeed(1000.0);
  stepper.setAcceleration(200.0);
  analogReadResolution(12);
  attachInterrupt(LIM_1, lim1ISR, FALLING);
  attachInterrupt(LIM_2, lim2ISR, FALLING);
  stepperTimer.begin(stepperISR, 1000000/STEPPER_FREQ);
  init_motors();
}

void science_loop () {
  if(digitalRead(LIM_1) == 1 && digitalRead(LIM_2) == 1){
    leadScrewState = Limits::off;
  }

  readAllMoistures();

  writeAllMotors();
}

void lim1ISR(){
  leadScrewState = Limits::up;
  brake();
  brakeTimer.begin(brakeCB, 100000);
  brakeTimeDone = 0;
}

void lim2ISR(){
  leadScrewState = Limits::down;
  brake();
  brakeTimer.begin(brakeCB, 100000);
  brakeTimeDone = 0;
}


#endif //LEAVE THIS AT THE BOTTOM OF THIS FILE