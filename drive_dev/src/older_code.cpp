#include <Servo.h>
#include <stdint.h>
#include <stdbool.h>
#include <tm4c123gh6pm.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include <Rover_SerialAPI.h>
#include <Arduino.h>
#include <driverlib/timer.h>
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
//#include "driverlib/watchdog.h"

/*
 * PINOUT NOTE:
 * 
 * All PWM output pins are configured
 * as open-drain for them to be able to work with 5V logic
 * level. As for Hall Sensor inputs, the TM4C pins are 5V-tolerant
 * to input already.
 * 
 */

#define CW 1
#define CCW -1

#define LB_MOTOR_HALL_A PA_2
#define RB_MOTOR_HALL_A PA_3
#define LF_MOTOR_HALL_A PA_4
#define RF_MOTOR_HALL_A PA_5

#define LB_MOTOR_HALL_B PC_4
#define RB_MOTOR_HALL_B PC_5
#define LF_MOTOR_HALL_B PC_6
#define RF_MOTOR_HALL_B PC_7

#define LB_MOTOR_HALL_C PF_1
#define RB_MOTOR_HALL_C PF_2
#define LF_MOTOR_HALL_C PF_3
#define RF_MOTOR_HALL_C PF_4

#define LB_SERVO_PIN PB_4
#define RB_SERVO_PIN PB_5
#define LF_SERVO_PIN PB_6
#define RF_SERVO_PIN PB_7

#define LB_CURRENT_SENSE_PIN PD_0
#define RB_CURRENT_SENSE_PIN PD_1
#define LF_CURRENT_SENSE_PIN PD_2
#define RF_CURRENT_SENSE_PIN PD_3

#define LB_CURRENT_NFAULT_PIN PE_0
#define RB_CURRENT_NFAULT_PIN PE_1
#define LF_CURRENT_NFAULT_PIN PE_2
#define RF_CURRENT_NFAULT_PIN PE_3


/**
 * Motors and the maximum acceleration we can do without fucking up their gearboxes,
 * expressed in arbitrary -100 to 100 units. Acceleration is per
 * loop (approx. 100ms right now, could go down to 10ms)
 */
Servo LBservo, LFservo, RBservo, RFservo;
#define MAX_ACCEL 5

volatile int direction;
int getRPM(long time_elapsed, long pulses);

void setPinAsOpenDrain(char port, int pin, int output);
void LFHallSensorA(), LFHallSensorB(), LFHallSensorC(),
     LBHallSensorA(), LBHallSensorB(), LBHallSensorC(),
     RFHallSensorA(), RFHallSensorB(), RFHallSensorC(),
     RBHallSensorA(), RBHallSensorB(), RBHallSensorC();
//void ConnectionLostISR();

/**
 * Serial structs and function signatures. Could we make these into a library pls
 * 
 */
#define SERIAL_RX_BUFFER_SIZE 64
typedef struct {
    float b[SERIAL_RX_BUFFER_SIZE/sizeof(float)];
    size_t count;
} FloatBuffer;
FloatBuffer fbuf;
void AddFloatToBuffer(FloatBuffer fb, float val);
void print_byte_array(byte* byte_array, size_t size);
void char_to_float(char* str_byte, float* f);
bool syn2master(HardwareSerial Serial);
void AccelInt(void);
static char buffer[SERIAL_RX_BUFFER_SIZE];
static float fbuffer[SERIAL_RX_BUFFER_SIZE];
volatile int lb_cur_speed = 1500;
volatile int lf_cur_speed = 1500;
volatile int rb_cur_speed = 1500;
volatile int rf_cur_speed = 1500;
char buf[17];
#define ID 'a'
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

// the setup routine runs once when you press reset:
void setup() {           
  SerialAPI::init('1', 9600);    
  setPinAsOpenDrain('B', 4, 1);
  setPinAsOpenDrain('B', 5, 1);
  setPinAsOpenDrain('B', 6, 1);
  setPinAsOpenDrain('B', 7, 1);

  // pinMode(LB_MOTOR_HALL_A, INPUT);
  // pinMode(LB_MOTOR_HALL_B, INPUT);
  // pinMode(LB_MOTOR_HALL_C, INPUT);
  // pinMode(RB_MOTOR_HALL_A, INPUT);
  // pinMode(RB_MOTOR_HALL_B, INPUT);
  // pinMode(RB_MOTOR_HALL_C, INPUT);
  // pinMode(LF_MOTOR_HALL_A, INPUT);
  // pinMode(LF_MOTOR_HALL_B, INPUT);
  // pinMode(LF_MOTOR_HALL_C, INPUT);
  // pinMode(RF_MOTOR_HALL_A, INPUT);
  // pinMode(RF_MOTOR_HALL_B, INPUT);
  // pinMode(RF_MOTOR_HALL_C, INPUT);

  // pinMode(LB_CURRENT_SENSE_PIN, INPUT);
  // pinMode(RB_CURRENT_SENSE_PIN, INPUT);
  // pinMode(LF_CURRENT_SENSE_PIN, INPUT);
  // pinMode(RF_CURRENT_SENSE_PIN, INPUT);
  // pinMode(LB_CURRENT_NFAULT_PIN, INPUT);
  // pinMode(RB_CURRENT_NFAULT_PIN, INPUT);
  // pinMode(LF_CURRENT_NFAULT_PIN, INPUT);
  // pinMode(RF_CURRENT_NFAULT_PIN, INPUT);

  LBservo.attach(LB_SERVO_PIN);
  LFservo.attach(LF_SERVO_PIN);
  RBservo.attach(RB_SERVO_PIN);
  RFservo.attach(RF_SERVO_PIN);

  // attachInterrupt(LF_MOTOR_HALL_A, LFHallSensorA, CHANGE);
  // attachInterrupt(LF_MOTOR_HALL_B, LFHallSensorB, CHANGE);
  // attachInterrupt(LF_MOTOR_HALL_C, LFHallSensorC, CHANGE);

  // attachInterrupt(LB_MOTOR_HALL_A, LBHallSensorA, CHANGE);
  // attachInterrupt(LB_MOTOR_HALL_B, LBHallSensorB, CHANGE);
  // attachInterrupt(LB_MOTOR_HALL_C, LBHallSensorC, CHANGE);

  // attachInterrupt(RB_MOTOR_HALL_A, RBHallSensorA, CHANGE);
  // attachInterrupt(RB_MOTOR_HALL_B, RBHallSensorB, CHANGE);
  // attachInterrupt(RB_MOTOR_HALL_C, RBHallSensorC, CHANGE);

  // attachInterrupt(RF_MOTOR_HALL_A, RFHallSensorA, CHANGE);
  // attachInterrupt(RF_MOTOR_HALL_B, RFHallSensorB, CHANGE);
  // attachInterrupt(RF_MOTOR_HALL_C, RFHallSensorC, CHANGE);
  
  // Lock motors and get ready to go
  LBservo.writeMicroseconds(1500);
  LFservo.writeMicroseconds(1500);
  RBservo.writeMicroseconds(1500);
  RFservo.writeMicroseconds(1500);

  // Timer interrupts for acceleration control
  // SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
  // while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4)){}
  // TimerConfigure(TIMER4_BASE, TIMER_CFG_A_PERIODIC);
  // TimerIntRegister(TIMER4_BASE, TIMER_A, AccelInt);
  // TimerLoadSet(TIMER4_BASE, TIMER_A, SysCtlClockGet());
  // TimerEnable(TIMER4_BASE, TIMER_A);
  // IntEnable(INT_TIMER4A);
  // TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

  // Watchdog timer for connection loss, set it for 2 seconds
  // SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
  // WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet() * 2);
  // WatchdogIntRegister(WATCHDOG0_BASE, &ConnectionLostISR);
  // WatchdogEnable(WATCHDOG0_BASE);

  // Watchdog timer to reset MCU if serial connection isn't recovered in 10 seconds.
  //SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);

  // Configure this timer to reset the system on its second interrupt (10 seconds)
  // WatchdogReloadSet(WATCHDOG1_BASE, SysCtlClockGet() * 5);
  // WatchdogResetEnable(WATCHDOG1_BASE);
  // WatchdogEnable(WATCHDOG1_BASE);

  // delay(100);
  // float temp[4] = {0.0};
  // buf[0] = '1';
  // memcpy(buf+1, temp, 16);
  // SerialAPI::send_bytes('0', buf, 17);
}

volatile long lb_hall_a_interrupts_raw = 0;
volatile long lb_hall_b_interrupts_raw = 0;
volatile long lb_hall_c_interrupts_raw = 0;

volatile long lf_hall_a_interrupts_raw = 0;
volatile long lf_hall_b_interrupts_raw = 0;
volatile long lf_hall_c_interrupts_raw = 0;

volatile long rb_hall_a_interrupts_raw = 0;
volatile long rb_hall_b_interrupts_raw = 0;
volatile long rb_hall_c_interrupts_raw = 0;

volatile long rf_hall_a_interrupts_raw = 0;
volatile long rf_hall_b_interrupts_raw = 0;
volatile long rf_hall_c_interrupts_raw = 0;

volatile int lb_direction = 1, rb_direction = 1, rf_direction = 1, lf_direction = 1;

float lb_regress_value, lf_regress_value, rb_regress_value, rf_regress_value;
float lb_hall_a_interrupts_per_second, rb_hall_a_interrupts_per_second,
    rf_hall_a_interrupts_per_second, lf_hall_a_interrupts_per_second;

char us_buf[20];
float speeds[4];

float lb_new_us, lf_new_us, rb_new_us, rf_new_us, lb_prev_us=1500, lf_prev_us=1500, rb_prev_us=1500, rf_prev_us=1500;
unsigned long integration_period_start, integration_period_end;
long time_elapsed;
float lb_target_speed, rb_target_speed, lf_target_speed, rf_target_speed;
float filtered_lb_target_speed, filtered_rb_target_speed, filtered_lf_target_speed, filtered_rf_target_speed;
int filtered_lb_us, filtered_lf_us, filtered_rb_us, filtered_rf_us;

//int loops = 0;
bool lb_leap_up=false, lb_leap_down=false, rb_leap_up=false, rb_leap_down=false,
     rf_leap_up=false, rf_leap_down=false, lf_leap_up=false, lf_leap_down=false;
  
volatile int lb_increment, rb_increment, lf_increment, rf_increment;

void loop() {
  // put your main code here, to run repeatedly:
  // lb_cur_speed = LBservo.readMicroseconds();
  // lf_cur_speed = LFservo.readMicroseconds();
  // rb_cur_speed = RBservo.readMicroseconds();
  // rf_cur_speed = RFservo.readMicroseconds();

  // integration_period_start = millis();
  // delay(100);
  // integration_period_end = millis();

  // time_elapsed = integration_period_end - integration_period_start;

  // lb_hall_a_interrupts_per_second = lb_hall_a_interrupts_raw * (1000.0f / (float)time_elapsed);
  // lb_regress_value = (lb_hall_a_interrupts_per_second - 24.995f) / 30.178f;

  // rb_hall_a_interrupts_per_second = rb_hall_a_interrupts_raw * (1000.0f / (float)time_elapsed);
  // rb_regress_value = (rb_hall_a_interrupts_per_second - 24.995f) / 30.178f;

  // lf_hall_a_interrupts_per_second = lf_hall_a_interrupts_raw * (1000.0f / (float)time_elapsed);
  // lf_regress_value = (lf_hall_a_interrupts_per_second - 24.995f) / 30.178f;

  // rf_hall_a_interrupts_per_second = rf_hall_a_interrupts_raw * (1000.0f / (float)time_elapsed);
  // rf_regress_value = (rf_hall_a_interrupts_per_second - 24.995f) / 30.178f;

  // lb_hall_a_interrupts_raw = 0;
  // rb_hall_a_interrupts_raw = 0;
  // lf_hall_a_interrupts_raw = 0;
  // rf_hall_a_interrupts_raw = 0;

  /**
   * Get new commands from main computer, and send the actual speed
   * 
   */
   if(SerialAPI::update()){

    // Feed watchdog as soon as serial communication is established
    // WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet() * 2);
    // WatchdogReloadSet(WATCHDOG1_BASE, SysCtlClockGet() * 5);

    memset(buffer, 0, SERIAL_RX_BUFFER_SIZE);
    int cur_pack_id = SerialAPI::read_data(buffer,sizeof(buffer));

    memcpy(speeds, buffer+1, 16);

    lb_target_speed = speeds[0];
    lf_target_speed = speeds[1] * -1; 
    rb_target_speed = speeds[2];
    rf_target_speed = speeds[3] * -1;
    //  delay(100);

    buf[0] = '1';

    memcpy(buf+1, speeds, 16);

    SerialAPI::send_bytes('0', buf, 17);

    // us_buf[0] = '1';
    // memcpy(us_buf+1, &lb_regress_value, 4);
    // memcpy(us_buf+5, &rb_regress_value, 4);
    // memcpy(us_buf+9, &lf_regress_value, 4);
    // memcpy(us_buf+13, &rf_regress_value, 4);
    // SerialAPI::send_bytes('0', us_buf, 17);

    // delay(100); 

  }
  lb_new_us = (1500.0f + 4.0f * lb_target_speed);
  lf_new_us = (1500.0f + 4.0f * lf_target_speed);
  rb_new_us = (1500.0f + 4.0f * rb_target_speed);
  rf_new_us = (1500.0f + 4.0f * rf_target_speed);

  // int motorsDone = 0;
  // int lb_dir = (lb_new_us - lb_cur_speed >= 0) ? 1 : -1;
  // int lf_dir = (lf_new_us - lf_cur_speed >= 0) ? 1 : -1;
  // int rb_dir = (rb_new_us - rb_cur_speed >= 0) ? 1 : -1;
  // int rf_dir = (rf_new_us - rf_cur_speed >= 0) ? 1 : -1;
  // for(int i = 0; i < 400; i+=5){
  //   if(abs(lb_new_us - lb_cur_speed) > i){
  //     LBservo.writeMicroseconds(lb_cur_speed+(i*lb_dir));
  //     motorsDone += 1;
  //   }
  //   if(abs(lf_new_us - lf_cur_speed) > i){
  //     LFservo.writeMicroseconds(lf_cur_speed+(i*lf_dir));
  //     motorsDone += 1;
  //   }
  //   if(abs(rb_new_us - rb_cur_speed) > i){
  //     RBservo.writeMicroseconds(rb_cur_speed+(i*rb_dir));
  //     motorsDone += 1;
  //   }
  //   if(abs(rf_new_us - rf_cur_speed) > i){
  //     RFservo.writeMicroseconds(rf_cur_speed+(i*rf_dir));
  //     motorsDone += 1;
  //   }
  //   if(motorsDone == 0) break;
  //   motorsDone = 0;
  // }


  LBservo.writeMicroseconds(lb_new_us);
  LFservo.writeMicroseconds(lf_new_us);
  RBservo.writeMicroseconds(rb_new_us);
  RFservo.writeMicroseconds(rf_new_us);
  // int temp_lb = (int) lb_cur_speed + lb_increment;
  // int temp_lf = (int) lf_cur_speed + lf_increment;
  // int temp_rb = (int) rb_cur_speed + rb_increment;
  // int temp_rf = (int) rf_cur_speed + rf_increment;
  // if(!(temp_lb == 1500 && lb_increment == 0)) LBservo.writeMicroseconds((int) lb_cur_speed + lb_increment);
  // if(!(temp_lf == 1500 && lf_increment == 0)) LFservo.writeMicroseconds((int) lf_cur_speed + lf_increment);
  // if(!(temp_rb == 1500 && rb_increment == 0)) RBservo.writeMicroseconds((int) rb_cur_speed + rb_increment);
  // if(!(temp_rf == 1500 && rf_increment == 0)) RFservo.writeMicroseconds((int) rf_cur_speed + rf_increment);

  // lb_prev_us = lb_new_us;
  // lf_prev_us = lf_new_us;
  // rb_prev_us = rb_new_us;
  // rf_prev_us = rf_new_us;

  //loops++;

}

void LFHallSensorA() {        
  lf_direction = (digitalRead(LF_MOTOR_HALL_A) == digitalRead(LF_MOTOR_HALL_B)) ? CW : CCW;   
  lf_hall_a_interrupts_raw = lf_hall_a_interrupts_raw + lf_direction; 
}

void LFHallSensorB() {
  lf_direction = (digitalRead(LF_MOTOR_HALL_B) == digitalRead(LF_MOTOR_HALL_C)) ? CW : CCW;
  lf_hall_a_interrupts_raw = lf_hall_a_interrupts_raw + lf_direction; 
}

void LFHallSensorC() {
  lf_direction = (digitalRead(LF_MOTOR_HALL_C) == digitalRead(LF_MOTOR_HALL_A)) ? CW : CCW;
  lf_hall_a_interrupts_raw = lf_hall_a_interrupts_raw + lf_direction; 
}

////////////////////////////////////////////////////////////////////////////////////////////

void LBHallSensorA() {        
  lb_direction = (digitalRead(LB_MOTOR_HALL_A) == digitalRead(LB_MOTOR_HALL_B)) ? CW : CCW;   
  lb_hall_a_interrupts_raw = lb_hall_a_interrupts_raw + lb_direction; 
}

void LBHallSensorB() {
  lb_direction = (digitalRead(LB_MOTOR_HALL_B) == digitalRead(LB_MOTOR_HALL_C)) ? CW : CCW;
  lb_hall_a_interrupts_raw = lb_hall_a_interrupts_raw + lb_direction; 
}

void LBHallSensorC() {
  lb_direction = (digitalRead(LB_MOTOR_HALL_C) == digitalRead(LB_MOTOR_HALL_A)) ? CW : CCW;
  lb_hall_a_interrupts_raw = lb_hall_a_interrupts_raw + lb_direction; 
}

////////////////////////////////////////////////////////////////////////////////////////////

void RBHallSensorA() {        
  lb_direction = (digitalRead(RB_MOTOR_HALL_A) == digitalRead(RB_MOTOR_HALL_B)) ? CW : CCW;   
  lb_hall_a_interrupts_raw = lb_hall_a_interrupts_raw + lb_direction; 
}

void RBHallSensorB() {
  rb_direction = (digitalRead(RB_MOTOR_HALL_B) == digitalRead(RB_MOTOR_HALL_C)) ? CW : CCW;
  rb_hall_a_interrupts_raw = rb_hall_a_interrupts_raw + rb_direction; 
}

void RBHallSensorC() {
  rb_direction = (digitalRead(RB_MOTOR_HALL_C) == digitalRead(RB_MOTOR_HALL_A)) ? CW : CCW;
  rb_hall_a_interrupts_raw = rb_hall_a_interrupts_raw + rb_direction; 
}

////////////////////////////////////////////////////////////////////////////////////////////

void RFHallSensorA() {        
  rf_direction = (digitalRead(RF_MOTOR_HALL_A) == digitalRead(RF_MOTOR_HALL_B)) ? CW : CCW;   
  rf_hall_a_interrupts_raw = rf_hall_a_interrupts_raw + rf_direction; 
}

void RFHallSensorB() {
  rf_direction = (digitalRead(RF_MOTOR_HALL_B) == digitalRead(RF_MOTOR_HALL_C)) ? CW : CCW;
  rf_hall_a_interrupts_raw = rf_hall_a_interrupts_raw + rf_direction; 
}

void RFHallSensorC() {
  rf_direction = (digitalRead(RF_MOTOR_HALL_C) == digitalRead(RF_MOTOR_HALL_A)) ? CW : CCW;
  rf_hall_a_interrupts_raw = rf_hall_a_interrupts_raw + rf_direction; 
}

////////////////////////////////////////////////////////////////////////////////////////////

void AccelInt(){
  TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

  //lb
  if(lb_new_us - lb_cur_speed > 10){
    lb_increment += 5;
  }
  else if(lb_new_us - lb_cur_speed < -10){
    lb_increment-= 5;
  }else{
    lb_increment = 0;
  }

  //lf
  if(lf_new_us - lf_cur_speed > 10){
    lf_increment += 5;
  }
  else if(lf_new_us - lf_cur_speed < -10){
    lf_increment -= 5;
  }else{
    lf_increment = 0;
  }

  //rb
  if(rb_new_us - rb_cur_speed > 10){
    rb_increment += 5;
  }
  else if(rb_new_us - rb_cur_speed < -10){
    rb_increment -= 5;
  }else{
    rb_increment = 0;
  }

  //rf
  if(rf_new_us - rf_cur_speed > 10){
    rf_increment += 5;
  }
  else if(rf_new_us - rf_cur_speed < -10){
    rf_increment -= 5;
  }else{
    rf_increment = 0;
  }

  
}

// When connection is lost, set acceleration target to 0 for a smooth stop.
// Resetting microcontroller isn't feasible because that will force the motors
// to stop with a jerk, with unknown results.
// void ConnectionLostISR(){
//   WatchdogIntClear(WATCHDOG0_BASE);
//   rf_new_us = 0;
//   lf_new_us = 0;
//   rb_new_us = 0;
//   lb_new_us = 0;
// }

////////////////////////////////////////////////////////////////////////////////////////////

void setPinAsOpenDrain(char port, int pin, int output){
  int amsel_register, pctl_register, dir_register, afsel_register, odr_register, den_register, data_register, pin_idx = pin;

  if(1 > pin || pin > 7){return;}
  if('A' > port || port > 'F'){return;}
  if(!(output == 0 || output == 1)){return;}

  // PD4, PD5, PB0 and PB1 are not 5V tolerant
  if(port == 'D' && pin == 4){return;}
  if(port == 'D' && pin == 5){return;}
  if(port == 'B' && pin == 0){return;}
  if(port == 'B' && pin == 1){return;}

  switch(port){
    case 'A':
      SYSCTL_RCGCGPIO_R |= 0x01;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0001) == 0){};               // ready?
      GPIO_PORTA_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTA_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTA_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTA_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTA_ODR_R |= (1 << pin_idx);                     // open drain enable
      // GPIO_PORTA_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

      GPIO_PORTA_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTA_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;

    case 'B':
      SYSCTL_RCGCGPIO_R |= 0x02;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0002) == 0){};               // ready?
      GPIO_PORTB_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTB_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTB_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTB_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTB_ODR_R |= (1 << pin_idx);                     // open drain enable
      // GPIO_PORTB_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

      GPIO_PORTB_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTB_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;
    
    case 'C':
      SYSCTL_RCGCGPIO_R |= 0x03;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0003) == 0){};               // ready?
      GPIO_PORTC_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTC_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTC_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTC_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTC_ODR_R |= (1 << pin_idx);                     // open drain enable
      // GPIO_PORTC_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

      GPIO_PORTC_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTC_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;
   
    case 'D':
      SYSCTL_RCGCGPIO_R |= 0x04;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0004) == 0){};               // ready?
      GPIO_PORTD_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTD_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTD_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTD_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTD_ODR_R |= (1 << pin_idx);                     // open drain enable
      // GPIO_PORTD_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

      GPIO_PORTD_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTD_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;
   
    case 'E':
      SYSCTL_RCGCGPIO_R |= 0x05;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0005) == 0){};               // ready?
      GPIO_PORTE_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTE_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTE_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTE_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTE_ODR_R |= (1 << pin_idx);                     // open drain enable
      // GPIO_PORTD_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

      GPIO_PORTE_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTE_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;
    
    case 'F':
      SYSCTL_RCGCGPIO_R |= 0x06;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0006) == 0){};               // ready?
      GPIO_PORTF_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTF_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTF_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTF_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTF_ODR_R |= (1 << pin_idx);                     // open drain enable
      // GPIO_PORTD_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

      GPIO_PORTF_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTF_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;

    default:
      return;        
  }
  return;
}