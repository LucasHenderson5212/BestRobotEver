#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define displayOn false

#define TAPE_FOLLOW_STATE 1
#define COLLISION_STATE 2

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible

#define RIGHT_EYE PA3
#define LEFT_EYE PA2
#define RIGHT_EYE_OUT PA4
#define LEFT_EYE_OUT PA1

#define STEERING_SERVO PA_0
#define LEFT_MOTOR PB_7
#define LEFT_MOTOR_2 PB_6
#define RIGHT_MOTOR PB_9
#define RIGHT_MOTOR_2 PB_8

#define BOX_MOTOR PA_9
#define MAG_SENSOR PB4
#define MAG_SENSOR_2 PB5
#define MAG_SENSOR_3 PB3
#define MAG_SENSOR_4 PA15

#define MAG_OUT_SENSOR PB_9
#define MAG_OUT_SENSOR_2 PB_10

#define LEFT_DOOR PA_0
#define RIGHT_DOOR PA_0

#define LEFT_DOOR_CLOSE 500
#define LEFT_DOOR_OPEN 200
#define RIGHT_DOOR_CLOSE 950
#define RIGHT_DOOR_OPEN 1250

#define STEERING_NEUTRAL 1500
#define STEERING_MIN_INPUT 1040
#define STEERING_MAX_INPUT 1960

#define MAX_LEFTOVER (4095+MOTOR_SPEED)/kdif

#define PID_MIN STEERING_MIN_INPUT-MAX_LEFTOVER
#define PID_MAX STEERING_MAX_INPUT+MAX_LEFTOVER

#define MOTOR_FREQUENCY 100
#define MOTOR_SPEED 1100

#define THRESHOLD 180

#define STATE_1 1
#define STATE_2 2
#define STATE_3 4
#define STATE_4 7

//PID constants
#define kp 110
#define kd 250
#define ki 1
#define kdif 4.5

#define kWheelSpeedUp (4000-MOTOR_SPEED)/(MAX_LEFTOVER*kdif)*0.1
//leftOver*kdif*0.x < 4000 - MOTOR_SPEED

#define dDuration 12

#define maxi 25

int steeringState = 0;
int i = 0;

int dCount = 0;
int lastD = 0;

int currentServoPos = STEERING_NEUTRAL;
int currentPIDNum = STEERING_NEUTRAL;

int lastTime;

void magnet_interrupt(void);
void magnet_out_interrupt(void);


void follow_tape(void);
int get_state(void);

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  pinMode(STEERING_SERVO, OUTPUT);
  pinMode(BOX_MOTOR, OUTPUT);

  if(!displayOn){
    pinMode(RIGHT_MOTOR, OUTPUT);
    pinMode(RIGHT_MOTOR_2, OUTPUT);
    pinMode(LEFT_MOTOR, OUTPUT);
    pinMode(LEFT_MOTOR_2, OUTPUT);
  }

  //Set Servo to neutral position
  pwm_start(STEERING_SERVO, 50, currentServoPos, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  // //Tape Following
  pinMode(RIGHT_EYE, INPUT);
  pinMode(LEFT_EYE, INPUT);
  pinMode(RIGHT_EYE_OUT, INPUT);
  pinMode(LEFT_EYE_OUT, INPUT);

  //Bomb Detection
  pinMode(MAG_SENSOR, INPUT);
  pinMode(MAG_SENSOR_2, INPUT);
  pinMode(MAG_SENSOR_3, INPUT);
  pinMode(MAG_SENSOR_4, INPUT);
  pinMode(MAG_OUT_SENSOR, INPUT);
  pinMode(MAG_OUT_SENSOR_2, INPUT);

  attachInterrupt(digitalPinToInterrupt(MAG_SENSOR), magnet_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(MAG_SENSOR_2), magnet_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(MAG_SENSOR_3), magnet_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(MAG_SENSOR_4), magnet_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(MAG_OUT_SENSOR), magnet_out_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(MAG_OUT_SENSOR_2), magnet_out_interrupt, FALLING);

  //Start Box Motor
  pwm_start(BOX_MOTOR, 100, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);

  //Opens doors
  pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_DOOR, 50, LEFT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);


  //for testing
  if(displayOn){
    display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display_handler.display();
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println("Hi Rudy!");
    display_handler.display();
  }

  delay(100);

  lastTime = getCurrentMillis();
}


void loop() {
  //50 loops per sec
  if((getCurrentMillis() - lastTime) > 20){
    lastTime = getCurrentMillis();

    switch (get_state())
    {
    case TAPE_FOLLOW_STATE:
      follow_tape();
      break;
    case COLLISION_STATE:
      break;
    default:
      break;
    }
  }
}

void follow_tape(){
    //Only for display stuff
    // int r = analogRead(RIGHT_EYE);
    // int ro = analogRead(RIGHT_EYE_OUT);
    // int l = analogRead(LEFT_EYE);
    // int lo = analogRead(LEFT_EYE_OUT);

    bool rightOn = analogRead(RIGHT_EYE) > THRESHOLD;
    bool rightOutOn = analogRead(RIGHT_EYE_OUT) > THRESHOLD;
    bool leftOn = analogRead(LEFT_EYE) > THRESHOLD;
    bool leftOutOn = analogRead(LEFT_EYE_OUT) > THRESHOLD;

    int leftOver;
    int lastState = steeringState;

    // FOR 4 REFLECTENCE SENSORS, find the position state
    if (!rightOn && !leftOn && !rightOutOn && !leftOutOn && lastState > 0) {
      steeringState = STATE_4;
    } else if (!rightOn && !leftOn && !rightOutOn && !leftOutOn && lastState < 0) {
      steeringState = -STATE_4;
    } else if (leftOutOn && !leftOn && !rightOn && !rightOutOn) {
      steeringState = STATE_3;
    } else if (leftOutOn && leftOn && !rightOn && !rightOutOn) {
      steeringState = STATE_2;
    } else if (!leftOutOn && leftOn && !rightOn && !rightOutOn){
      steeringState = STATE_1;
    } else if (!leftOutOn && leftOn && rightOn && !rightOutOn) {
      steeringState = 0;
    } else if (!leftOutOn && !leftOn && !rightOn && rightOutOn){
      steeringState = -STATE_3;
    } else if (!leftOutOn && !leftOn && rightOn && rightOutOn){
      steeringState = -STATE_2;
    } else if (!leftOutOn && !leftOn && rightOn && !rightOutOn){
      steeringState = -STATE_1;
    }

    // Once robot is back on tape, set motors back to normal immediately (to prevent overshooting)
    if(steeringState != -STATE_4 && steeringState != STATE_4){
      if (currentPIDNum > STEERING_MAX_INPUT) {
        currentPIDNum = STEERING_MAX_INPUT;
      } else if (currentPIDNum < STEERING_MIN_INPUT) {
        currentPIDNum = STEERING_MIN_INPUT;
      }
    }
    
    //for testing reflectance sensors    
    if (displayOn){
      int i = digitalRead(MAG_SENSOR);

      display_handler.clearDisplay();
      display_handler.setTextSize(1);
      display_handler.setTextColor(SSD1306_WHITE);
      display_handler.setCursor(0,0);
      char msg[100];
      sprintf(msg, "MAG 1: %d", i);
      display_handler.println(msg);
      display_handler.display();

      /*
      display_handler.clearDisplay();
      display_handler.setTextSize(1);
      display_handler.setTextColor(SSD1306_WHITE);
      display_handler.setCursor(0,0);
      char msg[100];
      char msg2[100];
      sprintf(msg, "Steering State: %d", steeringState);
      sprintf(msg2, "%d %d %d %d", lo, l, r, ro);
      display_handler.println(msg);
      display_handler.println(msg2);
      display_handler.display();
      */
    }
   
  
    // calculate PID change
    int p = (int)(kp*steeringState);
    int d = (int)(kd*(steeringState-lastState));
    //d decays after 12 loops
    if (d == 0 && dCount > 0){
      d = lastD;
      dCount--;
    } else if (d == 0){
      d = lastD/2;
    } else if (d != 0) {
      dCount = dDuration;
      d += lastD/2;
    }
    lastD = d;
    i = (int)(ki*steeringState) + i;

    //Keep i within max
    if (i > maxi){
      i = maxi;
    } else if (i < -maxi){
      i = -maxi;
    }

    // if(abs(lastState) < abs(steeringState)){
    //   //immediately reduce i to 0
    //   i = 0;
    // }

    int g = p + d + i;
    
    //Keep numbers within min/max
    currentPIDNum = STEERING_NEUTRAL + g;
    if(currentPIDNum > PID_MAX) {
      currentPIDNum = PID_MAX;
    } else if (currentPIDNum < PID_MIN){
      currentPIDNum = PID_MIN;
    }

    //Calculate amount of differential steering to add on top of steering, and set steering
    if (currentPIDNum > STEERING_MAX_INPUT) {
      leftOver = currentPIDNum - STEERING_MAX_INPUT;
      currentServoPos = STEERING_MAX_INPUT;
    } else if (currentPIDNum < STEERING_MIN_INPUT) {
      leftOver = currentPIDNum - STEERING_MIN_INPUT;
      currentServoPos = STEERING_MIN_INPUT;
    } else {
      leftOver = 0;
      currentServoPos = currentPIDNum;
    }
    
    //Set new servo position
    pwm_start(STEERING_SERVO, 50, currentServoPos, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

    //keep differential within maximum 
    if (kdif*leftOver-MOTOR_SPEED > 4095) {
      leftOver = (4095+MOTOR_SPEED)/kdif;
    } else if (kdif*leftOver+MOTOR_SPEED < -4095) {
      leftOver = -(4095+MOTOR_SPEED)/kdif;
    }
    
    //Set Motors
    if(getCurrentMillis() > 2000 && !displayOn){
      if(leftOver > 0){
        if(leftOver*kdif > MOTOR_SPEED){
          pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, (int)(leftOver*kdif-MOTOR_SPEED), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        } else {
          pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)(MOTOR_SPEED-leftOver*kdif), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        }
        pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, MOTOR_SPEED+leftOver*kdif*kWheelSpeedUp, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      } else if (leftOver < 0){
        if(-leftOver*kdif > MOTOR_SPEED){
          pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, (int)(-leftOver*kdif-MOTOR_SPEED), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        } else {
          pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)(MOTOR_SPEED+leftOver*kdif), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        }
        pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, MOTOR_SPEED-leftOver*kdif*kWheelSpeedUp, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      } else {
        pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, MOTOR_SPEED, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, MOTOR_SPEED, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      }
    }
}

int get_state(){
  //This function will use data from the collision sensors to figure out if something other than tape following needs to happen
  //Most of the time will return TAPE_FOLLOW_STATE
  return TAPE_FOLLOW_STATE;
}

void magnet_interrupt(){
  //Stop motor
  pwm_start(BOX_MOTOR, 0, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  //doors close
  pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_CLOSE, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_DOOR, 50, LEFT_DOOR_CLOSE, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

void magnet_out_interrupt(){
  //start motor
  pwm_start(BOX_MOTOR, 100, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  //doors open
  pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_DOOR, 50, LEFT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}



