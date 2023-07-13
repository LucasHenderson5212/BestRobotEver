#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible
#define LED_BUILTIN PC13
#define RIGHT_EYE PA2
#define LEFT_EYE PA1
#define DETECT_THRESHOLD PA0
#define STEERING_SERVO PA_7
#define RIGHT_MOTOR PA_8
#define LEFT_MOTOR PA_9

#define STEERING_NEUTRAL 1400
#define STEERING_MIN_INPUT 1080
#define STEERING_MAX_INPUT 2000

#define MOTOR_SPEED 450

//PID constants Levi Rocks
#define kp 50
#define kd 50
#define ki 10
#define kdif 1

#define maxi 200

int steeringState = 0;

int currentServoPos = STEERING_NEUTRAL;
int currentPIDNum = STEERING_NEUTRAL;

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
 display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 display_handler.display();
 
display_handler.clearDisplay();
display_handler.setCursor(0,0);
display_handler.println("on tape");

//Set Servo to neutral position
  pwm_start(STEERING_SERVO, 50, currentServoPos, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  //Tape Following
  pinMode(RIGHT_EYE, INPUT);
  pinMode(LEFT_EYE, INPUT);
  pinMode(DETECT_THRESHOLD, INPUT);
}

void loop() {
  //Digital System
  int threshold = analogRead(DETECT_THRESHOLD);
  bool rightOn = analogRead(RIGHT_EYE) > threshold;
  bool leftOn = analogRead(LEFT_EYE) > threshold;

  int lastState = steeringState;
  int leftOver;

  display_handler.clearDisplay();
  display_handler.setCursor(0,0);

  if(rightOn && leftOn){
    steeringState = 0;
    display_handler.println("on tape");
  } else if (rightOn && !leftOn){
    steeringState = -1;
    display_handler.println("little right");
  } else if (!rightOn && leftOn){
    steeringState = 1;
    display_handler.println("far right");
  } else if(!rightOn && !leftOn){
    steeringState = 5;
    display_handler.println("little left");
  } else if(!rightOn && !leftOn){
    steeringState = -5;
    display_handler.println("far left");
  }
  

  int p = kp*steeringState;
  int d = kd*(steeringState-lastState);
  int i = ki*steeringState + i;

  if (i > maxi){
    i = maxi;
  } else if (i < -maxi){
    i = -maxi;
  }

  int g = p + d + i;
  if (currentPIDNum + g > STEERING_MAX_INPUT) {
    leftOver = currentPIDNum + g - STEERING_MAX_INPUT;
    currentServoPos = STEERING_MAX_INPUT;
  } else if (currentServoPos + g < STEERING_MIN_INPUT) {
    leftOver = currentServoPos + g - STEERING_MIN_INPUT;
    currentServoPos = STEERING_MIN_INPUT;
  } else {
    leftOver = 0;
    currentServoPos = currentPIDNum + g;
  }
  currentPIDNum = currentPIDNum + g;

  pwm_start(STEERING_SERVO, 50, currentServoPos, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  //Motor speed adjustment based on leftOver
  /*
  if(leftOver > 0){
    pwm_start(RIGHT_MOTOR, 1000, MOTOR_SPEED, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    if(leftOver*kdif > MOTOR_SPEED){
      pwm_start(LEFT_MOTOR, 1000, 0, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    } else {
      pwm_start(LEFT_MOTOR, 1000, MOTOR_SPEED-leftOver*kdif, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    }
  } else if (leftOver < 0){
    if(-leftOver*kdif > MOTOR_SPEED){
      pwm_start(RIGHT_MOTOR, 1000, 0, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    } else {
      pwm_start(RIGHT_MOTOR, 1000, MOTOR_SPEED+leftOver*kdif, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    }
    pwm_start(LEFT_MOTOR, 1000, MOTOR_SPEED, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  } else {
    pwm_start(RIGHT_MOTOR, 1000, MOTOR_SPEED, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    pwm_start(LEFT_MOTOR, 1000, MOTOR_SPEED, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  }
  */
}


