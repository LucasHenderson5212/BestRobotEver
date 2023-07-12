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

#define STEERING_NEUTRAL 2000
#define STEERING_MIN_INPUT 1700
#define STEERING_MAX_INPUT 2600

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
   //initialize LED digital pin as an output
  pinMode(LED_BUILTIN, OUTPUT);

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

//If the last state was centered
  if (lastState == 0){
    //Turn right
    if (rightOn && !leftOn){
      steeringState = -1;
    }
    //Turn Left
    if (!rightOn && leftOn){
      steeringState = 1;
      }
  //If the last state was slightly to right
  } else if (lastState == 1){
    if(rightOn && leftOn){
      steeringState = 0;
    } else if(!rightOn && !leftOn){
      steeringState = 5;
    }
  //If the last state was slightly to left
  } else if (lastState == -1){
    if(rightOn && leftOn){
      steeringState = 0;
    } else if(!rightOn && !leftOn){
      steeringState = -5;
    }
    //If the last state was far right
  } else if (lastState == 5){
    if(!rightOn && leftOn){
      steeringState = 1;
    }
  //If the last state was far right
  } else if (lastState == -5){
    if(rightOn && !leftOn){
      steeringState = -1;
    }
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
    currentServoPos = currentPIDNum + g;
  }
  currentPIDNum = currentPIDNum + g;


  pwm_start(PA_7, 50, currentServoPos, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  //Motor speed adjustment based on leftOver
  if(leftOver > 0){
    //pwm_start(PA_right_MOTOR, 50?, motorSpeed-leftOver*kdiff);
  } else if (leftOver < 0){
    //pwm_start(PA_LEFT_MOTOR, 50?, motorSpeed+leftOver*kdiff);
  }
}


