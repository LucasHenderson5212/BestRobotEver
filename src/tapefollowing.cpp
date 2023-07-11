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

#define STEERING_NEUTRAL 1500
#define STEERING_MIN_INPUT 1000
#define STEERING_Max_INPUT 2000

//PID constants
#define kp 1
#define kd 1
#define ki 0

#define maxi 10

int steeringState = 0;

int currentServoPos;

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
   //initialize LED digital pin as an output
  pinMode(LED_BUILTIN, OUTPUT);

//Set Servo to neutral position
  currentServoPos = STEERING_NEUTRAL;
  pwm_start(STEERING_SERVO, 50, currentServoPos, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  //Tape Following
  pinMode(RIGHT_EYE, INPUT);
  pinMode(LEFT_EYE, INPUT);
  pinMode(DETECT_THRESHOLD, INPUT);
}

void loop() {
  int threshold = analogRead(DETECT_THRESHOLD);
  bool rightOn = analogRead(RIGHT_EYE) > threshold;
  bool leftOn = analogRead(LEFT_EYE) > threshold;
  int error = 0;

  int lastState = steeringState;

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
  int i = ki*error + i;
  if (i > maxi){
    i = maxi;
  } else if (i < -maxi){
    i = -maxi;
  }
  int g = p + d + i;
  pwm_start(PA_7, 50, STEERING_NEUTRAL+g, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}


