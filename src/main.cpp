#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible
#define RIGHT_EYE PA2
#define LEFT_EYE PA1
//#define DETECT_THRESHOLD PA0
#define STEERING_SERVO PA_0
#define LEFT_MOTOR PB_6
#define LEFT_MOTOR_2 PB_7
#define RIGHT_MOTOR PB_8
#define RIGHT_MOTOR_2 PB_9

#define STEERING_NEUTRAL 1400
#define STEERING_MIN_INPUT 1080
#define STEERING_MAX_INPUT 2000

#define PID_MIN STEERING_MIN_INPUT-2*MOTOR_SPEED
#define PID_MAX STEERING_MAX_INPUT+2*MOTOR_SPEED

#define MOTOR_SPEED 400
#define THRESHOLD 300

#define STATE_1 2
#define STATE_2 4

//PID constants
#define kp 10
#define kd 20
#define ki 5
#define kdif 1

#define maxi 50

int steeringState = 0;

int currentServoPos = STEERING_NEUTRAL;
int currentPIDNum = STEERING_NEUTRAL;

int lastTime;

//Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
   //initialize LED digital pin as an output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STEERING_SERVO, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR_2, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR_2, OUTPUT);

  //Set Servo to neutral position
  currentServoPos = STEERING_NEUTRAL;
  pwm_start(STEERING_SERVO, 50, currentServoPos, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  //Tape Following
  pinMode(RIGHT_EYE, INPUT);
  pinMode(LEFT_EYE, INPUT);
  //pinMode(DETECT_THRESHOLD, INPUT);

  //Motors
  pwm_start(RIGHT_MOTOR, 1000, MOTOR_SPEED, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, 1000, MOTOR_SPEED, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  // //for testing
  // display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // display_handler.display();
  // display_handler.clearDisplay();
  // display_handler.setTextSize(1);
  // display_handler.setTextColor(SSD1306_WHITE);
  // display_handler.setCursor(0,0);
  // display_handler.println("Hi Rudy!");
  // display_handler.display();
  
  delay(10000);

 lastTime = getCurrentMillis();
}

void loop() {

  if((getCurrentMillis() - lastTime) > 25){
    lastTime = getCurrentMillis();

    //int threshold = analogRead(DETECT_THRESHOLD);
    bool rightOn = analogRead(RIGHT_EYE) > THRESHOLD;
    bool leftOn = analogRead(LEFT_EYE) > THRESHOLD;

    int leftOver;
    int lastState = steeringState;

    if(rightOn && leftOn){
        steeringState = 0;
      } else if (rightOn && !leftOn){
        steeringState = -STATE_1;
      } else if (!rightOn && leftOn){
        steeringState = STATE_1;
      } else if(!rightOn && !leftOn && lastState == STATE_1){
        steeringState = STATE_2;
      } else if(!rightOn && !leftOn && lastState == -STATE_1){
        steeringState = -STATE_2;
      }
    
    //for testing
    
    // display_handler.clearDisplay();
    // display_handler.setTextSize(1);
    // display_handler.setTextColor(SSD1306_WHITE);
    // display_handler.setCursor(0,0);
    // if(steeringState == 0){
    //   display_handler.println("On tape");
    // }
    // else if(steeringState == STATE_1){
    //   display_handler.println("Slightly right");
    // }
    // else if(steeringState == -STATE_1){
    //   display_handler.println("Slightly left");
    // }
    // else if(steeringState == STATE_2){
    //   display_handler.println("Far right");
    // }
    // else if(steeringState == -STATE_2){
    //   display_handler.println("Far left");
    // }
    // display_handler.display();
  

    int p = kp*steeringState;
    int d = kd*(steeringState-lastState);
    int i = ki*steeringState + i;

    if (i > maxi){
      i = maxi;
    } else if (i < -maxi){
      i = -maxi;
    }

    int g = p + d + i;
    
    currentPIDNum = currentPIDNum + g;
    if(currentPIDNum > PID_MAX) {
      currentPIDNum = PID_MAX;
    } else if (currentPIDNum < PID_MIN){
      currentPIDNum = PID_MIN;
    }

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
    

    pwm_start(STEERING_SERVO, 50, currentServoPos, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);


    if(leftOver > 0){
      if(leftOver*kdif > MOTOR_SPEED){
        pwm_start(LEFT_MOTOR, 1000, 0, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR_2, 1000, leftOver*kdif-MOTOR_SPEED, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
      } else {
        pwm_start(LEFT_MOTOR_2, 1000, 0, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR, 1000, MOTOR_SPEED-leftOver*kdif, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
      }
      pwm_start(RIGHT_MOTOR, 1000, MOTOR_SPEED+400, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    } else if (leftOver < 0){
      if(-leftOver*kdif > MOTOR_SPEED){
        pwm_start(RIGHT_MOTOR, 1000, 0, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR_2, 1000, -leftOver*kdif, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
      } else {
        pwm_start(RIGHT_MOTOR_2, 1000, 0, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR, 1000, MOTOR_SPEED+leftOver*kdif, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
      }
      pwm_start(LEFT_MOTOR, 1000, MOTOR_SPEED+400, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    } else {
      pwm_start(RIGHT_MOTOR_2, 1000, 0, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
      pwm_start(LEFT_MOTOR_2, 1000, 0, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
      pwm_start(RIGHT_MOTOR, 1000, MOTOR_SPEED, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
      pwm_start(LEFT_MOTOR, 1000, MOTOR_SPEED, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    }
  }
  
}


