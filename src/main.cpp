#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

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

#define BOX_MOTOR PA8
#define MAG_SENSOR PB4
#define MAG_SENSOR_2 PB5
#define MAG_SENSOR_3 PB3
#define MAG_SENSOR_4 PA15

#define MAG_OUT_SENSOR PB_9
#define MAG_OUT_SENSOR_2 PB_10

#define LEFT_DOOR PA_0
#define RIGHT_DOOR PA_0

#define LEFT_DOOR_CLOSE 0
#define LEFT_DOOR_OPEN 0
#define RIGHT_DOOR_CLOSE 0
#define RIGHT_DOOR_OPEN 0

#define STEERING_NEUTRAL 1380
#define STEERING_MIN_INPUT 1000
#define STEERING_MAX_INPUT 1800

#define PID_MIN STEERING_MIN_INPUT//-1000//-1*MOTOR_SPEED
#define PID_MAX STEERING_MAX_INPUT//+1000//+1*MOTOR_SPEED

#define MOTOR_FREQUENCY 100
#define MOTOR_SPEED 1000

#define THRESHOLD 180

#define STATE_1 1
#define STATE_2 2
#define STATE_3 4
#define STATE_4 7

#define displayOn true

//PID constants
#define kp 11
#define kd 145
#define kdout 65
#define ki 1 //start in the order of ~1 in testing
#define kdif 3.5 //0.05

#define maxi 36

int steeringState = 0;
int i = 0;

int currentServoPos = STEERING_NEUTRAL;
int currentPIDNum = STEERING_NEUTRAL;

int lastTime;

void magnet_interrupt(void);
void magnet_out_interrupt(void);
void boxes();

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
  currentServoPos = STEERING_NEUTRAL;
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
  // pinMode(MAG_OUT_SENSOR, INPUT);
  // pinMode(MAG_OUT_SENSOR_2, INPUT);

  attachInterrupt(digitalPinToInterrupt(MAG_SENSOR), magnet_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(MAG_SENSOR_2), magnet_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(MAG_SENSOR_3), magnet_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(MAG_SENSOR_4), magnet_interrupt, FALLING);
  // attachInterrupt(MAG_OUT_SENSOR, magnet_out_interrupt, FALLING);
  // attachInterrupt(MAG_OUT_SENSOR_2, magnet_out_interrupt, FALLING);

  //Start Box Motor
  digitalWrite(BOX_MOTOR, HIGH);

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

  digitalWrite(PA8, HIGH);

  delay(10000);

  lastTime = getCurrentMillis();
}

void magnet_interrupt(){
  // digitalWrite(BOX_MOTOR, LOW);

  display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println("THIS HAPPENED!");
    display_handler.display();

    // delay(100000);

  //doors close
  /*
  pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_CLOSE, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_DOOR, 50, LEFT_DOOR_CLOSE, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  */
}

void magnet_out_interrupt(){
  //digitalWrite(BOX_MOTOR, HIGH);
  //doors open
  /*
  pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_DOOR, 50, LEFT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  */
}


void loop() {
  digitalWrite(PA8, LOW);
  if((getCurrentMillis() - lastTime) > 20){
    
    lastTime = getCurrentMillis();

    int r = analogRead(RIGHT_EYE);
    int ro = analogRead(RIGHT_EYE_OUT);
    int l = analogRead(LEFT_EYE);
    int lo = analogRead(LEFT_EYE_OUT);

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
    int d;
    i = (int)(ki*steeringState) + i;
    if(abs(lastState) > abs(steeringState)){
      d = (int)(kd*(steeringState-lastState));
    } else {
      //immediately reduce i to 0
      i = 0;
      d = (int)(kdout*(steeringState-lastState));
    }

    if (i > maxi){
      i = maxi;
    } else if (i < -maxi){
      i = -maxi;
    }

    int g = p + d + i;
    
    //Keep numbers within min/max
    currentPIDNum = currentPIDNum + g;
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
    if (kdif*leftOver > 4000) {
      leftOver = 4000/kdif;
    } else if (kdif*leftOver < -4000) {
      leftOver = -4000/kdif;
    }

//
    if(getCurrentMillis() > 2000 && !displayOn){
      if(leftOver > 0){
        if(leftOver*kdif > MOTOR_SPEED){
          pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, (int)(leftOver*kdif-MOTOR_SPEED), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        } else {
          pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)(MOTOR_SPEED-leftOver*kdif), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        }
        pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, MOTOR_SPEED+leftOver*kdif*0.3, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      } else if (leftOver < 0){
        if(-leftOver*kdif > MOTOR_SPEED){
          pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, (int)(-leftOver*kdif-MOTOR_SPEED), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        } else {
          pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)(MOTOR_SPEED+leftOver*kdif), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        }
        pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, MOTOR_SPEED-leftOver*kdif*0.3, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      } else {
        pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, MOTOR_SPEED, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, MOTOR_SPEED, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      }
    }
    //Magnet Stuff?
  }

  
}


