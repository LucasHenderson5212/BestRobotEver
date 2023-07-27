#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define displayOn false

#define TAPE_FOLLOW_STATE 1
#define COLLISION_STATE 2

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible

#define RIGHT_EYE PA1
#define LEFT_EYE PA4
#define RIGHT_EYE_OUT PA2
#define LEFT_EYE_OUT PA3

#define STEERING_SERVO PA_0
#define LEFT_MOTOR PB_7
#define LEFT_MOTOR_2 PB_6
#define RIGHT_MOTOR PB_9
#define RIGHT_MOTOR_2 PB_8

#define BOX_MOTOR PA9
#define MAG_SENSOR PB4
#define MAG_SENSOR_2 PB5
#define MAG_SENSOR_3 PB3
#define MAG_SENSOR_4 PA15

//#define MAG_OUT_SENSOR PB_9
//#define MAG_OUT_SENSOR_2 PB_10

#define LEFT_DOOR PA_8
#define RIGHT_DOOR PA_10

#define LEFT_DOOR_CLOSE 850
#define LEFT_DOOR_OPEN 350
#define RIGHT_DOOR_CLOSE 1200
#define RIGHT_DOOR_OPEN 1690

#define LEFT_COLLISION PB13
#define RIGHT_COLLISION PB14
#define MIDDLE_COLLISION PB15

#define STEERING_NEUTRAL 1300
#define STEERING_MIN_INPUT 700
#define STEERING_MAX_INPUT 1800

#define MAX_LEFTOVER (4095+MOTOR_SPEED-2*MOTOR_DEADZONE)/kdif

#define PID_MIN STEERING_MIN_INPUT-MAX_LEFTOVER
#define PID_MAX STEERING_MAX_INPUT+MAX_LEFTOVER

#define MOTOR_FREQUENCY 100
#define MOTOR_SPEED 1300
#define MOTOR_DEADZONE 475

#define THRESHOLD 220

#define STATE_1 1
#define STATE_2 2
#define STATE_3 4
#define STATE_4 7

//PID constants
#define kp 125
#define kd 160
#define kdout 20
#define ki 3
#define kdif 2.2

#define kWheelSpeedUp (4000-MOTOR_SPEED)/(MAX_LEFTOVER*kdif)*0//.3
//leftOver*kdif*0.x < 4000 - MOTOR_SPEED

#define dDuration 12

#define maxi 400

int steeringState = 0;
int i = 0;

int dCount = 0;
int lastD = 0;

int speedMultiplier = 1;
int collisionTime = 0;

int currentServoPos = STEERING_NEUTRAL;
int currentPIDNum = STEERING_NEUTRAL;

int lastTime;

bool doorsClosed = false;
int doorCloseTime = 0;
bool shouldStart = false;

void magnet_interrupt(void);
void magnet_out_interrupt(void);


void follow_tape();
void collision();
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
  
  //pinMode(RIGHT_DOOR, OUTPUT);
  //pinMode(LEFT_DOOR, OUTPUT);

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

  //Collision Detection
  pinMode(LEFT_COLLISION, INPUT);
  pinMode(RIGHT_COLLISION, INPUT);
  pinMode(MIDDLE_COLLISION, INPUT);

  //Opens doors
  pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_DOOR, 50, LEFT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  //delay(200);
  //digitalWrite(BOX_MOTOR, HIGH);

 
  //Set Servo to neutral position
  pwm_start(STEERING_SERVO, 50, currentServoPos, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  delay(100);

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

     // //Check for magnet
    int mag1 = digitalRead(MAG_SENSOR);
    int mag2 = digitalRead(MAG_SENSOR_2);
    int mag3 = digitalRead(MAG_SENSOR_3);
    int mag4 = digitalRead(MAG_SENSOR_4);
    if(!doorsClosed && (!mag1 || !mag2 || !mag3 || !mag4)){
      magnet_interrupt();
      // display_handler.clearDisplay();
      // display_handler.setTextSize(1);
      // display_handler.setTextColor(SSD1306_WHITE);
      // display_handler.setCursor(0,0);
      // //char msg[100];
      // char msg2[100];
      // //sprintf(msg, "Steering State: %d", steeringState);
      // sprintf(msg2, "%d %d %d %d", digitalRead(MAG_SENSOR), digitalRead(MAG_SENSOR_2), digitalRead(MAG_SENSOR_3), digitalRead(MAG_SENSOR_4));
      // //display_handler.println(msg);
      // display_handler.println(msg2);
      // display_handler.display();
    }
    //Only like this if no exit sensors
    // if(doorsClosed && (getCurrentMillis()-doorCloseTime)>2500){
    //   magnet_out_interrupt();
    // } else if (shouldStart){
    //   shouldStart = false;
    //   digitalWrite(BOX_MOTOR, HIGH);
    // }

    //Increase speedMultiplier towards 1 (in case it was decreased by collision)
    if (getCurrentMillis()-collisionTime > 500 && speedMultiplier < 1){
      speedMultiplier += 0.02;
    }

    //find what to do
    switch (get_state())
    {
    case TAPE_FOLLOW_STATE:
      follow_tape();
      break;
    case COLLISION_STATE:
      collision();
      break;
    default:
      break;
    }
  }
}

void follow_tape(){
    //Only for display stuff
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
    } else if (leftOutOn && !leftOn && !rightOn && !rightOutOn){//} && lastState > -STATE_3) {
      steeringState = STATE_3;
    } else if (leftOutOn && leftOn && !rightOn && !rightOutOn){//} && lastState > -STATE_3) {
      steeringState = STATE_2;
    } else if (!leftOutOn && leftOn && !rightOn && !rightOutOn){//} && lastState > -STATE_3){
      steeringState = STATE_1;
    } else if (!leftOutOn && leftOn && rightOn && !rightOutOn) {
      steeringState = 0;
    } else if (!leftOutOn && !leftOn && !rightOn && rightOutOn){//} && lastState < STATE_3){
      steeringState = -STATE_3;
    } else if (!leftOutOn && !leftOn && rightOn && rightOutOn){//} && lastState < STATE_3){
      steeringState = -STATE_2;
    } else if (!leftOutOn && !leftOn && rightOn && !rightOutOn){//} && lastState < STATE_3){
      steeringState = -STATE_1;
    }

    // Once robot is back on tape, set motors back to normal immediately (to prevent overshooting?)
    if(abs(steeringState) < STATE_4){
      if (currentPIDNum > STEERING_MAX_INPUT) {
        currentPIDNum = STEERING_MAX_INPUT;
      } else if (currentPIDNum < STEERING_MIN_INPUT) {
        currentPIDNum = STEERING_MIN_INPUT;
      }
    }
    
    //for testing reflectance sensors    
    if (displayOn){
      
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
      
    }
   
  
    // calculate PID change
    int p = (int)(kp*steeringState);
    int d = (int)(kd*(steeringState-lastState));
    //d only kicks in if approaching the tape
    if (abs(steeringState) > abs(lastState)){
      d = (int)(kdout*(steeringState-lastState));
    }
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
    if(abs(steeringState) < STATE_4){
      i = 0;
    }

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

    //corrects for non-linearity in servo
    if (g < 0 && g > -(STEERING_MAX_INPUT-STEERING_NEUTRAL)){
      g = g*(STEERING_NEUTRAL-STEERING_MIN_INPUT)/(STEERING_MAX_INPUT-STEERING_NEUTRAL);
    } else if (g <= -(STEERING_MAX_INPUT-STEERING_NEUTRAL)){
      g = g+(STEERING_MAX_INPUT-STEERING_NEUTRAL)-(STEERING_NEUTRAL-STEERING_MIN_INPUT);
    }
    
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

    // if(abs(steeringState) < STATE_4){
    //   leftOver = 0;
    // }

    //keep differential within maximum (shouldn't happen but safety) 
    if (kdif*leftOver-MOTOR_SPEED+2*MOTOR_DEADZONE > 4095) {
      leftOver = (4095+MOTOR_SPEED)/kdif;
    } else if (kdif*leftOver+MOTOR_SPEED-2*MOTOR_DEADZONE < -4095) {
      leftOver = -(4095+MOTOR_SPEED)/kdif;
    }
    
    //Set Motors
    if(getCurrentMillis() > 2000 && !displayOn){
      if(leftOver > 0){
        if(leftOver*kdif > MOTOR_SPEED-MOTOR_DEADZONE){
          pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, (int)((leftOver*kdif-MOTOR_SPEED+2*MOTOR_DEADZONE)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        } else {
          pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)((MOTOR_SPEED-leftOver*kdif)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        }
        pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)(MOTOR_SPEED+leftOver*kdif*kWheelSpeedUp*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      } else if (leftOver < 0){
        if(-leftOver*kdif > MOTOR_SPEED-MOTOR_DEADZONE){
          pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, (int)((-leftOver*kdif-MOTOR_SPEED+2*MOTOR_DEADZONE)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        } else {
          pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)((MOTOR_SPEED+leftOver*kdif)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        }
        pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)(MOTOR_SPEED-leftOver*kdif*kWheelSpeedUp*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      } else {
        pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)(MOTOR_SPEED*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)(MOTOR_SPEED*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      }
    }
}

void collision(){
  //This will be what happens during first collision, etc. 
  //After many collisions or something else, other methods must be implemented.
  //speedMultiplier = 0;
  //collisionTime = getCurrentMillis();
  follow_tape();
}

int get_state(){
  //This function will use data from the collision sensors to figure out if something other than tape following needs to happen
  //Most of the time will return TAPE_FOLLOW_STATE
  if (digitalRead(LEFT_COLLISION) || digitalRead(RIGHT_COLLISION) || digitalRead(MIDDLE_COLLISION)){
    return COLLISION_STATE;
  } else {
    return TAPE_FOLLOW_STATE;
  }
}

void magnet_interrupt(){
  doorCloseTime = getCurrentMillis();
  //Stop motor
  digitalWrite(BOX_MOTOR, LOW);
  //doors close
  pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_CLOSE, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_DOOR, 50, LEFT_DOOR_CLOSE, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  doorsClosed = true;

  // if(displayOn){
  //   display_handler.clearDisplay();
  //   display_handler.setTextSize(1);
  //   display_handler.setTextColor(SSD1306_WHITE);
  //   display_handler.setCursor(0,0);
  //   display_handler.println("Interrupt happened");
  //   display_handler.display();
  // }

}

void magnet_out_interrupt(){
  //doors open
  pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_DOOR, 50, LEFT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  shouldStart = true;
  doorsClosed = false;
}



