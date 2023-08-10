#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
//https://www.amazon.ca/dp/B07SSNJ6Y5/ref=sspa_dk_detail_0?pd_rd_i=B07SWW9NDR&pd_rd_w=7Bw2g&content-id=amzn1.sym.43f51e91-471e-46fd-9eb7-f35b3f7790d8&pf_rd_p=43f51e91-471e-46fd-9eb7-f35b3f7790d8&pf_rd_r=G9XBGQHSD56J52QQ9RB2&pd_rd_wg=ftXRi&pd_rd_r=f276fa57-2491-46d2-b009-a5b08aa60fa2&s=toys&sp_csd=d2lkZ2V0TmFtZT1zcF9kZXRhaWwy&th=1

#define GREEN_LIGHT PC13
#define displayOn false
#define HARD_CODE_RIGHT false
#define HARD_CODE_LEFT false

#define HARD_CODE_PIN PB11

#define TAPE_FOLLOW_STATE 1
#define COLLISION_STATE 2

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible
//chicken
// gabe was here

#define RIGHT_EYE_OUT PA5
#define RIGHT_EYE PA6
#define LEFT_EYE PA7
#define LEFT_EYE_OUT PB0

#define LEFT_SIDE_EYE PA4
#define RIGHT_SIDE_EYE PA3

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

#define MAG_OUT_SENSOR PA12
#define MAG_OUT_SENSOR_2 PA11

#define BOX_COUNTER PB1
#define MAX_BOXES 8

#define BOMB_DROP_SERVO PA_1
#define BOMB_DROP_CLOSED 1800
#define BOMB_DROP_OPEN 2600

#define LEFT_DOOR PA_8
#define RIGHT_DOOR PA_10

#define LEFT_DOOR_CLOSE 1575
#define LEFT_DOOR_OPEN 1000

#define RIGHT_DOOR_CLOSE 1150
#define RIGHT_DOOR_OPEN 1725

#define LEFT_COLLISION PB12
#define MIDDLE_COLLISION PB15
#define MIDDLE_SIDES_COLLISION PB14
#define RIGHT_COLLISION PB13

#define STEERING_NEUTRAL 1430
#define STEERING_MIN_INPUT 1240
#define STEERING_MAX_INPUT 1720

#define MAX_LEFTOVER (4095+MOTOR_SPEED)/kdif

#define PID_MIN STEERING_MIN_INPUT-MAX_LEFTOVER
#define PID_MAX STEERING_MAX_INPUT+MAX_LEFTOVER

#define MOTOR_FREQUENCY 100
#define MOTOR_SPEED 3500
#define MOTOR_SPEED_START 1800
#define START_SPEED_TIME 0//15000

#define THRESHOLD 450

#define STATE_1 1
#define STATE_2 2
#define STATE_3 6
#define STATE_4 15

//PID constants
#define kp 25
#define kd 9
#define kdout kd
#define ki 0.8
#define kdif 42

//Slow PID constants
#define kp_s 26
#define kd_s -3
#define kdout_s 3
#define ki_s 1
#define kdif_s 50

#define kWheelSpeedUp 0//(4000-MOTOR_SPEED)/(MAX_LEFTOVER*kdif)*0
//leftOver*kdif*0.x < 4000 - MOTOR_SPEED
#define kWheelSpeedUp_s (4095-MOTOR_SPEED_START)/(4095+MOTOR_SPEED_START)*0.3

//#define dDuration 10

#define maxi 75

int steeringState = 0;
int i = 0;
int lastStateChange = 0;

int dCount = 0;
double lastD = 0;

int boxCount = 0;
int lastBoxState = 0;

double speedMultiplier = 1;
int collisionTime = 0;
int lastCollisionState = 0;
int lastBoxReading = 0;

int currentServoPos = STEERING_NEUTRAL;
int currentPIDNum = STEERING_NEUTRAL;

int lastTime;
int offTapeCount = 0;
bool caughtTape = false;

bool doorsClosed = false;
int doorCloseTime = 0;
bool shouldStart = false;
int stopTime = 0;
bool shouldStop = false;

#define MAGNET_TO_BOX_SENSOR_DELAY 3000

//Delay Box Motor Start Until After Doors
int exitTime = 0;
#define BOX_MOTOR_START_DELAY 150

void magnet_interrupt(void);
void magnet_out_interrupt(void);

void rightTurn();
void leftTurn();
void rightStartLoop();
void leftStartLoop();

void follow_tape();
void collision();
void get_state(void);

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  pinMode(GREEN_LIGHT, OUTPUT);

  pinMode(STEERING_SERVO, OUTPUT);
  pinMode(BOX_MOTOR, OUTPUT);

  if(!displayOn){
    pinMode(RIGHT_MOTOR, OUTPUT);
    pinMode(RIGHT_MOTOR_2, OUTPUT);
    pinMode(LEFT_MOTOR, OUTPUT);
    pinMode(LEFT_MOTOR_2, OUTPUT);
  }

  pinMode(RIGHT_DOOR, OUTPUT);
  pinMode(LEFT_DOOR, OUTPUT);

  // //Tape Following
  pinMode(RIGHT_EYE, INPUT);
  pinMode(LEFT_EYE, INPUT);
  pinMode(RIGHT_EYE_OUT, INPUT);
  pinMode(LEFT_EYE_OUT, INPUT);
  pinMode(RIGHT_SIDE_EYE, INPUT);
  pinMode(LEFT_SIDE_EYE, INPUT);

  //Bomb Detection
  pinMode(MAG_SENSOR, INPUT);
  pinMode(MAG_SENSOR_2, INPUT);
  pinMode(MAG_SENSOR_3, INPUT);
  pinMode(MAG_SENSOR_4, INPUT);
  pinMode(MAG_OUT_SENSOR, INPUT);
  pinMode(MAG_OUT_SENSOR_2, INPUT);

  // //Collision Detection
  pinMode(LEFT_COLLISION, INPUT);
  pinMode(MIDDLE_COLLISION, INPUT);
  pinMode(MIDDLE_SIDES_COLLISION, INPUT);
  pinMode(RIGHT_COLLISION, INPUT);

  //Box Counter
  pinMode(BOX_COUNTER, INPUT_PULLUP);
  //Opens doors
  pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_DOOR, 50, LEFT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  // delay(300);
  // digitalWrite(BOX_MOTOR, HIGH);

 
  //Set Servo to neutral position
  pwm_start(STEERING_SERVO, 50, currentServoPos, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);


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

  // digitalWrite(GREEN_LIGHT, HIGH);
  // delay (1000);
  // digitalWrite(GREEN_LIGHT, LOW);
  // delay (1000);
  // digitalWrite(GREEN_LIGHT, HIGH);
  // delay (1000);
  // digitalWrite(GREEN_LIGHT, LOW);

  if (!displayOn && HARD_CODE_RIGHT/*digitalRead(HARD_CODE_PIN)*/){
    rightStartLoop();
  } else if (!displayOn && HARD_CODE_LEFT){
    leftStartLoop();
  }
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
    }
    int mago = digitalRead(MAG_OUT_SENSOR);
    int mago2 = digitalRead(MAG_OUT_SENSOR_2);
    if (doorsClosed && ((!mago || !mago2) || (((getCurrentMillis() - doorCloseTime) > MAGNET_TO_BOX_SENSOR_DELAY) && (lastBoxReading < doorCloseTime)))) {
            magnet_out_interrupt();
        }
    
    if (shouldStart && (getCurrentMillis()-exitTime) > BOX_MOTOR_START_DELAY){
      shouldStart = false;
      digitalWrite(BOX_MOTOR, HIGH);
    }

    if (shouldStop && (getCurrentMillis()-stopTime) > 700){
      magnet_interrupt();
    }

    if (lastTime > 5000 && lastTime < 5100){
      pwm_start(BOMB_DROP_SERVO, 50, BOMB_DROP_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    } else if (lastTime > 5100 && lastTime < 5200) {
      pwm_stop(BOMB_DROP_SERVO);
    } else if (lastTime > 10000 && lastTime < 10100){
      pwm_start(BOMB_DROP_SERVO, 50, BOMB_DROP_CLOSED, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    } else if (lastTime > 10100 && lastTime < 10200){
      pwm_stop(BOMB_DROP_SERVO);
    }

    int boxSensor = digitalRead(BOX_COUNTER);

    if (boxSensor){
      if (!lastBoxState && !doorsClosed && (getCurrentMillis()-lastBoxReading) > 100) {
          boxCount++;
          if (boxCount >= MAX_BOXES) {
              shouldStop = true;
              stopTime = getCurrentMillis();
          }
      }
      lastBoxReading = getCurrentMillis(); 
    }
    lastBoxState = boxSensor;

    //Increase speedMultiplier towards 1 (in case it was decreased by collision)
    if ((getCurrentMillis()-collisionTime) > 2000 && speedMultiplier < 1){
      speedMultiplier += 0.02;
    } else if (speedMultiplier > 1){
      speedMultiplier -= 0.02;
    }

    //find what to do re: collisions
    get_state();
    follow_tape();
  }
}

void follow_tape(){
    //Only for display stuff
    int r = analogRead(RIGHT_EYE);
    int ro = analogRead(RIGHT_EYE_OUT);
    int l = analogRead(LEFT_EYE);
    int lo = analogRead(LEFT_EYE_OUT);

    int re = analogRead(RIGHT_SIDE_EYE);
    int le = analogRead(LEFT_SIDE_EYE);

    bool rightOn = analogRead(RIGHT_EYE) > THRESHOLD;
    bool rightOutOn = analogRead(RIGHT_EYE_OUT) > THRESHOLD;
    bool leftOn = analogRead(LEFT_EYE) > THRESHOLD;
    bool leftOutOn = analogRead(LEFT_EYE_OUT) > THRESHOLD;

    bool leftSideOn = analogRead(LEFT_SIDE_EYE) > 750;// && analogRead(LEFT_SIDE_EYE) < 1000;
    bool rightSideOn = analogRead(RIGHT_SIDE_EYE) > 750;// && analogRead(RIGHT_SIDE_EYE) < 1000;

    int leftOver;
    int lastState = steeringState;


    // FOR 4 REFLECTENCE SENSORS, find the position state
    if (!rightOn && !leftOn && !rightOutOn && !leftOutOn && lastState > 0) {
      steeringState = STATE_4;
    } else if (!rightOn && !leftOn && !rightOutOn && !leftOutOn && lastState < 0) {
      steeringState = -STATE_4;
    } else if (leftOutOn && !leftOn && !rightOn && !rightOutOn){//} && lastState > -STATE_3) {
      steeringState = STATE_3;
    } else if (leftOutOn && leftOn && !rightOutOn){//} && lastState > -STATE_3) {
      steeringState = STATE_2;
    } else if (!leftOutOn && leftOn && !rightOn && !rightOutOn){//} && lastState > -STATE_3){
      steeringState = STATE_1;
    } else if (!leftOutOn && leftOn && rightOn && !rightOutOn) {
      steeringState = 0;
    } else if (!leftOutOn && !leftOn && !rightOn && rightOutOn){//} && lastState < STATE_3){
      steeringState = -STATE_3;
    } else if (!leftOutOn && rightOn && rightOutOn){//} && lastState < STATE_3){
      steeringState = -STATE_2;
    } else if (!leftOutOn && !leftOn && rightOn && !rightOutOn){//} && lastState < STATE_3){
      steeringState = -STATE_1;
    } else if (!leftOutOn && !leftOn && !rightOn && !rightOutOn){ //If all goes to shit, try to fix it with edge sensors
      offTapeCount++;
      if (leftSideOn && offTapeCount > 4 && lastStateChange != 0 && !caughtTape && lastState != STATE_4){ //10 loops is 0.2 seconds, may need to be changed
        steeringState = STATE_4;
        caughtTape = true;
        speedMultiplier = 0;
        collisionTime = getCurrentMillis()+1000;
      } else if (rightSideOn && offTapeCount > 4 && lastStateChange != 0 && !caughtTape && lastState != -STATE_4){
        steeringState = -STATE_4;
        caughtTape = true;
        speedMultiplier = 0;
        collisionTime = getCurrentMillis()+1000;
      }
    } else {
      caughtTape = false;
      offTapeCount = 0;
    }

    // if(abs(steeringState-lastState) > 2){
    //   steeringState = lastState;
    // }

    
    if (steeringState != lastState){
      digitalWrite(GREEN_LIGHT, LOW);
      lastStateChange = getCurrentMillis();
    }
    if((getCurrentMillis() - lastStateChange) > 10000 && abs(steeringState) == STATE_4){
      lastStateChange = getCurrentMillis();
      steeringState = -steeringState;
    }

    // Once robot is back on tape, set motors, i back to normal immediately (to prevent overshooting?)
    if(abs(steeringState) < STATE_4){
      //i = 0;
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
      char msg3[100];
      char stateTime[100];
      char currTime[100];
      // sprintf(msg, "Box Count: %d", boxCount);
      // display_handler.println(msg);
      sprintf(msg, "Steering State: %d", steeringState);
      sprintf(msg2, "%d %d %d %d", lo, l, r, ro);
      sprintf(msg3, "%d %d", le, re);
      
      sprintf(stateTime, "Last State Change Time: %d", lastStateChange);
      sprintf(currTime, "CurrentTime: %d", getCurrentMillis());
      
      display_handler.println(msg);
      display_handler.println(msg2);
      display_handler.println(msg3);
      // display_handler.println(stateTime);
      // display_handler.println(currTime);
      display_handler.display();
      
    }
   
  
    // calculate PID change
    int p;
    double d;
    if (getCurrentMillis() > START_SPEED_TIME){
      p = (int)(kp*steeringState);
      d = (int)(kd*(steeringState-lastState));
      i = (int)(ki*steeringState) + i;
    } else {
      p = (int)(kp_s*steeringState);
      d = (int)(kd_s*(steeringState-lastState));
      i = (int)(ki_s*steeringState) + i;
    }
    // //d only uses kd if approaching the tape, otherwise kdout
    // if (abs(steeringState) > abs(lastState)){
    //   d = kdout*(steeringState-lastState);
    // }
    //d decays
    if (d == 0){
      d = lastD*0.94;
    } else if (d != 0) {
      d += lastD*0.94;
    }
    lastD = d;

    //Keep i within max
    if (i > maxi){
      i = maxi;
    } else if (i < -maxi){
      i = -maxi;
    }

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

    //Calculate amount of differential steering to add on top of steering, and keep servo within max
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

    if(abs(steeringState) < STATE_4){
      leftOver = 0;
    }

    int motorSpeed;
    int kDiffLoop;
    int wheelSpeedUp;
    if (getCurrentMillis() < START_SPEED_TIME){
      motorSpeed = MOTOR_SPEED_START;
      kDiffLoop = kdif_s;
      wheelSpeedUp = kWheelSpeedUp_s;
    } else {
      motorSpeed = MOTOR_SPEED;
      kDiffLoop = kdif;
      wheelSpeedUp = kWheelSpeedUp;
    }


    //keep differential within maximum (shouldn't happen but safety) 
    if (kDiffLoop*leftOver-motorSpeed > 4095) {
      leftOver = (4095+motorSpeed)/kDiffLoop;
    } else if (kDiffLoop*leftOver+motorSpeed < -4095) {
      leftOver = -(4095+motorSpeed)/kDiffLoop;
    }
    
    //Set Motors
    if(getCurrentMillis() > 0 && !displayOn){
      if(leftOver > 0){
        if(leftOver*kDiffLoop > motorSpeed){
          pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, (int)((leftOver*kDiffLoop-motorSpeed)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        } else {
          pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)((motorSpeed-leftOver*kDiffLoop)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        }
        pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)((motorSpeed+leftOver*kdif*wheelSpeedUp)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      } else if (leftOver < 0){
        if(-leftOver*kDiffLoop > motorSpeed){
          pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, (int)((-leftOver*kDiffLoop-motorSpeed)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        } else {
          pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)((motorSpeed+leftOver*kDiffLoop)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        }
        pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)((motorSpeed-leftOver*kdif*wheelSpeedUp)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      } else {
        pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)(motorSpeed*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)(motorSpeed*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      }
    }
}

void get_state(){
  // //This function will use data from the collision sensors to figure out if something other than tape following needs to happen
  // //Most of the time will return TAPE_FOLLOW_STATE
  if (digitalRead(MIDDLE_COLLISION) || !digitalRead(MIDDLE_SIDES_COLLISION)){
    if (!lastCollisionState){
      lastCollisionState = 1;
      speedMultiplier = 0;
      digitalWrite(GREEN_LIGHT, HIGH);
      // digitalWrite(BOX_MOTOR, LOW);
      // delay(1000);
      // digitalWrite(BOX_MOTOR, HIGH);
      collisionTime = getCurrentMillis();
    } else {
      lastCollisionState = 1;
    }
  } else if (digitalRead(LEFT_COLLISION) || digitalRead(RIGHT_COLLISION)) {
    //speedMultiplier = 2;
    lastCollisionState = 0;
    // digitalWrite(GREEN_LIGHT, HIGH);
    //   digitalWrite(BOX_MOTOR, LOW);
    //   delay(5000);
    //   digitalWrite(BOX_MOTOR, HIGH);
  } else {
    lastCollisionState = 0;
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
  if (boxCount <= MAX_BOXES) {
    //doors open
    pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    pwm_start(LEFT_DOOR, 50, LEFT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    exitTime = getCurrentMillis();
    shouldStart = true;
    doorsClosed = false;
  }
}

void rightTurn(){
  pwm_start(STEERING_SERVO, 50, STEERING_MIN_INPUT, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(75);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 2170, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(1000);
  pwm_start(STEERING_SERVO, 50, STEERING_NEUTRAL, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT); 
  delay(1000000);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(425);
}

void leftTurn(){
  pwm_start(STEERING_SERVO, 50, STEERING_MAX_INPUT, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(75);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 1850, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(970);
  pwm_start(STEERING_SERVO, 50, STEERING_NEUTRAL-30, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  // pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  // pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT); 
  // delay(1000000);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(400);
  // pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  // pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);  

  //  delay(10000000);
}


void rightStartLoop(){
  bool inLoop = true;
  while (inLoop) {
    int boxSensor = digitalRead(BOX_COUNTER);
    if (boxSensor){
      if (!lastBoxState && !doorsClosed && (getCurrentMillis()-lastBoxReading) > 100) {
          boxCount++;
          if (boxCount >= MAX_BOXES) {
              shouldStop = true;
              stopTime = getCurrentMillis();
          }
      }
      lastBoxReading = getCurrentMillis(); 
    }
    lastBoxState = boxSensor;

    if (getCurrentMillis() > (75 + 1000 + 425)){
      inLoop = false;
    } else if (getCurrentMillis() > (75 + 1000)){
      pwm_start(STEERING_SERVO, 50, STEERING_NEUTRAL, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
      pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      delay(20);
    } else if (getCurrentMillis() > (75)){
      pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 2270, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      delay(20);
    } else {
      pwm_start(STEERING_SERVO, 50, STEERING_MIN_INPUT, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
      pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      delay(20);
    }
    // pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
    // pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT); 
    // delay(1000000);
  }
}

void leftStartLoop(){
bool inLoop = true;
  while (inLoop) {
    int boxSensor = digitalRead(BOX_COUNTER);
    if (boxSensor){
      if (!lastBoxState && !doorsClosed && (getCurrentMillis()-lastBoxReading) > 100) {
          boxCount++;
          if (boxCount >= MAX_BOXES) {
              shouldStop = true;
              stopTime = getCurrentMillis();
          }
      }
      lastBoxReading = getCurrentMillis(); 
    }
    lastBoxState = boxSensor;

    if (getCurrentMillis() > (75 + 790 + 480)){
      inLoop = false;
    } else if (getCurrentMillis() > (75 + 790 + 440)){
      pwm_start(STEERING_SERVO, 50, STEERING_NEUTRAL-40, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
;     pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      delay(20);
    }else if (getCurrentMillis() > (75 + 790)){
      pwm_start(STEERING_SERVO, 50, STEERING_NEUTRAL, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
;     pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      delay(20);
    } else if (getCurrentMillis() > (75)){
      pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 2250, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 4095, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      delay(20);
    } else {
      pwm_start(STEERING_SERVO, 50, STEERING_MAX_INPUT, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
      pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      delay(20);
    }
  }
}
