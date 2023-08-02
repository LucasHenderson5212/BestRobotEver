#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
//https://www.amazon.ca/dp/B07SSNJ6Y5/ref=sspa_dk_detail_0?pd_rd_i=B07SWW9NDR&pd_rd_w=7Bw2g&content-id=amzn1.sym.43f51e91-471e-46fd-9eb7-f35b3f7790d8&pf_rd_p=43f51e91-471e-46fd-9eb7-f35b3f7790d8&pf_rd_r=G9XBGQHSD56J52QQ9RB2&pd_rd_wg=ftXRi&pd_rd_r=f276fa57-2491-46d2-b009-a5b08aa60fa2&s=toys&sp_csd=d2lkZ2V0TmFtZT1zcF9kZXRhaWwy&th=1
#define displayOn false
#define HARD_CODE_RIGHT true
#define HARD_CODE_LEFT false

#define HARD_CODE_PIN PB11

#define TAPE_FOLLOW_STATE 1
#define COLLISION_STATE 2

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible

#define RIGHT_EYE PA1
#define LEFT_EYE PA4
#define RIGHT_EYE_OUT PA2
#define LEFT_EYE_OUT PA3

#define LEFT_SIDE_EYE PA6
#define RIGHT_SIDE_EYE PA5

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
#define MAX_BOXES 5

#define LEFT_DOOR PA_8
#define RIGHT_DOOR PA_10

#define LEFT_DOOR_CLOSE 1575
#define LEFT_DOOR_OPEN 1000
#define RIGHT_DOOR_CLOSE 1150
#define RIGHT_DOOR_OPEN 1725

#define LEFT_COLLISION PB12
#define MIDDLE_LEFT_COLLISION PB13
#define MIDDLE_RIGHT_COLLISION PB14
#define RIGHT_COLLISION PB15

#define STEERING_NEUTRAL 1450
#define STEERING_MIN_INPUT 1100
#define STEERING_MAX_INPUT 2000

#define MAX_LEFTOVER (4095+MOTOR_SPEED)/kdif

#define PID_MIN STEERING_MIN_INPUT-MAX_LEFTOVER
#define PID_MAX STEERING_MAX_INPUT+MAX_LEFTOVER

#define MOTOR_FREQUENCY 100
#define MOTOR_SPEED 1200

#define THRESHOLD 350

#define STATE_1 1
#define STATE_2 2
#define STATE_3 4
#define STATE_4 7

//PID constants
#define kp 103
#define kd 163
#define kdout 20
#define ki 1
#define kdif 6

#define kWheelSpeedUp (4000-MOTOR_SPEED)/(MAX_LEFTOVER*kdif)*0.2
//leftOver*kdif*0.x < 4000 - MOTOR_SPEED

#define dDuration 12

#define maxi 50

int steeringState = 0;
int i = 0;

int dCount = 0;
int lastD = 0;

int boxCount = 0;
int lastBoxState = 0;

double speedMultiplier = 1;
int collisionTime = 0;
int lastCollisionState = 0;

int currentServoPos = STEERING_NEUTRAL;
int currentPIDNum = STEERING_NEUTRAL;

int lastTime;
int offTapeCount = 0;

bool doorsClosed = false;
int doorCloseTime = 0;
bool shouldStart = false;
int stopTime = 0;
bool shouldStop = false;

void magnet_interrupt(void);
void magnet_out_interrupt(void);

void rightTurn();
void leftTurn();

void follow_tape();
void collision();
void get_state(void);

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
  pinMode(MIDDLE_LEFT_COLLISION, INPUT);
  pinMode(MIDDLE_RIGHT_COLLISION, INPUT);
  pinMode(RIGHT_COLLISION, INPUT);

  //Box Counter
  pinMode(BOX_COUNTER, INPUT_PULLUP);
  //Opens doors
  pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_DOOR, 50, LEFT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

  //delay(200);
  digitalWrite(BOX_MOTOR, HIGH);

 
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

  if (!displayOn && HARD_CODE_RIGHT/*digitalRead(HARD_CODE_PIN)*/){
    rightTurn();
  } else if (!displayOn && HARD_CODE_LEFT){
    leftTurn();
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
    if(doorsClosed && (!mago || !mago2)){
      magnet_out_interrupt();
    }
    
    if (shouldStart){
      shouldStart = false;
      digitalWrite(BOX_MOTOR, HIGH);
    }

    if (shouldStop && (getCurrentMillis()-stopTime) > 700){
      magnet_interrupt();
    }

    int boxSensor = digitalRead(BOX_COUNTER);

    if(!boxSensor && lastBoxState && !doorsClosed){
      boxCount++;
      if(boxCount >= MAX_BOXES){
        shouldStop = true;
        stopTime = getCurrentMillis();
      }
    }
    lastBoxState = boxSensor;



    //Increase speedMultiplier towards 1 (in case it was decreased by collision)
    if ((getCurrentMillis()-collisionTime) > 1000 && speedMultiplier < 1){
      speedMultiplier += 0.02;
    } else if (speedMultiplier > 1){
      speedMultiplier -= 0.02;
    }

    //find what to do re: collisions
    get_state();
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

    bool leftSideOn = analogRead(LEFT_SIDE_EYE) > 650;// && analogRead(LEFT_SIDE_EYE) < 1000;
    bool rightSideOn = analogRead(RIGHT_SIDE_EYE) > 650;// && analogRead(RIGHT_SIDE_EYE) < 1000;

    int lastStateChange = 0;
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
    if (!leftOutOn && !leftOn && !rightOn && !rightOutOn){ //If all goes to shit, try to fix it with edge sensors
      offTapeCount++;
      if (leftSideOn && offTapeCount > 5){ //10 loops is 0.2 seconds, may need to be changed
        steeringState = STATE_4;
        if (steeringState != lastState){
          //speedMultiplier = 0;
          collisionTime = getCurrentMillis();
        }
      } else if (rightSideOn && offTapeCount > 5){
        steeringState = -STATE_4;
        if (steeringState != lastState){
          //speedMultiplier = 0;
          collisionTime = getCurrentMillis();
        }
      }
    } else {
      offTapeCount = 0;
    }

    
    if (steeringState != lastState){
      lastStateChange = getCurrentMillis();
    }
    if((getCurrentMillis() - lastStateChange) > 5000){
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
      // sprintf(msg, "Box Count: %d", boxCount);
      // display_handler.println(msg);
      sprintf(msg, "Steering State: %d", steeringState);
      sprintf(msg2, "%d %d %d %d", lo, l, r, ro);
      sprintf(msg3, "%d %d", le, re);
      display_handler.println(msg);
      display_handler.println(msg2);
      display_handler.println(msg3);
      display_handler.display();
      
    }
   
  
    // calculate PID change
    int p = (int)(kp*steeringState);
    int d = (int)(kd*(steeringState-lastState));
    //d only uses kd if approaching the tape, otherwise kdout
    if (abs(steeringState) > abs(lastState)){
      d = (int)(kdout*(steeringState-lastState));
    }
    //d decays after dDuration (12) loops
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

    //keep differential within maximum (shouldn't happen but safety) 
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
          pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, (int)((leftOver*kdif-MOTOR_SPEED)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        } else {
          pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)((MOTOR_SPEED-leftOver*kdif)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        }
        pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)((MOTOR_SPEED+leftOver*kdif*kWheelSpeedUp)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      } else if (leftOver < 0){
        if(-leftOver*kdif > MOTOR_SPEED){
          pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, (int)((-leftOver*kdif-MOTOR_SPEED)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        } else {
          pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)((MOTOR_SPEED+leftOver*kdif)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        }
        pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)((MOTOR_SPEED-leftOver*kdif*kWheelSpeedUp)*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      } else {
        pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, (int)(MOTOR_SPEED*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, (int)(MOTOR_SPEED*speedMultiplier), TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
      }
    }
}

void get_state(){
  //This function will use data from the collision sensors to figure out if something other than tape following needs to happen
  //Most of the time will return TAPE_FOLLOW_STATE
  if (digitalRead(MIDDLE_LEFT_COLLISION || digitalRead(MIDDLE_RIGHT_COLLISION))){
    if (!lastCollisionState){
      lastCollisionState = 1;
      //speedMultiplier = 0;
      digitalWrite(BOX_MOTOR, LOW);
      delay(1000);
      digitalWrite(BOX_MOTOR, HIGH);
      collisionTime = getCurrentMillis();
    } else {
      lastCollisionState = 1;
    }
  } else if (digitalRead(LEFT_COLLISION) || digitalRead(RIGHT_COLLISION)) {
    speedMultiplier = 2;
    lastCollisionState = 0;
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
}

void magnet_out_interrupt(){
  if (boxCount <= MAX_BOXES) {
    //doors open
    pwm_start(RIGHT_DOOR, 50, RIGHT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    pwm_start(LEFT_DOOR, 50, LEFT_DOOR_OPEN, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
    shouldStart = true;
    doorsClosed = false;
  }
}

void rightTurn(){
  pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(50);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 2200, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 2200, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(300);
  pwm_start(STEERING_SERVO, 50, STEERING_MIN_INPUT, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 2000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 500, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(500);
  pwm_start(STEERING_SERVO, 50, STEERING_NEUTRAL+50, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 2000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 2000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(600);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);  

  delay(10000000);
}

void leftTurn(){
  pwm_start(LEFT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR_2, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 1000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(50);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 2200, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 2200, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(300);
  pwm_start(STEERING_SERVO, 50, STEERING_MAX_INPUT, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 2000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 500, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(666);
  pwm_start(STEERING_SERVO, 50, STEERING_NEUTRAL, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
  pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 2000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 2000, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  delay(450);
  // pwm_start(LEFT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);
  // pwm_start(RIGHT_MOTOR, MOTOR_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_12B_COMPARE_FORMAT);  

  // delay(10000000);
}



