#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible
#define LED_BUILTIN PC13
#define RIGHT_EYE PA2
#define LEFT_EYE PA1
#define DETECT_THRESHOLD PA0

volatile int i=0;
volatile int j=0;
int loopcounter;

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
   //initialize LED digital pin as an output
  pinMode(LED_BUILTIN, OUTPUT);

  //Tape Following
  pinMode(RIGHT_EYE, INPUT);
  pinMode(LEFT_EYE, INPUT);
  pinMode(DETECT_THRESHOLD, INPUT);


  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  delay(2000);
}

void loop() {

display_handler.clearDisplay();
display_handler.setCursor(0,0);


int right = analogRead(RIGHT_EYE);
int left = analogRead(LEFT_EYE);
int threshold = analogRead(DETECT_THRESHOLD);
if(right < threshold){
  display_handler.println("Turn left!");
}
if(left < threshold){
  display_handler.println("Turn right!");
}
else{
  display_handler.println("On tape!");
}
 display_handler.display();
}
