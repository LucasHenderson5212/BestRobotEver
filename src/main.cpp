#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
#define INTERRUPTPIN PB1
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void handle_interrupt();
volatile int i=0;
volatile int j=0;

void setup() {
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  

  // Displays "Hello world!" on the screen
  display_handler.clearDisplay();
  display_handler.setTextSize(1.1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(30,30);
  display_handler.println("Hello World");
  display_handler.display();
  delay(2000);
  // display_handler.println("no collision");
  // display_handler.display();
  pinMode(INTERRUPTPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPTPIN), handle_interrupt, RISING);
}

void loop() {
  display_handler.clearDisplay();
  display_handler.setTextSize(1.1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(30,30);
  char str[25];
  sprintf(str, "Collisions: %d", i);
  display_handler.println(str);
  display_handler.display();

}

void handle_interrupt(){
  i++;
}
