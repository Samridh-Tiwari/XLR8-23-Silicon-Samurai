# XLR8-23-Silicon-Samurai
A brief of our journey through XLR8 23, how even after multiple failures we persevered and made a bot with a cruel sense of humour. 
Let first begin from square one ie designing a good-looking yet effective bot
Our basic idea for the chassis was a unique-looking design having proper areas allocated for things we planned to put on it which were the ultrasonic sensor and an LED display.
But it turns out designing a chassis and being able to practically laser cut it is a difficult process and with the overworked laser printer in the TL our first chassis was imperfect and asymmetric + when you decided to drill holes for our clamp due to some mechanical errors the clamp was not sitting perfectly on chassis
Finally, after making the required changes and buying a new acrylic sheet we again printed a new chassis
But… this time the horizontal dimension was too wide and we were forced to redesign it again
Finally, with our last chasis, our team member managed to include proper holes for the clamp in the soft copy and the laser cutting was also perfect so we moved on with integrating the mechanical and electrical components on it.
You can find link to our final chasis design which we made in fusion 360 with a days worth of pre-experience

Moving on to the soldering and electrical integration part

Finally here is our code which includes the basic driving instructions along with Ultrasonic detector and LED dependencies, Ultrasonic and motor dependencies which allowed the detector to detect when our bot came in close proximity to a wall and automatically stop and at the same time show the distance to the nearest wall which lied in front of the bot


#include <WiFi.h>
#include <esp_now.h>

// Define a structure to hold IMU (Inertial Measurement Unit) data
typedef struct {
  float gx, gy, gz;
} IMUData;

IMUData myMessage; // Create a variable to store received IMU data
int cmd = 0;       // Initialize motor control command variable
int spd = 0;       // Initialize motor speed variable

// Function to handle received data from another device
void onDataReceiver(const uint8_t* mac, const uint8_t* incomingData, int len) {
  Serial.println("Message received.");
  // Copy the received data into the myMessage variable
  memcpy(&myMessage, incomingData, sizeof(myMessage));
  
  // Display received data
  Serial.println("=== Data ===");
  Serial.print("Mac address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(mac[i], HEX);  // Display MAC address
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  Serial.print("gx: ");
  Serial.print(myMessage.gx);    // Display x-axis gyro value
  Serial.print(", gy: ");
  Serial.print(myMessage.gy);    // Display y-axis gyro value
  Serial.print(", gz: ");
  Serial.println(myMessage.gz);   // Display z-axis gyro value
}
int trigPin = 5;    // Trigger
int echoPin = 4;    // Echo
long duration, cm;

//some experiments for the cheap 1.8 tft display working with the ESP32

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>

//Pin for ESP32
  #define TFT_CS         19  //case select connect to pin 5
  #define TFT_RST        15 //reset connect to pin 15
  #define TFT_DC         21 //AO connect to pin 32  (not sure that this is really used)  try pin 25
  #define TFT_MOSI       23 //Data = SDA connect to pin 23
  #define TFT_SCLK       18 //Clock = SCK connect to pin 18

// For ST7735-based displays, we will use this call
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

//powering the tft display...

//GND is GND  that is the easy one
//LED pin on TFT will control brightness.  Using DAC on the ESP32 can control brightness by connecting
//to this pin.  See code below.  If this pin is high, VCC on the TFT does not need to be connected, but
//things seem to work better when it is connected.  VCC can go to either the 3.3 volt or the 5 volt (Vin)
//of the ESP32.  There does not appear to be any appreciable brightness difference when 5v is used instead of 3.3
//But the screen is brightest when 3.3 volts for VCC and 5volts for LED.
  

// Function to update motor control based on received IMU data
void updateMotorControl() {
  float gx = myMessage.gx;
  float gy = myMessage.gy;
  float gz = myMessage.gz;

  // Motor control logic based on IMU data
  if ((gz != 0) && (gx != 0) && (abs(gy) < 2)) {
    spd = constrain(abs(map((atan2(gx, gz) * 180 / PI), 0, 90, 0, 255)), 0, 255);
    cmd = (gx > 0) ? 1 : 2; // Forward or backward
  } else if ((gz != 0) && (gy != 0) && (abs(gx) < 2)) {
    spd = constrain(abs(map((atan2(gy, gz) * 180 / PI), 0, 90, 0, 255)), 0, 255);
    cmd = (gy > 0) ? 3 : 4; // Right or left
  } else {
    cmd = 0; // Stop
    spd = 0;
  }

  // Adjust motor speed thresholds
  if (spd > 60 && spd < 150) {
    spd = 150;
  }
  if (spd > 150 && spd < 255) {
    spd = 255;
  }

  // Display motor control information
  Serial.print("cmd: ");
  Serial.print(cmd);   // Display motor command
  Serial.print(", speed: ");
  Serial.println(spd); // Display motor speed
}

// Pin assignments for motor control
// These pins are the Enable pins of the L298N motor driver which connects to esp32 gpio pins to implement the PWM function

const int ENA = 12;  // choose the GPIO pin of esp32;
const int ENB = 13;  // choose the GPIO pin of esp32;

// These pins are the input pins of l298N on the left side
const int IN1 = 14;  // Choose your GPIO pin of esp32 for the input 1
const int IN2 = 27;  // Choose your GPIO pin of esp32 for the input 2

// These pins are the input pins of l298N on the right side
const int IN3 = 26;  // Choose your GPIO pin of esp32 for the input 3
const int IN4 = 25;  // Choose your GPIO pin of esp32 for the input 4

// Setup function
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  Serial.println("Setup started");

  // Configure motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);    //fill in the blanks 
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);    //fill in the blanks   
  pinMode(IN3, OUTPUT);    //fill in the blanks 
  pinMode(IN4, OUTPUT);

  Serial.print("Mac Address: ");
  Serial.print(WiFi.macAddress());
  Serial.println(" ESP32 ESP-Now Broadcast");

  // Initialize ESP-NOW communication
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }

  // Register the data receiver callback function
  esp_now_register_recv_cb(onDataReceiver);

  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(115200);
  // Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab

  Serial.println("Initialized");

}//end of void setup
/**
 Possible colors that are predefined...
ST7735_BLACK ST77XX_BLACK
ST7735_WHITE ST77XX_WHITE
ST7735_RED ST77XX_RED
ST7735_GREEN ST77XX_GREEN
ST7735_BLUE ST77XX_BLUE
ST7735_CYAN ST77XX_CYAN
ST7735_MAGENTA ST77XX_MAGENTA
ST7735_YELLOW ST77XX_YELLOW
ST7735_ORANGE ST77XX_ORANGE
*/


// Function to apply motor control based on command and speed
void applyMotorControl() {
  switch (cmd) {

    // You have to develop the logic that, when the IMU is tilted front to go forward, Then the esp32 executes the following commands
    // Refer to get electrified slides for more help
    case 1:  // Forward
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      break;
    case 2:  // Backward
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      break;
    case 3:  // Right
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      break;
    case 4:  // Left
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      break;
    default:  // Stop
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      spd = 0;
      break;
    if(cm<5){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      delay(5000);
    }
  }

  // Apply the calculated motor speed to both motors
  // fill in the blanks to finalize the code
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

// Main loop
void loop() {
  
  // Delay to control loop speed
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  
  // Continuously update and apply motor control
  updateMotorControl();
  applyMotorControl();
tft.fillScreen(ST77XX_WHITE);
tft.setCursor(0, 40);
tft.setTextColor(ST7735_BLACK);
tft.setTextSize(2);
tft.print(cm);
if(cm<5){
  tft.setCursor(0, 50);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(5);
  tft.print("STOP");
  delay(5000);
}
}
