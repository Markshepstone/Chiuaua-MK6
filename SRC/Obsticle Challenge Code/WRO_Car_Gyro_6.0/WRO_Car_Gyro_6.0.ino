// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>


#include <Servo.h>

#include <Wire.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Servo myservo;
Servo TOFServo;

int targetAngle = 0;


// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1



int trueAngleLookup[] = { 0, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 15, 15, 16, 16, 17, 18, 18, 19, 20, 20, 21, 21, 22, 23, 23, 24, 25, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 36, 37, 38, 38, 39, 40, 40, 41, 41, 42, 43, 43, 44, 45, 45, 46, 46, 47, 48, 48, 49, 50, 50, 51, 52, 53, 53, 54, 55, 56, 57, 58, 58, 59, 60, 61, 62, 63, 63, 64, 65, 66, 67, 68, 68, 69, 70, 71, 72, 73, 73, 74, 75, 76, 77, 78, 78, 79, 80, 81, 82, 83, 83, 84, 85, 86, 87, 88, 88, 89, 90, 91, 92, 94, 95, 96, 98, 99, 100, 101, 102, 104, 105, 106, 107, 109, 110, 111, 112, 114, 115, 116, 117, 119, 120, 121, 122, 124, 125, 126, 127, 129, 130, 131, 133, 134, 135, 137, 138, 140, 142, 145, 147, 150, 153, 157, 161, 165, 170, 175, 180, 180 };

const int servoPin = 9;



Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;


int trueAngle = 0;
int trueAngleOffset = 0;
int displayAngle = 0;
int angle;
int angleOld = 9999;
int lookUpAngle;
void setup(void) {
  pinMode(25, OUTPUT);

  Serial.begin(115200);
  delay(1000);
    Serial.println("Hello World");
      delay(200);
      Serial.println("Hello World");
        delay(200);
        Serial.println("Hello World");
          Serial.println("Hello World");

 






   
    digitalWrite(25, HIGH);
    delay(250);
    digitalWrite(25, LOW);
    delay(500);

  



     digitalWrite(25, HIGH);
    delay(100);
    digitalWrite(25, LOW);
    delay(200);



  Serial.println("FLASH");


  delay(500);
  Wire1.setSDA(14);
  Wire1.setSCL(15);
  Serial.println("Wire1 SET");
  Wire1.begin(12);
  Serial.println("wire1 BEGIN");


 digitalWrite(25, HIGH);
    delay(100);
    digitalWrite(25, LOW);
    delay(200);

  Wire1.onReceive(receiveEvent);  // register event
  Wire1.onRequest(requestEvent);

  Serial.println("Adafruit BNO08x test!");
  delay(500);
      digitalWrite(25, HIGH);
    delay(1000);
    digitalWrite(25, LOW);
  // Try to initialize!

  if (!bno08x.begin_I2C()) {
    //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
  
    
   watchdog_enable(100, 0);
    while (1) {
      delay(10);
    }
  }

 digitalWrite(25, HIGH);
    delay(100);
    digitalWrite(25, LOW);
    delay(200);
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  setReports();

  Serial.println("Reading events");
  delay(100);



  if (watchdog_caused_reboot()) {

    Serial.println("Watchdog caused last reboot, proceeding.");
  } else {
    Serial.println("Watchdog did not cause last reboot, Reset.");
    watchdog_enable(100, 0);
    while (1) {
      delay(10);
    }
  }

    
    myservo.attach(servoPin, 500, 2500);
    TOFServo.attach(17, 500, 2500);
    

   if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)){
      Serial.println("Display failiure");
    }else{
      Serial.println("Display initialised");
    }


     display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!

   display.display();
   delay(1000);



  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}


void loop() {

  steer();

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

    case SH2_GAME_ROTATION_VECTOR:

      lookUpAngle = round(sensorValue.un.rotationVector.k * 180);


      // Serial.print(lookUpAngle);


      if (lookUpAngle >= 0 && sensorValue.un.rotationVector.k <= 0.71) {
        trueAngle = trueAngleLookup[abs(lookUpAngle % 180)];

        int measuredAngle = trueAngleLookup[abs(lookUpAngle % 180)];
        // Serial.print("  measured ");
        // Serial.print(measuredAngle);

        //  Serial.print(" NORM POS: ");
      } else if (lookUpAngle < 0 && sensorValue.un.rotationVector.k >= -0.71) {
        int measuredAngle = (-trueAngleLookup[abs(lookUpAngle % 180)]);
        // Serial.print("  measured ");
        // Serial.print(measuredAngle);


        trueAngle = (-trueAngleLookup[abs(lookUpAngle % 180)]);
        //  Serial.print(" NORM NEG: ");
      }

      if (sensorValue.un.rotationVector.k > 0.71) {


        lookUpAngle = round((1 - sensorValue.un.rotationVector.real) * 180);

        int measuredAngle = trueAngleLookup[abs(180 - lookUpAngle % 180)];
        // Serial.print("  measured ");
        // Serial.print(measuredAngle);

        if (measuredAngle != 180) {
          trueAngle = (180 - trueAngleLookup[abs(180 - lookUpAngle % 180)]);
        } else {
          trueAngle = 180;
        }
        // Serial.print(" REV POS: ");
      }


      if (sensorValue.un.rotationVector.k < -0.71) {
        //Serial.print(" REV NEG: ");
        lookUpAngle = round((1 - sensorValue.un.rotationVector.real) * 180);

        int measuredAngle = (int)abs(180 - lookUpAngle % 180);
        // Serial.print("  measured ");
        // Serial.print(measuredAngle);
        if (measuredAngle != 180) {
          trueAngle = (-(180 - trueAngleLookup[abs(180 - lookUpAngle % 180)]));
        } else {
          trueAngle = -180;
        }
      }

              Serial.print(" TRUE: ");
             Serial.print(displayAngle);
              Serial.print(" : ");
     Serial.print(angleOld+trueAngleOffset);

           Serial.print(" Offset: ");
            Serial.println(trueAngleOffset);
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(0, 0);
            display.print("Act: ");
            display.println(displayAngle);
            display.print("tgt: ");
            display.print(targetAngle);
            display.display();


      if (angleOld != trueAngle) {

        if (angleOld >= 170 && trueAngle <= -170) {

          trueAngleOffset += 360;
          //Serial.println("Applying offset POS");
        }
        if (angleOld <= -170 && trueAngle >= 170) {

          // Serial.println("Applying offset NEG");
          trueAngleOffset -= 360;
        }
        
      displayAngle = trueAngle + trueAngleOffset;

        angleOld = trueAngle;
      }

      //Serial.println("");
      break;
  }
}



void steer() {

  int angleDiff = (displayAngle - targetAngle);
 


  int maxSteering = 60;
  if (angleDiff > maxSteering) {
    angleDiff = maxSteering;
  } else if (angleDiff < -maxSteering) {
    angleDiff = -maxSteering;
  }

  float multiplier = 0.5;

 

  
  float steeringAngle = angleDiff * multiplier;





  steeringAngle += 88;
  Serial.print(steeringAngle);
  Serial.print("    ");
  Serial.print(angleDiff);
  Serial.print("    ");
  //erial.println(targetAngle);

  myservo.write(steeringAngle);
}



int recieveInt() {
  //digitalWrite(25, HIGH);
  byte b1 = Wire1.read();
  byte b2 = Wire1.read();

  int returnInt = b1;

  returnInt = returnInt << 8;
  returnInt += b2;

  return returnInt;
}
void sendInt(int sendInt) {
  byte b1 = sendInt >> 8;
  byte b2 = sendInt;
  Wire1.write(b1);
  Wire1.write(b2);
}

void receiveEvent(int len) {


  int16_t tatmp = recieveInt();
  targetAngle = tatmp;
  //delay(5);
  Serial.print("RECIEVED\t\t\t\t Target angle  =");
  Serial.println(targetAngle);
  //int motor = Wire.read();
  //delay(1000);
}

void requestEvent() {

  sendInt(displayAngle);
}


