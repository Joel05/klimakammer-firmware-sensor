#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_LTR390.h"
#include "RPi_Pico_TimerInterrupt.h"

Adafruit_BMP280 bmp; // I2C
Adafruit_LTR390 ltr = Adafruit_LTR390();
RPI_PICO_Timer ITimer(1);

const char I2C_ADDR = 0x71; //Set to desired i2c-adress
#undef DEBUG    //Define for various debug outputs (#undef to disable) - !!!ENABLING SLOWS DOWN CODE SIGNIFICANTLY!!!

#define Pressure 0x01
#define Temperature 0x02
#define Brightness 0x03
#define UVBrightness 0x04

char module = 0x00;  //Variable to store the module that is being called

float BrightnessValue = 0;
float UVBrightnessValue = 0;
bool switchState = 0;

#define BUILTIN_LED 25 //GPIO of BUILTIN_LED for pico
#ifdef esp32dev
  #undef BUILTIN_LED
  #define BUILTIN_LED 2 //GPIO of BUILTIN_LED for esp32dev
#endif


void sendData(float data1, float data2);  //Function to send data back to the master
void sendData(int data1, int data2);  //Overload to accept int as argument
void sendData(char data1, char data2);  //Overload to accept char as argument
void onRequest(); //Code to execute when master requests data from the slave
void onReceive(int len);  //Code to execute when master sends data to the slave

#pragma region sendData

void sendData(float data1 = 0, float data2 = 0){  //Function to send data back to the master
  //Pointer to the float
  uint8_t *bytePointer1 = reinterpret_cast<uint8_t*>(&data1);

  //Iterate throught the adresses of the pointer, to read the bytes of the float, and send them via i2c
  for (uint8_t i = 0; i < sizeof(float); ++i) {
      Wire1.write((*bytePointer1));
      bytePointer1++;
  }

  //Pointer to the second float
  uint8_t *bytePointer2 = reinterpret_cast<uint8_t*>(&data2);

  //Iterate throught the adresses of the pointer, to read the bytes of the float, and send them via i2c
  for (uint8_t i = 0; i < sizeof(float); ++i) {
      Wire1.write((*bytePointer2));
      bytePointer2++;
  }
}

void sendData(int data1 = 0, int data2 = 0){ //Overload to accept int as argument
  sendData((float)data1, (float)data2);
}

void sendData(char data1 = 0, char data2 = 0){  //Overload to accept char as argument
  sendData((float)data1, (float)data2);
}

#pragma endregion

#ifdef DEBUG
void blink(){
  for (char i = 0; i<10; i++){
    digitalWrite(BUILTIN_LED, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED, LOW);
    delay(50);
  }
}
#endif

void onRequest(){ //Code to execute when master requests data from the slave
  #ifdef DEBUG
  Serial.println("OnRequest");
  Serial.println(Wire1.peek());
  blink();
  #endif
  //Data is already saved in the module variable
  switch(module){
    case Pressure:
      #ifdef DEBUG
        Serial.println("Module 1 called");
      #endif
      //Code to execute when Module1 is being called
      sendData(bmp.readPressure());
      break;
    case Temperature:
      #ifdef DEBUG
        Serial.println("Module 2 called");
      #endif
      //Code to execute when Module2 is being called
      sendData(bmp.readTemperature());
      break;
    case Brightness:
      #ifdef DEBUG
        Serial.println("Module 3 called");
      #endif
      //Code to execute when Module3 is being called
      sendData(BrightnessValue);
      break;
    case UVBrightness:
      #ifdef DEBUG
        Serial.println("Module 4 called");
      #endif
      //Code to execute when Module4 is being called
      sendData(UVBrightnessValue);
      break;
    default:
      //Code to execute when unkown module is being called
      #ifdef DEBUG
        Serial.println("Unknown module called");
      #endif
      Wire1.write(0);  //Send 0 back to the master
      break;
  }
}

void onReceive(int len){
  #ifdef DEBUG
  Serial.println("OnReceive");
  blink();
  #endif
  //Code to execute when master sends data to the slave
  module = Wire1.read();  //Read from which sensor/module the master wants to change
  if (!Wire1.available()){  //Check if there is no more data to read, which means that the master wants to read data from the slave
    return;
  }
  char data = Wire1.read();  //Read the data the master wants to send
  switch(module){
    case Pressure:
      #ifdef DEBUG
        Serial.println("Module 1 called");
      #endif
      //Code to execute when Module1 is being called
      break;
    case Temperature:
      #ifdef DEBUG
        Serial.println("Module 2 called");
      #endif
      //Code to execute when Module2 is being called
      break;
    case Brightness:
      #ifdef DEBUG
        Serial.println("Module 3 called");
      #endif
      //Code to execute when Module3 is being called
      break;
    default:
      //Code to execute when unkown module is being called
      #ifdef DEBUG
        Serial.println("Unknown module called");
      #endif
      break;
  }
}

void readBrightness(){
  if(switchState){
    BrightnessValue = (ltr.readALS()*0.6)/(3*4);
    ltr.setMode(LTR390_MODE_UVS);
  }
  else{
    UVBrightnessValue = ltr.readUVS();
    ltr.setMode(LTR390_MODE_ALS);
  }
  switchState = !switchState;
  return;
}

void test(){
  return;
}

void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG
  pinMode(BUILTIN_LED, OUTPUT);
  #endif

  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);  //Initialize the BMP280 sensor

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  ltr.begin();  //Initialize the LTR390 sensor
  ltr.setGain(LTR390_GAIN_3);  //Set the gain of the sensor
  ltr.setResolution(LTR390_RESOLUTION_20BIT);  //Set the resolution of the sensor

  ITimer.setInterval(500*1000, (pico_timer_callback)readBrightness);  //Set the interval of the timer to 500ms

  Serial.begin(115200);
  Wire1.setSDA(10);
  Wire1.setSCL(11);
  Wire1.onReceive(onReceive);  //Function to be called when a master sends data to the slave
  Wire1.onRequest(onRequest);  //Function to be called when a master requests data from the slave
  Wire1.begin((uint8_t)I2C_ADDR);  //Register this device as a slave on the i2c-bus (on bus 0)

}

void loop() {
  // put your main code here, to run repeatedly:


}