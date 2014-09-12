#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>
#define PIN 13

Adafruit_NeoPixel strip = Adafruit_NeoPixel(18, PIN, NEO_GRB + NEO_KHZ800);

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
float currentX=0.0;
float currentY=0.0;
float currentZ=0.0;

int *tiltArray;

int y1[] = {
  2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,11};
int y2[] = {
  2,2,2,2,2,2,2,2,2,2,2,2,1,2,11,12};
int y3[] ={
  2,2,2,2,2,2,2,2,2,0,1,2,11,12,13};

int y4[] ={
  0,0,0,0,0,0,0,1,2,11,12,13,14,15,0,0};

int ny1[] = {
  3,10,3,3,3,3,3,3,3,3,3,3,3,3,3,3};
int ny2[] = {
  3,4,9,10,3,3,3,3,3,3,3,3,3,3,3,3};
int ny3[] ={
  3,4,5,8,9,10,3,3,3,3,3,3,3,3,3};
int ny4[] ={
  3,4,5,6,7,8,9,10,3,3,3,3,3,3};


volatile boolean flashLightStatus = 0;
volatile uint8_t adxlIntSource = 0;
volatile boolean adxlInterruptState = 0;


void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); 
  Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); 
  Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); 
  Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); 
  Serial.print(sensor.max_value); 
  Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); 
  Serial.print(sensor.min_value); 
  Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); 
  Serial.print(sensor.resolution); 
  Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


static uint8_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(ADXL345_ADDRESS);
  //i2cwrite(reg);
  Wire.write((uint8_t) reg);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_ADDRESS, 1);
  return Wire.read();  
}

void writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}


void setup(void) 
{
  interrupts();
  attachInterrupt(0, accelInterrupt, RISING);
  
  strip.begin();
  strip.show();
  Serial.begin(9600);
  Serial.println("Accelerometer Test"); 
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  //accel.setRange(ADXL345_RANGE_16_G);
  accel.setRange(ADXL345_RANGE_2_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  //displaySetRange(ADXL345_RANGE_2_G);
  writeRegister(ADXL345_REG_THRESH_TAP, 29);
  Serial.print("The value of the tap threshold is ");
  Serial.println(readRegister(ADXL345_REG_THRESH_TAP));
  Serial.print("The value of the device ID is ");
  Serial.println(readRegister(ADXL345_REG_DEVID));
  Serial.print("The value of the activity threshold is ");
  Serial.println(readRegister(ADXL345_REG_THRESH_ACT));
  //interrupts
  Serial.print("The value of interrupt enable ctrl is ");
  Serial.println(readRegister(ADXL345_REG_INT_ENABLE));
  Serial.print("The value of the interrupt mapping is ");
  Serial.println(readRegister(ADXL345_REG_INT_MAP));

  //set ADXL interrupts
  //enable single tap interrupt
  //writeRegister(ADXL345_REG_INT_ENABLE, 0b01000000);

  //enable single and double tap interrupt
  writeRegister(ADXL345_REG_INT_ENABLE, 0b01100000);

  //map all interrupts to pin1
  writeRegister(ADXL345_REG_INT_MAP, 0b00000000);

  // single tap configuration
  writeRegister(ADXL345_REG_DUR, 0x1F); // 625us/LSB
  writeRegister(ADXL345_REG_THRESH_TAP, 48); // 62.5mg/LSB  <==> 3000mg/62.5mg = 48 LSB as datasheet suggestion
  writeRegister(ADXL345_REG_TAP_AXES, 0b111); // enable tap detection on x,y,z axes

  //double tap configuration
  writeRegister(ADXL345_REG_LATENT, 0x50);
  writeRegister(ADXL345_REG_WINDOW, 0xff);

  //read and clear interrupts
  Serial.print("Interrupt source ");
  Serial.println(readRegister(ADXL345_REG_INT_SOURCE));
  //setup interrupts
 
}

//*******Single Tap Interrupt Routine*************
void accelInterrupt()
{
  adxlInterruptState=1;
}

void blank()
{
  for (int i =0; i< 18; i++)
  {
    strip.setPixelColor(i,strip.Color(0,0,0));
  }
  strip.show();
}

void happy()
{
  blank();
  happyRainbowCycle(20);
}

void flashLight()
{
  if (flashLightStatus == 0)
  {
    for (int i=0; i<18; i++)
    {
      strip.setPixelColor(i,strip.Color(250,0,0));
    }
    strip.show();
  }
  else
    blank();
  flashLightStatus = !flashLightStatus;
}

void displayTilt(int tiltArray2[])
{
  for (int i=0; i<16; i++)
  {
    strip.setPixelColor(tiltArray2[i],strip.Color(0,0,250));
  }
  strip.show();
  delay(200);
  blank();
}

void happyRainbowCycle(uint8_t wait) {
  uint16_t i, j;
  for(j=0; j<256*1; j++) { // x1 cycles of all colors on wheel
    //for(i=0; i< strip.numPixels(); i++) {
    for(i=0; i<16; i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
  blank();
}

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void loop(void) 
{
  if (adxlInterruptState)
  {
    adxlIntSource = readRegister(ADXL345_REG_INT_SOURCE);
    if (adxlIntSource & 0b00100000)
    {
      Serial.println("Two taps interrupt caught");
      happyRainbowCycle(20);  
    }
    else if (adxlIntSource & 0b01000000)
    {
      flashLight();
    }
    adxlInterruptState = 0;
  }

  /* Get a new sensor event */
  sensors_event_t event; 
  accel.getEvent(&event);
  currentX = event.acceleration.x;
  currentY = event.acceleration.y;
  currentZ = event.acceleration.z;

  if (currentY < -1)
  {
    tiltArray = ny1;
  }
  if (currentY < -2)
  {
    tiltArray = ny2; 
  }
  if (currentY < -3)
  {
    tiltArray =ny3;
  }
  if (currentY < -4)
  {
    tiltArray =ny4;
  }
  if (currentY > 1)
  {
    tiltArray = y1; 
  }
  if (currentY > 2 )
  { 
    tiltArray = y2;
  }
  if (currentY > 3 )
  { 
    tiltArray=y3;
  }
  if (currentY > 4 )
  { 
    tiltArray=y4;  
  }
  if (!flashLightStatus)
  {
    if ( (currentY > -1) and (currentY < 1))
    {
      blank();
    }
    else
    {
      displayTilt(tiltArray); 
    }
  } 
}






