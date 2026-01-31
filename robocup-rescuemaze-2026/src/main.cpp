#include <Arduino.h>
#include <Wire.h>
#include <Pins_ID.h>
#include <VLX.h>
#include <BNO.H>

#define vDelay 33

SemaphoreHandle_t i2cSemaphore;
const int Sensors_Amount = 6;
float X;
float Y;
VLX sensors[Sensors_Amount];
//BNO bno;

volatile unsigned long LastTime[Sensors_Amount] = {0};
volatile unsigned long CurrentTime[Sensors_Amount] = {};
volatile unsigned long IntraDelta[Sensors_Amount] = {};
volatile unsigned long LastReadTime = 0;
volatile unsigned long InterDelta[Sensors_Amount] = {};

void VLXTaskPriority1(void *pv);
void VLXTaskPriority2(void *pv);
void PrintDistances(void *pv);

/*void setup() {
    Serial.begin(9600);
    Wire.begin(21, 22);
    //Wire1.begin(SDA_PIN, SCL_PIN);
    //Wire1.setClock(400000);
    Wire.setClock(100000);
    for (uint8_t ch = 0; ch < 6; ch++) 
      {
      Wire.beginTransmission(0x70);
      Wire.write(1 << ch);
      Wire.endTransmission();
      delay(100);

       Serial.println(ch);
      for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
        Serial.print("Found device at 0x");
        Serial.println(addr, HEX);
        }
        delay(50);
      }
    }

    delay(1000);

  }
*/
void loop() {
/*
  for (int i = 0; i < 100; i++) {
    X = bno.getOrientationX();
    delay(100);
    Y = bno.getOrientationY();
    delay(100);
    Serial.println("X: " + String(X) + " Y: " + String(Y));
  }
  bno.resetOrientation();
*/
}

void setup(){
    Serial.begin(115200);
    Wire.begin(21, 22);
    //Wire1.begin(SDA_PIN, SCL_PIN);
    //Wire1.setClock(400000);
    Wire.setClock(100000);
    
    for (uint8_t ch = 0; ch < 7; ch++) 
      {
      Wire.beginTransmission(0x70);
      Wire.write(1 << ch);
      Wire.endTransmission();
      delay(100);

      Serial.print("Scanning channel "); 
      Serial.println(ch);
      for (byte addr = 27; addr < 71; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
        Serial.print("Found device at 0x");
        Serial.println(addr, HEX);
        }
        delay(50);
      }
    }

    delay(1000);
    
    Serial.println("Scanning..."); 
    i2cSemaphore = xSemaphoreCreateMutex();
    if (i2cSemaphore == NULL) {
    Serial.println("ERROR: Failed to create semaphore!");
    while(1);
    }
    Serial.println("Semaphore created successfully.");
    
    //bno.setupBNO();
    
    //sensors[vlxID::frontRight].setMux(vlxID::frontRight);
    //sensors[vlxID::frontRight].begin();

    sensors[vlxID::back].setMux(vlxID::back);
    sensors[vlxID::back].begin();
    sensors[vlxID::left].setMux(vlxID::left);
    sensors[vlxID::left].begin();
    sensors[vlxID::right].setMux(vlxID::right);
    sensors[vlxID::right].begin();
    sensors[vlxID::frontLeft].setMux(vlxID::frontLeft);
    sensors[vlxID::frontLeft].begin();
    sensors[vlxID::front].setMux(vlxID::front);
    sensors[vlxID::front].begin();
    sensors[vlxID::frontRight].setMux(vlxID::frontRight);
    sensors[vlxID::frontRight].begin();
    
    
    

    delay(1500); 

    //sensors[vlxID::rightBack].begin();
    //sensors[vlxID::leftUp].begin();
    //sensors[vlxID::leftBack].begin();

    xTaskCreatePinnedToCore(
    VLXTaskPriority1,
    "VLXTaskPriority1",
    4096,
    NULL,
    2,
    NULL,
    0);

  xTaskCreatePinnedToCore(
    VLXTaskPriority2,
    "VLXTaskPriority2",
    4096,
    NULL,
    1,
    NULL,
    1);
  

    xTaskCreatePinnedToCore(
    PrintDistances,
    "PrintDistances",
    2096,
    NULL,
    1,
    NULL,
    1);
}


void VLXTaskPriority1(void *pv) {
  while (true) {
    xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
    for (uint8_t id : TaskVLX1) {
      unsigned long now = esp_timer_get_time() / 1000; 
      IntraDelta[id] = now - LastTime[id];
      LastTime[id] = now;
      sensors[id].updateDistance();
      InterDelta[id] = now - LastReadTime;
      LastReadTime = now;
    }
    xSemaphoreGive(i2cSemaphore);
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}


void VLXTaskPriority2(void *pv) {
  while (true) {
    xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
    for (uint8_t id : TaskVLX2) {
      unsigned long now = esp_timer_get_time() / 1000;
      IntraDelta[id] = now - LastTime[id];
      LastTime[id] = now;
      sensors[id].updateDistance();
      InterDelta[id] = now - LastReadTime;
      LastReadTime = now;
    }
    xSemaphoreGive(i2cSemaphore);
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}

void PrintDistances(void *pv) {
  while (true) {
    Serial.println("VLXPriority1 Task:");
    for (uint8_t id : TaskVLX1) {
      Serial.print("VLX ID "); Serial.print(id);
      Serial.print(": Distance "); Serial.print(sensors[id].getDistance());
      Serial.print(" cm | IntraDelta "); Serial.print(IntraDelta[id]);
      Serial.print(" ms | InterDelta "); Serial.print(InterDelta[id]);
      Serial.println(" ms");
    }


    Serial.println("VLXPriority2 Task:");
    for (uint8_t id : TaskVLX2) {
      Serial.print("VLX ID "); Serial.print(id);
      Serial.print(": Distance "); Serial.print(sensors[id].getDistance());
      Serial.print(" cm | IntraDelta "); Serial.print(IntraDelta[id]);
      Serial.print(" ms | InterDelta "); Serial.print(InterDelta[id]);
      Serial.println(" ms");
    }
  
    Serial.println("------------------------");
    vTaskDelay(pdMS_TO_TICKS(vDelay + 1000));
  }
}