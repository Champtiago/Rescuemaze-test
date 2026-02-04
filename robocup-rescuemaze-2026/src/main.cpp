
#include <Arduino.h>
#include "Encoder.h"
#include "Test.h"

constexpr uint8_t Sensors_Amount=4;
static constexpr uint32_t vDelay = 33;
SemaphoreHandle_t i2cSemaphore;

volatile unsigned long LastTime[Sensors_Amount] = {0};
volatile unsigned long CurrentTime[Sensors_Amount] = {};
volatile unsigned long IntraDelta[Sensors_Amount] = {};
volatile unsigned long LastReadTime = 0;
volatile unsigned long InterDelta[Sensors_Amount] = {};

void VLXTaskPriority1(void *pv);
void VLXTaskPriority2(void *pv);
void PrintDistances(void *pv);

int servopos=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  robot.setupMotors();
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontLeft]), Interrups::frontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]), Interrups::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]), Interrups::backLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackRight]), Interrups::backRightEncoder, RISING);
  i2cSemaphore = xSemaphoreCreateMutex();
  if (i2cSemaphore == NULL) {
    Serial.println("ERROR: Failed to create semaphore!");
    while(1);
    }
  Serial.println("Semaphore created successfully.");
  if (robot.innit == true) {
    xTaskCreatePinnedToCore(
      VLXTaskPriority1, "VLXTaskPriority1", 4096, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(
      VLXTaskPriority2, "VLXTaskPriority2", 4096, NULL, 2, NULL, 1);
    
    xTaskCreatePinnedToCore(
      PrintDistances, "PrintDistances", 2096, NULL, 2, NULL, 1);

  }
  
  // robot.reloadKits();
}

void loop() {
  //testEncoders();
  //testPIDWheel();
  // // robot.leds.sequency();

  // robot.kitLeft(1);
  // delay(1000);
  // robot.kitRight(1);
  // delay(1000);

  // robot.ahead();
  // testTCS();
  // robot.setahead();
  // robot.setSpeed(50);

  // jeetson.getDetection();
  // delay(300);
  // testButton();
  // robot.ahead();
  //robot.setSpeed(40);
  //robot.moveDistance(30, true);
  //delay(5000);
  //testPIDWheel();
  /*testPIDWheel();
  delay(500);
  pidTest();
  delay(500);
  robot.right();
  delay(1000);
  robot.stop();
  while(1);
  */
  // calibrateColors();
  // robot.checkpointElection(); 
  // robot.buttonPressed=false;
  // testTCS();
  // testLimits();
  // testBnoY();
  
  // testVlxFrontDistance();
  // testVlxFrontLeft();
  // testVlxFrontRigth();
  // testVlxRight();
  // testVlxLeft();
  // testVlxFront();
  // testVlxBack();
}

void VLXTaskPriority1(void *pv) {
  while (true) {
    xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
    for (uint8_t id : TaskVLX1) {
      unsigned long now = esp_timer_get_time() / 1000; 
      IntraDelta[id] = now - LastTime[id];
      LastTime[id] = now;
      robot.vlx[id].updateDistance();
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
      robot.vlx[id].updateDistance();
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
      Serial.print(": Distance "); Serial.print(robot.vlx[id].getDistance());
      Serial.print(" cm | IntraDelta "); Serial.print(IntraDelta[id]);
      Serial.print(" ms | InterDelta "); Serial.print(InterDelta[id]);
      Serial.println(" ms");
    }


    Serial.println("VLXPriority2 Task:");
    for (uint8_t id : TaskVLX2) {
      Serial.print("VLX ID "); Serial.print(id);
      Serial.print(": Distance "); Serial.print(robot.vlx[id].getDistance());
      Serial.print(" cm | IntraDelta "); Serial.print(IntraDelta[id]);
      Serial.print(" ms | InterDelta "); Serial.print(InterDelta[id]);
      Serial.println(" ms");
    }
  
    Serial.println("------------------------");
    vTaskDelay(pdMS_TO_TICKS(vDelay + 1000));
    }
  }
