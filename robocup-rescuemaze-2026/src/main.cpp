#include <Arduino.h>
#include "Encoder.h"
#include "Test.h"
#include "motors.h"
#include "Pins_ID.h"
#include "maze.h"

bool calibrateColor = false;
maze m;
static constexpr uint32_t vDelay = 33;
SemaphoreHandle_t i2cSemaphore;

void VLXTaskPriority1(void *pv);
void VLXTaskPriority2(void *pv);

int servopos = 0;

void setup() {
  Serial.begin(115200);

  robot.setupMotors();
  
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontLeft]), 
                  Interrups::frontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]), 
                  Interrups::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]), 
                  Interrups::backLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackRight]), 
                  Interrups::backRightEncoder, RISING);
  
  i2cSemaphore = xSemaphoreCreateMutex();
  if (i2cSemaphore == NULL) {
    Serial.println("ERROR: Failed to create semaphore!");
    while(1);
  }
  Serial.println("Semaphore created successfully.");
  
  delay(500);
  
  robot.resetOrientation();
  
  if (robot.innit == true) {
    BaseType_t result2 = xTaskCreatePinnedToCore(
      VLXTaskPriority2, 
      "VLXTaskPriority2", 
      8192,
      NULL, 
      1, 
      NULL, 
      0
    );
    if (result2 != pdPASS) {
      Serial.println("ERROR: No se pudo crear VLXTaskPriority2");
    }
    Serial.println("Todas las tareas creadas exitosamente.");
  } else {
    Serial.println("ERROR: robot.innit = false");
  }
}

void VLXTaskPriority2(void *pv) {
  while (true) {
    if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (uint8_t id : TaskVLX2) {
        robot.vlx[id].updateDistance();
    }
    robot.tcs_.startIntegration();
    xSemaphoreGive(i2cSemaphore);
    }

    vTaskDelay(pdMS_TO_TICKS(170));

    if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
        robot.tcs_.updateRGBC();
        xSemaphoreGive(i2cSemaphore);
    }
    
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}


void loop() {

  m.run_algs();
  
}