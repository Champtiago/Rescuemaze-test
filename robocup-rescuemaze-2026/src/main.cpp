#include <Arduino.h>
#include "Encoder.h"
#include "Test.h"
#include "motors.h"
#include "Pins_ID.h"

constexpr uint8_t Sensors_Amount = 5;
static constexpr uint32_t vDelay = 33;
SemaphoreHandle_t i2cSemaphore;

volatile unsigned long LastTime[Sensors_Amount] = {0};
volatile unsigned long CurrentTime[Sensors_Amount] = {0};
volatile unsigned long IntraDelta[Sensors_Amount] = {0};
volatile unsigned long LastReadTime = 0;
volatile unsigned long InterDelta[Sensors_Amount] = {0};

// Variables para el algoritmo de mano derecha
bool rightHandStarted = true;

// Declaración d//*  *//e funciones
void VLXTaskPriority1(void *pv);
void VLXTaskPriority2(void *pv);
void PrintDistances(void *pv);
void RightHandNavigationTask(void *pv);

// Función para implementar la regla de mano derecha
void rightHandRule() {
    Serial.println("\n=== Evaluando situación ===");
    
    // Obtener distancias de los sensores
    uint16_t distFrontLeft = robot.vlx[vlxID::frontLeft].getDistance();
    uint16_t distFrontRight = robot.vlx[vlxID::frontRight].getDistance();
    uint16_t distFront = (distFrontLeft + distFrontRight) / 2;
    uint16_t distRight = robot.vlx[vlxID::rightUp].getDistance();
    uint16_t distLeft = robot.vlx[vlxID::leftUp].getDistance();
    
    // Umbral para considerar que hay pared (en cm)
    const uint16_t WALL_THRESHOLD = 20;
    
    bool wallFront = (distFront < WALL_THRESHOLD);
    bool wallRight = (distRight < WALL_THRESHOLD);
    bool wallLeft = (distLeft < WALL_THRESHOLD);
    
    // Debug
    Serial.print("Distancias - Frente: "); Serial.print(distFront);
    Serial.print(" cm | Derecha: "); Serial.print(distRight);
    Serial.print(" cm | Izquierda: "); Serial.print(distLeft);
    Serial.println(" cm");
    Serial.print("Paredes - Frente: "); Serial.print(wallFront ? "SI" : "NO");
    Serial.print(" | Derecha: "); Serial.print(wallRight ? "SI" : "NO");
    Serial.print(" | Izquierda: "); Serial.println(wallLeft ? "SI" : "NO");
    Serial.print("Ángulo actual: "); Serial.println(robot.bno.getOrientationX());
    
    // REGLA DE MANO DERECHA:
    // 1. Primero intentar girar a la derecha (mantener la mano derecha en la pared)
    if (!wallRight) {
        Serial.println("DECISIÓN: Girando DERECHA - no hay pared a la derecha");
        robot.right();
        robot.ahead();
        return;
    }
    
    // 2. Si hay pared a la derecha pero no al frente, avanzar recto
    if (!wallFront) {
        Serial.println("DECISIÓN: Avanzando RECTO - pared a la derecha, libre al frente");
        robot.ahead();
        return;
    }
    
    // 3. Si hay pared al frente y a la derecha, girar a la izquierda
    if (!wallLeft) {
        Serial.println("DECISIÓN: Girando IZQUIERDA - bloqueado al frente y derecha");
        robot.left();
        robot.ahead();
        return;
    }
    
    // 4. Si está completamente bloqueado (callejón sin salida), dar media vuelta
    Serial.println("DECISIÓN: Dando MEDIA VUELTA - completamente bloqueado");
    robot.left();
    robot.left();
    robot.ahead();
}

int servopos = 0;

void setup() {
  Serial.begin(115200);
  
  Serial.println("\n\n=================================");
  Serial.println("  ROBOT MANO DERECHA - INICIO");
  Serial.println("=================================\n");
  
  robot.setupMotors();
  
  // Configurar interrupciones para encoders
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontLeft]), 
                  Interrups::frontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kFrontRight]), 
                  Interrups::frontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackLeft]), 
                  Interrups::backLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::encoder[MotorID::kBackRight]), 
                  Interrups::backRightEncoder, RISING);
  
  // Crear semáforo para I2C
  i2cSemaphore = xSemaphoreCreateMutex();
  if (i2cSemaphore == NULL) {
    Serial.println("ERROR: Failed to create semaphore!");
    while(1);
  }
  Serial.println("Semaphore created successfully.");
  
  //  Aumentar delay para estabilización
  delay(500);
  
  // Calibrar orientación inicial
  robot.resetOrientation();
  
  if (robot.innit == true) {
    Serial.println("Verificando sensores VLX antes de crear tareas...");
    
    //  VERIFICACIÓN CRÍTICA: Probar cada sensor
    for (uint8_t i = 0; i < Sensors_Amount; i++) {
      Serial.print("  Sensor VLX["); 
      Serial.print(i); 
      Serial.print("]: ");
      
      // Intentar lectura
      uint16_t dist = robot.vlx[i].getDistance();
      Serial.print(dist);
      Serial.println(" cm");
      delay(50);
    }
    
    Serial.println("\nCreando tareas FreeRTOS...");
    
    //  Tarea para sensores VLX prioritarios (frente) - Stack aumentado
    BaseType_t result1 = xTaskCreatePinnedToCore(
      VLXTaskPriority1, 
      "VLXTaskPriority1", 
      8192,  //  Aumentado de 4096 a 8192
      NULL, 
      2, 
      NULL, 
      0
    );
    if (result1 != pdPASS) {
      Serial.println("ERROR: No se pudo crear VLXTaskPriority1");
    }

    // ⭐ Tarea para sensores VLX secundarios - Stack aumentado
    BaseType_t result2 = xTaskCreatePinnedToCore(
      VLXTaskPriority2, 
      "VLXTaskPriority2", 
      8192,  // ⭐ Aumentado de 4096 a 8192
      NULL, 
      2, 
      NULL, 
      1
    );
    if (result2 != pdPASS) {
      Serial.println("ERROR: No se pudo crear VLXTaskPriority2");
    }
    
    // ⭐ Tarea para imprimir distancias - Stack aumentado
    BaseType_t result3 = xTaskCreatePinnedToCore(
      PrintDistances, 
      "PrintDistances", 
      3072,  // ⭐ Aumentado de 2096 a 3072
      NULL, 
      1, 
      NULL, 
      1
    );
    if (result3 != pdPASS) {
      Serial.println("ERROR: No se pudo crear PrintDistances");
    }
    
    // ⭐ Tarea para navegación - Stack aumentado
    BaseType_t result4 = xTaskCreatePinnedToCore(
      RightHandNavigationTask, 
      "RightHandNav", 
      6144,  // ⭐ Aumentado de 4096 a 6144
      NULL, 
      3, 
      NULL, 
      0
    );
    if (result4 != pdPASS) {
      Serial.println("ERROR: No se pudo crear RightHandNavigationTask");
    }
    
    Serial.println("Todas las tareas creadas exitosamente.");
  } else {
    Serial.println("ERROR: robot.innit = false");
  }
  
  Serial.println("\n=================================");
  Serial.println("ROBOT LISTO - INICIANDO EN 3 SEG");
  Serial.println("=================================\n");
   // Dar tiempo para posicionar el robot
}


// ⭐ Tarea para actualizar sensores VLX prioritarios (frontales)
void VLXTaskPriority1(void *pv) {
  // ⭐ Delay inicial para asegurar que todo esté listo
  vTaskDelay(pdMS_TO_TICKS(500));
  
  Serial.println("VLXTaskPriority1 iniciada");
  
  while (true) {
    // ⭐ Timeout en semáforo para evitar bloqueos permanentes
    if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (uint8_t id : TaskVLX1) {
        // ⭐ VALIDACIÓN CRÍTICA: Verificar índice
        if (id >= Sensors_Amount) {
          Serial.print("ERROR VLXTask1: ID inválido: ");
          Serial.println(id);
          continue;
        }
        
        unsigned long now = esp_timer_get_time() / 1000; 
        IntraDelta[id] = now - LastTime[id];
        LastTime[id] = now;
        
        // ⭐ Protección adicional
        robot.vlx[id].updateDistance();
        
        InterDelta[id] = now - LastReadTime;
        LastReadTime = now;
      }
      xSemaphoreGive(i2cSemaphore);
    } else {
      // Timeout - semáforo no disponible
      Serial.println("WARN: VLXTask1 timeout en semáforo");
    }
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}

// ⭐ Tarea para actualizar sensores VLX secundarios
void VLXTaskPriority2(void *pv) {
  // ⭐ Delay inicial
  vTaskDelay(pdMS_TO_TICKS(600));
  
  Serial.println("VLXTaskPriority2 iniciada");
  
  while (true) {
    // ⭐ Timeout en semáforo
    if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (uint8_t id : TaskVLX2) {
        // ⭐ VALIDACIÓN CRÍTICA
        if (id >= Sensors_Amount) {
          Serial.print("ERROR VLXTask2: ID inválido: ");
          Serial.println(id);
          continue;
        }
        
        unsigned long now = esp_timer_get_time() / 1000;
        IntraDelta[id] = now - LastTime[id];
        LastTime[id] = now;
        
        robot.vlx[id].updateDistance();
        
        InterDelta[id] = now - LastReadTime;
        LastReadTime = now;
      }
      xSemaphoreGive(i2cSemaphore);
    } else {
      Serial.println("WARN: VLXTask2 timeout en semáforo");
    }
    vTaskDelay(pdMS_TO_TICKS(vDelay));
  }
}

// Tarea para imprimir distancias (debug)

void PrintDistances(void *pv) {
  // ⭐ Delay inicial
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  while (true) {
    /*
    Serial.println("\n========== SENSORES VLX ==========");
    Serial.println("VLX Priority 1 (Frontales):");
    for (uint8_t id : TaskVLX1) {
      // ⭐ Validación
      if (id >= Sensors_Amount) continue;
      
      Serial.print("  VLX["); Serial.print(id);
      Serial.print("]: "); Serial.print(robot.vlx[id].getDistance());
      Serial.print(" cm | IntraΔ: "); Serial.print(IntraDelta[id]);
      Serial.print(" ms | InterΔ: "); Serial.print(InterDelta[id]);
      Serial.println(" ms");
    }

    Serial.println("VLX Priority 2 (Laterales/Trasero):");
    for (uint8_t id : TaskVLX2) {
      // ⭐ Validación
      if (id >= Sensors_Amount) continue;
      
      Serial.print("  VLX["); Serial.print(id);
      Serial.print("]: "); Serial.print(robot.vlx[id].getDistance());
      Serial.print(" cm | IntraΔ: "); Serial.print(IntraDelta[id]);
      Serial.print(" ms | InterΔ: "); Serial.print(InterDelta[id]);
      Serial.println(" ms");
    }
  
    Serial.println("===================================\n");
    */
    vTaskDelay(pdMS_TO_TICKS(2000)); // Imprimir cada 2 segundos
  }
}

// Tarea principal para navegación con mano derecha
void RightHandNavigationTask(void *pv) {
  // ⭐ Delay inicial mayor para que los sensores se estabilicen
  vTaskDelay(pdMS_TO_TICKS(3000));
  
  Serial.println("\n*** NAVEGACIÓN CON MANO DERECHA INICIADA ***\n");
  
  while (true) {
    // Imprimir estado actual
    /*
    Serial.println("\n--- Estado actual ---");
    Serial.print("Frente L: "); Serial.print(robot.vlx[vlxID::frontLeft].getDistance());
    Serial.print(" cm | Frente R: "); Serial.print(robot.vlx[vlxID::frontRight].getDistance());
    Serial.println(" cm");
    Serial.print("Derecha: "); Serial.print(robot.vlx[vlxID::rightUp].getDistance());
    Serial.print(" cm | Izquierda: "); Serial.print(robot.vlx[vlxID::leftUp].getDistance());
    Serial.println(" cm");
    Serial.print("Atrás: "); Serial.print(robot.vlx[vlxID::back].getDistance());
    Serial.println(" cm");
    */
    
    // Ejecutar lógica de mano derecha
    //rightHandRule();
    
    // Esperar antes de la siguiente decisión
    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

void loop() {
  //Serial.println(robot.motor[MotorID::kBackRight].tics);
  robot.calibrateColors();
  checkTileColor();
  /*
  robot.ahead();
  robot.smoothRotate(90);
  delay(5000);
  robot.ahead();
  robot.ahead();
  robot.right();
  delay(5000);
  */
}