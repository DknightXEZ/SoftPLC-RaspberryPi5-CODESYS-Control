#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <EEPROM.h>
#include "CodesysLink.h"   // 🔵 [CODESYS] ช่องรับคำสั่งจาก CODESYS + สิทธิ์ควบคุม (ดูแท็บ CodesysLink)

// ==========================================
// 🟢 1. การตั้งค่า I2C Multiplexer (TCA9548A)
// ==========================================
#define TCAADDR 0x70
//                              J1  J2  J3  J4  J5  J6
const uint8_t portMap[6]    = { 0,  1,  6,  5,  7,  4 }; // J6 ไม่มีเซ็นเซอร์ (ใส่ 0 ไว้เฉยๆ)
const bool hasSensor[6]     = { true, true, true, true, true, false }; // J6 = false

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// ==========================================
// 🟢 2. ตั้งค่าขา Pin ของสเต็ปมอเตอร์ (STEP, DIR, ENA)
// ==========================================
const int STEP_PINS[6] = {  2,      4,      6,      8,         10,        12  };
const int DIR_PINS[6]  = {  3,      5,      7,      9,         11,        13  };
const int ENA_PINS[6]  = { 30,     31,     32,     33,         34,        35  };

AccelStepper motors[6] = {
  AccelStepper(1, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(1, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(1, STEP_PINS[2], DIR_PINS[2]),
  AccelStepper(1, STEP_PINS[3], DIR_PINS[3]),
  AccelStepper(1, STEP_PINS[4], DIR_PINS[4]),
  AccelStepper(1, STEP_PINS[5], DIR_PINS[5])
};

MultiStepper steppers;
long targetPositions[6];

// ==========================================
// 🟢 3. พารามิเตอร์กลไก
// ==========================================
//                           J1     J2     J3     J4       J5         J6
float gearRatios[6]  = {   40.0,  50.0,  50.0,  44.8,   9.837112,  20.0  };
float pulsePerRev[6] = {  800.0, 800.0, 800.0, 800.0,   1600.0,   800.0  };

float stepsPerDegree[6];    // steps ต่อ 1 องศาข้อต่อ (สำหรับสั่งมอเตอร์)
float stepsPerSensorDeg[6]; // steps ต่อ 1 องศาที่เซ็นเซอร์อ่านได้

// 📌 เซ็นเซอร์ AS5600 ติดที่เพลา Stepper Motor ทุกแกน
// sensorRatio = gearRatio (เพราะมอเตอร์หมุน gearRatio รอบ ต่อข้อต่อ 1 รอบ)
//                           J1     J2     J3     J4       J5         J6
float sensorRatio[6]  = {  40.0,  50.0,  50.0,  44.8,   9.837112,  20.0  };

// 📌 ทิศทางเซ็นเซอร์: +1.0 = ทิศปกติ, -1.0 = กลับทิศ
// ถ้าปล่อย Free แล้วโมเดลขยับสวนทางกับของจริง → เปลี่ยนเป็น -1.0
//                           J1    J2    J3     J4    J5    J6
float sensorDir[6]    = {  1.0,  1.0, -1.0,   1.0,  1.0,  1.0  };

// ==========================================
// 🟢 4. ระบบล้าง EEPROM อัตโนมัติ (ครั้งเดียวหลังอัปโหลดโค้ดใหม่)
// ==========================================
// 📌 เปลี่ยนเลข FIRMWARE_VERSION ทุกครั้งที่แก้สูตรคำนวณ เพื่อบังคับล้าง EEPROM
#define FIRMWARE_VERSION 3
#define EEPROM_VERSION_ADDR 500  // ที่อยู่สำหรับเก็บเวอร์ชัน (ห่างจาก savedState)

// ==========================================
// 🟢 5. ระบบตัวแปรสถานะ
// ==========================================
struct SavedData {
  float homeOffset;
  long lastSteps;
};
SavedData savedState[6];

int minAngle = -360;
int maxAngle = 360;
bool isMoving = false;
bool isFree[6] = {false, false, false, false, false, false};

String inputBuffer = "";
unsigned long lastLiveTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(25000, true);
  delay(500);

  Serial.println("\n=============================================");
  Serial.println("  🤖 AR4 Robotic Arm: Pro Controller v2");
  Serial.println("=============================================\n");

  // 🟢 เช็คว่าเวอร์ชันใน EEPROM ตรงกับเฟิร์มแวร์ปัจจุบันหรือไม่
  int savedVersion = 0;
  EEPROM.get(EEPROM_VERSION_ADDR, savedVersion);

  if (savedVersion != FIRMWARE_VERSION) {
    Serial.println("⚠️ ตรวจพบเฟิร์มแวร์ใหม่! ล้าง EEPROM อัตโนมัติ...");
    for (int i = 0; i < 6; i++) {
      savedState[i].homeOffset = 0.0;
      savedState[i].lastSteps = 0;
    }
    EEPROM.put(0, savedState);
    EEPROM.put(EEPROM_VERSION_ADDR, FIRMWARE_VERSION);
    Serial.println("✅ EEPROM ถูกรีเซ็ต → กรุณากด HOME เพื่อตั้งจุดอ้างอิงใหม่");
  } else {
    EEPROM.get(0, savedState);
  }

  for (int i = 0; i < 6; i++) {
    pinMode(ENA_PINS[i], OUTPUT);
    digitalWrite(ENA_PINS[i], LOW);

    stepsPerDegree[i] = (pulsePerRev[i] * gearRatios[i]) / 360.0;
    stepsPerSensorDeg[i] = stepsPerDegree[i] / sensorRatio[i];

    motors[i].setPinsInverted(false, false, false);
    motors[i].setMaxSpeed(1500);
    steppers.addStepper(motors[i]);

    if (isnan(savedState[i].homeOffset) || savedState[i].homeOffset < 0 || savedState[i].homeOffset > 360) {
      savedState[i].homeOffset = 0.0;
      savedState[i].lastSteps = 0;
    }

    // กู้คืนตำแหน่งเมื่อเปิดเครื่อง
    Serial.print("J"); Serial.print(i+1); Serial.print(": ");

    // 🟢 คูณ sensorDir ตอนแปลง steps → องศาเซ็นเซอร์
    float expectedSensorDeg = savedState[i].lastSteps / stepsPerSensorDeg[i] * sensorDir[i];
    // ใช้ while loop แทน fmod เพื่อความเข้ากันได้กับทุกบอร์ด
    float wrappedDeg = expectedSensorDeg;
    while (wrappedDeg >= 360.0) wrappedDeg -= 360.0;
    while (wrappedDeg < 0.0) wrappedDeg += 360.0;
    float expectedRaw = savedState[i].homeOffset + wrappedDeg;
    while (expectedRaw >= 360.0) expectedRaw -= 360.0;
    while (expectedRaw < 0.0) expectedRaw += 360.0;

    float currentRaw = -1.0;
    if (hasSensor[i]) {
      currentRaw = getRawAngle(i);
      int retry = 0;
      while (currentRaw < 0 && retry < 5) {
        delay(50);
        currentRaw = getRawAngle(i);
        retry++;
      }
    }

    if (currentRaw < 0) {
      currentRaw = expectedRaw;
      Serial.print(hasSensor[i] ? "[No Sensor!] " : "[No Sensor Config] ");
    }

    float diff = currentRaw - expectedRaw;
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    // 🟢 คูณ sensorDir ตอนแปลง องศาเซ็นเซอร์ → steps
    long actualSteps = savedState[i].lastSteps + (long)(diff * stepsPerSensorDeg[i] * sensorDir[i]);
    motors[i].setCurrentPosition(actualSteps);
    targetPositions[i] = actualSteps;

    Serial.print("sensor="); Serial.print(currentRaw, 1);
    Serial.print(" expected="); Serial.print(expectedRaw, 1);
    Serial.print(" diff="); Serial.print(diff, 1);
    Serial.print(" steps="); Serial.print(actualSteps);
    Serial.print(" spsd="); Serial.print(stepsPerSensorDeg[i], 3);
    Serial.println(" OK");
  }
  cdsLinkBegin();   // 🔵 [CODESYS] เปิด Serial1 (ขา 18/19) สำหรับ Modbus RTU
  Serial.println("\n✅ System Ready! (กรุณากด HOME ถ้าเป็นการอัปโหลดครั้งแรก)");
}

void loop() {
  cdsLinkPoll();    // 🔵 [CODESYS] รับคำสั่งจาก CODESYS และตรวจ watchdog (ไม่หยุดรอ)

  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      processCommand(inputBuffer);
      inputBuffer = ""; 
    } else {
      inputBuffer += inChar;
    }
  }

  for(int i = 0; i < 6; i++) {
    if(isFree[i]) {
      syncMotorToSensor(i);
    }
  }

  if (steppers.run()) {
    isMoving = true;
  } 
  else if (isMoving) {
    for (int i = 0; i < 6; i++) {
      savedState[i].lastSteps = motors[i].currentPosition();
    }
    isMoving = false;
  }

  unsigned long now = millis();
  if (now - lastLiveTime >= 50) { 
    lastLiveTime = now;
    Serial.print("LIVE:");
    for (int i = 0; i < 6; i++) {
      long liveAngle = round(motors[i].currentPosition() / stepsPerDegree[i]);
      Serial.print(liveAngle);
      if (i < 5) Serial.print(",");
    }
    Serial.println();
  }
}

// ==========================================
// 🧠 ซิงค์ตำแหน่งจากเซ็นเซอร์
// ==========================================
void syncMotorToSensor(int i) {
  if (!hasSensor[i]) return;
  float currentRaw = getRawAngle(i);
  if (currentRaw < 0) return;
  
  // 🟢 คูณ sensorDir เพื่อแปลง steps → องศาเซ็นเซอร์ ให้ตรงทิศ
  float currentSensorDeg = motors[i].currentPosition() / stepsPerSensorDeg[i] * sensorDir[i];
  
  // ใช้ while loop แทน fmod
  float wrappedDeg = currentSensorDeg;
  if (wrappedDeg > 36000.0 || wrappedDeg < -36000.0) {
    wrappedDeg = wrappedDeg - ((long)(wrappedDeg / 360.0)) * 360.0;
  }
  while(wrappedDeg >= 360.0) wrappedDeg -= 360.0;
  while(wrappedDeg < 0.0) wrappedDeg += 360.0;
  
  float expectedRaw = savedState[i].homeOffset + wrappedDeg;
  while(expectedRaw >= 360.0) expectedRaw -= 360.0;
  while(expectedRaw < 0.0) expectedRaw += 360.0;
  
  float deltaRaw = currentRaw - expectedRaw;
  while(deltaRaw > 180.0) deltaRaw -= 360.0;
  while(deltaRaw < -180.0) deltaRaw += 360.0;
  
  // 🟢 คูณ sensorDir เพื่อแปลง องศาเซ็นเซอร์ → steps ให้ตรงทิศ
  long newSteps = motors[i].currentPosition() + round(deltaRaw * stepsPerSensorDeg[i] * sensorDir[i]);
  
  motors[i].setCurrentPosition(newSteps);
  targetPositions[i] = newSteps; 
}

// ==========================================
// 🧠 อ่านค่า AS5600 
// ==========================================
float getRawAngle(uint8_t jointIndex) {
  if (jointIndex > 5) return -1.0;
  
  uint8_t targetPort = portMap[jointIndex]; 
  tcaselect(targetPort);
  delayMicroseconds(1500); // 🟢 เพิ่มเวลารอให้ MUX สลับเสร็จสมบูรณ์
  
  Wire.beginTransmission(0x36);
  Wire.write(0x0E); 
  if (Wire.endTransmission() != 0) return -1.0; // 🟢 เช็ค I2C error
  Wire.requestFrom(0x36, 2);

  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int rawAngle = (highByte << 8) | lowByte;
    return rawAngle * (360.0 / 4096.0);
  }
  return -1.0; 
}

// ==========================================
// 🧠 ระบบประมวลผลคำสั่ง Serial
// ==========================================
void processCommand(String input) {
  input.trim();
  input.toUpperCase(); 
  if (input.length() == 0) return;

  // 🔵 [CODESYS] สิทธิ์ควบคุม
  if (cdsHandleOwnerCommand(input)) return;   // REQ / REL / WHO
  if (cdsBlocksPcCommand(input)) return;      // CODESYS คุมอยู่: ไม่รับคำสั่งที่ทำให้แขนขยับ

  if (input == "SAVE") {
    for(int i = 0; i < 6; i++) {
      savedState[i].lastSteps = motors[i].currentPosition();
    }
    EEPROM.put(0, savedState);
    Serial.println("✅ [MEMORY] Position saved to EEPROM!");
  }
  // 🟢 คำสั่ง DEBUG: แสดงค่าเซ็นเซอร์ทุกตัวแบบ Real-time
  else if (input == "DEBUG") {
    Serial.println("\n===== DEBUG: Sensor Readings =====");
    for(int i = 0; i < 6; i++) {
      float raw = getRawAngle(i);
      Serial.print("J"); Serial.print(i+1); Serial.print(": ");
      if (raw < 0) {
        Serial.print("❌ FAIL (I2C error, port=");
        Serial.print(portMap[i]);
        Serial.println(")");
      } else {
        Serial.print("✅ "); Serial.print(raw, 2); Serial.print("°");
        Serial.print("  steps="); Serial.print(motors[i].currentPosition());
        Serial.print("  joint="); Serial.print(motors[i].currentPosition() / stepsPerDegree[i], 2);
        Serial.print("°  home="); Serial.print(savedState[i].homeOffset, 2);
        Serial.print("°  spsd="); Serial.println(stepsPerSensorDeg[i], 3);
      }
    }
    Serial.println("==================================\n");
  }
  else if (input == "FREE_ALL") {
    for(int i=0; i<6; i++) {
      digitalWrite(ENA_PINS[i], HIGH);
      isFree[i] = true;
    }
    Serial.println("🟢 All joints FREE");
  } 
  else if (input == "LOCK_ALL") {
    for(int i=0; i<6; i++) {
      digitalWrite(ENA_PINS[i], LOW);
      isFree[i] = false;
      targetPositions[i] = motors[i].currentPosition(); 
    }
    steppers.moveTo(targetPositions);
    Serial.println("🔴 All joints LOCKED");
  } 
  else if (input.startsWith("FREE")) {
    int axis = input.substring(4).toInt() - 1; 
    if(axis >= 0 && axis < 6) {
      digitalWrite(ENA_PINS[axis], HIGH);
      isFree[axis] = true;
      Serial.print("🟢 J"); Serial.print(axis+1); Serial.println(" FREE");
    }
  } 
  else if (input.startsWith("LOCK")) {
    int axis = input.substring(4).toInt() - 1; 
    if(axis >= 0 && axis < 6) {
      digitalWrite(ENA_PINS[axis], LOW);
      isFree[axis] = false;
      targetPositions[axis] = motors[axis].currentPosition();
      steppers.moveTo(targetPositions);
      Serial.print("🔴 J"); Serial.print(axis+1); Serial.println(" LOCKED");
    }
  }
  else if (input == "HOME") {
    setNewHome();
  } 
  else if (input == "STOP") {
    // 🔵 [CODESYS] ของเดิม 4 บรรทัดนี้หยุดแขนกลางทางไม่ได้ เพราะ steppers.moveTo() ไม่ทำอะไรเมื่อระยะทุกแกนเป็น 0
    // for(int i = 0; i < 6; i++) {
    //   targetPositions[i] = motors[i].currentPosition();
    // }
    // steppers.moveTo(targetPositions); 
    cdsStopAll();
    Serial.println("🛑 Program Stopped!");
  }
  else if (input.startsWith("S")) {
    int speedPercent = input.substring(1).toInt();
    if (speedPercent < 1) speedPercent = 1;
    if (speedPercent > 100) speedPercent = 100;
    
    long mappedSpeed = map(speedPercent, 1, 100, 100, 3000);
    for (int i = 0; i < 6; i++) {
      motors[i].setMaxSpeed((float)mappedSpeed);
    }
  }
  else if (input.startsWith("A")) {
    String temp = input.substring(1); 
    for (int i = 0; i < 6; i++) {
      int commaIndex = temp.indexOf(',');
      String valStr = (commaIndex != -1) ? temp.substring(0, commaIndex) : temp;
      
      long targetAngle = valStr.toInt(); 
      if (targetAngle >= minAngle && targetAngle <= maxAngle) {
        targetPositions[i] = round(targetAngle * stepsPerDegree[i]); 
      }
      
      if (commaIndex == -1) break; 
      temp = temp.substring(commaIndex + 1);
    }
    steppers.moveTo(targetPositions); 
  }
}

void setNewHome() {
  Serial.println("\n🏠 Setting new HOME position...");
  for (int i = 0; i < 6; i++) {
    float rawNow = getRawAngle(i); 
    Serial.print("J"); Serial.print(i+1); Serial.print(": ");
    if (rawNow >= 0.0) {
      savedState[i].homeOffset = rawNow;         
      savedState[i].lastSteps = 0;
      motors[i].setCurrentPosition(0); 
      targetPositions[i] = 0; 
      Serial.print("offset="); Serial.print(rawNow, 2); Serial.println("° ✅");
    } else {
      Serial.println("❌ Sensor not found!");
    }
  }
  EEPROM.put(0, savedState); 
  Serial.println("✅ [HOME] All Joints Zeroed!\n");
}
