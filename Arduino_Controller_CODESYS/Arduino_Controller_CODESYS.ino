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
const uint8_t portMap[6]    = { 0,  1,  6,  7,  5,  4 }; // J6 ไม่มีเซ็นเซอร์ (ใส่ค่าไว้เฉยๆ)
// J4 = พอร์ต 7, J5 = พอร์ต 5 (เดิมตั้งสลับกัน: ผลตรวจ 30 รอบ 12 พ.ย. เซ็นเซอร์ J4 อ่าน -8.04 ขณะสั่ง -7 และ J5 อ่าน +4.70 ขณะสั่ง 0
//  ตรงกับที่คำนวณไว้เมื่อ J4 อ่านเพลามอเตอร์ J5 และ J5 อ่านเพลามอเตอร์ J4 ทุกค่า)
const bool hasSensor[6]     = { true, true, true, true, true, false }; // J6 = false
// register ที่ใช้อ่านมุมของแต่ละแกน: 0x0E = ANGLE, 0x0C = RAW ANGLE (ค่าก่อนผ่านการตั้งช่วงมุมภายในชิป)
const uint8_t angleReg[6]   = { 0x0C, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E };

uint8_t angleRegForPort(uint8_t p) {
  for (int i = 0; i < 6; i++) if (hasSensor[i] && portMap[i] == p) return angleReg[i];
  return 0x0E;
}

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
float sensorDir[6]    = {  1.0,  1.0, -1.0,   1.0, -1.0,  1.0  };
// J5 = -1.0: ผลค้นหาพอร์ตและทิศเซ็นเซอร์ 12 พ.ย. เซ็นเซอร์พอร์ต 5 หมุนสวนทางกับ step ของ J5

// ==========================================
// 🟢 4. ระบบล้าง EEPROM อัตโนมัติ (ครั้งเดียวหลังอัปโหลดโค้ดใหม่)
// ==========================================
// 📌 เปลี่ยนเลข FIRMWARE_VERSION ทุกครั้งที่แก้สูตรคำนวณ เพื่อบังคับล้าง EEPROM
#define FIRMWARE_VERSION 3
#define EEPROM_VERSION_ADDR 500  // ที่อยู่สำหรับเก็บเวอร์ชัน (ห่างจาก savedState)
// ค่า homeOffset ของ J4/J5 ที่บันทึกตอนพอร์ตยังสลับกันอยู่ เป็นมุมของเซ็นเซอร์อีกแกน จึงสลับค่าให้ครั้งเดียวหลังอัปโหลด
// (ไม่ต้อง Set Home ใหม่ และไม่ล้างตำแหน่งที่บันทึกไว้) เครื่องหมายว่าสลับแล้วเก็บที่ EEPROM_PORTFIX_ADDR
#define EEPROM_PORTFIX_ADDR 510
#define PORTFIX_DONE        0x45
// บูตครั้งแรกหลังเปลี่ยน register ตาม angleReg: ตั้ง homeOffset ของ J1 จากค่าที่อ่านได้ ณ ตำแหน่งที่บันทึกไว้ (ค่าเดิมอ่านจาก ANGLE)
#define EEPROM_REGSEL_ADDR  511
#define REGSEL_DONE         0x52

// ==========================================
// 🟢 5. ระบบตัวแปรสถานะ
// ==========================================
struct SavedData {
  float homeOffset;
  long lastSteps;
};
SavedData savedState[6]; 

// 🟢 ขอบเขตมุมของแต่ละข้อต่อ (องศา) ให้ตรงกับตารางขอบเขตการเคลื่อนที่ในเล่ม
//    J1 ±75, J2 ±60, J3 ±60, J4 ±180, J5 ±180, J6 ±180
//    ใช้ตรวจทั้งคำสั่ง A จาก PC และคำสั่ง MOVE จาก CODESYS (ต้องตรงกับ JOINT_LIMITS ใน index.html)
const float jointMinDeg[6] = { -75.0, -60.0, -60.0, -180.0, -180.0, -180.0 };
const float jointMaxDeg[6] = {  75.0,  60.0,  60.0,  180.0,  180.0,  180.0 };
bool isMoving = false;
float userMaxSpeed = 1500;                 // ความเร็วที่ผู้ใช้ตั้งด้วยคำสั่ง S (steps/s) ใช้กับคำสั่ง A ปกติ
const float MAX_STEP_RATE = 3000.0;        // ความเร็วสูงสุดที่ยอมให้ (เท่าความเร็ว 100%)
bool isFree[6] = {false, false, false, false, false, false};

String inputBuffer = "";
unsigned long lastLiveTime = 0;
// 🟣 [LOG] TGT:<step เป้าหมาย J1..J6> ส่งเมื่อเป้าหมายของบอร์ดเปลี่ยน และส่งซ้ำทุก 1 วินาที ไม่ว่าคำสั่งมาจาก PC หรือ CODESYS
//   ให้โปรแกรมบนคอมพิวเตอร์บันทึกเป้าหมายที่บอร์ดกำลังทำจริง (เดิมโหมด CODESYS บันทึกมุมจริงลงช่องเป้าหมาย)
unsigned long lastTgtCheck = 0, lastTgtSent = 0;
long tgtSent[6] = { 0, 0, 0, 0, 0, 0 };
bool tgtEverSent = false;

// ==========================================
// 🟣 6. ตรวจตำแหน่งด้วยเซ็นเซอร์ AS5600 (Sensor Check)
// ==========================================
// หลังแขนหยุดนิ่งทุกครั้ง อ่านเซ็นเซอร์ทุกแกนแล้วส่งบรรทัด SENS: ไปที่ PC
//   SENS:<มุมจาก step J1..J6>;<มุมจากเซ็นเซอร์ J1..J6>   (องศาข้อต่อ ทศนิยม 3 ตำแหน่ง, NA = ไม่มีเซ็นเซอร์/อ่านไม่ได้)
//   ตามด้วย SENX:<มุมดิบ AS5600>;<ผลต่างที่เพลามอเตอร์>;<step จริง>;<step เป้าหมาย> (ดู senCheckAll)
// ความคลาดเคลื่อน = มุมจากเซ็นเซอร์ - มุมจาก step
// ⚠️ เซ็นเซอร์ติดที่เพลามอเตอร์ จึงแยกแยะได้เฉพาะความต่างไม่เกินครึ่งรอบมอเตอร์ (= 180/gearRatio องศาที่ข้อต่อ)
// ⚠️ ไม่อ่านระหว่างเคลื่อนที่ (ค่าเริ่มต้น) เพราะการอ่าน 1 แกนใช้เวลาราว 2 ms ทำให้พัลส์สะดุด
//    คำสั่ง CHKMOVE1 เปิดการอ่านระหว่างเคลื่อนที่ทีละแกนทุก 25 ms (แขนจะช้าลงเล็กน้อย), CHKMOVE0 ปิด
//    คำสั่ง CHK อ่านทุกแกนทันที (ใช้ตอนแขนหยุด)
const unsigned long SEN_SETTLE_MS      = 200;   // รอให้แขนนิ่งก่อนอ่าน
const unsigned long SEN_MOVE_PERIOD_MS = 25;
bool senPending = false;
unsigned long senDueTime = 0;
bool senDuringMove = false;
uint8_t senNextAxis = 0;
unsigned long senLastMoveRead = 0;

// ==========================================
// 7. บันทึกตำแหน่งลง EEPROM อัตโนมัติ (Auto-save)
// ==========================================
// การเปิดพอร์ตจาก PC ทำให้บอร์ดรีเซ็ต และ setup() กู้ตำแหน่งจาก lastSteps ใน EEPROM
// จึงบันทึก lastSteps ให้เองเมื่อแขนหยุดนิ่งครบ AUTOSAVE_IDLE_MS และตำแหน่งต่างจากที่บันทึกไว้
// - เขียนทีละแกนต่อรอบ loop (ไม่เกินราว 13 ms) เพื่อไม่ให้ Modbus/คำสั่งสะดุด
// - EEPROM.put เขียนเฉพาะไบต์ที่เปลี่ยน จึงไม่สึกหรอเมื่อค่าเดิม
// - แกนที่ปลดล็อก (FREE) ใช้ช่วงยอม 0.2 องศา กันการเขียนซ้ำจากสัญญาณรบกวนของเซ็นเซอร์
const unsigned long AUTOSAVE_IDLE_MS = 1000;
const float FREE_SAVE_TOL_DEG = 0.2;
unsigned long lastActivityMs = 0;
long eepromSteps[6];       // ค่า lastSteps ที่อยู่ใน EEPROM ขณะนี้ (แต่ละแกน)
long freeRefSteps[6];      // ตำแหน่งอ้างอิงสำหรับตรวจว่ามือยังขยับแกนที่ FREE อยู่หรือไม่
uint8_t autoSaveNext = 0;

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
    if (EEPROM.read(EEPROM_PORTFIX_ADDR) != PORTFIX_DONE) {
      float t = savedState[3].homeOffset;            // J4 <-> J5
      savedState[3].homeOffset = savedState[4].homeOffset;
      savedState[4].homeOffset = t;
      EEPROM.put(0, savedState);
      Serial.println("[PORTFIX] swapped J4/J5 home offsets to match the corrected sensor ports");
    }
  }
  EEPROM.update(EEPROM_PORTFIX_ADDR, PORTFIX_DONE);   // update = เขียนเฉพาะเมื่อค่าต่าง (ไม่สึก)
  bool regSelNew = (EEPROM.read(EEPROM_REGSEL_ADDR) != REGSEL_DONE);
  bool regSelSet = false;

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
      if (regSelNew && angleReg[i] != 0x0E && currentRaw >= 0) {
        savedState[i].homeOffset = wrap360(currentRaw - wrappedDeg);
        expectedRaw = currentRaw;
        EEPROM.put(0, savedState);
        regSelSet = true;
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
    eepromSteps[i] = savedState[i].lastSteps;   // ค่าที่อยู่ใน EEPROM (ถ้าเซ็นเซอร์ปรับตำแหน่ง ระบบจะบันทึกค่าใหม่ให้เองหลังนิ่ง 1 วินาที)
    freeRefSteps[i] = actualSteps;

    Serial.print("sensor="); Serial.print(currentRaw, 1);
    Serial.print(" expected="); Serial.print(expectedRaw, 1);
    Serial.print(" diff="); Serial.print(diff, 1);
    Serial.print(" steps="); Serial.print(actualSteps);
    Serial.print(" spsd="); Serial.print(stepsPerSensorDeg[i], 3);
    Serial.println(" OK");
  }
  if (regSelNew && regSelSet) EEPROM.update(EEPROM_REGSEL_ADDR, REGSEL_DONE);
  lastActivityMs = millis();
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
    senPending = true;                              // 🟣 [SENSOR] นัดตรวจหลังแขนนิ่ง
    senDueTime = millis() + SEN_SETTLE_MS;
  }

  autoSavePosition();   // บันทึกตำแหน่งลง EEPROM เมื่อแขนนิ่งครบ 1 วินาที

  // 🟣 [SENSOR] ตรวจทุกแกนเมื่อแขนหยุดนิ่งครบเวลา (ข้ามถ้ามีแกนที่ FREE อยู่)
  if (!isMoving && senPending && (long)(millis() - senDueTime) >= 0) {
    senPending = false;
    if (!anyFree()) senCheckAll();
  }

  // 🟣 [SENSOR] อ่านระหว่างเคลื่อนที่ทีละแกน (เปิดด้วย CHKMOVE1)
  if (isMoving && senDuringMove && millis() - senLastMoveRead >= SEN_MOVE_PERIOD_MS) {
    senLastMoveRead = millis();
    for (uint8_t n = 0; n < 6; n++) {             // หาแกนถัดไปที่มีเซ็นเซอร์
      senNextAxis = (senNextAxis + 1) % 6;
      if (hasSensor[senNextAxis]) break;
    }
    float d;
    if (readSensorJointDeg(senNextAxis, d)) {
      // SEN1:<แกน 1-6>,<มุมจาก step>,<มุมจากเซ็นเซอร์>
      Serial.print("SEN1:"); Serial.print(senNextAxis + 1); Serial.print(",");
      Serial.print(motors[senNextAxis].currentPosition() / stepsPerDegree[senNextAxis], 2); Serial.print(",");
      Serial.println(d, 2);
    }
  }

  unsigned long now = millis();
  if (now - lastLiveTime >= 50) { 
    lastLiveTime = now;
    // 🟣 [LOG] LIVS:<J1..J6> ตัวนับ step จริงของแต่ละแกน (จำนวนเต็ม) ให้โปรแกรมบนคอมพิวเตอร์แปลงเป็นองศาเอง
    //   แทน LIVE:<องศา ทศนิยม 2 ตำแหน่ง> เดิม ซึ่งปัดค่า (1 step ของ J2/J3 = 0.009°) จนคำนวณจำนวน step ย้อนกลับได้คลาด ±1
    //   บรรทัดสั้นกว่าเดิม จึงไม่เพิ่มเวลาที่ใช้ส่ง Serial
    Serial.print("LIVS:");
    for (int i = 0; i < 6; i++) {
      Serial.print(motors[i].currentPosition());
      if (i < 5) Serial.print(",");
    }
    Serial.println();
  }

  // 🟣 [LOG] ตรวจเป้าหมายทุก 50 ms โดยเว้นห่างจาก LIVS อย่างน้อย 25 ms ไม่ให้สองบรรทัดส่งพร้อมกันจนบัฟเฟอร์ Serial เต็ม
  if (now - lastTgtCheck >= 50 && now - lastLiveTime >= 25) {
    lastTgtCheck = now;
    bool changed = !tgtEverSent;
    for (int i = 0; i < 6; i++) if (motors[i].targetPosition() != tgtSent[i]) changed = true;
    if (changed || now - lastTgtSent >= 1000) {
      lastTgtSent = now;
      tgtEverSent = true;
      Serial.print("TGT:");
      for (int i = 0; i < 6; i++) {
        tgtSent[i] = motors[i].targetPosition();
        Serial.print(tgtSent[i]);
        if (i < 5) Serial.print(",");
      }
      Serial.println();
    }
  }
}

// ==========================================
// บันทึกตำแหน่งลง EEPROM อัตโนมัติ
// ==========================================
void autoSavePosition() {
  unsigned long now = millis();
  if (isMoving) { lastActivityMs = now; return; }
  // แกนที่ FREE: ถ้ามือยังขยับอยู่ (เกิน 0.2 องศา) ให้นับว่ายังไม่นิ่ง
  for (int i = 0; i < 6; i++) {
    if (!isFree[i]) continue;
    long p = motors[i].currentPosition();
    if (labs(p - freeRefSteps[i]) > (long)(stepsPerDegree[i] * FREE_SAVE_TOL_DEG)) {
      freeRefSteps[i] = p;
      lastActivityMs = now;
    }
  }
  if (now - lastActivityMs < AUTOSAVE_IDLE_MS) return;
  for (uint8_t n = 0; n < 6; n++) {
    uint8_t i = (autoSaveNext + n) % 6;
    long p = motors[i].currentPosition();
    long tol = isFree[i] ? (long)(stepsPerDegree[i] * FREE_SAVE_TOL_DEG) : 0;
    if (labs(p - eepromSteps[i]) > tol) {
      savedState[i].lastSteps = p;
      EEPROM.put(i * sizeof(SavedData), savedState[i]);   // เขียนเฉพาะแกนนี้ (ตำแหน่งเดียวกับใน savedState[6])
      eepromSteps[i] = p;
      autoSaveNext = (i + 1) % 6;
      return;                                             // รอบละ 1 แกน
    }
  }
}

// ==========================================
// 🧠 ซิงค์ตำแหน่งจากเซ็นเซอร์
// ==========================================
unsigned long freeSenFailMs[6] = {0, 0, 0, 0, 0, 0};

void syncMotorToSensor(int i) {
  if (!hasSensor[i]) return;
  float currentRaw = getRawAngle(i);
  if (currentRaw < 0) {
    // แกนนี้ปลด Free อยู่แต่อ่านเซ็นเซอร์ไม่ได้: ตัวนับ step จึงไม่เปลี่ยนตามมือ แจ้ง PC ไม่เกินวินาทีละครั้ง
    // FREESEN:<แกน 1-6>,<พอร์ต MUX>
    if (millis() - freeSenFailMs[i] >= 1000) {
      freeSenFailMs[i] = millis();
      Serial.print("FREESEN:"); Serial.print(i + 1); Serial.print(","); Serial.println(portMap[i]);
    }
    return;
  }
  
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
// 🟣 แปลงค่าเซ็นเซอร์เป็นมุมข้อต่อ (องศา)
// ==========================================
// ใช้หลักเดียวกับ syncMotorToSensor(): จำนวน step บอกว่าอยู่รอบที่เท่าไรของมอเตอร์
// ส่วนเซ็นเซอร์บอกมุมละเอียดภายในรอบนั้น แล้วแปลงกลับเป็นองศาข้อต่อ
// คืนค่า false ถ้าแกนนี้ไม่มีเซ็นเซอร์หรืออ่านไม่ได้ (ไม่แก้ตัวนับ step)
float wrap360(float d) {
  if (d > 36000.0 || d < -36000.0) d = d - ((long)(d / 360.0)) * 360.0;
  while (d >= 360.0) d -= 360.0;
  while (d < 0.0) d += 360.0;
  return d;
}

// 🟣 [LOG] แบบละเอียด: คืนมุมดิบที่ AS5600 อ่านได้ (0-360° ที่เพลา) และผลต่างจากมุมดิบที่ควรอ่านได้ตามจำนวน step (±180°)
//   ผลต่างใกล้ 0 = เซ็นเซอร์สอดคล้องกับตัวนับ step / ผลต่างมาก = เซ็นเซอร์ไม่หมุนตาม พอร์ตหรือทิศผิด หรือสเต็ปหาย
bool readSensorDetail(int i, float &jointDeg, float &rawDeg, float &residDeg) {
  if (!hasSensor[i]) return false;
  float currentRaw = getRawAngle(i);
  if (currentRaw < 0) return false;
  long steps = motors[i].currentPosition();
  float expectedRaw = wrap360(savedState[i].homeOffset + wrap360(steps / stepsPerSensorDeg[i] * sensorDir[i]));
  float deltaRaw = currentRaw - expectedRaw;
  while (deltaRaw > 180.0) deltaRaw -= 360.0;
  while (deltaRaw < -180.0) deltaRaw += 360.0;
  float stepsFromSensor = steps + deltaRaw * stepsPerSensorDeg[i] * sensorDir[i];
  jointDeg = stepsFromSensor / stepsPerDegree[i];
  rawDeg = currentRaw;
  residDeg = deltaRaw;
  return true;
}

bool readSensorJointDeg(int i, float &jointDeg) {
  float rawDeg, residDeg;
  return readSensorDetail(i, jointDeg, rawDeg, residDeg);
}

bool anyFree() {
  for (int i = 0; i < 6; i++) if (isFree[i]) return true;
  return false;
}

// อ่านทุกแกนแล้วส่ง SENS:<มุมจาก step J1..J6>;<มุมจากเซ็นเซอร์ J1..J6>
void senCheckAll() {
  float senDeg[6], rawDeg[6], resDeg[6]; bool ok[6];
  long stepNow[6], tgtNow[6];
  for (int i = 0; i < 6; i++) {
    stepNow[i] = motors[i].currentPosition();
    tgtNow[i] = motors[i].targetPosition();
    ok[i] = readSensorDetail(i, senDeg[i], rawDeg[i], resDeg[i]);
  }
  // 🟣 [LOG] ทศนิยม 3 ตำแหน่ง (เซ็นเซอร์ละเอียดที่ข้อต่อประมาณ 0.002-0.009°)
  Serial.print("SENS:");
  for (int i = 0; i < 6; i++) {
    Serial.print(stepNow[i] / stepsPerDegree[i], 3);
    if (i < 5) Serial.print(",");
  }
  Serial.print(";");
  for (int i = 0; i < 6; i++) {
    if (ok[i]) Serial.print(senDeg[i], 3); else Serial.print("NA");
    if (i < 5) Serial.print(",");
  }
  Serial.println();
  // 🟣 [LOG] SENX:<มุมดิบ AS5600 J1..J6>;<ผลต่างที่เพลามอเตอร์ J1..J6>;<step จริง J1..J6>;<step เป้าหมาย J1..J6>
  Serial.print("SENX:");
  for (int i = 0; i < 6; i++) {
    if (ok[i]) Serial.print(rawDeg[i], 2); else Serial.print("NA");
    if (i < 5) Serial.print(",");
  }
  Serial.print(";");
  for (int i = 0; i < 6; i++) {
    if (ok[i]) Serial.print(resDeg[i], 2); else Serial.print("NA");
    if (i < 5) Serial.print(",");
  }
  Serial.print(";");
  for (int i = 0; i < 6; i++) {
    Serial.print(stepNow[i]);
    if (i < 5) Serial.print(",");
  }
  Serial.print(";");
  for (int i = 0; i < 6; i++) {
    Serial.print(tgtNow[i]);
    if (i < 5) Serial.print(",");
  }
  Serial.println();
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
  Wire.write(angleReg[jointIndex]); 
  if (Wire.endTransmission() != 0) return -1.0; // 🟢 เช็ค I2C error
  Wire.requestFrom(0x36, 2);

  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int rawAngle = ((highByte & 0x0F) << 8) | lowByte;
    return rawAngle * (360.0 / 4096.0);
  }
  return -1.0; 
}

// ==========================================
// 8. ตรวจการเชื่อมต่อ I2C (คำสั่ง I2C)
// ==========================================
// ตอบกลับ 1 บรรทัด:
//   I2C:<mux>,<timeout>,<ports>;<J1>;<J2>;<J3>;<J4>;<J5>;<J6>
//   <mux>     1 = TCA9548A (0x70) ตอบรับ, 0 = ไม่ตอบ
//   <timeout> 1 = บัส I2C เคยค้างจนหมดเวลา (Wire timeout) ตั้งแต่ตรวจครั้งก่อน
//   <ports>   bitmask ของพอร์ต MUX 0-7 ที่มีอุปกรณ์ตอบที่ 0x36 (AS5600)
//   <Jn>      <พอร์ต>,<ตอบ 1/0>,<STATUS>,<AGC>,<MAGNITUDE>,<ANGLE องศา>   หรือ NA ถ้าแกนนั้นไม่มีเซ็นเซอร์
//             STATUS = รีจิสเตอร์ 0x0B ของ AS5600: bit5 MD (พบแม่เหล็ก), bit4 ML (แม่เหล็กอ่อนไป), bit3 MH (แม่เหล็กแรงไป)
//             AGC = รีจิสเตอร์ 0x1A, MAGNITUDE = 0x1B-0x1C, มุม = รีจิสเตอร์ตาม angleReg (ค่าเดียวกับที่ getRawAngle ใช้)
//             ค่าที่อ่านไม่ได้ส่งเป็น -1
//   ต่อท้ายด้วย ;P:<มุมพอร์ต 0..7> ;D:<sensorDir J1..J6> ;R:<sensorRatio J1..J6>
//             P = มุมของอุปกรณ์ที่ตอบที่ 0x36 ในแต่ละพอร์ต (register ตาม angleReg ของแกนที่ใช้พอร์ตนั้น, -1 = ไม่มี) ใช้หาว่าเซ็นเซอร์ของแต่ละแกนอยู่พอร์ตไหนจริง
// ไม่ทำระหว่างแขนเคลื่อนที่ (ตอบ I2C:BUSY) เพราะอ่านครบทุกพอร์ตใช้เวลาราว 25 ms ทำให้พัลส์สะดุด
#define AS5600_ADDR 0x36

bool as5600ReadBytes(uint8_t reg, uint8_t n, uint8_t *buf) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(AS5600_ADDR, (int)n);
  if (Wire.available() < n) return false;
  for (uint8_t k = 0; k < n; k++) buf[k] = Wire.read();
  return true;
}

void i2cReport() {
  bool timeoutSeen = Wire.getWireTimeoutFlag();
  Wire.clearWireTimeoutFlag();

  // 1) MUX ตอบรับที่ 0x70 หรือไม่
  Wire.beginTransmission(TCAADDR);
  bool muxOk = (Wire.endTransmission() == 0);

  // 2) ไล่ทุกพอร์ตของ MUX หาอุปกรณ์ที่ 0x36 (ช่วยดูว่าสายเสียบผิดพอร์ตหรือไม่)
  uint8_t portMask = 0;
  float portAng[8];
  for (uint8_t p = 0; p < 8; p++) {
    portAng[p] = -1.0;
    tcaselect(p);
    delayMicroseconds(1500);
    Wire.beginTransmission(AS5600_ADDR);
    if (Wire.endTransmission() == 0) {
      portMask |= (1 << p);
      uint8_t b[2];
      if (as5600ReadBytes(angleRegForPort(p), 2, b)) portAng[p] = (((b[0] & 0x0F) << 8) | b[1]) * (360.0 / 4096.0);
    }
  }

  Serial.print("I2C:");
  Serial.print(muxOk ? 1 : 0); Serial.print(",");
  timeoutSeen = timeoutSeen || Wire.getWireTimeoutFlag();
  Serial.print(timeoutSeen ? 1 : 0); Serial.print(",");
  Serial.print(portMask);

  // 3) อ่านสถานะของเซ็นเซอร์แต่ละแกนตาม portMap
  for (int i = 0; i < 6; i++) {
    Serial.print(";");
    if (!hasSensor[i]) { Serial.print("NA"); continue; }
    tcaselect(portMap[i]);
    delayMicroseconds(1500);
    uint8_t b[2];
    int status = -1, agc = -1;
    long mag = -1;
    float ang = -1.0;
    bool ack = as5600ReadBytes(0x0B, 1, b);
    if (ack) {
      status = b[0];
      if (as5600ReadBytes(0x1A, 1, b)) agc = b[0];
      if (as5600ReadBytes(0x1B, 2, b)) mag = ((long)(b[0] & 0x0F) << 8) | b[1];
      if (as5600ReadBytes(angleReg[i], 2, b)) ang = (((b[0] & 0x0F) << 8) | b[1]) * (360.0 / 4096.0);
    }
    Serial.print(portMap[i]); Serial.print(",");
    Serial.print(ack ? 1 : 0); Serial.print(",");
    Serial.print(status); Serial.print(",");
    Serial.print(agc); Serial.print(",");
    Serial.print(mag); Serial.print(",");
    Serial.print(ang, 2);
  }
  Serial.print(";P:");
  for (uint8_t p = 0; p < 8; p++) { Serial.print(portAng[p], 2); if (p < 7) Serial.print(","); }
  Serial.print(";D:");
  for (int i = 0; i < 6; i++) { Serial.print((int)sensorDir[i]); if (i < 5) Serial.print(","); }
  Serial.print(";R:");
  for (int i = 0; i < 6; i++) { Serial.print(sensorRatio[i], 4); if (i < 5) Serial.print(","); }
  Serial.println();
  Wire.clearWireTimeoutFlag();
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
    for (int i = 0; i < 6; i++) eepromSteps[i] = savedState[i].lastSteps;
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
  // 🟣 [SENSOR] ตรวจตำแหน่งด้วยเซ็นเซอร์ (ขึ้นต้นด้วย C จึงไม่ชนกับคำสั่งความเร็ว S และใช้ได้แม้ CODESYS ถือสิทธิ์)
  else if (input == "CHK") {
    if (isMoving) Serial.println("CHK: BUSY");
    else senCheckAll();
  }
  // ตรวจการเชื่อมต่อ I2C (MUX + AS5600 ทุกแกน) ใช้ได้แม้ CODESYS ถือสิทธิ์ เพราะไม่ทำให้แขนขยับ
  else if (input == "I2C") {
    if (isMoving) Serial.println("I2C:BUSY");
    else i2cReport();
  }
  else if (input == "CHKMOVE1") { senDuringMove = true;  Serial.println("CHK: read during motion ON"); }
  else if (input == "CHKMOVE0") { senDuringMove = false; Serial.println("CHK: read during motion OFF"); }
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
    userMaxSpeed = (float)mappedSpeed;
    for (int i = 0; i < 6; i++) {
      motors[i].setMaxSpeed((float)mappedSpeed);
    }
  }
  else if (input.startsWith("A")) {
    // A<J1>,<J2>,<J3>,<J4>,<J5>,<J6>[,T<ms>]   มุมเป็นองศา (ทศนิยมได้)
    //   ไม่มี T: ทุกแกนไปถึงพร้อมกันด้วยความเร็วที่ตั้งด้วยคำสั่ง S (เหมือนเดิม)
    //   มี T  : ทุกแกนไปถึงเป้าหมายพร้อมกันภายใน <ms> มิลลิวินาที (MoveL ส่งช่วงสั้น ๆ ต่อกันจึงเคลื่อนต่อเนื่องไม่หยุด)
    // อ่านมุมให้ครบก่อน แล้วตรวจขอบเขตรายแกน ถ้าเกินแม้แกนเดียวจะไม่ขยับเลย (ตอบ RANGE กลับไปที่ PC)
    String temp = input.substring(1); 
    long newTarget[6];
    for (int i = 0; i < 6; i++) newTarget[i] = targetPositions[i];
    bool inRange = true;
    long segMs = 0;
    for (int i = 0; i < 7; i++) {
      int commaIndex = temp.indexOf(',');
      String valStr = (commaIndex != -1) ? temp.substring(0, commaIndex) : temp;
      if (i == 6) {                                   // ช่องที่ 7 (ถ้ามี) = เวลาที่ต้องไปถึง
        if (valStr.startsWith("T")) segMs = valStr.substring(1).toInt();
        break;
      }
      float targetAngle = valStr.toFloat();   // รับมุมทศนิยมได้ ส่วนจำนวนเต็มแบบเดิมก็ใช้ได้เหมือนเดิม
      if (targetAngle < jointMinDeg[i] || targetAngle > jointMaxDeg[i]) {
        inRange = false;
        Serial.print("RANGE: J"); Serial.print(i + 1); Serial.print(" = "); Serial.print(targetAngle, 1);
        Serial.print(" (limit "); Serial.print(jointMinDeg[i], 0); Serial.print(" to "); Serial.print(jointMaxDeg[i], 0); Serial.println(")");
      } else {
        newTarget[i] = lround(targetAngle * stepsPerDegree[i]); 
      }
      
      if (commaIndex == -1) break; 
      temp = temp.substring(commaIndex + 1);
    }
    if (inRange) {
      float spd = userMaxSpeed;
      if (segMs > 0) {
        long maxDist = 0;
        for (int i = 0; i < 6; i++) {
          long d = labs(newTarget[i] - motors[i].currentPosition());
          if (d > maxDist) maxDist = d;
        }
        spd = (float)maxDist * 1000.0 / (float)segMs;
        if (spd < 1.0) spd = 1.0;
        if (spd > MAX_STEP_RATE) spd = MAX_STEP_RATE;
      }
      for (int i = 0; i < 6; i++) {
        motors[i].setMaxSpeed(spd);                   // MultiStepper ใช้ค่านี้คำนวณเวลา: แกนที่ไกลที่สุดใช้เวลา = ระยะ/spd
        targetPositions[i] = newTarget[i];
      }
      steppers.moveTo(targetPositions); 
    }
  }
}

void setNewHome() {
  // ตั้งท่าปัจจุบันเป็น HOME: ล้างข้อมูลเก่าของทุกแกนแล้วบันทึกชุดใหม่ลง EEPROM ทันที
  // ตอบกลับ PC 1 บรรทัด: HOME:OK  หรือ  HOME:FAIL,<สาเหตุ>  (ถ้าล้มเหลวจะไม่แตะค่าเดิมเลย)
  Serial.println("\n[HOME] Setting new HOME position...");
  if (isMoving) { Serial.println("HOME:FAIL,BUSY"); return; }

  // 1) อ่านเซ็นเซอร์ทุกแกนที่มีเซ็นเซอร์ก่อน (ลองซ้ำได้ 5 ครั้ง) ถ้าแกนใดอ่านไม่ได้ ยกเลิกทั้งหมด
  float rawNow[6];
  String failed = "";
  for (int i = 0; i < 6; i++) {
    rawNow[i] = 0.0;                       // J6 ไม่มีเซ็นเซอร์: ใช้ค่า 0
    if (!hasSensor[i]) continue;
    float r = getRawAngle(i);
    for (int k = 0; r < 0 && k < 5; k++) { delay(50); r = getRawAngle(i); }
    if (r < 0) {
      if (failed.length() > 0) failed += " ";
      failed += "J"; failed += String(i + 1);
    } else {
      rawNow[i] = r;
    }
  }
  if (failed.length() > 0) {
    Serial.print("HOME:FAIL,SENSOR "); Serial.println(failed);
    return;
  }

  // 2) ล้างข้อมูลเก่าทั้งหมด (ทุกแกน ทั้งค่า homeOffset และ lastSteps)
  for (int i = 0; i < 6; i++) {
    savedState[i].homeOffset = 0.0;
    savedState[i].lastSteps = 0;
  }

  // 3) ตั้งค่าใหม่: ท่าปัจจุบัน = 0 องศาทุกแกน
  for (int i = 0; i < 6; i++) {
    savedState[i].homeOffset = rawNow[i];
    motors[i].setCurrentPosition(0);
    targetPositions[i] = 0;
    eepromSteps[i] = 0;
    freeRefSteps[i] = 0;
    Serial.print("J"); Serial.print(i + 1); Serial.print(": ");
    if (hasSensor[i]) { Serial.print("offset="); Serial.println(rawNow[i], 2); }
    else Serial.println("no sensor, set to 0");
  }

  // 4) บันทึกลง EEPROM แล้วอ่านกลับมาตรวจว่าตรงกันทุกไบต์
  EEPROM.put(0, savedState);
  EEPROM.put(EEPROM_VERSION_ADDR, FIRMWARE_VERSION);
  SavedData check[6];
  EEPROM.get(0, check);
  lastActivityMs = millis();
  if (memcmp(check, savedState, sizeof(savedState)) == 0) {
    Serial.println("HOME:OK");
  } else {
    Serial.println("HOME:FAIL,EEPROM");
  }
}
