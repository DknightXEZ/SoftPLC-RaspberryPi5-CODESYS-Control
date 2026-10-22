// =====================================================================
//  CodesysLink.ino — ให้ CODESYS สั่งแขนผ่าน Modbus RTU (Serial1) ได้
//  โดยอินเตอร์เฟสบน PC (USB Serial) ยังใช้ชุดคำสั่งเดิมทั้งหมด
//
//  กติกา
//   - มีผู้ควบคุมครั้งละ 1 ฝั่ง เปิดเครื่องมาเป็นของ PC
//   - ฝั่งไหนก็ขอสิทธิ์ได้ แต่สลับให้เฉพาะตอนแขนหยุดนิ่ง
//   - คำสั่งที่ทำให้แขนขยับรับจากผู้ควบคุมเท่านั้น  STOP รับจากทั้งสองฝั่งเสมอ
//   - CODESYS ถือสิทธิ์แล้วเงียบเกิน CDS_WATCHDOG_MS -> หยุดแขน คืนสิทธิ์ให้ PC
//
//  ตาราง holding register (อ่าน FC03 หรือ FC04, เขียน FC06 หรือ FC16)
//   เขียนจาก CODESYS (ส่งทั้งก้อน 0..9 ด้วย FC16 ในเฟรมเดียว)
//     0      CMD_CODE   0=ไม่ทำอะไร 1=ขอสิทธิ์ 2=คืนสิทธิ์ 3=MOVE 4=STOP 5=FREE 6=LOCK
//     1      CMD_SEQ    เลขลำดับ  ค่าเปลี่ยน = ให้ทำ CMD_CODE หนึ่งครั้ง
//     2..7   TARGET     เป้าหมาย J1..J6 หน่วย 0.1 องศา (INT16)
//     8      SPEED      1..100 %  (ความหมายเดียวกับคำสั่ง S ของ PC)
//     9      AXIS_MASK  bit0..bit5 = J1..J6  แกนที่ MOVE / FREE / LOCK มีผล
//   อ่านอย่างเดียว
//     10     STATUS     bit0=CODESYS คุมอยู่ bit1=กำลังเคลื่อนที่ bit2=มีแกน Free bit3=watchdog เคยตัด
//     11     ACK_SEQ    CMD_SEQ ล่าสุดที่ทำแล้ว
//     12     RESULT     0=OK 1=BUSY(แขนยังวิ่ง) 2=ไม่ได้ถือสิทธิ์ 3=ค่าเกินช่วง 4=แกนนั้น Free อยู่
//                       5=ไม่รู้จักคำสั่ง 6=Arduino เพิ่งรีสตาร์ต คำสั่งที่ค้างไม่ถูกทำ
//     13     TICK       เพิ่มทุก 10 ms  ใช้ดูว่า Arduino ยังทำงาน
//     14..19 ACT        ตำแหน่งปัจจุบัน J1..J6 หน่วย 0.1 องศา (ค่าเดียวกับ LIVE แต่ละเอียดกว่า)
//     20..25 TGT        เป้าหมายที่กำลังวิ่งไป J1..J6 หน่วย 0.1 องศา
//     26     FREE_MASK  bit0..bit5 = แกนที่ Free อยู่
// =====================================================================
#include "CodesysLink.h"

#define CDS_REG_CMD_CODE   0
#define CDS_REG_CMD_SEQ    1
#define CDS_REG_TARGET     2
#define CDS_REG_SPEED      8
#define CDS_REG_AXIS_MASK  9
#define CDS_REG_STATUS     10
#define CDS_REG_ACK_SEQ    11
#define CDS_REG_RESULT     12
#define CDS_REG_TICK       13
#define CDS_REG_ACT        14
#define CDS_REG_TGT        20
#define CDS_REG_FREE_MASK  26
#define CDS_REG_COUNT      27
#define CDS_REG_WRITABLE   10      // เขียนได้เฉพาะ 0..9

#define CDS_CMD_NOP        0
#define CDS_CMD_REQUEST    1
#define CDS_CMD_RELEASE    2
#define CDS_CMD_MOVE       3
#define CDS_CMD_STOP       4
#define CDS_CMD_FREE       5
#define CDS_CMD_LOCK       6

#define CDS_RES_OK         0
#define CDS_RES_BUSY       1
#define CDS_RES_NOT_OWNER  2
#define CDS_RES_RANGE      3
#define CDS_RES_AXIS_FREE  4
#define CDS_RES_BAD_CMD    5
#define CDS_RES_RESTARTED  6

#define CDS_GAP_US         3000UL  // ไบต์ค้างนานกว่านี้โดยเฟรมไม่ครบ = ทิ้ง

static uint16_t cdsReg[CDS_REG_COUNT];
static bool     cdsOwner = false;         // false = PC, true = CODESYS
static bool     cdsWdTripped = false;
static bool     cdsSeqKnown = false;      // ยังไม่เคยได้เฟรมเขียนตั้งแต่เปิดเครื่อง
static uint16_t cdsLastSeq = 0;
static uint32_t cdsLastWriteMs = 0;       // เวลาที่ได้เฟรมเขียนถูกต้องล่าสุดจาก CODESYS
static uint32_t cdsLastBlockedMs = 0;
static uint8_t  cdsRx[64];
static uint8_t  cdsRxLen = 0;
static uint32_t cdsLastByteUs = 0;
static bool     cdsResync = false;        // true = ทิ้งไบต์จนกว่าสายจะเงียบ
static bool     cdsUnknownFc = false;
static uint8_t  cdsTx[64];
static float    cdsDeg10PerStep[6];
static uint8_t  cdsSliceAxis = 0;
static uint32_t cdsLastSliceUs = 0;
static float    cdsPcMaxSpeed[6];         // ความเร็วของฝั่ง PC เก็บไว้คืนตอน PC ได้สิทธิ์กลับ

// ---------------------------------------------------------------------
//  สถานะแขน
// ---------------------------------------------------------------------
bool cdsOwnsArm() { return cdsOwner; }

bool cdsArmIsMoving() {
  if (isMoving) return true;
  for (uint8_t i = 0; i < 6; i++) {
    if (motors[i].distanceToGo() != 0) return true;
  }
  return false;
}

// หยุดทุกแกนทันที: ตั้งเป้าหมายของมอเตอร์แต่ละตัวเป็นตำแหน่งปัจจุบันและความเร็วเป็น 0
// (steppers.moveTo() ใช้หยุดไม่ได้ เพราะมันไม่ทำอะไรเลยเมื่อระยะของทุกแกนเป็น 0)
void cdsStopAll() {
  for (uint8_t i = 0; i < 6; i++) {
    long p = motors[i].currentPosition();
    motors[i].setCurrentPosition(p);
    targetPositions[i] = p;
  }
}

static void cdsMoveTo() {
  bool any = false;
  for (uint8_t i = 0; i < 6; i++) {
    if (targetPositions[i] != motors[i].currentPosition()) any = true;
  }
  if (any) steppers.moveTo(targetPositions);
  else cdsStopAll();
}

// ---------------------------------------------------------------------
//  ผู้ควบคุม
// ---------------------------------------------------------------------
// แจ้งอินเตอร์เฟสบน PC:  OWNER:PC | OWNER:CDS  ตามด้วยเหตุผล (BUSY / BLOCKED / WD) ถ้ามี
static void cdsAnnounce(const char *why) {
  Serial.print("OWNER:");
  Serial.print(cdsOwner ? "CDS" : "PC");
  if (why[0] != 0) { Serial.print(' '); Serial.print(why); }
  Serial.println();
}

static void cdsSetOwner(bool toCds, const char *why) {
  if (toCds && !cdsOwner) {
    for (uint8_t i = 0; i < 6; i++) cdsPcMaxSpeed[i] = motors[i].maxSpeed();
    cdsWdTripped = false;
    cdsLastWriteMs = millis();
  }
  if (!toCds && cdsOwner) {
    for (uint8_t i = 0; i < 6; i++) motors[i].setMaxSpeed(cdsPcMaxSpeed[i]);
  }
  cdsOwner = toCds;
  cdsAnnounce(why);
}

// คำสั่งจากอินเตอร์เฟสบน PC:  REQ = ขอสิทธิ์,  REL = ส่งสิทธิ์ให้ CODESYS,  WHO = ถามว่าใครคุมอยู่
bool cdsHandleOwnerCommand(String input) {
  if (input == "WHO") { cdsAnnounce(""); return true; }
  if (input == "REQ") {
    if (!cdsOwner) cdsAnnounce("");
    else if (cdsArmIsMoving()) cdsAnnounce("BUSY");
    else cdsSetOwner(false, "");
    return true;
  }
  if (input == "REL") {
    if (cdsOwner) cdsAnnounce("");
    else if (cdsArmIsMoving()) cdsAnnounce("BUSY");
    else cdsSetOwner(true, "");
    return true;
  }
  return false;
}

// ขณะ CODESYS คุมอยู่ ไม่รับคำสั่งจาก PC ที่ทำให้แขนขยับหรือเปลี่ยนโหมด (A, S, HOME, FREE.., LOCK..)
// ส่วน STOP, SAVE, DEBUG ยังรับตามปกติ
bool cdsBlocksPcCommand(String input) {
  if (!cdsOwner) return false;
  if (input == "STOP" || input == "SAVE" || input == "DEBUG") return false;
  bool motion = input.startsWith("A") || input.startsWith("S") || input == "HOME"
             || input.startsWith("FREE") || input.startsWith("LOCK");
  if (motion && (uint32_t)(millis() - cdsLastBlockedMs) >= 500UL) {
    cdsLastBlockedMs = millis();
    cdsAnnounce("BLOCKED");
  }
  return motion;
}

// ---------------------------------------------------------------------
//  รีจิสเตอร์ฝั่งอ่าน: แปลงตำแหน่งจาก step เป็นหน่วย 0.1 องศา
// ---------------------------------------------------------------------
static uint16_t cdsDeg10(long steps, uint8_t axis) {
  float v = steps * cdsDeg10PerStep[axis];
  if (v > 32767.0) v = 32767.0;
  if (v < -32768.0) v = -32768.0;
  return (uint16_t)((int16_t)round(v));
}

static void cdsRefreshAxis(uint8_t i) {
  cdsReg[CDS_REG_ACT + i] = cdsDeg10(motors[i].currentPosition(), i);
  cdsReg[CDS_REG_TGT + i] = cdsDeg10(motors[i].targetPosition(), i);
}

// เรียกตอน CODESYS ขออ่าน
// ระหว่างแขนเคลื่อนที่ ตำแหน่งถูกคำนวณทีละแกนใน cdsLinkPoll() อยู่แล้ว (ค่าเก่าไม่เกินราว 12 ms)
// ตรงนี้จึงคำนวณครบทุกแกนเฉพาะตอนแขนหยุด เพื่อไม่ให้จังหวะพัลส์สะดุดตอนถูกอ่าน
static void cdsRefresh() {
  uint16_t st = 0;
  uint16_t fm = 0;
  bool moving = cdsArmIsMoving();
  for (uint8_t i = 0; i < 6; i++) {
    if (isFree[i]) fm |= (uint16_t)(1U << i);
    if (!moving) cdsRefreshAxis(i);
  }
  if (cdsOwner)         st |= 0x0001;
  if (moving)           st |= 0x0002;
  if (fm != 0)          st |= 0x0004;
  if (cdsWdTripped)     st |= 0x0008;
  cdsReg[CDS_REG_STATUS]    = st;
  cdsReg[CDS_REG_FREE_MASK] = fm;
  cdsReg[CDS_REG_TICK]      = (uint16_t)(millis() / 10UL);
}

// ---------------------------------------------------------------------
//  ทำคำสั่งจาก CODESYS  (คืนค่า RESULT)
// ---------------------------------------------------------------------
static uint16_t cdsExecute(uint16_t code) {
  uint16_t mask = cdsReg[CDS_REG_AXIS_MASK] & 0x003F;

  if (code == CDS_CMD_NOP) return CDS_RES_OK;

  if (code == CDS_CMD_STOP) {                 // รับเสมอ ไม่ว่าใครคุมอยู่
    cdsStopAll();
    return CDS_RES_OK;
  }

  if (code == CDS_CMD_REQUEST) {
    if (cdsOwner) return CDS_RES_OK;
    if (cdsArmIsMoving()) return CDS_RES_BUSY;
    cdsSetOwner(true, "");
    return CDS_RES_OK;
  }

  if (code == CDS_CMD_RELEASE) {
    if (!cdsOwner) return CDS_RES_OK;
    cdsStopAll();
    cdsSetOwner(false, "");
    return CDS_RES_OK;
  }

  if (code == CDS_CMD_MOVE) {
    if (!cdsOwner) return CDS_RES_NOT_OWNER;
    uint16_t spd = cdsReg[CDS_REG_SPEED];
    if (mask == 0 || spd < 1 || spd > 100) return CDS_RES_RANGE;
    for (uint8_t i = 0; i < 6; i++) {         // ตรวจให้ครบทุกแกนก่อน ถ้าผิดแม้แกนเดียวไม่ขยับเลย
      if ((mask & (1U << i)) == 0) continue;
      int16_t d10 = (int16_t)cdsReg[CDS_REG_TARGET + i];
      if (d10 < (int16_t)(minAngle * 10) || d10 > (int16_t)(maxAngle * 10)) return CDS_RES_RANGE;
      if (isFree[i]) return CDS_RES_AXIS_FREE;
    }
    long mappedSpeed = map((long)spd, 1, 100, 100, 3000);     // สูตรเดียวกับคำสั่ง S
    for (uint8_t i = 0; i < 6; i++) motors[i].setMaxSpeed((float)mappedSpeed);
    for (uint8_t i = 0; i < 6; i++) {
      if ((mask & (1U << i)) == 0) continue;
      float deg = (int16_t)cdsReg[CDS_REG_TARGET + i] / 10.0;
      targetPositions[i] = round(deg * stepsPerDegree[i]);
    }
    cdsMoveTo();
    return CDS_RES_OK;
  }

  if (code == CDS_CMD_FREE) {
    if (!cdsOwner) return CDS_RES_NOT_OWNER;
    if (mask == 0) return CDS_RES_RANGE;
    if (cdsArmIsMoving()) return CDS_RES_BUSY;
    for (uint8_t i = 0; i < 6; i++) {
      if ((mask & (1U << i)) == 0) continue;
      digitalWrite(ENA_PINS[i], HIGH);
      isFree[i] = true;
    }
    return CDS_RES_OK;
  }

  if (code == CDS_CMD_LOCK) {
    if (!cdsOwner) return CDS_RES_NOT_OWNER;
    if (mask == 0) return CDS_RES_RANGE;
    for (uint8_t i = 0; i < 6; i++) {
      if ((mask & (1U << i)) == 0) continue;
      digitalWrite(ENA_PINS[i], LOW);
      isFree[i] = false;
      long p = motors[i].currentPosition();
      motors[i].setCurrentPosition(p);
      targetPositions[i] = p;
    }
    return CDS_RES_OK;
  }

  return CDS_RES_BAD_CMD;
}

// เรียกหลังได้เฟรมเขียนที่ถูกต้อง: ถ้า CMD_SEQ เปลี่ยน ให้ทำคำสั่งหนึ่งครั้ง
static void cdsAfterWrite() {
  cdsLastWriteMs = millis();
  uint16_t seq = cdsReg[CDS_REG_CMD_SEQ];
  if (!cdsSeqKnown) {
    // เฟรมแรกหลังเปิดเครื่องหรือรีเซ็ต: จำเลขไว้เฉยๆ ไม่ทำคำสั่งที่อาจค้างมาจากก่อนรีเซ็ต
    cdsSeqKnown = true;
    cdsLastSeq = seq;
    cdsReg[CDS_REG_ACK_SEQ] = seq;
    cdsReg[CDS_REG_RESULT] = CDS_RES_RESTARTED;
    return;
  }
  if (seq == cdsLastSeq) return;
  cdsLastSeq = seq;
  cdsReg[CDS_REG_RESULT] = cdsExecute(cdsReg[CDS_REG_CMD_CODE]);
  cdsReg[CDS_REG_ACK_SEQ] = seq;
  for (uint8_t i = 0; i < 6; i++) {           // เป้าหมายใหม่ให้ CODESYS เห็นทันทีพร้อม ACK
    cdsReg[CDS_REG_TGT + i] = cdsDeg10(motors[i].targetPosition(), i);
  }
}

// ---------------------------------------------------------------------
//  Modbus RTU slave แบบไม่หยุดรอ (ไม่มี delay และไม่รอให้ส่งเสร็จ)
// ---------------------------------------------------------------------
// CRC-16 ของ Modbus คิดทีละครึ่งไบต์ด้วยตาราง 16 ช่อง (เร็วกว่าคิดทีละบิตราว 3 เท่า)
static const uint16_t cdsCrcTab[16] = {
  0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
  0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
};

static uint16_t cdsCrc16(const uint8_t *p, uint8_t n) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < n; i++) {
    crc ^= p[i];
    crc = (crc >> 4) ^ cdsCrcTab[crc & 0x000F];
    crc = (crc >> 4) ^ cdsCrcTab[crc & 0x000F];
  }
  return crc;
}

// เติม CRC แล้วส่ง  ถ้าบัฟเฟอร์ส่งว่างไม่พอจะทิ้งคำตอบนี้ (CODESYS ถามใหม่เอง) ดีกว่าหยุดรอแล้วมอเตอร์สะดุด
static void cdsSend(uint8_t len) {
  uint16_t crc = cdsCrc16(cdsTx, len);
  cdsTx[len++] = (uint8_t)(crc & 0xFF);
  cdsTx[len++] = (uint8_t)(crc >> 8);
  if (CDS_SERIAL.availableForWrite() >= (int)len) CDS_SERIAL.write(cdsTx, len);
}

static void cdsException(uint8_t fc, uint8_t code) {
  cdsTx[0] = CDS_SLAVE_ID;
  cdsTx[1] = fc | 0x80;
  cdsTx[2] = code;
  cdsSend(3);
}

// ความยาวเฟรมจากไบต์ที่ได้มาแล้ว:  -1 = ยังบอกไม่ได้,  0 = ไม่รู้จักฟังก์ชัน,  >0 = ความยาวทั้งเฟรม
static int8_t cdsFrameLength() {
  if (cdsRxLen < 2) return -1;
  uint8_t fc = cdsRx[1];
  if (fc == 0x03 || fc == 0x04 || fc == 0x06) return 8;
  if (fc == 0x10) {
    if (cdsRxLen < 7) return -1;
    if (cdsRx[6] > (uint8_t)(sizeof(cdsRx) - 9)) return 0;
    return (int8_t)(9 + cdsRx[6]);
  }
  return 0;
}

static void cdsProcess(uint8_t len) {
  uint16_t crc = cdsCrc16(cdsRx, len - 2);
  if (cdsRx[len - 2] != (uint8_t)(crc & 0xFF) || cdsRx[len - 1] != (uint8_t)(crc >> 8)) {
    cdsResync = true;                         // เฟรมเสีย: ไม่ตอบ รอให้สายเงียบแล้วเริ่มใหม่
    return;
  }
  uint8_t  fc   = cdsRx[1];
  uint16_t addr = ((uint16_t)cdsRx[2] << 8) | cdsRx[3];
  uint16_t val  = ((uint16_t)cdsRx[4] << 8) | cdsRx[5];   // จำนวนรีจิสเตอร์ หรือค่าที่เขียน (FC06)

  if (fc == 0x03 || fc == 0x04) {
    if (val < 1 || val > CDS_REG_COUNT) { cdsException(fc, 0x03); return; }
    if (addr >= CDS_REG_COUNT || (uint16_t)(addr + val) > CDS_REG_COUNT) { cdsException(fc, 0x02); return; }
    cdsRefresh();
    cdsTx[0] = CDS_SLAVE_ID;
    cdsTx[1] = fc;
    cdsTx[2] = (uint8_t)(val * 2);
    for (uint8_t i = 0; i < (uint8_t)val; i++) {
      uint16_t r = cdsReg[addr + i];
      cdsTx[3 + 2 * i] = (uint8_t)(r >> 8);
      cdsTx[4 + 2 * i] = (uint8_t)(r & 0xFF);
    }
    cdsSend((uint8_t)(3 + 2 * val));
    return;
  }

  if (fc == 0x06) {
    if (addr >= CDS_REG_WRITABLE) { cdsException(fc, 0x02); return; }
    cdsReg[addr] = val;
  } else {                                    // 0x10 เขียนหลายรีจิสเตอร์
    uint8_t bc = cdsRx[6];
    if (val < 1 || val > CDS_REG_WRITABLE || bc != (uint8_t)(val * 2)) { cdsException(fc, 0x03); return; }
    if (addr >= CDS_REG_WRITABLE || (uint16_t)(addr + val) > CDS_REG_WRITABLE) { cdsException(fc, 0x02); return; }
    for (uint8_t i = 0; i < (uint8_t)val; i++) {
      cdsReg[addr + i] = ((uint16_t)cdsRx[7 + 2 * i] << 8) | cdsRx[8 + 2 * i];
    }
  }
  for (uint8_t i = 0; i < 6; i++) cdsTx[i] = cdsRx[i];     // คำตอบของการเขียน = 6 ไบต์แรกของคำขอ
  cdsSend(6);
  cdsAfterWrite();
}

void cdsLinkBegin() {
  for (uint8_t i = 0; i < CDS_REG_COUNT; i++) cdsReg[i] = 0;
  cdsReg[CDS_REG_SPEED] = 50;
  cdsReg[CDS_REG_AXIS_MASK] = 0x003F;
  for (uint8_t i = 0; i < 6; i++) {
    cdsDeg10PerStep[i] = 10.0 / stepsPerDegree[i];
    cdsPcMaxSpeed[i] = motors[i].maxSpeed();
  }
  CDS_SERIAL.begin(CDS_BAUD);
  pinMode(CDS_RX_PIN, INPUT_PULLUP);
  while (CDS_SERIAL.available() > 0) CDS_SERIAL.read();
  cdsAnnounce("");
}

void cdsLinkPoll() {
  // 1) รับไบต์ที่มาถึงแล้ว และประมวลผลทันทีที่ครบเฟรม
  while (CDS_SERIAL.available() > 0) {
    uint8_t b = (uint8_t)CDS_SERIAL.read();
    cdsLastByteUs = micros();
    if (cdsResync) continue;
    if (cdsRxLen >= sizeof(cdsRx)) { cdsResync = true; cdsRxLen = 0; continue; }
    cdsRx[cdsRxLen++] = b;
    if (cdsRxLen == 1 && b != CDS_SLAVE_ID) { cdsResync = true; cdsRxLen = 0; continue; }   // ไม่ใช่เฟรมของเรา
    if (cdsUnknownFc) continue;               // เก็บไว้จนสายเงียบ แล้วค่อยตอบว่าไม่รองรับ
    int8_t need = cdsFrameLength();
    if (need == 0) cdsUnknownFc = true;
    else if (need > 0 && cdsRxLen >= (uint8_t)need) {
      cdsProcess((uint8_t)need);
      cdsRxLen = 0;
    }
  }

  // 2) สายเงียบเกิน CDS_GAP_US: จบการรอ
  if ((cdsRxLen > 0 || cdsResync) && (uint32_t)(micros() - cdsLastByteUs) > CDS_GAP_US) {
    if (cdsUnknownFc && !cdsResync && cdsRxLen >= 4) {
      uint16_t crc = cdsCrc16(cdsRx, cdsRxLen - 2);
      if (cdsRx[cdsRxLen - 2] == (uint8_t)(crc & 0xFF) && cdsRx[cdsRxLen - 1] == (uint8_t)(crc >> 8)) {
        cdsException(cdsRx[1], (cdsRx[1] == 0x10) ? 0x03 : 0x01);   // ไม่รองรับฟังก์ชันนี้ / จำนวนไม่ถูกต้อง
      }
    }
    cdsRxLen = 0;
    cdsResync = false;
    cdsUnknownFc = false;
  }

  // 3) ระหว่างเคลื่อนที่: คำนวณตำแหน่งสำหรับ CODESYS ทีละแกน ทุก 2 ms
  if (isMoving && (uint32_t)(micros() - cdsLastSliceUs) >= 2000UL) {
    cdsLastSliceUs = micros();
    cdsRefreshAxis(cdsSliceAxis);
    cdsSliceAxis = (uint8_t)((cdsSliceAxis + 1) % 6);
  }

  // 4) watchdog: CODESYS คุมอยู่แต่ไม่ส่งเฟรมเขียนมาเกินเวลา
  if (cdsOwner && (uint32_t)(millis() - cdsLastWriteMs) > CDS_WATCHDOG_MS) {
    cdsStopAll();
    cdsWdTripped = true;
    cdsSetOwner(false, "WD");
  }
}
