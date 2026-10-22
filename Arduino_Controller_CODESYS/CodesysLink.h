// =====================================================================
//  CodesysLink.h — ช่องรับคำสั่งจาก CODESYS (Modbus RTU บน Serial1)
//  และตัวแปร "ใครเป็นผู้ควบคุมแขน" (อินเตอร์เฟสบน PC หรือ CODESYS)
//  โค้ดอยู่ในแท็บ CodesysLink.ino ไฟล์นี้มีเฉพาะค่าตั้งและรายชื่อฟังก์ชัน
// =====================================================================
#ifndef CODESYS_LINK_H
#define CODESYS_LINK_H
#include <Arduino.h>

// ---------------- ค่าตั้ง ----------------
#define CDS_SERIAL       Serial1     // Mega 2560: TX1 = ขา 18, RX1 = ขา 19
#define CDS_RX_PIN       19          // เปิด pull-up ไว้ กันสัญญาณรบกวนตอนยังไม่ได้เสียบสาย
#define CDS_BAUD         115200UL    // 8N1 ต้องตรงกับ Modbus COM ใน CODESYS (ถ้าสื่อสารผิดพลาดบ่อย ลอง 57600 ทั้งสองฝั่ง)
#define CDS_SLAVE_ID     1           // ต้องตรงกับ Slave/Server address ใน CODESYS
#define CDS_WATCHDOG_MS  1000UL      // CODESYS ถือสิทธิ์อยู่แต่เงียบเกินเวลานี้ -> หยุดแขนแล้วคืนสิทธิ์ให้ PC

// ---------------- ฟังก์ชันที่ไฟล์หลักเรียกใช้ ----------------
void cdsLinkBegin();                        // เรียกท้าย setup()
void cdsLinkPoll();                         // เรียกทุกรอบของ loop()
bool cdsOwnsArm();                          // true = CODESYS เป็นผู้ควบคุม
bool cdsArmIsMoving();                      // true = ยังมีแกนที่วิ่งไม่ถึงเป้า
void cdsStopAll();                          // หยุดทุกแกนทันที
bool cdsHandleOwnerCommand(String input);   // คำสั่ง REQ / REL / WHO จากอินเตอร์เฟสบน PC (true = จัดการแล้ว)
bool cdsBlocksPcCommand(String input);      // true = ไม่รับคำสั่งนี้ เพราะ CODESYS คุมอยู่

#endif
