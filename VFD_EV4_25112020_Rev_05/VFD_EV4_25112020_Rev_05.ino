// SUN VFD Drive for Motor 3 Phase
// Edit by sakesun support SUN Team
// Edit by Tawatchai support SUN Team
// Edit Date 05/10/2020
// Code for IPM-EV4 Model
// Soft Start and Soft Speed Variable Adjust Freq , Read Volt , SET Volt Upper-Lower
// MOSFET and IGBT
//Update:01/11/2020 AUTOMODE READ VOLT
//Update:03/11/2020 AUTOMODE Soft Start
//Update:11/11/2020 FIX LCD and Timer
//Update:25/11/2020 Single Phase

#include <EEPROM.h>
#include <LiquidCrystal.h> //เรียกโปรแกรมคำสั่งการใช้งานจอแสดงผล LCD เข้ามาร่วมด้วย
// ตารางข้อมูลซายเวฟ ชุดละ 128 ตัว //Amplitude สูงสุดที่ไม่เกิน 255 ใช้สูงสุดที่ 245
// table for High Speed Normal for Compresser Air
PROGMEM const unsigned char Sine_Table[128] = {
  125, 131, 137, 143, 149, 155, 161, 167, 173, 178, 184, 189, 194, 199, 204, 209, 213, 218, 222, 225, 229, 232, 235, 238, 240, 243, 245, 246, 248, 249, 249, 250,
  250, 250, 249, 249, 248, 246, 245, 243, 240, 238, 235, 232, 229, 225, 222, 218, 213, 209, 204, 199, 194, 189, 184, 178, 173, 167, 161, 155, 149, 143, 137, 131,
  125, 119, 113, 107, 101, 95, 89, 83, 77, 72, 66, 61, 56, 51, 46, 41, 37, 32, 28, 25, 21, 18, 15, 12, 10, 7, 5, 4, 2, 1, 1, 0,
  0, 0, 1, 1, 2, 4, 5, 7, 10, 12, 15, 18, 21, 25, 28, 32, 37, 41, 46, 51, 56, 61, 66, 72, 77, 83, 89, 95, 101, 107, 113, 119
};

// เปลี่ยนวิธีการสั่งเซทค่าของรีจีสเตอร์ภายในของ Arduino ใหม่เพื่อให้เรียกใช้ง่ายขึ้น
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// เปลี่ยนชื่อเรียกขาของ Arduino ใหม่เพื่อให้จำง่ายในการต่อวงจร
#define PWM_OUT_UL 6 // PWM1 UL output  // Timer 0 name OC0A
#define PWM_OUT_UH 5 // PWM1 UH output  // Timer 0 name OC0B
#define PWM_OUT_VL 9 // PWM2 VL output // Timer 1 name OC1A
#define PWM_OUT_VH 10 // PWM2 VH output // Timer 1 name OC1B
#define PWM_OUT_WL 11 // PWM3 WL output // Timer 2 name OC2A
#define PWM_OUT_WH 3 // PWM3 WH output // Timer 2 name OC2B

#define testPin 13 // Test OutPut เพื่อกระพริบหลอดไฟ LED
//#define enablePin 7 // for dream Enable pin 7
#define enablePin 12

#define single_3ph A1
//#define FAULT_DETECT 2 //เพื่อตรวจรับสัญญาณผิดพลาดต่างๆจะได้ปิดภาค Power Driver ก่อนที่จะทำให้อุปกรณ์เสียหาย
#define FREQ_POT A0 // เพื่อเต่อกับวอลลุ่มปรับความถี่ analog input A0 
// ตั้งชื่อการกำหนดค่า DeadTime
//#define DT 3 // DeadTime Low Mosfet
byte DT; // DeadTime Low IGBT
// ตั้งชื่อการกำหนดค่าของ วอลลุ่มปรับความเร็วรอบ (Hz)ต่ำสุดและสูงสุด
#define FREQ_MIN 15 // default: 10
#define FREQ_MAX 60 // default: 90
//#define MAX_AMP_AT 45 // ให้ความแรง Amplitude สูงสุดที่ตั้งแต่ 42 Hz
// เปลี่ยนชื่อเรียกค่าตายตัวของตัวปรับวอลลุ่มของ Arduino ซึ่งเป็นความสามารถของ MCU ในแต่ละเบอร์อาจต่างกัน
#define POT_MIN 0 // default: 0
#define POT_MAX 1023 // default: 1023
#define MAX_Amplitude 60
#define MIN_Amplitude 20
#define TABLE_DIV 64
// การตั้งชื่อตัวแปรต่าง ๆ ที่ใช้ในตัวโปรแกรม
volatile float freq = 15;
// ค่าความถี่ PWM const double refclk=3921; // =16MHz /8/ 510 = 3921
const float refclk = 3921 ; // ค่าตายตัวที่ได้มาจากการคำนวน คริสตอลที่ใช้ 16000000Hz/8/510 = 3921
// ตั้งชื่อตัวแปรที่ใช้ในการคำนวนรอบของสัญญาณซายเวฟและต่อเนื่องกันอย่างสมบูรณ์ interrupt service declared as voilatile
volatile unsigned long sigma; // phase accumulator
volatile unsigned long delta; // phase increment
volatile unsigned long c4ms; // counter incremented every 4ms

// ค่าตัวแปรต่าง ๆ
byte phase0, phase1, phase2, freq_old, freq_new, temp, temp2, Amplitude;
unsigned int temp_ocr0a, temp_ocr0b, temp_ocr1a, temp_ocr1b, temp_ocr2a, temp_ocr2b;
bool single = false;
bool testPin_status = false;
// ตั้งชื่อค่าตายตัวที่เรากำหนดในการต่อขาต่าง ๆ ของ Arduino เข้ากับจอ LCD เพื่อให้จำง่าย
// Board Ninja pwm pin
#define RS A5
#define EN A4
#define D4 8
#define D5 7
#define D6 A2
#define D7 A3
// แล้วสั่งให้กำหนดตามนี้ได้เลย
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
//*********** เปลี่ยนให้แสดงผล Version ที่นี่ ****************************
char My_Version[16] = "  SUN INVERTER  ";
//*****************************************************************
void(* resetSoftware)(void) = 0; //ตั้งฟังชั่นการ RESET โปรแกรม
// เริ่มต้นตัวโปรแกรมโดยคำสั่งตั้งค่าต่าง ๆ ก่อน

//Read Volt
int Volt_VFD ;
int Upper_Volt ;
int Lower_Volt ;
byte VR_VOLT = 0;
int Volt_VFD_Old;
byte AUTO_Reset = 0;
// Analog I/O
#define KB A6 // keyboard input 

byte A_ = 0; //light DP
bool StrDefault = false;


// EEPROM Address
#define add_gMODE 0   // true false on-off system
#define add_gPowerType  1 // duty cycle 10-60 Max
#define add_gVup  2 // 2byte int
#define add_gVlo  4 // 2byte int
#define add_DT 6 //Dead Time

// Golble Var
byte gMODE;
byte gPowerType;

String Menu_Items[] = {"1.VFD MODE",      // index 0 --> gMODE   AUTO or Manual
                       "2.P Type  ",      // index 1 --> gPowerType Power Type MOSFET or IGBT
                       "3.V Upper  ",     // index 2 --> gVup
                       "4.V Lower   ",    // index 3 --> gVlo
                       "5.Read V     ",   // index 4 --> Read Volt
                       "6.Default     ",  // index 5
                       "7.About       ",  // index 6
                       "8.EXIT        "   // index 7
                      };
byte index = 0;

byte Key;
bool SET_Menu = false;

void SCANKEY() {  
  Scankey();
  // key up pass;
  if (Key == 3) { // key up
    Key = 0;
    if (index != 0) index--;
  }
  // key down pass;
  if (Key == 2) { // key down
    Key = 0;
    if (index != 7) index++;
  }

  if (Key == 4) { // show logo
    Key = 0;
    //Showlogo();
  }

  if (Key == 1) {
    Key = 0;
    /*
    delay(100);
    if (DP_ON_status)
      DP_ON_status = false;
    else DP_ON_status = true;
    digitalWrite(DP_ON, DP_ON_status);
    */
  }
  
  // show Menu
  lcd.print(Menu_Items[index]); // Menu_Items[]
  lcd.setCursor(12, 0);
  switch (index) {
    case 0: lcd.print(EncodeStr(gMODE)); break;
    case 1: lcd.print(PtypeStr(gPowerType)); break;
    case 2: lcd.print(Upper_Volt); break;
    case 3: lcd.print(Lower_Volt); break;
    //case 4: lcd.print(Default_(StrDefault)); break;
    //case 5: break;
  }

  lcd.setCursor(0, 1);
  lcd.print(" Up-Down Enter..");

  // in loop if key Enter
  if (Key == 5) {
    Key = 0;
    switch (index) {
      case 0: Menu_Index0(); break;
      case 1: Menu_Index1(); break;
      case 2: Menu_Index2(); break;
      case 3: Menu_Index3(); break;
      case 4: Menu_Index4(); break;
      case 5: Menu_Index5(); break;
      case 6: Menu_Index6(); break;
      case 7: Menu_Index7(); break;
    }
  }
}

void setup() {
  //Serial.begin(9600);
  //-----------------------------------------------------------------------------
  pinMode(KB, INPUT);
  pinMode(FREQ_POT,INPUT);
  pinMode(single_3ph, INPUT_PULLUP);
  pinMode(enablePin, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(testPin, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(PWM_OUT_UL, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(PWM_OUT_UH, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(PWM_OUT_VL, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(PWM_OUT_VH, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(PWM_OUT_WL, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(PWM_OUT_WH, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  // เซ็ตให้ขาที่ไปควบคุมโมดูลขับให้ถูกต้องก่อน ส่วนนี้สำคัญเพราะจะทำให้โมดูลขับเสียถ้าไม่เซ็ตให้ถูกต้องก่อน
  digitalWrite(PWM_OUT_UL, HIGH);
  digitalWrite(PWM_OUT_UH, LOW);
  digitalWrite(PWM_OUT_VL, HIGH);
  digitalWrite(PWM_OUT_VH, LOW);
  digitalWrite(PWM_OUT_WL, HIGH);
  digitalWrite(PWM_OUT_WH, LOW);
  //pinMode(FAULT_DETECT, INPUT_PULLUP); // สั่งเซตให้เป็นขารับสัญญาณเข้า และ พูลอัพ

  digitalWrite(enablePin, LOW); // สั่งให้ขานี้ ปิด (เป็น0) เพื่อทำให้ภาค Power Drive อย่าเพิ่งทำงานตอนนี้
  digitalWrite(testPin, HIGH); // Clear testPin

  //-----------------------------------------------------------------------------
  if (digitalRead(single_3ph) == LOW) {
    single = true; // ตรวจสอบว่าขาจั้มเปอร์จั้มเป็นซิงเกิลเฟสหรือเปล่า ถ้าจั้มก็บอกว่า จริง ถ้าไม่จ้ัมก็บอกว่า เท็จ
  }
  
  // ไปเรียกโปรแกรมการตั้งค่า ไทม์เเมอร์ เพื่อกำหนดค่าการสร้างสัญญาณ PWM 3 ชุด โดยใช้ตัวไทม์เมอร์ของ Arduino ทั้ง 3 ตัว Setup the timers
  setup_timer0();
  setup_timer1();
  setup_timer2();
  
  noInterrupts (); //ปิดอินเตอร์รัป
  interrupts ();   //เปิดอินเตอร์รัป
  
  lcd.begin(16, 2); // เริ่มเปิดใช้งานจอแสดงผล LCD
  lcd.setCursor(1, 0); // สังให้แสดงผลคอลั่มที่ 5 ในบรรทัดแรก (0)
  lcd.print(" IPM-EV4 VFD  "); //ถ้าจะใช้ภาษาอังกฤษ
  
  // show version
  lcd.setCursor(0, 1); // สังให้แสดงผล เวอร์ชั่น วัน เดือน ปี ที่เขียน คอลั่มที่ 1 (0) ในบรรทัดที่ 2 (1)
  for (temp = 0; temp <= 15; temp++) {
    lcd.print(My_Version[temp]);
    delay(100);
  }
  delay(3000); // หน่วงเวลาให้แสดงผลนี้ เป็นเวลานาน 3 วินาที
  lcd.clear();
  
  //-----------------------------------------------------------------------------
  if (single == true) { // ถ้าตัวจั้มเปอร์เป็นจริงก็ให้แสดงผล ที่ คอลั่ม 6 บรรทัดแรก J1P = จ่ายไฟแบบซิงเกิลเฟส
    lcd.setCursor(5, 0);
    lcd.print("1Phase");
  }
  else {
    lcd.setCursor(5, 0); // หรือถ้าตัวจั้มเปอร์เป็นเท็จ ก็ให้แสดงผลที่ คอลั่ม 6 บรรทัดแรก J3P = จ่ายไฟแบบ 3 เฟส
    lcd.print("3Phase");
  }
  //------------------ จะหยุดรอที่นี่ถ้าวอลลู่มปรับ Speed น้อยกว่า 18 Hz------------------
  
  do {
    freq_new = map(Readadc(), POT_MIN, POT_MAX, FREQ_MIN, FREQ_MAX); //รับค่า Speed จากวอลลุ่ม
    lcd.setCursor(0, 1); // แสดงผลที่คอลั่ม 1 บรรทัด 2 รอสวิตช์ลูกลอยเปิด WAIT SW
    lcd.print("Wait Keyboard/VR");
    delay (1000);
    lcd.setCursor(0, 1); // แสดงผลที่คอลั่ม 1 บรรทัด 2 รอสวิตช์ลูกลอยเปิด SpeedLow
    lcd.print("Motor OFF/LOW   ");
    delay (1000);
  } while (freq_new <= 18);
  //----------------------------------------------------------------------------
  
  freq_old = 10; //จะเริ่มครั้งแรกด้วยความถี่ตั้งแต่ 1 ก่อนเพื่อ WarmUp
  freq = 10;
  Amplitude = 10;
  delta = pow(2, 32) * freq / refclk ;
  lcd.clear();
  
  //SETUP_KEY// ENTER
  for(int z=0;z<100;z++){
    lcd.setCursor(4, 0);
    lcd.print("VFD-EV4");  
    lcd.setCursor(2, 1);
    lcd.print(">ENTER KEY<");  
    lcd.setCursor(14, 1);
    lcd.print(z);
    delay_(50);
    if((analogRead(KB)>=700) && (analogRead(KB)<=730)){
      delay_(250);
      ReadEEPROM();
      VR_VOLT = 0;
      z=100;
      Key = 0;
      lcd.clear();
      SET_Menu = true;
      loop();
    }
  }
  
  delay_(1000);
  //digitalWrite(enablePin, HIGH); // เปิด ภาค Power Drive ให้เริ่มทำงานได้
  //-----------------------------------------------------------------------------
  lcd.clear();
  // attachInterrupt(0, fault, LOW); //เริ่มให้ทำการตรวจการผิดปรกติ กันโหลดเกิน
  ReadEEPROM();//Load Save
 
  ////////////////////////After setting/////////////////////////////////////////
  if(gMODE == true){ //AUTO MODE
    sbi (TIMSK2, TOIE2); //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect เริ่มสร้างสัญญาณซายเวฟ 10 Hz.
    delay_(500);
    digitalWrite(enablePin,1);
    READ_VOLT();
    freq_new = map(Volt_VFD, Lower_Volt, Upper_Volt, FREQ_MIN, FREQ_MAX); 
      if(freq_new >= FREQ_MAX){
        freq_new = 50;
      }
    if(Volt_VFD <= 98){
      resetSoftware();
    }
    changeFreq_rampAUTO(freq_new);  
    freq_new = map(Volt_VFD, Lower_Volt, Upper_Volt, FREQ_MIN, FREQ_MAX); 
    changeFreq(freq_new);
    VR_VOLT = 2;
  }
  else if(gMODE == false){ //Manual MODE
    sbi (TIMSK2, TOIE2); //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect เริ่มสร้างสัญญาณซายเวฟ 10 Hz.
    delay_(500);
    digitalWrite(enablePin,1);
    freq_new = map(analogRead(FREQ_POT), POT_MIN, POT_MAX, FREQ_MIN, FREQ_MAX); //รับค่า Speed จากวอลลุ่ม
    changeFreq_ramp(freq_new);
    VR_VOLT = 1;
  }
  ////////////////////////Before setting/////////////////////////////////////////
  if(VR_VOLT == 0 ){
    lcd.setCursor(4, 0);
    lcd.print("VFD-EV4");  
    lcd.setCursor(2, 1);
    lcd.print(">ENTER KEY<");  
  }
}

/*
 * Read EEPROM
 */
void ReadEEPROM() {
  gMODE = EEPROM.read(add_gMODE); // Auto or Manual
  gPowerType = EEPROM.read(add_gPowerType); // Auto or Manual
  DT = EEPROM.read(add_DT);
  EEPROM.get(add_gVup,Upper_Volt); // Auto or Manual
  EEPROM.get(add_gVlo,Lower_Volt); // Auto or Manual
}


/////////////////////DefaultEEPROM///////////////////////////
void default_LV() {
  DT = 3;
  VR_VOLT = 2;
  gMODE = 2; //Auto
  PtypeStr(false);
  Upper_Volt = 145;
  Lower_Volt = 120;
  EEPROM.write(add_gMODE,gMODE);
  EEPROM.write(add_DT,DT);
  EEPROM.put(add_gVlo, Lower_Volt);
  EEPROM.put(add_gVup, Upper_Volt);
}
void default_HV() {
  DT = 3;
  VR_VOLT = 2;
  gMODE = 2; //Auto
  PtypeStr(false);
  Upper_Volt = 310;
  Lower_Volt = 240;
  EEPROM.write(add_gMODE,gMODE);
  EEPROM.write(add_DT,DT);
  EEPROM.put(add_gVlo, Lower_Volt);
  EEPROM.put(add_gVup, Upper_Volt);
}
/////////////////////////////////////////////////////////////
int Readadc () {
  int _x = 0 ;
  for (int i = 0; i < 9; i++) {
    _x += analogRead(FREQ_POT);
    delay_(1);
  }
  _x = _x / 9;
  return _x;
}

/*
 * Fuction:Delay
 * delay_() 
 */
int delay_(int time_){
  for(int Time_i=0 ; Time_i<10*time_ ; Time_i++){
    delay(1);
  }
  return;
}

//------------------------------- ทำงาน วน อยู่ในส่วนนี้เท่านั้น ---------------------
void loop() {
  Key = 0;
  while((analogRead(KB)>=700) && (analogRead(KB)<=730)){
    changeFreq_rampDown(freq_new);
    cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
    digitalWrite(enablePin,0);
    delay_(250);
    lcd.clear();
    SET_Menu = true;
  }
  while(SET_Menu == true){
    SCANKEY();  // Scankeyboard 
  }
  //Light DP//
  if((analogRead(KB)>=480) && (analogRead(KB)<=495)){
    delay_(250);
    A_++;
    if(A_>2)A_ = 1;
    switch (A_){
      case 1:digitalWrite(testPin, 0);break;
      case 2:digitalWrite(testPin, 1);break;
    }
  }
  
  //Manual  
  if(VR_VOLT == 1){
    lcd.setCursor(0, 0);
    lcd.print("Manual");
    if (testPin_status) { //ในช่วงที่ หลอด LED ติด จะเช็คว่ามีการเปลี่ยน Speed หรือไม่
      freq_new = map(Readadc(), POT_MIN, POT_MAX, FREQ_MIN, FREQ_MAX); //รับค่า Speed จากวอลลุ่ม
      if (freq_new != freq_old) { //ถ้าความถี่ใหม่ไม่เท่ากับความถี่เดิมแสดงว่าต้องการเปลี่ยน Speed ใหม่
        changeFreq(freq_new);
      }
    }
    //--------- ถ้าวอลลู่มปรับลงต่ำกว่า 18 Hz หรือสวิตช์ลูกลอยตัดอยู่ ให้หยุดเริ่มใหม่ -----------
    if (freq_new <= 18) {
      cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
      digitalWrite(enablePin,LOW);
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Reset...");
      delay_(1500);
      resetSoftware(); // RESET! เริ่มต้นใหม่
    }
  }
  //Auto
  if(VR_VOLT == 2){ 
    AUTO_Reset = 1;
    if (testPin_status) { //ในช่วงที่ หลอด LED ติด จะเช็คว่ามีการเปลี่ยน Speed หรือไม่
      freq_new = map(READ_VOLT(), Lower_Volt, Upper_Volt, FREQ_MIN, FREQ_MAX); //รับค่า Speed จากวอลลุ่ม
      if (freq_new != freq_old) { //ถ้าความถี่ใหม่ไม่เท่ากับความถี่เดิมแสดงว่าต้องการเปลี่ยน Speed ใหม่
        changeFreq(freq_new);
      }
    }     
  }
  delay_(10);
}

//----------------------------------------------------------------------------
// ชุดคำสั่งในการเปลี่ยนความถี่
void changeFreq(float _freq) {
  temp2 = _freq;
 
  if (_freq > freq_old) {
    for (temp2 = freq_old; temp2 <= _freq; temp2++) {
      Amplitude = temp2++; // new edit
      Amplitude++;
      if (Amplitude >= MAX_Amplitude) { // new edit
        Amplitude = MAX_Amplitude; // ล๊อค Amplitude
      }
      //lcd.setCursor(2, 0);
      //lcd.print(Amplitude);
      //lcd.setCursor(2, 1);
      //lcd.print(temp2);
      printlcd(Amplitude,temp2);
      lcd.print(" Hz. +");
      cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
      freq = _freq;
      delta = pow(2, 32) * freq / refclk; // อัพเดทความถี่ใหม่ update phase increment
      sbi (TIMSK2, TOIE2); //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect
      while (c4ms);
    }

  } else {
   for (temp2 = freq_old; temp2 >= _freq; temp2--) {
  
      Amplitude = temp2--; // new edit
      Amplitude--;
      if (Amplitude >= MAX_Amplitude) { // new edit
        Amplitude = MAX_Amplitude; // ล๊อค Amplitude
      }
      printlcd(Amplitude,temp2);
      lcd.print(" Hz. -");
      cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
      freq = _freq;
      delta = pow(2, 32) * freq / refclk; // อัพเดทความถี่ใหม่ update phase increment
      sbi (TIMSK2, TOIE2); //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect
      while (c4ms);
    }
  }
  freq_old = _freq;
}
//----------------------------------------------------------------------------

/*
 * Function Scankey 
 * Button: Left Up Down Right Enter
 */
void Scankey() {
  //Right//
  if((analogRead(KB)>=0) && (analogRead(KB)<=5)){
    delay_(80);
    lcd.clear();
    Key = 1;
  }
  //Down//
  else if((analogRead(KB)>=305) && (analogRead(KB)<=330)){
    delay_(250);
    lcd.clear();
    Key = 2;
  }
  //Up
  else if((analogRead(KB)>=125) && (analogRead(KB)<=140)){
    delay_(250);
    lcd.clear();
    Key = 3;
  }
  //Left// 
  else if((analogRead(KB)>=480) && (analogRead(KB)<=495)){
    delay_(80);
    lcd.clear();
    Key = 4;
  }
  //Enter//
  else if((analogRead(KB)>=700) && (analogRead(KB)<=730)){
    delay_(250);
    lcd.clear();
    Key = 5;
  }
  //return Key;
}

void ShowSuccress(String _oldstring) {
  lcd.setCursor(0, 1);
  lcd.print("Save Succress.. ");
  delay_(2000);
  lcd.print(_oldstring);
}

//MODE AUTO or Manual
void Menu_Index0() {  
  while (Key != 6) {
    Scankey();
    lcd.setCursor(0, 0);
    lcd.print("Press key.. " + EncodeStr(gMODE));
    lcd.setCursor(0, 1);
    lcd.print("L=AT.,R=MN.,D=Sav ");

    if (Key == 1) {
      gMODE = false;   
      Key = 0;
    }
    if (Key == 4){
      gMODE = true;  
      Key = 0;
    }

    if (Key == 2) { // save key
      EEPROM.write(add_gMODE, gMODE);
      ShowSuccress("L=AT.,R=MN.,D=Sav ");
      Key = 0;
    }

    if (Key == 5) {
      delay_(250);
      lcd.clear();
      Key = 6;  
      SCANKEY();
    }
    delay_(80);
  }
}
//Power Type Mosfet or IGBT
void Menu_Index1() {  
  while (Key != 6) {
    Scankey();
    lcd.setCursor(0, 0);
    lcd.print("Press key.. " + PtypeStr(gPowerType));
    lcd.setCursor(0, 1);
    lcd.print("L=IG.,R=MF.,D=Sav ");

    if (Key == 1) { //MOSFET
      gPowerType = false;
      DT = 1;   
      Key = 0;
    }
    if (Key == 4){ //IGBT
      gPowerType = true;  
      DT = 6; 
      Key = 0;
    }

    if (Key == 2) { // save key
      EEPROM.write(add_gPowerType, gPowerType);
      EEPROM.write(add_DT,DT);
      ShowSuccress("L=FET,R=IGBT.,D=Sav ");
      Key = 0;
    }

    if (Key == 5) {
      delay_(250);
      lcd.clear();
      Key = 6;  
      SCANKEY();
    }
    delay_(80);
  }
}
//Volt Upper
void Menu_Index2() { 
  while (Key != 6) {
    Scankey();
    lcd.setCursor(0, 0);
    lcd.print("Press key.. " + String(Upper_Volt));
    lcd.setCursor(0, 1);
    lcd.print("L=++,R=--,D=Sav ");

    if (Key == 1) {
      Upper_Volt--;
      Key = 0;   
    }
    if (Key == 4){ 
      Upper_Volt++;
      Key = 0;   
    }

    if (Key == 2) { // save key
      EEPROM.put(add_gVup, Upper_Volt);
      ShowSuccress("L=++,R=--,D=Sav ");
      Key = 0;
    }

    if (Key == 5) {
      delay_(250);
      lcd.clear();
      Key = 6;  
      SCANKEY();
    }
    delay_(10);
  }
}

//Volt Lower
void Menu_Index3() { 
  while (Key != 6) {
    Scankey();
    lcd.setCursor(0, 0);
    lcd.print("Press key.. " + String(Lower_Volt));
    lcd.setCursor(0, 1);
    lcd.print("L=++,R=--,D=Sav ");

    if (Key == 1) {
      Lower_Volt--;
      Key = 0;   
    }
    if (Key == 4){ 
      Lower_Volt++;
      Key = 0;   
    }

    if (Key == 2) { // save key
      EEPROM.put(add_gVlo, Lower_Volt);
      ShowSuccress("L=++,R=--,D=Sav ");
      Key = 0;
    }

    if (Key == 5) {
      delay_(250);
      lcd.clear();
      Key = 6;  
      SCANKEY();
    }
    delay_(10);
  }
}

//Read Volt
void Menu_Index4() { 
  while (Key != 6) {
    Scankey();
    lcd.setCursor(0, 0);
    lcd.print("Press key..");
    lcd.setCursor(0, 1);
    lcd.print("L=NoLoad,R=Load ");

    if ((Key == 1)and((DT == 1)or(DT == 12))) {//LOAD 
      Key = 0;
      cbi (TIMSK2, TOIE2);
      lcd.clear();
      AUTO_Reset = 0;
      freq_old = 5; //จะเริ่มครั้งแรกด้วยความถี่ตั้งแต่ 10 ก่อนเพื่อ WarmUp
      freq = 5;
      Amplitude = 5;
      sbi (TIMSK2, TOIE2); //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect
      delay_(500);
      digitalWrite(enablePin,1);
      freq_new = 50;
      changeFreq_ramp(freq_new);
      while(Key != 6){
        READ_VOLT();
        Scankey();
        if (testPin_status){
          changeFreq(freq_new);    
        }
        if (Key == 5) {
          changeFreq_rampDown(freq_new);
          cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
          digitalWrite(enablePin,0);
          delay_(250);
          lcd.clear();
          Key = 6;  
          SCANKEY();
        }
        delay_(50); 
      }
    }
    if (Key == 4){//NoLoad
      cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
      digitalWrite(enablePin,0);
      Key = 0;  
      lcd.clear();
      while(Key != 6){ 
        READ_VOLT();
        Scankey();
        if (Key == 5) {
          delay_(250);
          lcd.clear();
          Key = 6;  
          SCANKEY();
        }
        delay_(50);
      }   
    }
    if (Key == 5) {
      delay_(250);
      lcd.clear();
      Key = 6;  
      SCANKEY();
    }
    delay_(80);
  }
}

//Default
void Menu_Index5() {  
  while (Key != 6) {
    Scankey();
    lcd.setCursor(0, 0);
    lcd.print("Press key.. " + Default_(StrDefault));
    lcd.setCursor(0, 1);
    lcd.print("L=HV,R=LV,E=OK ");

    if (Key == 1) {
      StrDefault = true;
      default_LV();
      Key = 0;   
    }
    if (Key == 4){ 
      StrDefault = false;
      default_HV();
      Key = 0;   
    }
    if (Key == 5) {
      delay_(250);
      lcd.clear();
      Key = 6;  
      SCANKEY();
    }
    delay_(80);
  }
}

//About
void Menu_Index6() { 
  lcd.clear(); 
  while (Key != 6) {
    Scankey();
    lcd.setCursor(0, 0);
    lcd.print("VFD-EV4");
    lcd.setCursor(0, 1);
    lcd.print("V.04 Y:2020");

    if (Key == 5) {//ENTER Close to menu
      delay_(250);
      lcd.clear();
      Key = 6;  
      SCANKEY();
    }
    delay_(80);
  }
}

//Exit
void Menu_Index7() { 
  while (Key != 6) {
    Scankey();
    lcd.setCursor(0, 0);
    lcd.print("Press key.. ");
    lcd.setCursor(0, 1);
    lcd.print("L=OK , R=Cancel");

    if (Key == 1) {
      delay_(250);
      lcd.clear();
      Key = 6;  
      SCANKEY();
    }
    if ((Key == 4)and((DT == 1)or(DT == 6))){ 
      lcd.clear();
      
      if(gMODE == true){
        freq_old = 5; //จะเริ่มครั้งแรกด้วยความถี่ตั้งแต่ 10 ก่อนเพื่อ WarmUp
        freq = 5;
        Amplitude = 5;
        sbi (TIMSK2, TOIE2); //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect
        delay_(500);
        digitalWrite(enablePin,1);
        READ_VOLT();
        freq_new = map(Volt_VFD, Lower_Volt, Upper_Volt, FREQ_MIN, FREQ_MAX); 
        if(freq_new >= FREQ_MAX){
          freq_new = 50;
        }
        if(Volt_VFD <= 98){
          resetSoftware();
        }
        changeFreq_rampAUTO(freq_new);  
        freq_new = map(Volt_VFD, Lower_Volt, Upper_Volt, FREQ_MIN, FREQ_MAX); 
        changeFreq(freq_new);
        SET_Menu = false;
        Key = 5; 
        index = 0;
        return; 
      }
      else if(gMODE == false){
        freq_old = 5; //จะเริ่มครั้งแรกด้วยความถี่ตั้งแต่ 10 ก่อนเพื่อ WarmUp
        freq = 5;
        Amplitude = 5;
        sbi (TIMSK2, TOIE2); //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect
        delay_(500);
        digitalWrite(enablePin,1);
        freq_new = map(analogRead(FREQ_POT), POT_MIN, POT_MAX, FREQ_MIN, FREQ_MAX); //รับค่า Speed จากวอลลุ่ม
        changeFreq_ramp(freq_new);
        SET_Menu = false;
        Key = 5; 
        index = 0;
        return;  
      }   
    }
    delay_(80);
  }
}

String EncodeStr(byte _xx) {
  String _str;
  switch (_xx) {
    case true: 
      _str = "AT."; 
      VR_VOLT = 2; 
      break;
    case false: 
      _str = "MN."; 
      VR_VOLT = 1; 
      break;
  }
  return _str;
}

String PtypeStr(byte _xx) {
  String _str;
  switch (_xx) {
    case true: 
      _str = "IGBT"; 
      break;
    case false: 
      _str = "FET "; 
      break;
  }
  return _str;
}

String Default_(byte _xx) {
  String _str;
  switch (_xx) {
    case true:_str = "LV";break;
    case false:_str = "HV";break;
  }
  return _str;
}
void printlcd(byte _amp, byte _temp){
      lcd.setCursor(13, 1);
      if(single == true) lcd.print("1PH");
      else if(single == false) lcd.print("3PH");
      
      lcd.setCursor(12,0);
      if (DT == 3) lcd.print("FET");
      else if (DT==8) lcd.print("IGBT");
      
      lcd.setCursor(0, 1);
      lcd.print("F:");
      lcd.setCursor(2, 1);
      lcd.print(_temp);        
}

/*
 * Read Volt and SETUP Volt Upper - Lower
 */
int READ_VOLT(){
  int V_read = 0;
  V_read = 1023 - Readadc(); //  Inverting
  
  //Volt SL. PC817 Rank mark = B
  if(V_read >=76)Volt_VFD = map((V_read), 76, 82, 309, 325); 
  else if((V_read >=67)&&(V_read <76))Volt_VFD = map((V_read), 67, 76, 284, 309); 
  else if(V_read <67)Volt_VFD = map((V_read), 58, 67, 258, 284); //8
  /*
  //Volt SL. PC817 Rank mark = C
  if(V_read >=140)Volt_VFD = map((V_read), 140, 155, 305, 326); 
  else if((V_read >=123)&&(V_read <140))Volt_VFD = map((V_read), 123, 140, 282, 305); 
  else if(V_read <123)Volt_VFD = map((V_read), 107, 123, 258, 282); // 
  */

  if(Volt_VFD != Volt_VFD_Old){
    lcd.setCursor(4, 0); 
    lcd.print("     "); 
    //lcd.setCursor(1, 0); 
    //lcd.print("                ");
  }
  Volt_VFD_Old = Volt_VFD;  
  
  lcd.setCursor(0, 0); 
  lcd.print("Vin:");  
  lcd.setCursor(4, 0); 
  lcd.print(Volt_VFD,1); 
  lcd.setCursor(9, 0); 
  lcd.print("V"); 
  //lcd.setCursor(0, 1); 
  //lcd.print(V_read);  
  
  if(AUTO_Reset == 1){//Reset...
    if (Volt_VFD >= Upper_Volt){
      cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
      digitalWrite(enablePin,0);
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Reset...");
      delay_(1500);
      resetSoftware(); // RESET! เริ่มต้นใหม่
    }
    if (Volt_VFD <= Lower_Volt){
      cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
      digitalWrite(enablePin,0);
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Reset...");
      delay_(1500);
      resetSoftware(); // RESET! เริ่มต้นใหม่
    }  
  }    
  delay_(10);     
  return Volt_VFD;
}

// ชุดคำสั่งในการเปลี่ยนความถี่ แบบไต่ระดับ RAMP
void changeFreq_ramp(float _freq) {
  if (_freq > freq_old) {
    for (temp2 = freq_old; temp2 <= _freq; temp2++) {
      Amplitude = temp2 + 20;  // ++ 10 to 50
      //Amplitude++;
      if (Amplitude >= MAX_Amplitude) {
        Amplitude = MAX_Amplitude; // ล๊อค Amplitude
      }
      printlcd(Amplitude,temp2);
      lcd.print(" Hz. +    ");
      cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
      freq = temp2;
      delta = pow(2, 32) * freq / refclk; // อัพเดทความถี่ใหม่ update phase increment
      sbi (TIMSK2, TOIE2); //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect
      while (c4ms);
    }
  }
  freq_old = _freq;
}
// ชุดคำสั่งในการเปลี่ยนความถี่ แบบไต่ระดับ RAMP AUTO
void changeFreq_rampAUTO(float _freq) {
  if (_freq > freq_old) { 
    for (temp2 = freq_old; temp2 <= _freq; temp2++) {
      Amplitude = temp2++;  // ++ 10 to 50
      Amplitude++;
      if (Amplitude >= MAX_Amplitude) {
        Amplitude = MAX_Amplitude; // ล๊อค Amplitude
      }
      /*
      if (temp2>=60) {
        //Amplitude = MIN_Amplitude; // ล๊อค Amplitude
        freq_new = 61;
        temp2 = 60;
      }
      */
      freq_new = map(READ_VOLT(), Lower_Volt, Upper_Volt, FREQ_MIN, FREQ_MAX);
      if(temp2 >= freq_new){
        temp2 = _freq;
      }
      printlcd(Amplitude,temp2);
      lcd.print(" Hz. +");
      cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
      freq = temp2;
      delta = pow(2, 32) * freq / refclk; // อัพเดทความถี่ใหม่ update phase increment
      sbi (TIMSK2, TOIE2); //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect
      while (c4ms);
    }
  }
  freq_old = _freq;
}
//ไต่ระดับลง
void changeFreq_rampDown(float _freq) {
  if (_freq > 15) {
    
    for (temp2 = freq_old; temp2 <= _freq; temp2--) {
      delay_(500);
      Amplitude = temp2--;
      Amplitude--;
      if (Amplitude <= 15) {
        _freq = 13;
        Amplitude = 13; // ล๊อค Amplitude
      }
      printlcd(Amplitude,temp2);
      lcd.print(" Hz. -");
      cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
      freq = temp2;
      delta = pow(2, 32) * freq / refclk; // อัพเดทความถี่ใหม่ update phase increment
      sbi (TIMSK2, TOIE2); //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect
      while (c4ms);
    }
  }
  freq_old = _freq;
}
//------------------------------------------------------------------------------
// ชุดคำสั่งในการตั้งค่า ไทม์เมอร์ ตัวแรก (0)
// timer0 setup
// set prescaler to 8, PWM mode to phase correct PWM, 16000000/512/8 = 3.9kHz clock
void setup_timer0(void) {
  // Timer0 Clock Prescaler to :ตั้งค่าไทม์เมอร์ให้ หารอีก 8 เพื่อจะได้ความถี่ PWM ขนาด 3.9 Khz ให้เหมาะกับ IPM เบอร์เก่า ๆ
  cbi (TCCR0B, CS00);
  sbi (TCCR0B, CS01);
  cbi (TCCR0B, CS02);
  // Timer0 PWM Mode set to Phase Correct PWM
  sbi (TCCR0A, COM0B0); // ตั้งค่าให้ชุดส่งสัญญาณช่อง แรก (OCR0B) ให้ส่งสัญญาณออกเป็น Sine PWM ปรกติ +
  sbi (TCCR0A, COM0B1);
  cbi (TCCR0A, COM0A0); // ตั้งค่าให้ชุดส่งสัญญาณช่อง แรก (OCR0A) ให้ส่งสัญญาณออกเป็น Sine PWM กลับเฟส Invert เป็น -
  sbi (TCCR0A, COM0A1);
  sbi (TCCR0A, WGM00); //ตั้งค่าให้สัญญาณพัลซ์ทุกลูกอยู่แนวเดียวกันเสมอในทุกเฟส (Mode 1 / Phase Correct PWM)
  cbi (TCCR0A, WGM01);
  cbi (TCCR0B, WGM02);
}
//------------------------------------------------------------------------------
// timer1 setup
// set prescaler to 8, PWM mode to phase correct PWM, 16000000/512/8 = 3.9kHz clock ให้เหมาะกับ IPM เบอร์เก่า ๆ
void setup_timer1(void) {
  // Timer1 Clock Prescaler to :ตั้งค่าไทม์เมอร์ให้ หารอีก 8 เพื่อจะได้ความถี่ PWM ขนาด 3.9 Khz
  cbi (TCCR1B, CS10);
  sbi (TCCR1B, CS11);
  cbi (TCCR1B, CS12);
  // Timer1 PWM Mode set to Phase Correct PWM
  sbi (TCCR1A, COM1B0); // ตั้งค่าให้ชุดส่งสัญญาณช่อง 2(OCR1B) ให้ส่งสัญญาณออกเป็น Sine PWM ปรกติ +
  sbi (TCCR1A, COM1B1);
  cbi (TCCR1A, COM1A0); // ตั้งค่าให้ชุดส่งสัญญาณช่อง 2(OCR1A) ให้ส่งสัญญาณออกเป็น Sine PWM กลับเฟส Invert เป็น -
  sbi (TCCR1A, COM1A1);
  sbi (TCCR1A, WGM10); //ตั้งค่าให้สัญญาณพัลซ์ทุกลูกอยู่แนวเดียวกันเสมอในทุกเฟส (Mode 1 / Phase Correct PWM)
  cbi (TCCR1A, WGM11);
  cbi (TCCR1B, WGM12);
  cbi (TCCR1B, WGM13);
}
//------------------------------------------------------------------------------
// timer2 setup
// set prescaler to 8, PWM mode to phase correct PWM, 16000000/512/8 = 3.9kHz clock ให้เหมาะกับ IPM เบอร์เก่า ๆ
void setup_timer2(void) {
  // Timer1 Clock Prescaler to :ตั้งค่าไทม์เมอร์ให้ หารอีก 8 เพื่อจะได้ความถี่ PWM ขนาด 3.9 Khz
  // Timer2 Clock Prescaler to : /8
  cbi (TCCR2B, CS20);
  sbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);
  // Timer2 PWM Mode set to Phase Correct PWM
  sbi (TCCR2A, COM2B0); // ตั้งค่าให้ชุดส่งสัญญาณช่อง 3(OCR2B) ให้ส่งสัญญาณออกเป็น Sine PWM ปรกติ +
  sbi (TCCR2A, COM2B1);
  cbi (TCCR2A, COM2A0); // ตั้งค่าให้ชุดส่งสัญญาณช่อง 3(OCR2A) ให้ส่งสัญญาณออกเป็น Sine PWM กลับเฟส Invert เป็น -
  sbi (TCCR2A, COM2A1);
  sbi (TCCR2A, WGM20); //ตั้งค่าให้สัญญาณพัลซ์ทุกลูกอยู่แนวเดียวกันเสมอในทุกเฟส (Mode 1 / Phase Correct PWM)
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);
}
//------------------------------------------------------------------------------
/*
// ส่งInterrupt Service Routine attached to INT0 vector
void fault() {
  digitalWrite(enablePin, LOW); // สั่งให้ขานี้ ปิด (เป็น0) เพื่อทำให้ภาค Power Drive อย่าเพิ่งทำงานตอนนี้
  cbi (TIMSK2, TOIE2); // หยุดการทำงานของไทม์เมอร์ 2 disable timer2 overflow detect
  digitalWrite(PWM_OUT_UL, HIGH);
  digitalWrite(PWM_OUT_UH, LOW);
  digitalWrite(PWM_OUT_VL, HIGH);
  digitalWrite(PWM_OUT_VH, LOW);
  digitalWrite(PWM_OUT_WL, HIGH);
  digitalWrite(PWM_OUT_WH, LOW);
  lcd.clear();
  lcd.setCursor(0, 0); // สังให้แสดงผลคอลั่มที่ 1 ในบรรทัดแรก (0)
  lcd.print("OverLoad"); // แสดงผล Error
  lcd.setCursor(0, 1); // สังให้แสดงผลคอลั่มที่ 1 ในบรรทัด 2 (1)
  lcd.print("ResetOFF");
  while (1);
}
*/
//------------------------------------------------------------------------------
//การที่จะได้สัญญาณ ซายเวฟ ที่มีความถี่แม่นยำตามที่เราตั้งค่าต่าง ๆ ไว้ ต้องอาศัยการนับเวลาของตัวไทม์ เมอร์ 2 และเมื่อนับครบตามที่เราตั้งไว้ต้องมาทำตามคำสั่งชุดนี้
// Timer2 Interrupt Service at 31372,550 KHz = 32uSec
// this is the timebase REFCLOCK for the DDS generator
// runtime : 8 microseconds ( inclusive push and pop)
ISR(TIMER2_OVF_vect) {
  // สูตรการคำนวน ความถี่กลับมา แล้วเลื่อน จาก 32 บิต กลับมาใช้เพียง 8 บิตล่าง
  sigma = sigma + delta; // soft DDS, phase accu with 32 bits
  phase0 = sigma >> 25; // use upper 8 bits for phase accu as frequency information
  //ชุดคำสั่งนี้แบ่งเป็น 2 ส่วน ชุดแรกเพื่อ ส่งสัญญาณ ซายเวฟ ทั้ง 3 คู่ ให้มีเฟสต่างกัน 0-90-180 เพื่อใช้กับมอร์เตอร์ซิงเกิ้ลเฟส และ ชุดต่อมา สำหรับ 3 เฟส 0-120-120

  if (single == true) { // ถ้าจั้มเปอร์เป็นจริง ให้ทำงานส่วนนี้ คือแบบซิงเกิลเฟส
    phase1 = (phase0 + 64) & 0b01111111;
    phase2 = (phase0 + 32) & 0b01111111;

    //สำหรับแบบซิงเกิ้ลเฟส เฟสคู่ที่ 0 กับ เฟสคู่ที่ 1 จะกลับกัน เพื่อให้มอร์เตอร์ หมุนทิศทางเดียวกันทั้งแบบ ซิงเกิ้ล และ แบบ 3 เฟส
    // นำค่าที่อ่านได้จากตาราง ซายเวฟ + ความแรงของสัญญาณ (Amplitude) + เฟส + ค่า DeadTime ออกไปที่ ขาสัญญาณออกทั้ง 3 คู่
    // OCR0A=(uint8_t)pgm_read_byte_near((Sine_Table + ADD_AMPD) + phase1)+DT_LOW;
    temp_ocr0a = pgm_read_byte_near(Sine_Table + phase1) * (Amplitude) / (TABLE_DIV) ;
    temp_ocr0b = pgm_read_byte_near(Sine_Table + phase1) * (Amplitude) / (TABLE_DIV) + DT;
    temp_ocr1a = pgm_read_byte_near(Sine_Table + phase0) * (Amplitude) / (TABLE_DIV) ;
    temp_ocr1b = pgm_read_byte_near(Sine_Table + phase0) * (Amplitude) / (TABLE_DIV) + DT;
    temp_ocr2a = pgm_read_byte_near(Sine_Table + phase2) * (Amplitude) / (TABLE_DIV) ;
    temp_ocr2b = pgm_read_byte_near(Sine_Table + phase2) * (Amplitude) / (TABLE_DIV) + DT;
    OCR0A = (uint8_t) temp_ocr0a; // pin D6
    OCR0B = (uint8_t) temp_ocr0b; // pin D5
    OCR1A = (uint8_t) temp_ocr1a; // pin D9
    OCR1B = (uint8_t) temp_ocr1b; // pin D10
    OCR2A = (uint8_t) temp_ocr2a; // pin D11
    OCR2B = (uint8_t) temp_ocr2b; // pin D3
  }

  //---------------------- ถ้าจั้มเปอร์เป็นเท็จ ให้ทำงานส่วนนี้ คือแบบ 3 เฟส ที่มีเฟสต่างกัน 120 องศา------------------------------------------------------------------
  else {

    // 3 phase
    phase1 = (phase0 + 42) & 0b01111111;
    phase2 = (phase0 + 84) & 0b01111111;

    // นำค่าที่อ่านได้จากตาราง ซายเวฟ + ความแรงของสัญญาณ (Amplitude) + เฟส + ค่า DeadTime ออกไปที่ ขาสัญญาณออกทั้ง 3 คู่
    temp_ocr0a = pgm_read_byte_near(Sine_Table + phase0) * (Amplitude) / (TABLE_DIV);
    temp_ocr0b = pgm_read_byte_near(Sine_Table + phase0) * (Amplitude) / (TABLE_DIV)+ DT;
    temp_ocr1a = pgm_read_byte_near(Sine_Table + phase1) * (Amplitude) / (TABLE_DIV);
    temp_ocr1b = pgm_read_byte_near(Sine_Table + phase1) * (Amplitude) / (TABLE_DIV)+ DT;
    temp_ocr2a = pgm_read_byte_near(Sine_Table + phase2) * (Amplitude) / (TABLE_DIV);
    temp_ocr2b = pgm_read_byte_near(Sine_Table + phase2) * (Amplitude) / (TABLE_DIV)+ DT;
    OCR0A = (uint8_t) temp_ocr0a; // pin D6
    OCR0B = (uint8_t) temp_ocr0b; // pin D5
    OCR1A = (uint8_t) temp_ocr1a; // pin D9
    OCR1B = (uint8_t) temp_ocr1b; // pin D10
    OCR2A = (uint8_t) temp_ocr2a; // pin D11
    OCR2B = (uint8_t) temp_ocr2b; // pin D3
  }

  if (c4ms++ == 500) {
    c4ms = 0;
    //digitalWrite(testPin, testPin_status);
    testPin_status = !testPin_status;
  }
}
