#include <ps5Controller.h>
uint8_t dz = 15;
#define F1 12
#define F2 14
#define B1 27
#define B2 26

void setup() {
pinMode(F1,OUTPUT); // F
pinMode(F2,OUTPUT); // F
pinMode(B1,OUTPUT); //
pinMode(B2,OUTPUT);
  
  Serial.begin(115200);   // Debug
  Serial2.begin(115200, SERIAL_8N1, 3, 1); // TX=17, RX=16 (change as per wiring)

  ps5.begin("E8:47:3A:55:1F:5A");  // Replace with your ESP32 BT MAC
    Serial.println("Waiting for ps5...");
    Dead_centre();

}
void Dead_centre(void){
  digitalWrite(F1,0);
  digitalWrite(F2,0);
  digitalWrite(B1,0);
  digitalWrite(B2,0);
}
void All_up(void){
  digitalWrite(F1,1);
  digitalWrite(F2,0);
  digitalWrite(B1,1);
  digitalWrite(B2,0);
}
void F_ret(void){
  digitalWrite(F1,0);
  digitalWrite(F2,1);
}
void F_ext(void){
  digitalWrite(F1,1);
  digitalWrite(F2,0);
}
void B_ret(void){
  digitalWrite(B1,0);
  digitalWrite(B2,1);
}
void B_ext(void){
  digitalWrite(B1,1);
  digitalWrite(B2,0);
}
void All_down(void){
  digitalWrite(F1,0);
  digitalWrite(F2,1);
  digitalWrite(B1,0);
  digitalWrite(B2,1);
}
void loop() {
  if (ps5.isConnected()) {

  int lx = ps5.LStickX();   // 0–255
    int ly = ps5.LStickY();
    int rx = ps5.RStickX();
    int ry = ps5.RStickY();
    lx = map(lx,-128,127,-255,255);
    ly = map(ly,-128,127,-255,255);
    rx = map(rx,-128,127,-255,255);

    if(abs(ly) <= dz) ly = 0;
    if(abs(lx) <= dz) lx = 0;
    if(abs(rx) <= dz) rx = 0;

    int l2 = ps5.L2Value();  // 0–255
    int r2 = ps5.R2Value();

int l1 = ps5.L1();
int r1 = ps5.R1();

  //Serial.printf("LX:%d  LY:%d  RX:%d  RY:%d\n", lx, ly, rx, ry);  // Debug on PC
  Serial2.printf("%d,%d,%d,%d,%d,%d,%d,%d\n", lx, ly, rx, ry, l2, r2, l1,r1);  // Send to STM32

     char buf[50];
    sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d\n", lx, ly, rx, ry, l2, r2, l1,r1);
    Serial2.print(buf);   // TX → STM32
    // Step_down
if(l1 && r1) {All_up(); delay(700); Dead_centre(); }
else Dead_centre();
if(l2) F_ret();
if(r2) B_ret();

    // Step_down
if(l1){ F_ext();}
if(r1) B_ext();
if(l2 && r2) All_down();
    
  delay(10);
}

}