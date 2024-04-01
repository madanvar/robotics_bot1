#include <PS4BT.h>
// #include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#define motor1_pwm 8
#define motor1_dir 26
#define motor2_pwm 9
#define motor2_dir 28
#define motor3_pwm 13
#define motor3_dir 46
#define motor4_pwm 12
#define motor4_dir 44
#define pulley_pwm 12
#define pulley_dir 44
#define throw_pwm1 4
#define throw_dir1 33
USB Usb;
BTD Btd(&Usb); 
PS4BT PS4(&Btd, PAIR);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;
#include <Servo.h>

int counter=0;
const int a = 18;
const int b = 19;
int f=1;
int f1=1;
int f2=1;
Servo servo1;
Servo servo2;
Servo servo3;

// #define throw_pwm2 5
// #define throw_dir2 34
// int spd1=40;
int s=40;
int Speed=100;
int max=137;
int min=117;



void setup() {
  Serial.begin(115200);
#if !defined(MIPSEL)
  while (!Serial); 
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));

  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor3_pwm, OUTPUT);
  pinMode(motor4_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor3_dir, OUTPUT);
  pinMode(motor4_dir, OUTPUT);
  pinMode(pulley_pwm,OUTPUT);
  pinMode(pulley_dir,OUTPUT);

  servo1.attach(9);
  servo2.attach(8);
  servo3.attach(7);
  pinMode(a,INPUT);
  pinMode(b,INPUT);
  pinMode(throw_pwm1,OUTPUT);
  // pinMode(throw_pwm2,OUTPUT);
  pinMode(throw_dir1,OUTPUT);
  // pinMode(throw_dir2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(a),func,FALLING);

}
void loop() {
  Usb.Task();

  if (PS4.connected()) {
    Serial.println("\nPS4 controller connected");
    int lx=PS4.getAnalogHat(LeftHatX);
    int ly=PS4.getAnalogHat(LeftHatY);
    int rx=PS4.getAnalogHat(RightHatX);
    int ry=PS4.getAnalogHat(RightHatY);
    int l2=PS4.getAnalogButton(L2);
    int r2=PS4.getAnalogButton(R2);
    // int up=PS4.getButtonClick(TRIANGLE);
    
    if(ly > max && lx < max+20 && lx > min-20){
      Speed=map(ly,max,255,10,250);
      moveBackward(Speed);
      Serial.println("\B");
      
    }
    else if(ly < min && lx < max+20 && lx > min-20){
      Speed=map(ly,min,0,10,250);
      moveForward(Speed);
      Serial.println("\F");
    }
    else if(lx > max && ly < max+10 && ly > min-10){
      Speed=map(lx,max,255,10,250);
      MoveRight(Speed);
      Serial.println("R");
    }
    else if(lx<min && ly< max+10 && ly>min+10){
      Speed=map(lx,min,0,10,250);
      MoveLeft(Speed);
      Serial.println("\L");
    }
    else if(lx < max && lx > min && ly < max && ly > min){
      stopMotors();
    }
    else if(l2>15){
      Speed=map(l2,16,255,10,250);
      RotateLeft(Speed);
      Serial.println("\RL");

    }
    else if(r2>15){
      Speed=map(r2,16,255,10,250);
      RotateRight(Speed);
      Serial.println("\RR");

    }
    else if(ry > max && rx < max+30 && rx > min-30){
      int spd1=map(ry,max,255,10,250);
      PulleyDown(spd1);
      Serial.println("down");

    }
    
    else if(ry < min && rx < max+20 && rx > min-20){
      int spd1=map(ry,min,0,10,255);
      PulleyUp(spd1);
      Serial.println("upppp");
    }
    else if(PS4.getButtonClick(TRIANGLE)){
      if (f1==0){
        servo_open();
        Serial.println("open");
        f1=1;
      }else{
        f1==1;
        servo_close();
        Serial.println("close");

      }

    }
    
    else if(PS4.getButtonClick(SQUARE)){
      if(f2==0){
        mech1();
        Serial.println("throww");
        f2=1;
      }else{
        f2==1;
        mech2();
        Serial.println("down");
      }
    }
    
    else if(PS4.getButtonClick(UP)){
      if(f==0){
        lose();
        Serial.println("pick");
        f=1;
      }else{
        f==1;
        pick();
        Serial.println("lose");
      }
      }
      else if(rx < max && rx > min && ry < max && ry > min){
      stop1();
      }
    }
  }



void moveForward( int Speed) {
  analogWrite(motor1_pwm, Speed);
  analogWrite(motor2_pwm, Speed);
  analogWrite(motor3_pwm, Speed);
  analogWrite(motor4_pwm, Speed);
  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, HIGH);
  digitalWrite(motor3_dir, HIGH);
  digitalWrite(motor4_dir, HIGH);
}

void moveBackward(int Speed){
  analogWrite(motor1_pwm, Speed);
  analogWrite(motor2_pwm, Speed);
  analogWrite(motor3_pwm, Speed);
  analogWrite(motor4_pwm, Speed);
  digitalWrite(motor1_dir, LOW);
  digitalWrite(motor2_dir, LOW);
  digitalWrite(motor3_dir, LOW);
  digitalWrite(motor4_dir, LOW);
}

void MoveLeft(int Speed){
  analogWrite(motor1_pwm, Speed);
  analogWrite(motor2_pwm, Speed);
  analogWrite(motor3_pwm, Speed);
  analogWrite(motor4_pwm, Speed);
  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, LOW);
  digitalWrite(motor3_dir, HIGH);
  digitalWrite(motor4_dir, LOW);
}

void MoveRight(int Speed){
  analogWrite(motor1_pwm, Speed);
  analogWrite(motor2_pwm, Speed);
  analogWrite(motor3_pwm, Speed);
  analogWrite(motor4_pwm, Speed);
  digitalWrite(motor1_dir, LOW);
  digitalWrite(motor2_dir, HIGH);
  digitalWrite(motor3_dir, LOW);
  digitalWrite(motor4_dir, HIGH);
}

void RotateRight(int Speed){
  analogWrite(motor1_pwm, Speed);
  analogWrite(motor2_pwm, Speed);
  analogWrite(motor3_pwm, Speed);
  analogWrite(motor4_pwm, Speed);
  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, HIGH);
  digitalWrite(motor3_dir, HIGH);
  digitalWrite(motor4_dir, HIGH);
}

void RotateLeft(int Speed){
  analogWrite(motor1_pwm, Speed);
  analogWrite(motor2_pwm, Speed);
  analogWrite(motor3_pwm, Speed);
  analogWrite(motor4_pwm, Speed);
  digitalWrite(motor1_dir, LOW);
  digitalWrite(motor2_dir, LOW);
  digitalWrite(motor3_dir, LOW);
  digitalWrite(motor4_dir, LOW);
}

void PulleyUp(int spd1){
  
  analogWrite(pulley_pwm, spd1);
  digitalWrite(pulley_dir, LOW);
}

void PulleyDown(int spd1){
  analogWrite(pulley_pwm, spd1);
  digitalWrite(pulley_dir, HIGH);
}
void stop1(){
  analogWrite(pulley_pwm, 0);
}



void stopMotors(){
  analogWrite(motor1_pwm, 0);
  analogWrite(motor2_pwm, 0);
  analogWrite(motor3_pwm, 0);
  analogWrite(motor4_pwm, 0);
}
void servo_open(){
  servo1.write(0);
  servo2.write(90);
}
void servo_close(){
  servo1.write(90);
  servo2.write(0);
}
void pick(){
  servo3.write(0);
}
void lose(){
  servo3.write(90);

}
void throww(int s){
  analogWrite(throw_pwm1, s);
  // analogWrite(throw_pwm2, s);
  digitalWrite(throw_dir1, HIGH);
  // digitalWrite(throw_dir2, HIGH);
}
void down(int s){
  analogWrite(throw_pwm1, s);
  // analogWrite(throw_pwm2, s);
  digitalWrite(throw_dir1, LOW);
  // digitalWrite(throw_dir2, LOW);

}
void func(){

  int x = digitalRead(b);
  if (x>0){
    counter++;
  }
  else{
    counter--;
  }
}
void stopp(){  
  analogWrite(throw_pwm1, 0);
  // analogWrite(throw_pwm2, 0);
}
void mech1(){
    Serial.println(counter);
    if (counter>=-10 && counter<=25){
      throww(s);
    }
    else if (counter>25 && counter<50){ 
      stopp();
    }
    
}
void mech2(){
  down(s);
}
