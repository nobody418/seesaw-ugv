#include <Arduino.h>    //Incluye librerias para arduino
#include <Wire.h>       //Incluye libreria para I2C

#define MPU 0x69        //Direccion I2C de la IMU
//Radios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ;
//Angulos
float Acc[2];
String valores;


//DEFINE PINES DRIVER de MOTORES L298N
#define LM1 0    //=D3   //IN1
#define LM2 2    //=D4   //IN2
#define RM1 13   //=D7   //IN3
#define RM2 15   //=D8   //IN4
#define PWML 14  //=D5   //ENA
#define PWMR 12  //=D6   //ENB

int v = 0;

void setup() {
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(9600);
}

void loop() {
  //Serial.println(estado);
  delay(400);
  leeMPU();

//  mov(v);
//  
//  v += 90;
//  if (v >= 255) {
//    v = -255;
//  }
}

void mov(int v) { //Movimiento del robot
  velMot(abs(v));
  if (v > 255) {
    v = 255;
  }
  if (v < -255) {
    v = -255;
  }
  if (v >= 0) {
    motorFW();
  } else {
    motorBW();
  }
}


void velMot(int v) { //Velocidad de motores
  analogWrite(PWML, v);
  analogWrite(PWMR, v);
}

void motorFW() { //Avance de motores
  digitalWrite(RM1, 1);
  digitalWrite(RM2, 0);
  digitalWrite(LM1, 1);
  digitalWrite(LM2, 0);
}

void motorBW() { //Retroceso de motores
  digitalWrite(RM1, 0);
  digitalWrite(RM2, 1);
  digitalWrite(LM1, 0);
  digitalWrite(LM2, 1);
}

void motorStop() { //Paro de motores
  digitalWrite(RM1, 0);
  digitalWrite(RM2, 0);
  digitalWrite(LM1, 0);
  digitalWrite(LM2, 0);
}

void leeMPU() {
  //Leer los valores del Acelerometro de la IMU
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //A partir del 0x3B, se piden 6 registros
  AcX = Wire.read() << 8 | Wire.read(); //Cada valor ocupa 2 registros
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  //Calculo de los angulos Y, X respectivamente, con la formula de la tangente.
  //Acc2[1] = atan(-1*(AcX2/A_R2)/sqrt(pow((AcY2/A_R2),2) + pow((AcZ2/A_R2),2)))*RAD_TO_DEG;
  //Acc2[0] = atan((AcY2/A_R2)/sqrt(pow((AcX2/A_R2),2) + pow((AcZ2/A_R2),2)))*RAD_TO_DEG;

  Acc[0] = atan((AcY) / sqrt(pow((AcX), 2) + pow((AcZ), 2)));


  valores = "90, " + String(Acc[0]) + "," ;
  Serial.println("x="+String(AcX)+", y="+String(AcY)+", z="+String(AcZ));
  Serial.println("Angulos MPU");
  Serial.println(valores);

  delay(10);
}
