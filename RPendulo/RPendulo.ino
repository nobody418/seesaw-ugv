/***
   Intento de ajustar inercia de los motores
*/
#include <Arduino.h>    //Incluye librerias para arduino
#include <Wire.h>       //Incluye libreria para I2C

#define MPU 0x69        //Direccion I2C de la IMU
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY = 0, AcZ, AcY1;
int16_t GyX, GyY, GyZ;

//DEFINE PINES DRIVER de MOTORES L298N
#define LM1 0    //=D3   //IN1
#define LM2 2    //=D4   //IN2
#define RM1 13   //=D7   //IN3
#define RM2 15   //=D8   //IN4
#define PWML 14  //=D5   //ENA
#define PWMR 12  //=D6   //ENB

//Variable para el ancho de PWM de los motores
float e[3], u, a1, a2, a3;
int v = 0;

//Constantes de control
float kp = 2;
float kd = 0;
float ki = 5.8;

//Variables para control de tiempo
unsigned long t0 = 0;
float dt;

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

  e[0] = 0;
  e[1] = 0;
  e[2] = 0;
  u = 0;

  mov(0);
  motorStop();

  Serial.begin(9600);
}

void loop() {
  dt = float(millis() - t0) / 1000;
  t0 = millis();
  leeMPU();
  e[2] = e[1];
  e[1] = e[0];
  e[0] = 20 - 1.26 * float(AcY / 164) - 0.41 * float(GyX / 131);

  a1 = kp + 0.5 * ki * dt;
  a2 = 0.5 * ki * dt;
  a3 = 0;

  u = a1 * e[0] + a2 * e[1] + a3 * e[2];
  v = floor(u);

  Serial.println(String(v) + " , " + String(e[0]));
  mov(v);
  delay(10);
}


void mov(int v) { //Movimiento del robot
  if (v > 125) {
    v = 125;
  }
  if (v < -125) {
    v = -125;
  }
  if (v > 0) {
    motorFW();
    velMot(abs(140 + v));
  } else {
    motorBW();
    velMot(abs(-140 + v));
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
  AcY1 = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  AcY = floor(0.7 * float(AcY) + 0.3 * float(AcY1));

  //Leer los valores del Giroscopio
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //A partir del 0x43, se piden 6 registros
  GyX = Wire.read() << 8 | Wire.read(); //Cada valor ocupa 2 registros
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

}
