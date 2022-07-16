/*
 * Robot Balancin o Pendulo invertido
 * V2.0
 * Para arduino UNO, NANO
 */
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include <SoftwareSerial.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
#define MIN_ABS_SPEED 40
MPU6050 mpu;

//#include <SD.h>
//#include <SPI.h>
//File myFile;
// 4   CS/ SS uSD
//11   MOSI   uSD
//12   MISO   uSD
//13   CLK    uSD
  
//SoftwareSerial Serial_2 (4, 3);

//**** Control de Motores puente H - L298N
int ENA = 5;    //5;
int IN1 = 7;    //6;
int IN2 = 8;    //7;
int IN3 = 9;    //9;
int IN4 = 10;   //8;
int ENB = 6;    //10;

//**** MPU control/status variables
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//**** Variables de orientación
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//**** PID set - Los Valors de PID cambian con cada diseño
double Kp = 62;   //70;  //60;  //double Kp = 60; 
double Kd = 2.6;  //4.0;  //3.5;  //double Kd = 2.2;  
double Ki = 220;//220;  //220;  //250;  //250;  //double Ki = 270;  

//factor de 0 a 100% de activacion del PWM, sirve para corregir cuando un motor esta "lento"
double motorSpeedFactorLeft = 0.51;//0.46; //double motorSpeedFactorLeft = 0.5;
double motorSpeedFactorRight = 0.45;//0.40; //double motorSpeedFactorRight = 0.45;

//Set point buscado
double originalSetpoint = 179.2;   //double originalSetpoint = 172.50;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady(){
  mpuInterrupt = true;
}

//**** Para envio
String SerialData="";
String SerialData2="";
String SerialData3="";
unsigned long tiempoInicial = 0;
unsigned long tiempoFinal = 0;


void setup(){
  delay(500);
  Serial.begin(115200); // inicia el puerto serial para comunicacion //115200//250000
  //Serial_2.begin(9600);
  //Serial.setTimeout(50);
  /*
  if (SD.begin()){ // SD Card Initialization
    Serial.println(F("SD card is ready to use."));
  } else {
    Serial.println(F("SD card initialization failed"));
    return;
  }
  */
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);  //mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);   //mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);  //mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);  // turn on the DMP, now that it's ready
    // Habilita el pin 2 Interrupcion 0 (la interrupción 0 está relacionada con el pin 2) 
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
    Serial.print(F("Inicio Normal PID, MPU, Motores"));
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop(){  
  tiempoInicial = micros();
  //Serial.print(F("Loop principal"));
  // if programming failed, don't try to do anything  
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize){
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
    //Serial.println(output);
  }

  //reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  //get current FIFO count
  fifoCount = mpu.getFIFOCount();

  //check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024){
    //reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));    
    //otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02){
    //wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 57.29 + 180;    //180/M_pi = 180/3.1415

    //Serial.println(input);
    //Serial.write(input);

    //AQUI EL ERROR
    /*tiempoInicial = micros();    
    SerialData = String(input,2);    
    Serial.print(SerialData);
    Serial.print(',');
    SerialData2 = String(output,2);
    Serial.print(SerialData2);        
    Serial.print(',');
    Serial.print(micros()-tiempoInicial);
    Serial.println(',');
    */
    
    Serial.write(',');  //por la fecha del lector serial en el celular    
    SerialData = String(input,2);
    writeString(SerialData);
    Serial.write(',');
    SerialData2 = String(output,2);
    writeString(SerialData2);
    Serial.write(',');

    tiempoFinal = micros()-tiempoInicial;
    SerialData3 = String(tiempoFinal);
    writeString(SerialData3);
    Serial.write(';');
    Serial.write('\n');
    
    //writeString(SerialData2);
    //Serial.write(',');

    //writeString(SerialData+","+SerialData2+";");
    //writeSD();
  }
  
}
void writeString(String stringData){ //serially push out a String with Serial.write()
  for (int i=0; i<stringData.length(); i++)
     Serial.write(stringData[i]);   //Push each char 1 by 1 on each loop pass
}


/*
void writeSD(){
  myFile = SD.open("datalog.TXT", FILE_WRITE); // Abrir archivo y escribir valor
  if (myFile){
    myFile.print("Time(ms)=");
    myFile.print(millis());
    myFile.print(", input=");
    myFile.print(input);
    myFile.print(", output=");
    myFile.println(output);
    myFile.close();
  } 
  else  Serial.println(F("Error al abrir el archivo"));
}
*/

//**** Parametros con los cuales jugar
// MIN_ABS_SPEED 30  //Velocidad minima del PWM

// double PuntoEquilibrio = 179.7; //180.5; //SET POINT BUSCADO
//kp, ki, kd

//double motorSpeedFactorLeft = 0.6;    //CORRIGE SI UN MOTOR ES MAS LENTO QUE EL OTRO
//double motorSpeedFactorRight = 0.5;
