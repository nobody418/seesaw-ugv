#define LM1 13
#define LM2 15
#define RM1 7
#define RM2 6
#define PWML 14
#define PWMR 12

int v = 0;
bool sb = 0;

void setup() {
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);

  //Serial.begin(9600);
}

void loop() {
  //Serial.println(v);
  mov(v);
  delay(300);
  if (v < 255 && !sb) {
    v += 90;
  }
  if (v >= 255) {
    sb = 1;
  }
  if (v > -255 && sb) {
    v -= 90;
  }
  if (v <= -255) {
    sb = 0;
  }
}

void mov(int v) {
  velMot(abs(v));
  if (v > 255) {
    v = 255;
  }
  if (v < -255) {
    v = -255;
  }
  if (v >= 0) {
    motorAv();
  } else {
    motorRe();
  }
}

void velMot(int v) {
  analogWrite(PWML, v);
  analogWrite(PWMR, v);
}

void motorAv() {
  digitalWrite(RM1, 1);
  digitalWrite(RM2, 0);
  digitalWrite(LM1, 1);
  digitalWrite(LM2, 0);
}

void motorRe() {
  digitalWrite(RM1, 0);
  digitalWrite(RM2, 1);
  digitalWrite(LM1, 0);
  digitalWrite(LM2, 1);
}
