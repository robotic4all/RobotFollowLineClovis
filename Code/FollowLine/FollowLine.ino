/*
  #####################
	# ROBOT FOLLOW LINE #
	# TEAM ROBOTIC4ALL  #
	#	AUGUST 2019	      #
	#####################
*/

/* INCLUINDO BIBLIOTECAS */
#include <SparkFun_TB6612.h>
#include <QTRSensors.h>

/*Declarando Variveis dos Motores*/
#define motorLeftSpeed    11
#define motorLeftA        10
#define motorLeftB        9
#define stby              8
#define motorRightA       6
#define motorRightB       7
#define motorRightSpeed   5

const int offsetA =  1;
const int offsetB = -1;

//Etanciando biblioteca dos motores
Motor motorLeft  = Motor(motorLeftA,  motorLeftB,  motorLeftSpeed,  offsetA, stby);
Motor motorRight = Motor(motorRightA, motorRightB, motorRightSpeed, offsetB, stby);

//Variaveis Auxiliares Velocidade
#define speedBase  60
#define speedMin   0
#define speedMax   200

/*Declarando variaveis do Array*/
QTRSensors arraySensors;
const uint8_t sensorCount = 6;
uint16_t valuesSensors[sensorCount];

#define emitterPin 12

/*VARIAVEIS PID*/
float   Kp = 0.08,  //0.1;  0.8;    0.06
        Ki = 0.0,
        Kd = 0.6;   //1.0;  8.0;    1.0

uint16_t position = 0;

int error         = 0;
int setPoint      = 2500;

int P = 0;
int I = 0;
int D = 0;

int PIDValue  = 0;

int lastError = 0;
int countI    = 0;

/*SENSORES LATERAIS*/
QTRSensors sensorSideLeft;
QTRSensors sensorSideRight;

uint16_t vSSLeft  = 0,
         vSSRight = 0;

int media = 600;
int final = 0;

int geo  = 0;
int geo1 = 0;
int geo2 = 0;
int geo3 = 0;
int geo4 = 0;
int geo5 = 0;

/*BUZZER */
#define buzzer 13

/*LEDs*/
#define led 4

/*BOTÔES*/
#define btnLeft  3
#define btnRight 2

void onLed(int time){
  digitalWrite(led, HIGH);
  delay(time);
  digitalWrite(led, LOW);
}

void onLed(){
  digitalWrite(led, HIGH);
}

void offLed(){
  digitalWrite(led, LOW);
}

void onBuzzer(int time){
  digitalWrite(buzzer, HIGH);
  delay(time);
  digitalWrite(buzzer, LOW);
}

void onBuzzer(){
  digitalWrite(buzzer, HIGH);
}

void offBuzzer(){
  digitalWrite(buzzer, LOW);
}

void initRobot(){
  //Inicializando Array
  arraySensors.setTypeAnalog();
  arraySensors.setSensorPins((const uint8_t[]){A5, A4, A3, A2, A1, A0}, sensorCount);
  arraySensors.setEmitterPin(emitterPin);

  //Inicializando Sensor Lateral Esquerdo 
  sensorSideLeft.setTypeAnalog();
  sensorSideLeft.setSensorPins((const uint8_t[]){A6}, 1);
  sensorSideLeft.setEmitterPin(A6);

  //Inicializando Sensor Lateral Direito
  sensorSideRight.setTypeAnalog();
  sensorSideRight.setSensorPins((const uint8_t[]){A7}, 1);
  sensorSideRight.setEmitterPin(A7);

  //Led e Buzzer
  onLed(500);
  onBuzzer(100);
  onLed(500);
}

void calibrateArray() {
  Serial.println("Calibrando Sensores Array...");

  onLed();

  for(uint16_t i = 0; i < 100; i++){ arraySensors.calibrate(); }

  offLed();

  Serial.println("Sensores Calibrados!");

  delay(2000);
}

void calibrateSideSensors(){
  Serial.println("Calibrando Sensores Laterais...");

  onLed();

  for(uint16_t i = 0; i < 100; i++){
    sensorSideLeft.calibrate();
    sensorSideRight.calibrate();
  }

  offLed();

  Serial.println("Sensores Calibrados!");

  delay(2000);
}

bool btnPressed(int btn){
  if (digitalRead(btn) == HIGH)
    return true;
  else
    return false;
}

void followLine(){
  //Parada pelo Sensor Lateral
  //detectMarker();

  // Cálculo do PID.
  position = arraySensors.readLineWhite(valuesSensors);
  /*Serial.println(position);
  delay(250);*/

  error = position - setPoint;

  P = (Kp * error);
  I = (Ki * countI);
  D = (Kd * (error - lastError));

  PIDValue = P + I + D;

  // Atualiza os valores de I e do erroAnterior.
  lastError = error;
  countI += error;

  // Atribuição da velocidade dos motores.
  int speedRight = speedBase + PIDValue;
  int speedLeft = speedBase - PIDValue;

  // Checa se as velocidades ultrapassam 255 ou são mensores que 0, corrige caso necessario
  if (speedRight > speedMax)
    speedRight = speedMax;
  if (speedLeft > speedMax)
    speedLeft = speedMax;
  if (speedRight < speedMin)
    speedRight = 0;
  if (speedLeft < speedMin)
    speedLeft = 0;

  //Atribui as velocidades aos motores
  motorLeft.drive(speedLeft);
  motorRight.drive(speedRight);
}

void readSideSensors() {
  vSSLeft  = sensorSideLeft.readLineWhite(1);
  vSSRight = sensorSideRight.readLineWhite(1);

  if(vSSLeft < media) {vSSLeft = 1;} 
  else {vSSLeft = 0;}

  if(vSSRight < media) {vSSRight = 1;} 
  else {vSSRight = 0;}

  /*
  Serial.print("vSSLeft = ");
  Serial.println(vSSLeft);
  Serial.print("vSSRight = ");
  Serial.println(vSSRight);
  Serial.println();
  */
}

void detectMarker() {
  readSideSensors();

  if(vSSLeft == 0 && vSSRight == 0) { geo = 0; }
  if(vSSLeft == 1 && vSSRight == 0) { geo = 1; }
  if(vSSLeft == 0 && vSSRight == 1) { geo = 2; }
  if(vSSLeft == 1 && vSSRight == 1) { geo = 3; }

  if(geo1 == geo){
    offLed();
    offBuzzer();
  }
  else if(geo1 != geo) {
    if(geo == 0 && geo1 == 1 && geo2 == 0) {
      final++;
      markerRight();
    }
    else if(geo == 0 && geo1 == 2 && geo2 == 0) {
      markerLeft();
    }
    else if(geo == 0 && ((geo1 == 3) || (geo2 == 3) || (geo3 == 3))) {
      intersection();
    }
    geo5 = geo4;
    geo4 = geo3;
    geo3 = geo2;
    geo2 = geo1;
    geo1 = geo;
  }
}

void intersection(){
  onLed();
  onBuzzer();
}

void markerLeft(){
  onLed();
  onBuzzer();
}

void markerRight() {
  onLed();
  onBuzzer();

  if(final >= 2) {
    motorLeft.brake();
    motorRight.brake();
    delay(10000);
    final = 0;
  } 
}
void startRun(){
  onBuzzer(1000);
  delay(10);
  onLed();
  onBuzzer(100);
  delay(10);
  offLed();
  onBuzzer(100);
  delay(10);
  onLed();
  onBuzzer(100);
  delay(10);
  offLed();
  onBuzzer(1000);
}

void finishRun(){
  onBuzzer(100);
  delay(10);
  onLed();
  onBuzzer(100);
  delay(10);
  offLed();
  onBuzzer(100);
  delay(10);
  onLed();
  onBuzzer(100);
  delay(10);
  offLed();
  onBuzzer(100);
}

void setup() {
  Serial.begin(9600);

  //Setas os pinos como entrada ou saida
  pinMode(buzzer, OUTPUT);
  pinMode(led,    OUTPUT);
  pinMode(btnLeft, INPUT);
  pinMode(btnRight,INPUT);

  initRobot();  
  
}

void loop() {
  if(btnPressed(btnRight)) && btnPressed(btnLeft){
    startRun();
    while (!btnPressed(btnLeft)){
      followLine();
    }
    finishRun();
  }
  if(btnPressed(btnLeft)){
    calibrateSideSensors();
  }
  if(btnPressed(btnRight)){
    calibrateArray();
  }

}
