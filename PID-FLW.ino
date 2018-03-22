/*
 ****************CONTROL PID SEGUIDOR DE LINEA******************

   AUTOR: ARIEL ARTURO RÌOS SIERRA

   CORREO: arturi.marking@gmail.com

   FECHA: 12 de marzo del 2018

   VERSION: 1.0
 ***************************************************************
*/
#include <QTRSensors.h>

#define AIN_1 7
#define AIN_2 8

#define BIN_1 10
#define BIN_2 13

#define PWMA 6
#define PWMB 11

#define STBY 9

#define ENCO_A_IZQ 2
#define ENCO_B_IZQ 4
#define ENCO_A_DER 3
#define ENCO_B_DER 5

#define POS_OBJECTIVO 3500

#define BUTTON_STATUS 12

#define NUM_SENSOR_IR 8

unsigned char sensores_ir[] = {0, 1, 2, 3, 4, 5, 6, 7};
unsigned int valoresSensorIr[NUM_SENSOR_IR];

int estadoBoton = HIGH;
int esperaTiempo = 560;
long tiempo = 0;
int previous = LOW;
bool estadoRobot = false;

const float velocidadBase = 100.0;

const float kP = 0.0;
const float kI = 0.0;
const float kD = 0.0;

volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;

float rpmIzq = 0;
float rpmDer = 0;
float velocidadIzq = 0;
float velocidadDer = 0;

unsigned long tiempoAnterior = 0;
unsigned int pulsosPorRevolucion = 8;

unsigned int diametroRueda = 20;
int relacionRuedas = 10;

unsigned int posicion_actual = 0;
unsigned int posicion = 0;
unsigned int posicionAnterior = 0;
unsigned int derivativo = 0;
unsigned int integral = 0;

QTRSensorsAnalog qtra(sensores_ir, NUM_SENSOR_IR);


void setup() {
  pinMode(BUTTON_STATUS, INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN_1, OUTPUT);
  pinMode(AIN_2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);

  pinMode(STBY, OUTPUT);

  pinMode(ENCO_A_IZQ, INPUT);
  pinMode(ENCO_B_IZQ, INPUT);

  pinMode(ENCO_A_DER, INPUT);
  pinMode(ENCO_B_DER, INPUT);

  attachInterrupt(0, leftEncoderEvento, CHANGE);
  attachInterrupt(1, rightEncoderEvento, CHANGE);

  for (int i = 0; i < 5; i++) {
    qtra.calibrate();
  }
}

void leftEncoderEvento() {
  if (digitalRead(ENCO_A_IZQ) == HIGH) {
    if (digitalRead(ENCO_B_IZQ) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(ENCO_B_IZQ) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}

void rightEncoderEvento() {
  if (digitalRead(ENCO_A_DER) == HIGH) {
    if (digitalRead(ENCO_B_DER) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(ENCO_B_DER) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}

void loop() {

  estadoBoton = digitalRead(BUTTON_STATUS);
  delay(40);

  if (estadoBoton == HIGH && previous == LOW && millis() - tiempo > esperaTiempo) {
    if (estadoRobot)
      estadoRobot = false;
    else
      estadoRobot = true;
    tiempo = millis();
  }

  if (!estadoRobot) {
    int motoresEncendidos = digitalRead(STBY);
    if (motoresEncendidos == 1) {
      digitalWrite(STBY, LOW);
    }
    loop();
  }

  inicioRobot();
  previous = estadoBoton;
}

void inicioRobot() {
  digitalWrite(STBY, HIGH);

  leerVelocidades();

  int errorPID = calculoPID();

  float velocidadActualIzq = velocidadIzq - errorPID;
  float velocidadActualDer = velocidadDer + errorPID;

  float velocidadRPMIzq = rpmIzq - errorPID;
  float velocidadRPMDer = rpmDer + errorPID;

  float potenciaPWMIzq = map(velocidadRPMIzq, 0, 3000, 0, 255);
  float potenciaPWMDer = map(velocidadRPMDer, 0, 3000, 0, 255);

  if (potenciaPWMIzq >= velocidadBase) {
    potenciaPWMIzq = velocidadBase;
  }

  if (potenciaPWMDer >= velocidadBase) {
    potenciaPWMDer = velocidadBase;
  }

  analogWrite(PWMA, potenciaPWMDer);
  analogWrite(PWMB, potenciaPWMIzq);
  adelante();

  posicionAnterior = posicion;
}

void leerVelocidades() {
  long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior >= 1000) {
    noInterrupts();
    rpmIzq = 60 * leftCount / pulsosPorRevolucion * 1000 / (millis() - tiempoAnterior);

    rpmDer = 60 * rightCount / pulsosPorRevolucion * 1000 / (millis() - tiempoAnterior);

    velocidadIzq = rpmIzq / relacionRuedas * 3.1416 * diametroRueda * 60 / 1000000;

    velocidadIzq = rpmDer / relacionRuedas * 3.1416 * diametroRueda * 60 / 1000000;

    leftCount = 0;
    rightCount = 0;

    tiempoAnterior = millis();
    interrupts();
  }
}

unsigned int calculoPID() {
  posicion = leerPosicionError();

  derivativo = posicion - posicionAnterior;

  integral = integral + posicion;

  return (kP * posicion + kI * integral + kD * derivativo);

}

unsigned int leerPosicionError() {
  qtra.read(valoresSensorIr);

  posicion_actual = qtra.readLine(valoresSensorIr, QTR_EMITTERS_ON, 0);

  return (POS_OBJECTIVO - posicion_actual);
}

void adelante() {
  digitalWrite(AIN_1, LOW);
  digitalWrite(AIN_2, HIGH);

  digitalWrite(BIN_1, HIGH);
  digitalWrite(BIN_2, LOW);
}

void reversa() {
  digitalWrite(AIN_1, HIGH);
  digitalWrite(AIN_2, LOW);

  digitalWrite(BIN_1, LOW);
  digitalWrite(BIN_2, HIGH);
}
