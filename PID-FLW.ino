/*
 ****************CONTROL PID SEGUIDOR DE LINEA******************

   AUTOR: ARIEL ARTURO RÌOS SIERRA

   CORREO: arturi.marking@gmail.com

   FECHA: 12 de marzo del 2018

   VERSION: 1.0
 ***************************************************************
*/
#include <QTRSensors.h>
#include <Encoder.h>

#define ENCODER_OPTIMIZE_INTERRUPTS

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

bool estadoBoton = false;
bool estadoRobot = false;

float velocidadBase = 150.0;

const float kP = 0.0;
const float kI = 0.0;
const float kD = 0.0;


unsigned int posicion = 0;
unsigned int posicionAnterior = 0;
unsigned int derivativo = 0;
unsigned int integral = 0;

QTRSensorsAnalog qtra(sensores_ir, NUM_SENSOR_IR);

Encoder knobLeft(ENCO_A_IZQ, ENCO_B_IZQ);
Encoder knobRight(ENCO_A_DER, ENCO_B_DER);


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

  for (int i = 0; i < 5; i++) {
    qtra.calibrate();
  }

  adelante();
  digitalWrite(STBY, LOW);
}

void loop() {

  estadoBoton = digitalRead(BUTTON_STATUS);

  if (estadoBoton) {
    estadoRobot = !estadoRobot;
    delay(40);
  }

  if (!estadoRobot) {
    digitalWrite(STBY, LOW);
    loop();
  }

  inicioRobot();
}

void inicioRobot() {
  digitalWrite(STBY, HIGH);

  int errorPID = calculoPID;

  posicionAnterior = posicion;
}

unsigned int leerPosicionError() {
  qtra.read(valoresSensorIr);

  unsigned int posicion_actual = qtra.readLine(valoresSensorIr, QTR_EMITTERS_ON, 0);

  return (POS_OBJECTIVO - posicion_actual);
}

unsigned int calculoPID() {
  posicion = leerPosicionError();

  derivativo = posicion - posicionAnterior;

  integral = integral + posicion;

  return (kP * posicion + kI * integral + kD * derivativo);

}

void leerVelocidad() {
  long velocidadIzquierda = knobLeft.read();
  long velocidadDerecha = knobRight.read();
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
