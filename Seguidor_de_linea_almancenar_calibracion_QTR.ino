#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <QTRSensors.h>

#define Kp 1.45 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 16 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 255 // max speed of the robot
#define leftMaxSpeed 255 // max speed of the robot
#define rightBaseSpeed 255 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 255 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2
#define BATTERY_PIN A7
#define SAMPLE_BATTERY 1000
#define RESISTOR 11.0
#define rightMotorPWM          10
#define leftMotorPWM           11
/*#define rightMotor1            A1
 #define rightMotor2            A0
 #define rightMotorPWM          10
 // Motor B 
 #define leftMotor1             A3
 #define leftMotor2             A4
 #define leftMotorPWM           11
 #define motorPower             A2 */
const byte  AZUL =  13;
QTRSensorsRC qtrrc((unsigned char[]) {  
  2, 3, 4, 5, 6, 7, 8, 9 } 
,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19

unsigned int sensorValues[NUM_SENSORS];
unsigned int maximum_from_EEPROM[NUM_SENSORS];
unsigned int minimum_from_EEPROM[NUM_SENSORS];
byte resetButton=0; // Contador de veces que se reinicia el Arduino
int memory=511; // Posicion EEPROM en la que se almacenera el contador de reinicios EJ: Arduino UNO = 1000 Bytes pero Arduino Nano 512 Bytes
int SetPoint=3500;
float voltage;


void restore_sensor(){  // funcion para restaurar sensores una vez reiniciado el arduino
  qtrrc.resetCalibration();
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    minimum_from_EEPROM[i] = EEPROM.readInt(i*2); // se podria hacer en un solo for y evitar el otro, pero es para verlo de forma de ordenada el proceso
    maximum_from_EEPROM[i] = EEPROM.readInt((i*2)+(NUM_SENSORS*2));
  }
  qtrrc.calibrate(); // es obligatorio activar para poder restaurar los valores de calibracion
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    qtrrc.calibratedMinimumOn[i]=minimum_from_EEPROM[i] ;
    qtrrc.calibratedMaximumOn[i]=maximum_from_EEPROM[i];
  }

}

unsigned long timer=0;
void setup()
{
  timer=millis();

  DDRC = B11111100; // DECLARA PINES ANALOGICOS COMO SALIDA DEL A0-A5
  DDRB |= (1 << PORTB2); // DECLARA EL PIN 10 como salida PWM right
  DDRB |= (1 << PORTB3);// DECLARA EL PIN 11 como salida PWM left
  DDRB |= (1 << PORTB5); // LED 13

  resetButton=EEPROM.read(memory); //Guarda el valor de la EEPROM

  if (resetButton <= 20 || resetButton >40) resetButton=50;  // Se asigna el valor de 5 cada vez que sea evaluado
  resetButton-=10; // se Resta 1 cada vez que se resetea

  EEPROM.write(memory,resetButton); // se asigna el valor de resetButton a la EEPROM


  switch (resetButton){ // segun el numero de reinicios hay 3 acciones

  case 20:
    digitalWrite(AZUL,HIGH);
    delay(50);
    digitalWrite(AZUL,LOW);
    delay(250);
    while(1){
      stop();
    } 

    break;
  case 40:

    digitalWrite(AZUL,HIGH);
    int i;
    for (i = 0; i < 150; i++) {// calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead

      PORTC |= (1 << PORTC2) | (1 << PORTC0);
      PORTC &= ~(1 << PORTC1);
      analogWrite(rightMotorPWM, 0);
      PORTC |= (1 << PORTC2) | (1 << PORTC4);
      PORTC &= ~(1 << PORTC3);
      analogWrite(leftMotorPWM, 100); 

      qtrrc.calibrate();

    }

    stop();
    delay(500);
    digitalWrite(AZUL,LOW);
    for (int i=0; i <6; i++){  // parpadeo para indicar que termino la calibracion
      digitalWrite(AZUL,HIGH);
      delay(200);
      digitalWrite(AZUL,LOW);
      delay(200);
    } 
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      EEPROM.writeInt(i*2,qtrrc.calibratedMinimumOn[i]);     // la EEMPROM tiene capacidad de 8 bits osea que almacena un valor hasta 255, pero con la libreria EEMPROMex se usan 2 Bytes por Slot
      // y se puede guardar un int que es de 16 bits 
      EEPROM.writeInt((i*2)+(NUM_SENSORS*2),qtrrc.calibratedMaximumOn[i]);  
    }

    while(1){
      digitalWrite(AZUL,HIGH);
      stop(); 

    }
    break;
  case 30:

    digitalWrite(AZUL,LOW);
    restore_sensor(); 
    break;
  }
  delay(800);
} 

int lastError = 0;
int integral=0;
void loop()
{

  if( (millis()-timer >= SAMPLE_BATTERY)){
    voltage =(analogRead(BATTERY_PIN))*(5.0/1023.0)*RESISTOR;

    if(voltage <= 2.5){
      digitalWrite(AZUL,HIGH);
    }
 
    timer=millis(); 
  }
  int position = qtrrc.readLine(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - SetPoint;

  int motorSpeed = Kp * error +  (integral/100000) + Kd * (error - lastError) ;
  lastError = error;
  integral += error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

  {
    PORTC |= (1 << PORTC2) | (1 << PORTC0);
    PORTC &= ~(1 << PORTC1);
    analogWrite(rightMotorPWM, rightMotorSpeed);
    PORTC |= (1 << PORTC2) | (1 << PORTC4);
    PORTC &= ~(1 << PORTC3);
    analogWrite(leftMotorPWM, leftMotorSpeed);   
  }
}

void stop(){
  PORTC &= ~(1 << PORTC2);
}





