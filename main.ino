#include <Arduino.h>
#include "Motor.h"

#define LEFT_ENC_PIN_A PD2  //pin 2
#define LEFT_ENC_PIN_B PD3  //pin 3
#define RIGHT_ENC_PIN_A PC4  //pin A4
#define RIGHT_ENC_PIN_B PC5   //pin A5

#define LEFT_MOTOR_PWM 6
#define LEFT_MOTOR_DIR 5
#define RIGHT_MOTOR_PWM 10
#define RIGHT_MOTOR_DIR 9

Motor* LeftMotor;
Motor* RightMotor;


#define PID_RATE 30 //HZ
const int c_PIDInterval = 1000/PID_RATE;
unsigned long NextPID = c_PIDInterval;
#define AUTO_STOP_INTERVAL 2000
long LastMotorCommand = AUTO_STOP_INTERVAL;

char chr;


static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR (PCINT2_vect){
  static uint8_t enc_last=0;        
  enc_last <<=2; //shift previous state two places
  enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  LeftMotor->GetEncCounts() += ENC_STATES[(enc_last & 0x0f)];
}
  
/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR (PCINT1_vect){
  static uint8_t enc_last=0;      
  enc_last <<=2; //shift previous state two places
  enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  RightMotor->GetEncCounts() += ENC_STATES[(enc_last & 0x0f)];
}


void ParseEncoderCommand();
void ParseMotorCommand();
void ParseGainCommand();

void setup() {
  // put your setup code here, to run once:
  LeftMotor = new Motor(LEFT_MOTOR_DIR, LEFT_MOTOR_PWM);
  //RightMotor = new Motor(RIGHT_MOTOR_DIR, RIGHT_MOTOR_PWM);
  LeftMotor->SetMotorRate(30);
  
  DDRD &= ~(1<<LEFT_ENC_PIN_A);
  DDRD &= ~(1<<LEFT_ENC_PIN_B);
  DDRC &= ~(1<<RIGHT_ENC_PIN_A);
  DDRC &= ~(1<<RIGHT_ENC_PIN_B);
  
  // enable pull up resistors
  PORTD |= (1<<LEFT_ENC_PIN_A);
  PORTD |= (1<<LEFT_ENC_PIN_B);
  PORTC |= (1<<RIGHT_ENC_PIN_A);
  PORTC |= (1<<RIGHT_ENC_PIN_B);
  
  // tell pin change mask to listen to left encoder pins
  PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
  // tell pin change mask to listen to right encoder pins
  PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
  
  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  PCICR |= (1 << PCIE1) | (1 << PCIE2);


  Serial.begin(115200);
}


void loop() {
  /*
  // put your main code here, to run repeatedly:
 if (Serial.available() > 0) {
    // Read the next character
  chr = Serial.read();

  switch (chr)
  {
  case 'e': ParseEncoderCommand(); break;
  case 'm': ParseMotorCommand(); break;
  case 'g': ParseGainCommand(); break;
  default:
    Serial.write("Invalid Command");
    break;
  }

 }
  */
  LeftMotor->SetMotorRate(30);

  if (millis() > NextPID) {
    LeftMotor->Move();
    NextPID += c_PIDInterval;

    Serial.print("Enc Counts: ");
    Serial.println(LeftMotor->GetEncCounts());
  }


  
}

void ParseEncoderCommand()
{
  while (Serial.available() > 0)
  {
     chr = Serial.read();
    // Terminate a command with a CR
    if (chr == 13) 
    {
      //Serial.print(RightMotor->GetEncCounts());
      //Serial.print(" ");
      Serial.println(LeftMotor->GetEncCounts());
      return;    
    }
  }
  
}
void ParseMotorCommand()
{
  LastMotorCommand = millis();
  
  
}
void ParseGainCommand()
{
  
}
