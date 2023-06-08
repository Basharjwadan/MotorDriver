#include <Arduino.h>
#include "Motor.h"
#include "PID.h"

#define LEFT_ENC_PIN_A PD2  //pin 2
#define LEFT_ENC_PIN_B PD3  //pin 3
#define RIGHT_ENC_PIN_A PC4  //pin A4
#define RIGHT_ENC_PIN_B PC5   //pin A5

#define LEFT_MOTOR_PWM 6
#define LEFT_MOTOR_DIR 5
#define RIGHT_MOTOR_PWM 10
#define RIGHT_MOTOR_DIR 9


#define PID_TAU 0.02f
#define PID_LIM_MIN -255.0f
#define PID_LIM_MAX  255.0f
#define PID_LIM_MIN_INT -230.0f
#define PID_LIM_MAX_INT  230.0f
#define SAMPLE_TIME_S 0.03333f


#define PID_RATE 30 //HZ
const int c_PIDInterval = 1000/PID_RATE;
unsigned long NextPID = c_PIDInterval;
#define AUTO_STOP_INTERVAL 2000
long LastMotorCommand = AUTO_STOP_INTERVAL;

Motor LeftMotor;
Motor RightMotor;
PIDController LeftController;
PIDController RightController;
volatile long LeftEncCounts = 0;
volatile long RightEncCounts = 0;

static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR (PCINT2_vect){
  static uint8_t enc_last=0;        
  enc_last <<=2; //shift previous state two places
  enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  LeftEncCounts += ENC_STATES[(enc_last & 0x0f)];
};
  
/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR (PCINT1_vect){
  static uint8_t enc_last=0;      
  enc_last <<=2; //shift previous state two places
  enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  RightEncCounts += ENC_STATES[(enc_last & 0x0f)];
};


void setup() {
  // put your setup code here, to run once:
  LeftMotor.m_Dir = LEFT_MOTOR_DIR;
  LeftMotor.m_Pwm = LEFT_MOTOR_PWM;
  LeftMotor.m_CurrentMotorRate = 0;
  LeftMotor.m_DesiredMotorRate = 0;
  LeftMotor.m_PrevEncCounts = 0;

  RightMotor.m_Dir = RIGHT_MOTOR_DIR;
  RightMotor.m_Pwm = RIGHT_MOTOR_PWM;
  RightMotor.m_CurrentMotorRate = 0;
  RightMotor.m_DesiredMotorRate = 0;
  RightMotor.m_PrevEncCounts = 0;

  LeftController.Kp = 1.4f;
  LeftController.Ki = 6.0f;
  LeftController.Kd = 0.00f;
  LeftController.tau = PID_TAU;
  LeftController.T = SAMPLE_TIME_S;
  LeftController.limMin = PID_LIM_MIN;
  LeftController.limMax = PID_LIM_MAX;
  LeftController.limMinInt = PID_LIM_MIN_INT;
  LeftController.limMaxInt = PID_LIM_MAX_INT;
  LeftMotor.m_Pid = &LeftController;

  RightController.Kp = 1.4f;
  RightController.Ki = 6.0f;
  RightController.Kd = 0.00f;
  RightController.tau = PID_TAU;
  RightController.T = SAMPLE_TIME_S;
  RightController.limMin = PID_LIM_MIN;
  RightController.limMax = PID_LIM_MAX;
  RightController.limMinInt = PID_LIM_MIN_INT;
  RightController.limMaxInt = PID_LIM_MAX_INT;
  RightMotor.m_Pid = &RightController;

  
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


// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

bool SendEncoderCounts();
bool SendCurrentRates();
bool MotorSpeedCommand();
bool ShiftArrayBy(char* arr, int arrsize, int shifts);

void loop() {
   while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();
    Serial.print(chr);
    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      
      switch(cmd)
      {
        case 'e': Serial.println(SendEncoderCounts() ? "" : "invalid"); break;
        case 'c': Serial.println(SendCurrentRates() ? ""  : "invalid"); break;
        case 'm': Serial.println(MotorSpeedCommand() ? "ok" : "invalid"); break;
        default: Serial.println("invalid"); break;
      }
      
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }


  if (millis() > NextPID) {
    NextPID += c_PIDInterval;
    LeftMotor.m_CurrentMotorRate = LeftEncCounts - LeftMotor.m_PrevEncCounts;
    RightMotor.m_CurrentMotorRate = RightEncCounts - RightMotor.m_PrevEncCounts;

    MotorMove(&LeftMotor);
    MotorMove(&RightMotor);
    
    LeftMotor.m_PrevEncCounts = LeftEncCounts;
    RightMotor.m_PrevEncCounts = RightEncCounts;
  }
  
}


bool SendEncoderCounts()
{
  Serial.print(RightEncCounts);
  Serial.print(" ");
  Serial.println(LeftEncCounts);
  return true;
}

bool SendCurrentRates()
{
  Serial.print(RightMotor.m_CurrentMotorRate);
  Serial.print(" ");
  Serial.println(LeftMotor.m_CurrentMotorRate);
  return true;
}

bool MotorSpeedCommand()
{
  if (arg != 2)
    return false;
    
  RightMotor.m_DesiredMotorRate = atoi(argv2);
  LeftMotor.m_DesiredMotorRate = atoi(argv1);
  
  return true;
}
