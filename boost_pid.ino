#include<PID_v2.h>
 

const byte n = 199;  
const float Kp = 0.0121394716387684, Ki=0.135412833850176, Kd = 0;

double Setpoint;
double Input;
double Output;

int PWM = 3;
int feedback = A0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() 
 {
  pinMode (PWM, OUTPUT);
  pinMode(A1, INPUT);

  TCCR2A = bit (WGM20) | bit (WGM21) | bit (COM2B1); 
  TCCR2B = bit (WGM22) | bit (CS21) | bit (CS21);         
  OCR2A =  n;                               
  //OCR2B = 19;             
  Serial.begin(9600);
  
  Setpoint = 651;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,150);
  //myPID.SetTunnings(Kp, Ki, Kd);
  } 


void loop() { 
  int Input = analogRead(feedback);
  double Vo = Input*0.0048828125;
  Vo = Vo * 6.405;
  myPID.Compute();
  analogWrite(PWM, Output);
  Serial.println(Input);
  Serial.println(Vo);
}
