/* Position Control PID v1
 * Antonio Vasquez Briones
 * 
 * Control the position of a cart at certain distance by driving a dc motor with pwm. 
 * The position is sensed by a two channel optical encoder placed at the cart.
 */


#include <PID_v1.h>

//Pin Declaration
#define   motorPin   11 // motorPWM
#define   turnPin    13 // motor CC or CCW
#define   chanA      2  // encoder channel A
#define   chanB      3  // encoder channel B
#define   zeroPin    5  // limit switch for zero trimming

boolean   ZERO;   //flag for zero trim done

/* Compensator Variables
 * sp : Set Point, desired cart position
 * pv : Process Value, actual cart position
 * mv : Manipulated Value, pwm value given to motor controller
 */
 
double    sp, pv, mv;
double    kp = 5, ki =100, kd = 0.05; 

int m; //just for counting time

//Compensator declaration
PID myPID(&pv, &mv, &sp,5,100,0.05, DIRECT);

void setup() {
  
  pinMode(chanA,INPUT);
  pinMode(chanB,INPUT);
  pinMode(zeroPin,INPUT_PULLUP);
  pinMode(turnPin, OUTPUT);
  pinMode(motorPin,OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(chanA), changeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(chanB), changeB, CHANGE);
  
  Serial.begin(9600);
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // limits for pwm
  myPID.SetSampleTime(1);

  zeroTrim();

  sp=0;   // initial set point, position value

}

void loop() {

  readSerial();
  myPID.Compute();   // takes pv and sp value then changes mv value 
  driveMotor(mv);    // uses the PID output to drive the motor
  sendSerial();      // show values on serial monitor

}

// isr for encoder
void changeA(){
  
  boolean A=digitalRead(2);
  boolean B=digitalRead(3);
  
  if(A==B)
    pv--;
  if(A!=B)
    pv++;

}

// isr for encoder
void changeB(){
  
  boolean A=digitalRead(2);
  boolean B=digitalRead(3);
  
  if(B!=A)
     pv--;
  
  if(A==B)
     pv++;
  
}

void zeroTrim(){
/* this function drive motor backwards 
 * until activates a limit switch (D5)
 * then reset the actual Position (PV) to zero.
 */
  while(!ZERO){
    
    digitalWrite(turnPin,HIGH);
    analogWrite(motorPin,180);
    
    if(!digitalRead(5)){
      
      ZERO = true;  
      
    }
    
    pv = 0;
  
  }  
}

void readSerial(){
/* this function allows to 
 * change Set Point and tuning parameters
 * through serial port.
 */
  if(Serial.available()>0){
      
      if(Serial.read()=='S')
        sp=Serial.parseInt();

      if(Serial.read()=='P')
        kp=Serial.parseFloat();

      if(Serial.read()=='I')
        ki=Serial.parseFloat();

      if(Serial.read()=='D')
        kd=Serial.parseFloat();

      myPID.SetTunings(kp, ki, kd);
       
  }  
}

void driveMotor(int mvTemp){
/* this function takes the output value 
 * from PID and decides the turn CC or CCW
 * and the duty cycle applied to dc motor.
 */
  if(mvTemp>70){  
    digitalWrite(turnPin,LOW);
    analogWrite(motorPin,mvTemp);
  }

  // dead zone
  if((mvTemp<70)&&(mvTemp>-70)){
    analogWrite(motorPin,0);
  }
    
  if(mvTemp<-70){
    mvTemp=-1*mvTemp;  
    digitalWrite(turnPin,HIGH);
    analogWrite(motorPin,mvTemp);
  }
}

void sendSerial(){
/* this function shows relevant values on
 * serial monitor each 1000 loops of code.
 */
  m++;
  if(m>=1000){
    Serial.print((int)sp);
    Serial.print(",");
    Serial.print((int)mv);
    Serial.print(",");
    Serial.println((int)pv);
    m=0;
  }  
}
