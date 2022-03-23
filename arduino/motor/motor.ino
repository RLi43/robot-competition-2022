/* 
Connections
# Motor Driver
+5V, GND - Arduino Power
Mi_PWM - Speed - to analog pin(PWM acturally)
Mi_EN - Direction - to digital pin
# Motor
Red, Black - Motor Power - to Motor Driver
Green   - Encoder Ground
Blue    - Encoder Vcc(3.5-20V)
Yellow  - Encoder A output
White   - Encoder B output
*/
/*
No feedback, just some example functions!!!
*/

// Motor (TODO: pin number)
#define PIN_M1_PWM 
#define PIN_M2_PWM 
#define PIN_M1_EN 
#define PIN_M2_EN

// Encoder -- Actually, it's better to use 2 arduino?
volatile unsigned int count_pulses_1 = 0;
volatile unsigned int count_pulses_2 = 0;
#define PIN_M1_ENCA 2 // YELLOW
#define PIN_M1_ENCB 3 // WHITE
#define PIN_M2_ENCA 2 // YELLOW
#define PIN_M2_ENCB 3 // WHITE

void setup() {
  Serial.begin(9600);
  pinMode(PIN_M1_ENCA,INPUT);
  pinMode(PIN_M2_ENCA,INPUT);
  pinMode(PIN_M1_ENCB,INPUT);
  pinMode(PIN_M2_ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_ENCA),readEncoder_1,RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_ENCA),readEncoder_2,RISING);
  
  pinMode(PIN_M1_PWM,OUTPUT);
  pinMode(PIN_M2_PWM,OUTPUT);
  pinMode(PIN_M1_EN,OUTPUT);
  pinMode(PIN_M2_EN,OUTPUT);
  
  Serial.println("Hello");
}

void loop() {
    if(Serial.available()){
        char val = Serial.read();
        if(val != -1)
        {
            switch(val)
            {
            case 'w'://Move Forward
                advance (255,255);   //move forward in max speed
                break;
            case 's'://Move Backward
                back_off (255,255);   //move back in max speed
                break;
            case 'a'://Turn Left
                turn_L (100,100);
                break;
            case 'd'://Turn Right
                turn_R (100,100);
                break;
            case 'z':
                Serial.println("Hello");
                break;
            case 'x':
                stop();
                break;
            }
        }
        else stop();
    }
    Serial.print("Pulses: ")
    Serial.print(count_pulses_1);
    Serial.print(", ");
    Serial.print(count_pulses_2);
    Serial.println();
}

// Motor
void stop(void)                    //Stop
{
  digitalWrite(PIN_M1_EN,0);
  digitalWrite(PIN_M1_PWM,LOW);
  digitalWrite(PIN_M2_EN,0);
  digitalWrite(PIN_M2_PWM,LOW);
}
void advance(char a,char b)          //Move forward
{
  analogWrite (PIN_M1_EN,a);      //PWM Speed Control
  digitalWrite(PIN_M1_PWM,HIGH);
  analogWrite (PIN_M2_EN,b);
  digitalWrite(PIN_M2_PWM,HIGH);
}
void back_off (char a,char b)          //Move backward
{
  analogWrite (PIN_M1_EN,a);
  digitalWrite(PIN_M1_PWM,LOW);
  analogWrite (PIN_M2_EN,b);
  digitalWrite(PIN_M2_PWM,LOW);
}
void turn_L (char a,char b)             //Turn Left
{
  analogWrite (PIN_M1_EN,a);
  digitalWrite(PIN_M1_PWM,LOW);
  analogWrite (PIN_M2_EN,b);
  digitalWrite(PIN_M2_PWM,HIGH);
}
void turn_R (char a,char b)             //Turn Right
{
  analogWrite (PIN_M1_EN,a);
  digitalWrite(PIN_M1_PWM,HIGH);
  analogWrite (PIN_M2_EN,b);
  digitalWrite(PIN_M2_PWM,LOW);
}

// Encoder
void readEncoder_1(){
  int b = digitalRead(PIN_M1_ENCB);
  if(b > 0){
    count_pulses_1++;
  }
  else{
    count_pulses_1--;
  }
}
void readEncoder_2(){
  int b = digitalRead(PIN_M2_ENCB);
  if(b > 0){
    count_pulses_2++;
  }
  else{
    count_pulses_2--;
  }
}