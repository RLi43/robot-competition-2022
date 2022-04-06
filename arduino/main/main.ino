#include <Arduino.h>
/* ----- SETTINGS ----- */
#define PORT_BAUD_RATE 9600 // Serial for debug = Serial0
// Serial1 for Dynamixel
#define RPI_BAUD_RATE 115200
#define Serial_rpi Serial2
#define MAX_LOOP_MS 100

// Protocol TODO
#define MSG_WRITE_LENGTH 8
#define MSG_READ_LENGTH 8

// Motors & Encoders
#define MOTOR_SPEED_MAX 100
#define ENCODER_PULSE_PER_REVOLUTION 900 // 48/4*75
#define WHEEL_PERIMETER_MM (3.14159*120)
// Servos
#include<Servo.h>
Servo Servo_arm, Servo_gripper, Servo_gate;
#define SERVO_ARM_MID_US 1500 // TO CHECK
#define SERVO_ARM_LOW_US 750
#define SERVO_ARM_HIGH_US 2250
#define SERVO_GRIPPER_MID_US 1500
#define SERVO_GRIPPER_LOW_US 750
#define SERVO_GRIPPER_HIGH_US 2250
#define SERVO_GATE_MID_US 1500
#define SERVO_GATE_LOW_US 750
#define SERVO_GATE_HIGH_US 2250
#define STORAGE_EMPTY_MS 5000// TODO
// Digital Compass
#include <Wire.h>
#include <GY26Compass.h>
#define ADDR_COMPASS 0x70
#define COMPASS_DECLINATION_ANGLE 6.3

/* ----- PIN DEFINITION ----- */
// NOTE FOR MEGA 2560
// Interupt: 2, 3, *21, *20, 19, 18 -- * for I2C
// PWM: 2 - 13, 44 - 46; 490 Hz (pins 4 and 13: 980 Hz)
// Serial;   Serial1;        Serial2;        Serial3
// 0(RX), 1(TX); 19(RX), 18(TX); 17(RX), 16(TX); 15(RX), 14(TX)

// Motors
#define PIN_MOTOR_LDIR 11 // TODO
#define PIN_MOTOR_RDIR 12
#define PIN_MOTOR_LPWM 9
#define PIN_MOTOR_RPWM 10
// Encoders 12*ratio(polulu)
#define PIN_ENCODER_LA 2 // INT 0
#define PIN_ENCODER_LB 4
#define PIN_ENCODER_RA 3 // INT 1
#define PIN_ENCODER_RB 5
// Servos ~PWM
#define PIN_SERVO_ARM 6
#define PIN_SERVO_GRIPPER 7
#define PIN_SERVO_GATE 8
// Range Sensors -- use analog pins
#define PIN_RANGE_OBS_L1 43
#define PIN_RANGE_OBS_R1 42
#define PIN_RANGE_OBS_FM 41
#define PIN_RANGE_OBS_FL 41
#define PIN_RANGE_OBS_FR 40
#define PIN_RANGE_BOTTLE_H 39
#define PIN_RANGE_BOTTLE_L 38
// LED
// NOTE: LED_BUILTIN = D13
#define PIN_LED_STATUS LED_BUILTIN

/* ----- COMPONENTS' FUNCTIONS ----- */
// Motors
void _move(char left = MOTOR_SPEED_MAX, char right = MOTOR_SPEED_MAX, bool forw_l = true, bool forw_r = true){
    if(left > MOTOR_SPEED_MAX) left = MOTOR_SPEED_MAX;
    if(right > MOTOR_SPEED_MAX) right = MOTOR_SPEED_MAX;
    analogWrite(PIN_MOTOR_LPWM, left);
    analogWrite(PIN_MOTOR_RPWM, right);
    digitalWrite(PIN_MOTOR_LDIR, forw_l);
    digitalWrite(PIN_MOTOR_RDIR, forw_r);
}
void move_forward(char speed = MOTOR_SPEED_MAX){_move(speed, speed);}
void move_back(char speed = MOTOR_SPEED_MAX){_move(speed, speed, false, false);};
void move_stop(){_move(0, 0);}
void turn_left(char left = 0, char right = MOTOR_SPEED_MAX){_move(left, right);}
void turn_right(char left = MOTOR_SPEED_MAX, char right = 0){_move(left, right);}
void rotate_left(char left = MOTOR_SPEED_MAX, char right = MOTOR_SPEED_MAX){_move(left, right, false, true);}
void rotate_right(char left = MOTOR_SPEED_MAX, char right = MOTOR_SPEED_MAX){_move(left, right, true, false);}
// Encoders
volatile int encoder_left_count_pulses = 0;
volatile int encoder_right_count_pulses = 0;
void encoder_read_left(){
  int b = digitalRead(PIN_ENCODER_LB);
  if(b > 0) encoder_left_count_pulses++;
  else encoder_left_count_pulses--;
}
void encoder_read_right(){
  int b = digitalRead(PIN_ENCODER_RB);
  if(b > 0) encoder_right_count_pulses++;
  else encoder_right_count_pulses--;
}
int displacement(int pulse_count){//TO CHECK overflow?
    return pulse_count*WHEEL_PERIMETER_MM/ENCODER_PULSE_PER_REVOLUTION;
}
// Servos
void _arm_control(unsigned int us){Servo_arm.writeMicroseconds(us);}
void _gripper_control(unsigned int us){Servo_gripper.writeMicroseconds(us);}
void _gate_control(unsigned int us){Servo_gate.writeMicroseconds(us);}
void arm_forward(){
    // TODO
    _arm_control(SERVO_ARM_HIGH_US);
}
void arm_back(){
    // TODO
    _arm_control(SERVO_ARM_LOW_US);
}
void gripper_open(){
    // TODO
    _gripper_control(SERVO_GRIPPER_HIGH_US);
}
void gripper_close(){
    // TODO
    _gripper_control(SERVO_GRIPPER_LOW_US);
}
void gate_open(){
    // TODO
    _gate_control(SERVO_GATE_HIGH_US);
}
void gate_close(){
    // TODO
    _gate_control(SERVO_GATE_LOW_US);
}
// Range Sensors
#define RANGE_BOTTLE_CM 15
#define RANGE_OBS_WARNING_CM 80
#define RANGE_OBS_STOP_CM 5
float _range_voltage_distance(int volt){
    // TODO
    return volt*5.0/1024;
}
bool _bottle_low(){
    // TODO
    return _range_voltage_distance(analogRead(PIN_RANGE_BOTTLE_L)) < RANGE_BOTTLE_CM;
}
bool _bottle_high(){
    // TODO
    return _range_voltage_distance(analogRead(PIN_RANGE_BOTTLE_H)) < RANGE_BOTTLE_CM;
}
bool bottle_in_front(){
    return _bottle_low();
}
// obstacles
bool obstacle_infront(byte* distance){
    *distance = _range_voltage_distance(analogRead(PIN_RANGE_OBS_FM)); //TODO
    if(*distance < RANGE_OBS_STOP_CM){
        // TODO: stop, *report
        report_obs_stop();
        return true;
    }else if(*distance < RANGE_OBS_WARNING_CM){
        return true;
    }
    return false;
}
bool obstacle_side(byte* distance, bool left = true){
    int pin = left?PIN_RANGE_OBS_L1:PIN_RANGE_OBS_R1;
    *distance = _range_voltage_distance(analogRead(pin)); //TODO
    if(*distance < RANGE_OBS_WARNING_CM){
        // TODO: 
        return true;
    }
    return false;
}

// compass
GY26_I2C_Compass compass(ADDR_COMPASS);
float get_compass_angle(){
    return compass.getCompassAngle();
}

// Status
void report_error(char msg = 'X'){
    Serial.print(msg);
    Serial.print(" - ");
    Serial.println("Error!");
    digitalWrite(PIN_LED_STATUS, HIGH);
}

void setup(){
    pinMode(PIN_MOTOR_LDIR, OUTPUT);
    pinMode(PIN_MOTOR_RDIR, OUTPUT);
    pinMode(PIN_MOTOR_LPWM, OUTPUT);
    pinMode(PIN_MOTOR_RPWM, OUTPUT);
    pinMode(PIN_ENCODER_LA, INPUT);
    pinMode(PIN_ENCODER_LB, INPUT);
    pinMode(PIN_ENCODER_RA, INPUT);
    pinMode(PIN_ENCODER_RB, INPUT);    
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LA),encoder_read_left,RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RA),encoder_read_right,RISING);

    pinMode(PIN_SERVO_ARM, OUTPUT);
    pinMode(PIN_SERVO_GRIPPER, OUTPUT);
    pinMode(PIN_SERVO_GATE, OUTPUT);
    Servo_arm.attach(PIN_SERVO_ARM);
    Servo_gripper.attach(PIN_SERVO_GRIPPER);
    Servo_gate.attach(PIN_SERVO_GATE);

    pinMode(PIN_RANGE_OBS_L1, INPUT);
    pinMode(PIN_RANGE_OBS_R1, INPUT);
    pinMode(PIN_RANGE_OBS_FM, INPUT);
    pinMode(PIN_RANGE_OBS_FL, INPUT);
    pinMode(PIN_RANGE_OBS_FR, INPUT);
    pinMode(PIN_RANGE_BOTTLE_H, INPUT);
    pinMode(PIN_RANGE_BOTTLE_L, INPUT);

    Wire.begin();
    compass.setDeclinationAngle(COMPASS_DECLINATION_ANGLE);

    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);

    Serial.begin(PORT_BAUD_RATE);
    // Maybe write something to rpi
    Serial.println("Hello!");

    // Reset TODO
    // close the gate
    // arm
}

/* ----- GLOBAL VARIABLES ----- */
int encoder_last_reported_left = 0;
int encoder_last_reported_right = 0;

unsigned long loop_start = 0; // ms
byte grasp = 0; // indicate the grasp status
byte gate = 0; // indicate the gate status
bool check_bottle = false;
bool no_bottle = false;
byte last_report_byte = 0x00; // bottle, obs, xxxxxx
void report_obs_stop(){
    last_report_byte |= (1 << 6);
}

void loop(){
    loop_start = millis();

    // correspondance to rpi
    if(Serial){
        // read
        read_command();
        // write
        write_info();
    }
    else report_error('S');

    // Routine
    // check obstacles TODO
    // *check bottles
    // check the bottle
    if(check_bottle){
        if(grasp == 0){
            // it's not grasping
            if(bottle_in_front()){
                // do the grasp
                grasp = 1;
            }
            else{
                // report no bottle in front
                no_bottle = true;
            }
        }
    }
    do_grasp();
    do_gate();

    // check the frequency
    if(millis() - loop_start > MAX_LOOP_MS) report_error('T');
}

void read_command(){
    byte len = Serial.available();
    while(len >= MSG_READ_LENGTH){
        // TODO: Protocal
        // grasp
        // remind to check if it's grasping

        // check another command
        len -= MSG_READ_LENGTH;
    }
    if(len > 0) report_error('R'); // ? transmission is too slow?
    // else no info from rpi
}
void write_info(){
    // TODO
    byte msg[MSG_WRITE_LENGTH] = {0xFF,}; // string?
    // TODO
    int idx = 0;
    // displacement - left, right, 2x2bytes
    int curcountL = encoder_left_count_pulses;
    int curcountR = encoder_right_count_pulses;
    int displace_left = displacement(curcountL - encoder_last_reported_left);
    int displace_right = displacement(curcountR - encoder_last_reported_right);
    // orientation, 1 byte, 360 --map--> 255
    float angle = get_compass_angle(); // TODO check if it's -180~180
    msg[idx++] = byte(angle*255/360);
    // obstacles 3bytes cm; 0xFF for no obs
    byte dis = 0;
    if(obstacle_infront(&dis)){
        msg[idx++] = dis;
    }
    if(obstacle_side(&dis, true)){//left
        msg[idx++] = dis;
    }
    if(obstacle_side(&dis, false)){//right
        msg[idx++] = dis;
    }

    // feedback of the bottle
    if(no_bottle){
        // TODO
        last_report_byte |= (1 << 7);
        no_bottle = false;
    }

    // avoid blocking -- todo, doesn't deal with it, the msg will miss
    if(MSG_WRITE_LENGTH > Serial.availableForWrite()) report_error('W');
    else{
        Serial.write(msg, MSG_WRITE_LENGTH);
        encoder_last_reported_left = curcountL;
        encoder_last_reported_right = curcountR;
        last_report_byte = 0x00;
    }
}
// maybe useless reminder: if you send command when it's executing it, the latter one should be ignored
void do_grasp(){
    switch (grasp)
    {
    // TODO: how to check the former one has done - time?(by measurement?)
    case 0:
        break;
    case 1:
        // rotate the arm //TODO
        arm_forward();
        grasp += 1;
        break;
    case 2:
        // grasp // TODO
        gripper_close();
        grasp += 1;
        break;
    case 3:
        // pullback // TODO
        arm_back();
        grasp += 1;
        break;
    case 4:
        // release // TODO
        gripper_open();
        grasp = 0;
        break;
    default:
        break;
    }
}
unsigned long gate_counter = 0;
void do_gate(){
    if(gate>0){
        if(gate == 1){
            gate_open();
            gate = 2;
            gate_counter = millis();
        }
        else if (gate == 2 && (millis() - gate_counter) > STORAGE_EMPTY_MS)
        {
            gate_close();
            gate = 0;
        }        
    }
}
