/*
VCC
TRIG
ECHO
GND
*/
// TODO pin number

#define PIN_TRIG 
#define PIN_ECHO
#define Speed_of_sound 0.034 //cm/us

// defines variables

int measure(){
    long duration; // variable for the duration of sound wave travel
    // Clears the trigPin condition
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(PIN_ECHO, HIGH);
    // Calculating the distance
    int distance = duration * Speed_of_sound / 2; // Speed of sound wave divided by 2 (go and back)
    return distance;
}

void setup() {
    pinMode(PIN_TRIG, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(PIN_ECHO, INPUT); // Sets the echoPin as an INPUT
    Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
    Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
}

void loop() {
    // Displays the distance on the Serial Monitor
    int distance = measure();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
}