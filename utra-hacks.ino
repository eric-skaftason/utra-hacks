#include "Arduino.h"
#include <array>
#include <unordered_map>

class MotorController {
  private:
    int m1p1; // motor 1 pin 1
    int m1p2;
    int m2p1;
    int m2p2;

  public:
    MotorController() {
      m1p1 = 1;
      m1p2 = 2;
      m2p1 = 3;
      m2p2 = 4;

      pinMode(m1p1, OUTPUT);
      pinMode(m1p2, OUTPUT);
      pinMode(m2p1,  OUTPUT);
      pinMode(m2p2, OUTPUT);
    }

    void moveForward() {
      digitalWrite(m1p1, LOW);
      digitalWrite(m1p2, HIGH);

      // digitalWrite(m2p1, HIGH);
      // digitalWrite(m2p2, LOW);
    }

    void turnRight() {
      digitalWrite(m1p1, LOW);
      digitalWrite(m1p2, LOW);

      digitalWrite(m2p1, HIGH);
      digitalWrite(m2p2, LOW);
      // delay(300);
    }

    void turnLeft() {
      digitalWrite(m1p1, HIGH);
      digitalWrite(m1p2, LOW);

      digitalWrite(m2p1,LOW);
      digitalWrite(m2p2,LOW);
      // delay(300);
    }
    void backwards() {

      digitalWrite(m1p1, HIGH);
      digitalWrite(m1p2, LOW);

      digitalWrite(m2p1, HIGH);
      digitalWrite(m2p2, LOW);
    }
};

class ColourSensor {
  private:
    int S0;
    int S1;
    int S2;
    int S3;
    int OUT;

    void init() {
      pinMode(S0, OUTPUT);
      pinMode(S1, OUTPUT);
      pinMode(S2, OUTPUT);
      pinMode(S3, OUTPUT);
      pinMode(OUT, INPUT);

      // Frequency scaling 20%
      digitalWrite(S0, HIGH);
      digitalWrite(S1, LOW);
    }

    int percentDiff(int a, int b) {
      return int((float)abs(a - b) / ((a + b) / 2.0) * 100);
  }

  public:
    ColourSensor(int s0, int s1, int s2, int s3, int out) {
        S0 = s0;
        S1 = s1;
        S2 = s2;
        S3 = s3;
        OUT = out;

        init();
    }

    ColourSensor() {
        S0 = 8;
        S1 = 9;
        S2 = 10;
        S3 = 11;
        OUT = 12;

        init();
    }

    std::array<int, 4> getColourData() {

      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      int r = pulseIn(OUT, LOW); // Reading RED component of color
    
      digitalWrite(S2, HIGH);
      digitalWrite(S3, HIGH);
      int g = pulseIn(OUT, LOW); // Reading GREEN component of color
      
      digitalWrite(S2, LOW);
      digitalWrite(S3, HIGH);
      int b = pulseIn(OUT, LOW); // Reading BLUE component of color

      digitalWrite(S2, HIGH);
      digitalWrite(S3, LOW);
      int a = pulseIn(OUT, LOW);
      
      return {r, g, b, a};
    }

    char getColour(int threshold) {
      std::array<int, 4> data = getColourData();
      int r = data[0];
      int g = data[1];
      int b = data[2];

      // Check if red is dominant
      if (r <= g && r <= b &&
          percentDiff(r, g) >= threshold &&
          percentDiff(r, b) >= threshold) return 'R';

      // Check if green is dominant
      if (g <= r && g <= b &&
          percentDiff(g, r) >= threshold &&
          percentDiff(g, b) >= threshold) return 'G';

      // Check if blue is dominant
      if (b <= r && b <= g &&
          percentDiff(b, r) >= threshold &&
          percentDiff(b, g) >= threshold) return 'B';

      return 'U';
  }

};

class UltrasonicSensor {
  private:
    int pinTrig;
    int pinEcho;

  public:
    UltrasonicSensor() {
      pinTrig = 6;
      pinEcho = 7;

      pinMode(pinTrig, OUTPUT);
      pinMode(pinEcho, INPUT);
    }

    long getDist() {
      long duration, cm;

      digitalWrite(pinTrig, LOW);
      delayMicroseconds(2);
      digitalWrite(pinTrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(pinTrig, LOW);

      duration = pulseIn(pinEcho, HIGH);

      // convert the microseconds to cm
      cm = duration / 29 / 2;

      return cm;
    }
};

class IRSensor {
  private:
    int pin;

  public:
    IRSensor() {
    }
};

class CourseCorrection {
  public:
  char primary_color;
  bool colourDetected = false;
  double angle = 0;
  primary_color = sensor.getColourData(40);

  void Backtracking(char colour) {
    //turn left little increment
    while(angle <= 90 && !colourDetected){
      MotorController.turnLeft();
      delay(100);
      angle += 18;
      maxColour = sensor.getColour(40);
      if(maxColour == colour){
        MotorController.moveForward();
        return;
      }
    }

    //turn right little increment
    //turn back
    MotorController.turnRight();
    delay(500);
    angle = 0;
    while(angle <= 90 && !colourDetected){
      MotorController.turnRight();
      delay(100);
      angle += 18;
      maxColour = sensor.getColour(40);
      if(maxColour == colour){
        MotorController.moveForward();
        return;
      }
    }
  }
}

    IRSensor(int p) {
      pin = p;

      pinMode(pin, INPUT);
    }

    bool wallDetected() {
      // digitalWrite(pin, HIGH);  
      // delayMicroseconds(500);  
      // int a = analogRead(pin);

      // digitalWrite(pin, LOW);  
      // delayMicroseconds(500);  
      // int b = analogRead(pin);


      // int c = a - b;

      // Serial.print(a);
      // Serial.print(' ');
      // Serial.print(b);
      // Serial.print(' ');
      // Serial.println(c);

      int sensorValue = digitalRead(pin);

      if (sensorValue == LOW) {  // Object detected
        Serial.println("Object detected");
        return true;
      } else {                   // No object
        Serial.println("No object");
        return false;
      }
      

      return true;
    }

};

ColourSensor sensor;
MotorController motor_controller;
UltrasonicSensor ultrasonic_sensor;
IRSensor ir_sensor = IRSensor(5);
int prev_rgba[4] = {0, 0, 0, 0};

void setup() {
  Serial.begin(9600);

  // Table head
  Serial.println(F("  ΔR |  ΔG |  ΔB |  ΔA"));
  Serial.println(F("----------------------"));
}

void printCell(int value) {
  if (value >= 0) Serial.print(" ");
  if (abs(value) < 10)  Serial.print(" ");
  if (abs(value) < 100) Serial.print(" ");
  Serial.print(value);
}

void writeToMap(){
  std::unordered_map<int, std::string> umap = {};
}

void loop() {
  auto data = sensor.getColourData();

  int r = data[0] / 1;
  int g = data[1] / 1;
  int b = data[2] / 1;
  int a = data[3] / 1;

  // Serial.print("R: "); Serial.print(r);
  // Serial.print(" G: "); Serial.print(g);
  // Serial.print(" B: "); Serial.print(b);
  // Serial.print(" A: "); Serial.println(a);

  int deltaR = r - prev_rgba[0];
  int deltaG = g - prev_rgba[1];
  int deltaB = b - prev_rgba[2];
  int deltaA = a - prev_rgba[3];

  char maxColour;   // 'R', 'G', or 'B'

  maxColour = sensor.getColour(40);


  // printCell(r); Serial.print(" | ");
  // printCell(g); Serial.print(" | ");
  // printCell(b); Serial.print(" | ");
  // printCell(a); Serial.print(" | ");
  // Serial.println(maxColour);

  // printCell(deltaR); Serial.print(" | ");
  // printCell(deltaG); Serial.print(" | ");
  // printCell(deltaB); Serial.print(" | ");
  // printCell(deltaA); Serial.println();

  // motor_controller.moveForward();


  // long dist = ultrasonic_sensor.getDist();
  // Serial.print(dist); Serial.println(" cm");


  bool wall_detected = ir_sensor.wallDetected();

  prev_rgba[0] = r;
  prev_rgba[1] = g;
  prev_rgba[2] = b;
  prev_rgba[3] = a;

  delay(300);
}