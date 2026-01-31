#include "Arduino.h"
#include <array>

class MotorController {
  private:
    int m1p1;
    int m1p2;
    int m2p1;
    int m2p2;

    MotorController() {
      pinMode(m1p1, OUTPUT);
      pinMode(m1p2, OUTPUT);
      pinMode(m2p1,  OUTPUT);
      pinMode(m2p2, OUTPUT);

      m1p1 = 0;
      m1p2 = 1;
      m2p1 = 2;
      m2p2 = 3;
    }

  public:
    moveForward() {
      digitalWrite(m1p1, HIGH);
      digitalWrite(m1p2, LOW);

      digitalWrite(m2p1, HIGH);
      digitalWrite(m2p2, LOW);

      delay(300);
      moveForward();
    }

    turnRight() {
      digitalWrite(m1p1, LOW);
      digitalWrite(m1p2, LOW);

      digitalWrite(m2p1, HIGH);
      digitalWrite(m2p2, LOW);
      delay(300);
    }

    turnLeft() {
      digitalWrite(motor1pin, HIGH);
      digitalWrite(motor1pin, LOW);

      digitalWrite(motor2pin1,LOW);
      digitalWrite(motor2pin2,LOW)
      delay(300);
    }



}

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
        S0 = 12;
        S1 = 11;
        S2 = 10;
        S3 = 9;
        OUT = 8;

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


};

ColourSensor sensor;
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

void loop() {
  auto data = sensor.getColourData();

  int r = data[0] / 10;
  int g = data[1] / 10;
  int b = data[2] / 10;
  int a = data[3] / 10;

  // Serial.print("R: "); Serial.print(r);
  // Serial.print(" G: "); Serial.print(g);
  // Serial.print(" B: "); Serial.print(b);
  // Serial.print(" A: "); Serial.println(a);

  int deltaR = r - prev_rgba[0];
  int deltaG = g - prev_rgba[1];
  int deltaB = b - prev_rgba[2];
  int deltaA = a - prev_rgba[3];

  char maxColour;   // 'R', 'G', or 'B'

  if (r < g && r < b) {
    maxColour = 'R';
  } else if (g < r && g < b) {
    maxColour = 'G';
  } else if (b < r && b < g) {
    maxColour = 'B';
  } else {
    maxColour = 'U'; // unknown / tie
  }


  printCell(r); Serial.print(" | ");
  printCell(g); Serial.print(" | ");
  printCell(b); Serial.print(" | ");
  printCell(a); Serial.print(" | ");
  Serial.println(maxColour);

  // printCell(deltaR); Serial.print(" | ");
  // printCell(deltaG); Serial.print(" | ");
  // printCell(deltaB); Serial.print(" | ");
  // printCell(deltaA); Serial.println();


  prev_rgba[0] = r;
  prev_rgba[1] = g;
  prev_rgba[2] = b;
  prev_rgba[3] = a;

  delay(500);
}
