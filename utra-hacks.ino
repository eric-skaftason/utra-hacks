#include "Arduino.h"
#include <array>

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

    std::array<float, 3> getColourData() {
      // Get Red
      digitalWrite(this->S2, LOW);
      digitalWrite(this->S3, LOW);

      // freq is inverse of period or 1 / (duration of low + duration of high)
      // uses 1 million to convert from microseconds to seconds
      float redPeriod = pulseIn(OUT, LOW) + pulseIn(OUT, HIGH);
      float redFreq = 1000000.0 / redPeriod; // frequency in Hz

      // Get Green
      digitalWrite(this->S2, LOW);
      digitalWrite(this->S3, HIGH);
      float greenPeriod = pulseIn(OUT, LOW) + pulseIn(OUT, HIGH);
      float greenFreq = 1000000.0 / greenPeriod;


      // Get Blue
      digitalWrite(this->S2, HIGH);
      digitalWrite(this->S3, HIGH);
      float bluePeriod = pulseIn(OUT, LOW) + pulseIn(OUT, HIGH);
      float blueFreq = 1000000.0 / bluePeriod;

      return {redFreq, greenFreq, blueFreq};
    }

};

ColourSensor sensor;

void setup() {
  Serial.begin(9600);
}

void loop() {
  auto [r, g, b] = sensor.getColourData();

  Serial.print("Red: ");   Serial.print(r);
  Serial.print(" Green: "); Serial.print(g);
  Serial.print(" Blue: ");  Serial.println(b);

  delay(500);
}
