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

      digitalWrite(s2, LOW);
      digitalWrite(s3, LOW);
      r = pulseIn(outPin, LOW); // Reading RED component of color
    
      digitalWrite(s2, HIGH);
      digitalWrite(s3, HIGH);
      g = pulseIn(outPin, LOW); // Reading GREEN component of color
      
      digitalWrite(s2, LOW);
      digitalWrite(s3, HIGH);
      b = pulseIn(outPin, LOW); // Reading BLUE component of color
      

      return {r, g, b};
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

  char max_channel;
  if (r > g && r > b) {
      max_channel = 'r';
  } else if (g > r && g > b) {
      max_channel = 'g';
  } else {
      max_channel = 'b';
  }


  Serial.print( "max: "); Serial.println( max_channel);

  delay(500);
}
