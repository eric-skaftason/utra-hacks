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
int last_luminance = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  auto [r, g, b, a] = sensor.getColourData();

  Serial.print("Red: ");   Serial.print(r);
  Serial.print(" Green: "); Serial.print(g);
  Serial.print(" Blue: ");  Serial.println(b);
  Serial.print(" Luminance: ");  Serial.println(a);

  char max_channel;
  if (r > g && r > b) {
      max_channel = 'r';
  } else if (g > r && g > b) {
      max_channel = 'g';
  } else {
      max_channel = 'b';
  }


  // Print change in luminance
  Serial.print( "max: "); Serial.println( last_luminance - a);
  // Serial.print( "max: "); Serial.println( max_channel);

  last_luminance = a;

  delay(500);
}
