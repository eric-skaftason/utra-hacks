// Pin configuration
#define S0 8
#define S1 9
#define S2 10
#define S3 11
#define OUT 12

// Variables to store frequency values
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

void setup() {
  Serial.begin(9600);

  // Set control pins as OUTPUT
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  // Set frequency scaling to 20% (can also use 100% or 2%)
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void loop() {
  // Read RED
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(OUT, LOW); // Measure the LOW duration of the pulse

  // Read BLUE
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(OUT, LOW);

  // Read GREEN
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(OUT, LOW);

  // Print the values
  Serial.print("R: ");
  Serial.print(redFrequency);
  Serial.print("  G: ");
  Serial.print(greenFrequency);
  Serial.print("  B: ");
  Serial.println(blueFrequency);

  delay(500); // Small delay to make output readable
}
