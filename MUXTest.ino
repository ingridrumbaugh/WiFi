/**
   Multiplexer Test
   Ingrid Rumbaugh
   2/1/2018
*/

#define s0 4 // Digital pins - multiplexer 
#define s1 5
#define en 15
#define sig A0

int c0 = 0; // value of select pin @ s0
int c1 = 0; // value of select pin @ s1

int count = 0; // which Y pin is being selected
//int bin[] = {000, 1, 10, 11};

void setup() {
  Serial.begin(9600);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(en, OUTPUT);
}

void loop() {
  digitalWrite(en, LOW); // enable all the time for now
  //int row = bin[count];
  c0 = bitRead(0, 0);
  //c1 = bitRead(0, 0);
  c0 = analogRead(s0);
  //digitalWrite(s1, c1);
  // Serial.println("Count: ");
  // Serial.print(count);
  // Serial.println("-------------");
  Serial.println(analogRead(sig));
  delay(1000); // time to read - this will probs go away


}

void out0() {
  digitalWrite(en, LOW); // Apparently enable is LOW
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
}

void out1() {
  digitalWrite(en, LOW);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
}





