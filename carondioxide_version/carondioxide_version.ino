int greenLed = 8;
int redLed = 9;
int redLed1 = 10;
int redLed2 = 11;
int redLed3 = 12;
int gasPin = 0;
int analogSensor;
// Your threshold value

void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);  
  pinMode(redLed1, OUTPUT);
  pinMode(redLed2, OUTPUT);
  pinMode(redLed3, OUTPUT);
  Serial.begin(9600);
}

void loop() {
 analogSensor = analogRead(gasPin);
 Serial.println(analogSensor);
//  
//   Checks if it has reached the threshold value
  if (analogSensor < 50)
  {
    digitalWrite(greenLed, HIGH);
    digitalWrite(redLed, LOW);
    digitalWrite(redLed1, LOW);
    digitalWrite(redLed2, LOW);
    digitalWrite(redLed3, LOW);
  }
  else if (analogSensor < 60)
  {
    digitalWrite(redLed, HIGH);
    digitalWrite(redLed1, LOW);    
    digitalWrite(redLed2, LOW);
    digitalWrite(redLed3, LOW);
    digitalWrite(greenLed, LOW);
  }
    else if (analogSensor < 70)
  {
    digitalWrite(redLed, HIGH);
    digitalWrite(redLed1, HIGH);
    digitalWrite(redLed2, LOW);
    digitalWrite(redLed3, LOW);
    digitalWrite(greenLed, LOW);
  }
      else if (analogSensor < 80)
  {
    digitalWrite(redLed, HIGH);
    digitalWrite(redLed1, HIGH);
    digitalWrite(redLed2, HIGH);
    digitalWrite(redLed3, LOW);
    digitalWrite(greenLed, LOW);
  }
        else
        {
    digitalWrite(redLed, HIGH);
    digitalWrite(redLed1, HIGH);
    digitalWrite(redLed2, HIGH);
    digitalWrite(redLed3, HIGH);
    digitalWrite(greenLed, LOW);
  }
  delay(100);


}
