/**
   CODE FOR UNO - WITH MULTIPLEXER AND SERIAL COMMUNICATIONS
   Used some code from: http://thecodeinn.blogspot.com/2013/10/arduino-and-multiplexing.html
   Code for buttons: http://www.instructables.com/id/How-to-Multiple-Buttons-on-1-Analog-Pin-Arduino-Tu/
   
*/

#define joy1SW 6
#define joy0SW 7
#define s0 10 // Digital pins - multiplexer 
#define s1 11
#define s2 12
#define s3 13
#define en 5
#define sig A0

int c0 = 0; // value of select pin @ s0
int c1 = 0; // value of select pin @ s1
int c2 = 0; // value of select pin @ s2
int c3 = 0; // value of select pin @ s3

int count = 0; // which Y pin is being selected
int bin[] = {000, 1, 10, 11, 100, 101, 110, 111};

int buttonState1 = 0; 

void setup() {
  
  Serial.begin(9600);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(en, OUTPUT);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  
}

void loop() {
  // MUX code
  digitalWrite(en, LOW); // enable all the time for now
  for (int cnt = 0; cnt < 8; cnt++) {
    int row = bin[cnt];
    c0 = bitRead(row,0);
    c1 = bitRead(row,1);
    c2 = bitRead(row,2); 

    digitalWrite(s0, c0);
    digitalWrite(s1, c1);
    digitalWrite(s2, c2); 
    
//    c0 = analogRead(s0);
//    c1 = analogRead(s1);
//    c2 = analogRead(s2);
//    c3 = analogRead(s3);
    Serial.println("-----------------");
    Serial.print("Accessing Item @ Pin: ");
    Serial.print(cnt);
    Serial.print("  "); 
    Serial.print("Value @ This Item is: ");
    Serial.print(analogRead(sig));
    Serial.println("  "); 
    Serial.println("-----------------");

    delay(1000); // time to read - this will probs go down to 100
  }

  //delay(100); 
}




