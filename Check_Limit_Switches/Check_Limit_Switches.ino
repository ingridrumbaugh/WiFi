#define limit_up 48
#define limit_down 49 
bool allow_up = false;
bool allow_down = false; 


void setup() {
  pinMode(limit_up, INPUT);
  pinMode(limit_down, INPUT); 

  Serial.begin(9600); 
}

void loop() {
//  if (digitalRead(limit_up) == LOW) {
//    allow_up = true;
//    Serial.println("Limit switch UP is OPEN"); 
//  } else {
//    allow_up = false; 
//    Serial.println("Limit Switch UP is PRESSED"); 
//  }
//  delay(200); 

  if (digitalRead(limit_down) == LOW) {
    allow_down = true;
    Serial.println("Limit Switch DOWN is OPEN"); 
  } else {
    allow_down = false; 
    Serial.println("Limit Switch DOWN is PRESSED"); 
  }
  delay(200);

}
