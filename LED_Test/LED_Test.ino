#define LED1 4
#define LED2 3
#define LED3 2
#define LED4 9
#define LED5 8 

void setup() {
  pinMode(LED1, OUTPUT); 
  pinMode(LED2, OUTPUT); 
  pinMode(LED3, OUTPUT); 
  pinMode(LED4, OUTPUT); 
  pinMode(LED5, OUTPUT); 

}

void loop() {
  digitalWrite(LED1, HIGH); 
  delay(500); 
  digitalWrite(LED1, LOW); 
  digitalWrite(LED2, HIGH); 
  delay(500);
  digitalWrite(LED2, LOW); 
  digitalWrite(LED3, HIGH); 
  delay(500); 
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, HIGH); 
  delay(500); 
  digitalWrite(LED4, LOW); 
  digitalWrite(LED5, HIGH);
  delay(500);
  digitalWrite(LED5, LOW); 

}
