char recstr[10];
// char sendstr[20] = "Hello from Uno!"; 

void setup() {
  Serial.begin(9600);
  // Serial.begin(115200); 
}

void loop() {
  Serial.readBytes(recstr,10); 
  Serial.println(recstr); 
  delay(1000); 
  // Serial.write(sendstr,20); 
  // Serial.println(sendstr); 
  // Serial.println("UNO");
}

