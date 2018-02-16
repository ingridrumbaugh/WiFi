char str[10] = "Hello"; 
// char getstr[20]; // store received data 

void setup() {
  Serial.begin(9600); 
  // Serial.begin(115200); 
}

void loop() {
  // int value = 1234; 
  // itoa(value, str, 10);
  Serial.write(str,10); 
  Serial.print("message: "); 
  Serial.println(str);
  
  delay(1000);
//  Serial.readBytes(getstr,20); 
//  for (int i = 0; i < 20; i ++) {
//    Serial.print(getstr[i]); 
//  }
//  // Serial.println(getstr); 
//  Serial.println("Wifi"); 
//  delay(1000); 
}

