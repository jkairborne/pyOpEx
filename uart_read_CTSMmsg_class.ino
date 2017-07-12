void setup(){
  Serial.begin(57600);
}

void loop(){
  // int bytesSent = Serial.write("hello"); //send the string “hello” and return the length of the string.

 
  Serial.write(0xFF);
  Serial.write(0x00);
  Serial.write(0x41);
Serial.write(0x42);
Serial.write(0x43);
Serial.write(0x44);
Serial.write(0x45);
Serial.write(0x46);
Serial.write(0x47);
Serial.write(0x48);
Serial.write(0x49);
Serial.write(0x50);



  
  delay(150);
}
