char count =0;

void setup(){
  Serial.begin(57600);

}

int checksum(char *data, int length)
/* data does not include the checksum byte. length is the checksum length
 */
{
    int sum = 0;
    int checksum;
    for (int i = 0; i < length; i++)
    {
        sum += data[i];
    }
    sum = sum & 0xFF;
    checksum = 0xFF - sum;
    return checksum;
}


void loop(){
  // int bytesSent = Serial.write("hello"); //send the string “hello” and return the length of the string.
char arr[4];
Serial.write(0x58);
arr[0] = 0x00;
arr[1] = 0x01;
arr[2] = (count);
arr[3] = 0x03;

  Serial.write(arr[0]);
    Serial.write(arr[1]);
      Serial.write(arr[2]);
        Serial.write(arr[3]);
  Serial.write(checksum(arr,4));


count++;
  
  delay(150);
}


/*
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
*/
