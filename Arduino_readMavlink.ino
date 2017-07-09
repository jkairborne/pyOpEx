
#include <mavlink_types.h>
#include <mavlink.h>

// Example 3 - Receive with start- and end-markers

const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;

void setup() {
    Serial.begin(57600);
    Serial.println("<Arduino is ready>");
}


void MyPrintln(float imu1, float imu2)
{
 Serial.print(imu1,2);
 Serial.print(" imu X  ");
 Serial.print(imu2,2);
 Serial.print(" imu Y ");
 Serial.println();
}


void loop() {
    mavlink_message_t msg;
    mavlink_status_t status;
while(Serial.available() > 0){
uint8_t c = Serial.read();
if(mavlink_parse_char(MAVLINK_COMM_0,c,&msg,&status))
{

            switch (msg.msgid)
            {
                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    mavlink_highres_imu_t imu;
                    mavlink_msg_highres_imu_decode(&msg, &imu);
 
                    Serial.println("Got msg");
                    //Serial.println("\t time: %llu\n", imu.time_usec);
                    MyPrintln(imu.xacc,imu.yacc);
                    //Serial.println("\t acc  (NED):\t% f\t% f\t% f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc);
                    //Serial.println("\t gyro (NED):\t% f\t% f\t% f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
                    //Serial.println("\t mag  (NED):\t% f\t% f\t% f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);

                }
                break;


}
}
}
}

