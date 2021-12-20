#include <Wire.h>

#include <Servo.h>   //載入函式庫，這是內建的，不用安裝

Servo servo_1;  // 建立SERVO物件
Servo servo_2;  // 建立SERVO物件

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;

float servo_pitch;
float servo_roll;

void setup() {
  Wire.begin();                                                        //Start I2C as master
  Serial.begin(115200);                                                //Use only for debugging
  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  
 Serial.println("Initializing");
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  digitalWrite(13, HIGH);                                              //Set digital output 13 high to indicate startup
  Serial.write("Fixing\n");
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);  
    Serial.println(cal_int);//Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 1000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 1000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 1000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  digitalWrite(13, LOW);                                               //All done, turn the LEDoff
  Serial.print("Ready\n");
  delay(2000);

   servo_1.attach(9);
   servo_2.attach(10);
  loop_timer = micros();                                               //Reset the loop timer
}

void loop(){

  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  int pitchfix = angle_roll * sin(gyro_z * 0.000001066);
  int rollfix = angle_pitch * sin(gyro_z * 0.000001066);
  angle_pitch += pitchfix;            //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= rollfix;              //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 1.50;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= -2.3;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.97 + angle_pitch_acc * 0.03;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.97 + angle_roll_acc * 0.03;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
   Serial.print("Pitch: ");
   Serial.print(angle_pitch);
   Serial.print("  Roll: ");
   Serial.println(angle_roll+'\n');

   float servo_pitch_tmp = 90-angle_pitch;
   if(servo_pitch_tmp < 72) servo_pitch_tmp=72;
   else if(servo_pitch_tmp > 108) servo_pitch_tmp=108;
   else servo_pitch_tmp = int(servo_pitch_tmp);

   if(servo_pitch_tmp != servo_pitch)
   {
      servo_pitch = servo_pitch_tmp;
      servo_1.write(servo_pitch);
   }

   float servo_roll_tmp = 90+angle_roll;
   if(servo_roll_tmp < 72) servo_roll_tmp=72;
   else if(servo_roll_tmp > 108) servo_roll_tmp=108;
   else servo_roll_tmp = int(servo_roll_tmp);

   if(servo_roll_tmp != servo_roll)
   {
      servo_roll = servo_roll_tmp;
      servo_2.write(servo_roll);
   }

  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}
void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

//        Serial.print("a/g:\t");
//        Serial.print(acc_x); Serial.print("\t");
//        Serial.print(acc_y); Serial.print("\t");
//        Serial.print(acc_z); Serial.print("\t");
//        Serial.print(gyro_x); Serial.print("\t");
//        Serial.print(gyro_y); Serial.print("\t");
//        Serial.print(gyro_z);Serial.println("\t");
}
void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Serial.println("restarting...");
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050    //mpu6050 i2c address
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  delay(1000);

   Serial.println("set Acc...");
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
     Serial.println("set Gyro...");
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission*/
}
