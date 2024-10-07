
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}
float v_gps = 0;
void loop() {
  mpu6050.update();

  if(millis() - timer > 2000){
  
    
    float AngleRoll = mpu6050.getAccAngleX();
    float AnglePitch = mpu6050.getAccAngleY();
    float accX = mpu6050.getAccX();
    float accY = mpu6050.getAccY();
    float accZ = mpu6050.getAccZ();
    float accZInertial = -sin(AnglePitch*(3.142/180))*accX+cos(AnglePitch*(3.142/180))*sin(AngleRoll*(3.142/180))*accY+cos(AnglePitch*(3.142/180))*cos(AngleRoll*(3.142/180))*accZ;
    accZInertial = (accZInertial-1)*9.81*100;
    v_gps = v_gps + accZInertial*0.004;
    Serial.println("=======================================================");
    
    Serial.print("temp : ");Serial.println(v_gps);
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
  
    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
  
    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
    
    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    Serial.println("=======================================================\n");
    timer = millis();
    
  }
}







// }
// #include <MPU6050_tockn.h>
// #include <Wire.h>
// #define GRAVITY 9.81
// MPU6050 mpu6050(Wire);

// long timer = 0;
// float accX, accY, accZ; // Giá trị đọc được từ MPU6050
// float accAngleX, accAngleY; // Góc nghiêng tính từ MPU6050

// float velocityX , velocityY, velocityZ ; // Vận tốc theo các trục
// float prevVelocityX, prevVelocityY, prevVelocityZ; // Vận tốc trước đó
// float prevAccX = 0, prevAccY = 0, prevAccZ = 0;

// unsigned long prevUpdateTime; // Thời gian lần cập nhật trước
// float deltaTime; // Khoảng thời gian giữa 2 lần cập nhật
// void setup() {
//   Serial.begin(9600);
//   Wire.begin();
//   // mpu6050.begin();
//   mpu6050.calcGyroOffsets(true);
// }

// void loop() {
//   mpu6050.update();
//   float accX = mpu6050.getAccX();
//   float accY = mpu6050.getAccY();
//   float accZ = mpu6050.getAccZ();

//    // Tính khoảng thời gian giữa 2 lần đọc
//   unsigned long currentTime = millis();
//   deltaTime = (currentTime - prevUpdateTime) / 1000.0;

//   // // Lọc gia tốc tuyến tính
//   //   float accLinearX = accX - sin(accAngleX) * GRAVITY;
//   //   float accLinearY = accY - sin(accAngleY) * GRAVITY;

//   //   // Tích phân tính vận tốc
//   //   // velocityX = prevVelocityX + accLinearX * deltaTime;
//   //   // velocityY = prevVelocityY + accLinearY * deltaTime;
//   //   velocityX += (deltaTime / 6.0) * (prevVelocityX + 4 *  velocityX + accLinearX); 
//   //   velocityY += (deltaTime / 6.0) * (prevVelocityY + 4 *  velocityY + accLinearY);
//   //   // Cập nhật lại giá trị cho lần tính tiếp theo
//   //   prevVelocityX = velocityX; 
//   //   prevVelocityY = velocityY;
//    // Tích phân hai lần để tính tốc độ
//     velocityX += ((accX + prevAccX) / 2) * deltaTime;
//     velocityY += ((accY + prevAccY) / 2) * deltaTime;
//     velocityZ += ((accZ + prevAccZ) / 2) * deltaTime;

//   prevAccX = accX;
//   prevAccY = accY;
//   prevAccZ = accZ;

//     prevUpdateTime = currentTime;

//   // // Tính tốc độ 
//   // float speed = sqrt(velocityX*velocityX + velocityY*velocityY); 
//   // if (millis() - timer > 1000) {

//   //   // Tính tốc độ


//   // velocityX += accX * dt;
//   // velocityY += accY * dt;
//   // velocityZ += accZ * dt;

//   // Tốc độ tổng quát bằng căn bậc 2 tổng bình phương
//   // float speed = sqrt(sq(velocityX) + sq(velocityY));
  
//     // In tốc độ
//   Serial.print("Velocity X: ");
//   Serial.print(velocityX);
//   Serial.print("\tVelocity Y: ");
//   Serial.print(velocityY);
//   Serial.print("\tVelocity Z: ");
//   Serial.println(velocityZ);
    
    
    
//     // Serial.print("Tốc độ hiện tại: ");
//     // Serial.print(sp);
//     // Serial.println(" m/s");

//     // Serial.print("Tốc độ X: ");
//     // Serial.print(velocityX);
//     // Serial.println(" m/s");

//     //  Serial.print("----------------------------------------------------- ");
//     // Serial.print("\naccX : ");Serial.print(mpu6050.getAccX());
//     // Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
//     // Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
  
//     // Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
//     // Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
//     // Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
//     // Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
//     // Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
  
//     // Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
//     // Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
//     // Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
    
//     // Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
//     // Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
//     // Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());



//   //   timer = millis();
//   // }
//   delay(100);
// }


