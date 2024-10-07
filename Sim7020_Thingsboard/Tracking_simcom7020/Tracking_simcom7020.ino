


/*
 * https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/uart.html#uart-api-setting-communication-pins
 */
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "simcom7020.h"
#include "gps_lib.h"
#define URI_MQTT         "mqtt.thingsboard.cloud"
#define DEVICE_ID        "4c81ee00-9270-11ee-8c4b-05c61329bd02"
#define DEVICE_TOKEN     "bHl28FccGcNty4X6Wh0R"

MPU6050 mpu6050(Wire);

static char topic_pub[100];
static char topic_sub[100];
GPS gps_t;
DT DateTime;
char gps_rx[256];
simcom simcom_7020;
uint8_t tp_mess[10];
int a = 0;
bool Flag_gps_fix = false;
bool Flag_gps_acc = false;
bool Flag_receive = false;
bool Flag_connect_mqtt = false;
bool GPS_ON = false;
uint8_t Datetime[20];
char rssi[10];
char rsrp[10];
char rsrq[10];
float estimated_speed = 0;
float prev_velocity = 0; 
float velocity_x = 0;
float velocity_y = 0;
float velocity_z = 0;
float accX;
float accY;
float accZ;
float prevAccX = 0;
float prevAccY = 0;
float prevAccZ = 0;
unsigned long prevTime = 0;
// Biến lưu trữ vận tốc
float velocityX = 0;
float velocityY = 0;
float velocityZ = 0;
float speed = 0;

client client_mqtt = {
    URI_MQTT,
    1883,
    "batky",
    DEVICE_ID,
    DEVICE_TOKEN,
    0
};
struct timeval epoch;
struct timezone utc;
static const char * TAG = "";  

// ham callback se thay doi gia tri chu ky gui message
void subcribe_callback(char * data)
{
  char* _buff;
  _buff = strstr(data, "{");
  ESP_LOGI(TAG, "%s", _buff);
  filter_comma_t((char *)_buff, 3, 4, (char *)tp_mess);
  simcom_7020.tp = atoi((const char *)tp_mess);
}      



void setup() {

    pinMode(13, OUTPUT); 
    Serial.begin(9600); 
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // khoi tao uart voi simcom va gps
    init_simcom(NUMERO_PORTA_SERIALE2, U2TXD, U2RXD, 9600);
    init_GPS(NUMERO_PORTA_SERIALE1, U1TXD, U1RXD, 9600);
    //digitalWrite(13, HIGH);
    simcom_7020.tp = 0.5; // chu ky hoat dong default 

    // dinh dang topic sub va pub
    sprintf(topic_pub, "v1/devices/me/telemetry",DEVICE_ID);
    sprintf(topic_sub, "v1/devices/me/attributes",DEVICE_ID);
    
    //cai dat mpu6050
    Wire.begin();
    mpu6050.begin();
    //mpu6050.calcGyroOffsets(true);
   
}
 
 
 
void loop() {
 
  mpu6050.update();
   
    //   float dt = 0.01; // chu kỳ lặp 
    // accX = mpu6050.getAccX();
    // accY = mpu6050.getAccY();
    // accZ = mpu6050.getAccZ();
    // // Tích phân vận tốc
    // velocity_x = accX * dt;  
    // velocity_y = accY * dt;
    // velocity_z = accZ * dt;
    // // Vận tốc tổng quát
    // estimated_speed = sqrt(velocity_x*velocity_x + velocity_y*velocity_y + velocity_z*velocity_z);

/////////////////////////////////////////////////////////////////////////////
 // Lấy thời điểm hiện tại
  accX = mpu6050.getAccX();
  accY = mpu6050.getAccY();
  accZ = mpu6050.getAccZ();
  unsigned long currentTime = millis();
  // Tính thời gian trôi qua từ lần đọc trước đó
  float deltaTime = (currentTime - prevTime) / 1000.0; // Đổi sang giây
  // Tính toán vận tốc bằng cách tích phân giá trị gia tốc theo thời gian
  velocityX += (accX + prevAccX) / 2 * deltaTime;
  velocityY += (accY + prevAccY) / 2 * deltaTime;
  velocityZ += (accZ + prevAccZ) / 2 * deltaTime;

  // Lưu lại giá trị gia tốc cho lần đọc tiếp theo
  prevAccX = accX;
  prevAccY = accY;
  prevAccZ = accZ;

  // Lưu lại thời điểm cho lần đọc tiếp theo
  prevTime = currentTime;

  // Tính vận tốc tổng quát
  estimated_speed = sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ);

////////////////////////////////////
  bool res;
  // check connect between simcom vs mcu
  if(!Flag_connect_mqtt)
  {
    printf("\n------------->WAITING<------------\r\n");
    Check_network();
    RECONECT:
    delay(1000/portTICK_PERIOD_MS);
    if (mqtt_start(client_mqtt, 3, 64800, 1, 3)) 
    {
      isconverhex(3); // dinh dang message cua pub la string (AT+CREVHEX=1 la gui hex)
      ESP_LOGI(TAG_SIM, "MQTT IS CONNECTED");
      vTaskDelay(500/portTICK_PERIOD_MS);
      SUB:
      if (mqtt_subscribe(client_mqtt, topic_sub, 0, 3, subcribe_callback)) 
      {
        ESP_LOGI(TAG_SIM, "Sent subcribe is successfully"); 
        Flag_connect_mqtt = true;
        vTaskDelay(500/portTICK_PERIOD_MS);
        GPS_ON = true;
      }
      else 
      {
        ESP_LOGI(TAG_SIM, "Sent subcribe is failed");
        goto SUB;
      }
    }
    else
    {
      ESP_LOGI(TAG_SIM, "MQTT IS NOT CONNECTED");
      mqtt_stop(client_mqtt, 2);
      Flag_connect_mqtt = false;
    }
  }
  // kiem tra xem co gps hay khong
  if(Flag_gps_fix && Flag_gps_acc)
  {
    Flag_gps_fix = false;
    Flag_gps_acc=false;
    CV_payload(); // dong goi ban tin 
    mqtt_message_publish(client_mqtt, gps_rx, topic_pub, 0, 5);
    mqtt_message_publish(client_mqtt, gps_rx, topic_sub, 0, 5);
    //mqtt_stop(client_mqtt, 2);
    ESP_LOGI(TAG, "time period = %d", simcom_7020.tp);
    vTaskDelay(simcom_7020.tp * 60000/portTICK_PERIOD_MS);
    GPS_ON = true;
  } 
  // 
   vTaskDelay(1000/portTICK_PERIOD_MS);
}
// kiem tra lẹnh AT va network cua module
void Check_network()
{
  bool res;
  POWER_ON:
  if(isInit(4)) ESP_LOGW(TAG,"module connect physical success");
  else  ESP_LOGW(TAG,"module connect physical fail");
  FINE_NET:
  res = isRegistered(10);
  if(res) ESP_LOGW(TAG, "Module registed OK");
  else
  {
    if(!isInit(3)) 
    {
      ESP_LOGE(TAG, "Module registed FALSE");
      goto POWER_ON;
    }
    else
    {
       goto FINE_NET;
    }
  }
}

static bool prevCrashed = false;
// Hàm kiểm tra có tai nạn hay không
bool checkAccident() {
    // Đọc dữ liệu gia tốc từ cảm biến
    // int accX = mpu6050.getAccX();
    // int accY = mpu6050.getAccY();
    // int accZ = mpu6050.getAccZ();

    // Tính toán giá trị gia tốc tổng cộng (độ lớn của vectơ gia tốc)
    float totalAcc = sqrt(accX * accX + accY * accY + accZ * accZ);

    // Đặt ngưỡng để xác định sự kiện va chạm
    float thresholdAcc = 0.8; 
    // Kiểm tra góc nghiêng
  float pitch = getPitch(accX, accY, accZ);
  bool isRollover = abs(pitch) > 3; 
  // Kiểm tra giảm tốc độ đột ngột   
  bool isSuddenBrake = IsSuddenDrop(gps_t.speed_k);
  
  // Kết luận va chạm
  bool crashed = (isRollover || isSuddenBrake) && totalAcc > thresholdAcc;

  // So sánh với trạng thái trước
  if(crashed != prevCrashed){
     prevCrashed = crashed; 
     return true; //Phát hiện va chạm mới
       }
  return false;
}
// Hàm kiểm tra giảm tốc đột ngột
bool IsSuddenDrop(float curSpeed) {
  static float prevSpeed = 0; 
  float dropThreshold = 2    ; // km/h
  if(abs(prevSpeed - curSpeed) > dropThreshold){
     prevSpeed = curSpeed;
     return true;
  }
  prevSpeed = curSpeed;
  return false;  
}
// Hàm tính góc nghiêng dựa trên gia tốc
float getPitch(float ax, float ay, float az) {

  float pitch = atan2(ax, sqrt(ay * ay + az * az));

  return pitch * 180.0 / PI; // Chuyển về độ 
}

// //Bo Loc Kalman
// float estimate_accuracy = 1.0; 
// const float KALMAN_GAIN = 0.05;
// float kalman() {

//   // Đọc dữ liệu GPS
//   float measured_speed = gps_t.speed_k;  

//   // Dự đoán dựa trên trạng thái trước 
//   float speed_prediction = speed_estimate;

//   // Tính toán lọc Kalman
//   float prediction_error = abs(measured_speed - speed_prediction);
//   estimate_accuracy += (KALMAN_GAIN * prediction_error);
//   estimate_accuracy = constrain(estimate_accuracy, 0.01, 10);
  
//   speed_estimate = speed_prediction + (KALMAN_GAIN * (measured_speed - speed_prediction));

//   // Sử dụng biến speed_estimate đã lọc 
//   return speed_estimate; 

// }
// Hàm ổn định vận tốc từ GPS
float stabilizeGPSspeed(float speed) {
  // Nếu vận tốc nhỏ hơn 1, làm tròn về 0
  if (speed < 1.0) {
    return 0.0;
  } 
  // Nếu vận tốc lớn hơn 5, làm tròn xuống 5
  else if (speed > 5.0) {
    return 5.0;
  } 
  // Nếu vận tốc từ 1 đến 5, giữ nguyên giá trị
  else {
    return speed;
  }
}


// function lay ban tin sau khi da co lat, lon va cac thong so 
void CV_payload(void)
{
  memset(gps_rx, 0, strlen(gps_rx));
  get_signal_strength(rssi, rsrp, rsrq, 3);
  vTaskDelay(100/portTICK_PERIOD_MS);
  
  if(!a) 
  {
    a = 1;
    epoch.tv_sec = string_to_seconds((const char *)Datetime);
    settimeofday(&epoch, &utc);
  }
  gettimeofday(&epoch, 0);
// Kiểm tra nếu tốc độ và tai nạn
    
   bool crashed = checkAccident();
  float stabilizedSpeed = stabilizeGPSspeed(gps_t.speed_k);
  //float gps_speed = kalman();
  sprintf(gps_rx, "{\\\"lat\\\":%f,\\\"lon\\\":%f,\\\"speed\\\":%0.2f,\\\"overSpeed\\\":%s,\\\"Accident\\\":%s,\\\"mpu\\\":%f}", gps_t.latitude, gps_t.longitude, stabilizedSpeed, stabilizedSpeed> 3 ? "1" : "0", crashed ? "1" : "0", estimated_speed);
//sprintf(gps_rx, "{\\\"lat\\\":%f,\\\"lon\\\":%f,\\\"speed\\\":%f,\\\"overSpeed\\\":%f,\\\"Accident\\\":%f,\\\"gpt\\\":%f}", gps_t.speed_k, estimated_speed, accX, accY, accZ, velocity_z);
//sprintf(gps_rx, "{\\\"lat\\\":%f,\\\"lon\\\":%f,\\\"speed\\\":%0.2f,\\\"overSpeed\\\":%s,\\\"Accident\\\":%s}", gps_t.latitude, gps_t.longitude, estimated_speed, estimated_speed > 3 ? "1" : "0", crashed ? "1" : "0");
//sprintf(gps_rx, "{\\\"lat\\\":\\\"%f\\\",\\\"lon\\\":\\\"%f\\\",\\\"speed\\\":\\\"%0.2f\\\",\\\"overSpeed\\\":\\\"%s\\\",\\\"Accident\\\":\\\"%s\\\"}", gps_t.latitude, gps_t.longitude, gps_t.speed_k, gps_t.speed_k > 0.5 ? "1" : "0", crashed ? "1" : "0");


  Serial.print("accX : ");Serial.print(accX);
  Serial.print("\taccY : ");Serial.print(accY);
  Serial.print("\taccZ : ");Serial.println(accZ);
  printf("CarInfor: %s\n", gps_rx);
  ESP_LOGI(TAG, "Tran: %s", (char*) gps_rx);
}




