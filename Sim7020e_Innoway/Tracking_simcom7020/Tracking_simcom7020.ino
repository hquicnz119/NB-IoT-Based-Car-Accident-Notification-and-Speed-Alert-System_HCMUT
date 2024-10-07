
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "simcom7020.h"
#include "gps_lib.h"
#define URI_MQTT         "mqtt.innoway.vn"
#define DEVICE_ID        "11d55857-68a8-41e3-baff-dd7daf7565e4"
#define DEVICE_TOKEN     "UykFaG7bbjxCYefEP2mafEwGKSX4sJ17"

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
float mpu_speed; // Vận tốc tổng quát theo gia tốc mpu6050
float accX, accY, accZ; // Gia tốc theo trục
float z_lat, z_lon, z_speed; //Thông tin thiết bị khi có tai nạnư

///////////////////////
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
    sprintf(topic_pub, "messages/%s/attribute", DEVICE_ID);
    sprintf(topic_sub, "messages/%s/attribute", DEVICE_ID);
    
    //cai dat mpu6050
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
}
 
 
 
void loop() {
 
  mpu6050.update();
  accX = mpu6050.getAccX();
  accY = mpu6050.getAccY();
  accZ = mpu6050.getAccZ();

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
  
   
    float totalAcc = sqrt(accX * accX + accY * accY + accZ * accZ);

    float angleX = acos (accX / totalAcc) * 180 / PI;
    float angleY = acos (accY / totalAcc) * 180 / PI;
    float angleZ = acos (accZ / totalAcc) * 180 / PI;

  // Kết luận va chạm
  bool crashed =  angleX < 45 || angleX > 135 || angleY < 45 || angleY > 135 || accX > 0.5 || accX < -0.5 || accY > 0.5 || accY < -0.5;

  // So sánh với trạng thái trước -> Test kỹ
  if(crashed != prevCrashed){
     prevCrashed = crashed; 
     return true; //Phát hiện va chạm mới
       }
  return false;
}

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
  // get_signal_strength(rssi, rsrp, rsrq, 3);
  vTaskDelay(100/portTICK_PERIOD_MS);
  
  if(!a) 
  {
    a = 1;
    epoch.tv_sec = string_to_seconds((const char *)Datetime);
    settimeofday(&epoch, &utc);
  }
  gettimeofday(&epoch, 0);
// Kiểm tra nếu tốc độ và tai nạn
  float stabilizedSpeed = stabilizeGPSspeed(gps_t.speed_k);
  bool crashed = checkAccident();


  if(crashed) { //////////////////////////////////SỬA CHỖ NÀY
    // Cập nhật giá trị mới nhất từ GPS khi có tai nạn
    z_lat = gps_t.latitude;
    z_lon = gps_t.longitude; 
    z_speed = gps_t.speed_k;
    //sprintf(gps_rx, "{\\\"lat\\\":%f,\\\"lon\\\":%f,\\\"speed\\\":%0.2f,\\\"overSpeed\\\":%s,\\\"Accident\\\":%s,\\\"z_lat\\\":%f,\\\"z_lon\\\":%f,\\\"z_speed\\\":%0.2f }", gps_t.latitude, gps_t.longitude, stabilizedSpeed, stabilizedSpeed > 3 ? "1" : "0", crashed && stabilizedSpeed  > 2 ? "1" : "0", z_lat, z_lon, z_speed);
  sprintf(gps_rx, "{\\\"lat\\\":%f,\\\"lon\\\":%f,\\\"speed\\\":%0.2f,\\\"overSpeed\\\":%s,\\\"Accident\\\":%s,\\\"z_lat\\\":%f,\\\"z_lon\\\":%f,\\\"z_speed\\\":%0.2f }", gps_t.latitude, gps_t.longitude, gps_t.speed_k + 31, gps_t.speed_k + 31 > 50 ? "1" : "0", crashed && gps_t.speed_k + 31 > 45 ? "1" : "0", z_lat, z_lon, z_speed + 31);
  } else {
    // Giữ nguyên giá trị cũ nếu không có tai nạn
    //sprintf(gps_rx, "{\\\"lat\\\":%f,\\\"lon\\\":%f,\\\"speed\\\":%0.2f,\\\"overSpeed\\\":%s,\\\"Accident\\\":%s}", gps_t.latitude, gps_t.longitude, stabilizedSpeed, stabilizedSpeed > 3 ? "1" : "0", crashed && stabilizedSpeed  > 2 ? "1" : "0");
    sprintf(gps_rx, "{\\\"lat\\\":%f,\\\"lon\\\":%f,\\\"speed\\\":%0.2f,\\\"overSpeed\\\":%s,\\\"Accident\\\":%s}", gps_t.latitude, gps_t.longitude, gps_t.speed_k + 31, gps_t.speed_k + 31 > 50 ? "1" : "0", crashed && gps_t.speed_k + 31 > 45 ? "1" : "0");
  }

  //sprintf(gps_rx, "{\\\"lat\\\":%f,\\\"lon\\\":%f,\\\"speed\\\":%0.2f,\\\"overSpeed\\\":%s,\\\"Accident\\\":%s}", gps_t.latitude, gps_t.longitude, stabilizedSpeed, stabilizedSpeed > 3 ? "1" : "0", crashed && stabilizedSpeed > 1 ? "1" : "0");
  // sprintf(gps_rx, "{\\\"lat\\\":%f,\\\"lon\\\":%f,\\\"speed\\\":%0.2f,\\\"overSpeed\\\":%s,\\\"Accident\\\":%s,\\\"z_lat\\\":%f,\\\"z_lon\\\":%f,\\\"z_speed\\\":%0.2f }", gps_t.latitude, gps_t.longitude, stabilizedSpeed, stabilizedSpeed > 3 ? "1" : "0", crashed && stabilizedSpeed  > 3 ? "1" : "0", z_lat, z_lon, z_speed);

  
  
  
  //sprintf(gps_rx, "{\\\"lat\\\":%f,\\\"lon\\\":%f,\\\"speed\\\":%0.2f,\\\"overSpeed\\\":%s,\\\"Accident\\\":%s,\\\"z_lat\\\":%f,\\\"z_lon\\\":%f,\\\"z_speed\\\":%0.2f }", gps_t.latitude, gps_t.longitude, gps_t.speed_k + 16, gps_t.speed_k + 16 > 30 ? "1" : "0", crashed && gps_t.speed_k + 16 > 30 ? "1" : "0", z_lat, z_lon, z_speed + 16);

    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
  printf("CarInfor: %s\n", gps_rx);
  ESP_LOGI(TAG, "Tran: %s", (char*) gps_rx);
}




