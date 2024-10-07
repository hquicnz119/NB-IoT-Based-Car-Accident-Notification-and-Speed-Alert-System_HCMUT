
/*
 * https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/uart.html#uart-api-setting-communication-pins
 */
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "simcom7020.h"
#include "gps_lib.h"
#define URI_MQTT         "mqtt3.thingspeak.com"
#define DEVICE_ID        "2272001"
#define DEVICE_TOKEN     "PCkSmXt32j6XuCjpne8Quecl"

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

client client_mqtt = {
    URI_MQTT,
    1883,
    "HzYuHAAnIhsyLQMCASQ4DBo",
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
    sprintf(topic_pub, "channels/%s/publish", DEVICE_ID);
    sprintf(topic_sub, "messages/%s/attribute", DEVICE_ID);
    
    //cai dat mpu6050
    Wire.begin();
    mpu6050.begin();
    //mpu6050.calcGyroOffsets(true);
   
}
 
 
 
void loop() {
 
  mpu6050.update();

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


// Hàm kiểm tra có tai nạn hay không
bool checkAccident() {
    // Đọc dữ liệu gia tốc từ cảm biến
    int accX = mpu6050.getAccX();
    int accY = mpu6050.getAccY();
    int accZ = mpu6050.getAccZ();

    // Tính toán giá trị gia tốc tổng cộng (độ lớn của vectơ gia tốc)
    float totalAcc = sqrt(accX * accX + accY * accY + accZ * accZ);

    // Đặt ngưỡng để xác định sự kiện va chạm
    float thresholdAcc = 1; // Giả sử giá trị này cần được điều chỉnh dựa trên tình huống cụ thể

    // Kiểm tra xem có sự kiện va chạm hay không
    if (totalAcc > thresholdAcc) {
        return true; // Tai nạn xảy ra
    } else {
        return false; // Không có tai nạn
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
   sprintf(gps_rx, "field1=%f&field2=%f&field3=%0.2f&field4=%s&field5=%s&status=MQTTPUBLISH", gps_t.latitude, gps_t.longitude, gps_t.speed_k, crashed ? "1" : "0", gps_t.speed_k > 0.5 ? "1" : "0");
  // sprintf(gps_rx, "{\\\"location\\\":{\\\"lat\\\":\\\"%f\\\",\\\"lon\\\":\\\"%f\\\"},\\\"speed\\\":\\\"%0.2f\\\",\\\"overSpeed\\\":\\\"%s\\\",\\\"Accident\\\":\\\"%s\\\"}", gps_t.latitude, gps_t.longitude, gps_t.speed_k, gps_t.speed_k > 0.5 ? "true" : "false", crashed ? "true" : "false");

// if (gps_t.speed_k > 0.5) {
//   // Gửi cảnh báo vượt tốc độ
//   sprintf(gps_rx, "{\\\"location\\\":{\\\"lat\\\":\\\"%f\\\",\\\"lon\\\":\\\"%f\\\"},\\\"speed\\\":\\\"%0.2f\\\",\\\"overSpeed\\\":\\\"true\\\"}", gps_t.latitude, gps_t.longitude, gps_t.speed_k);
// } else {
//   // Gửi dữ liệu bình thường
//   sprintf(gps_rx, "{\\\"location\\\":{\\\"lat\\\":\\\"%f\\\",\\\"lon\\\":\\\"%f\\\"},\\\"speed\\\":\\\"%0.2f\\\",\\\"overSpeed\\\":\\\"false\\\"}", gps_t.latitude, gps_t.longitude, gps_t.speed_k);
// }
  Serial.print("accX : ");Serial.print(mpu6050.getAccX());
  Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
  Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
  
  printf("CarInfor: %s\n", gps_rx);
  ESP_LOGI(TAG, "Tran: %s", (char*) gps_rx);
}




