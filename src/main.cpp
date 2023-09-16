#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h> //thư viện truyền thông với PLC
#include <avr/interrupt.h>
#include <string.h>

//-----khai báo chân-------
#define soXungCanRot 200
#define cbBanXoay1 23
#define cbBanXoay2 25
#define cbCapLy 53
#define cbViTri_CapLy 51
#define cbViTri_chietRot 50
#define cbViTri_dapLy 48
#define cbViTri_dayLy 52
#define cbGioiHanTren_xilanh 45
#define in1 29
#define in2 31
#define in3 33
#define in4 35
#define ena 8
#define enb 9
#define chanXung 19
#define chanXung2 4
#define cbRobot 42
#define cbGiachut 22
#define vanNuoc1_on ModbusRTUClient.coilWrite(1, 0x03, 0x01)
#define vanNuoc1_off ModbusRTUClient.coilWrite(1, 0x03, 0x00)
#define vanNuoc2_on ModbusRTUClient.coilWrite(1, 0x05, 0x01)
#define vanNuoc2_off ModbusRTUClient.coilWrite(1, 0x05, 0x0)
#define Start_MotorCapLy ModbusRTUClient.coilWrite(1, 0x02, 0x01)
#define Stop_MotorCapLy ModbusRTUClient.coilWrite(1, 0x02, 0x00)
#define Start_MotorMangLy ModbusRTUClient.coilWrite(1, 0x01, 0x01)
#define Stop_MotorMangLy ModbusRTUClient.coilWrite(1, 0x01, 0x00)
#define Start_MotorCapDa ModbusRTUClient.coilWrite(1, 0x04, 0x01)
#define Stop_MotorCapDa ModbusRTUClient.coilWrite(1, 0x04, 0x00)
#define motorXoay220_Nghich ModbusRTUClient.coilWrite(1, 0x06, 0x01)
#define StartMotorHut ModbusRTUClient.coilWrite(1, 0x0a, 0x01)
#define StopMotorHut ModbusRTUClient.coilWrite(1, 0x0a, 0x00)
#define ngatCamBien ModbusRTUClient.coilWrite(1, 0x07, 0x01)
#define batCamBien ModbusRTUClient.coilWrite(1, 0x07, 0x00)
#define Reset 8
//---- các biến toàn cục---
TaskHandle_t taskHandle_hamChinh;
TaskHandle_t taskHandle_dayly;
TaskHandle_t taskHandle_daply;
TaskHandle_t taskHandle_captrasua;

unsigned int status = 0;
unsigned int soXung = 0;
unsigned int soXung2 = 0;
unsigned int soLy = 0; // biến chứa số ly mà khách order
int soluong = 0;
String dataRecieve;
String data_order = "1,1,0,2";
int mangOrder[50];

//---- nguyên mẫu hàm
void xoayBan();
void capLy();
void dapLy_xuong(); // dập nắp ly đi xuống
void dapLy_len();   // dập nắp ly đi lên
void dayLy_len();   // đẩy ly ra đi lên
void dayLy_xuong(); // đẩy ly ra đi xuống
void demxung();     // đếm xung rót nước
void demxung2();
void chayThuan();   // cơ cấu robot chay thuận
void chayNghich();  // cơ cấu robot chạy nghịch
void dung();        // hàm dừng cơ cấu robot
//-----khai báo các task---
void hamChinh(void *pvParameters);
void dieuKhienXilanhDay(void *pvParameters);
void capTraSua(void *pvParameters);


void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600);
  ModbusRTUClient.begin(9600);
  xTaskCreate(hamChinh, "hamChinh", 1024, NULL, -1, &taskHandle_hamChinh);
  xTaskCreate(dieuKhienXilanhDay, "dieuKhienXilanhDay", 256, NULL, 2, &taskHandle_dayly);
  xTaskCreate(capTraSua, "capTraSua", 1024, NULL, 3, &taskHandle_captrasua);

  pinMode(Reset,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(cbBanXoay1, INPUT_PULLUP); // đầu vào cảm biến góc ở bàn xoay
  pinMode(cbBanXoay2, INPUT_PULLUP); // đầu vào cảm biến góc ở bàn xoay
  pinMode(cbCapLy, INPUT_PULLUP);    // đầu vào cảm biến cấp ly
  pinMode(cbViTri_CapLy, INPUT_PULLUP);
  pinMode(cbViTri_chietRot, INPUT_PULLUP);
  pinMode(cbViTri_dapLy, INPUT_PULLUP);
  pinMode(cbViTri_dayLy, INPUT_PULLUP);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(cbRobot, INPUT_PULLUP);
  pinMode(cbGiachut,  INPUT_PULLUP);
  pinMode(cbGioiHanTren_xilanh,INPUT_PULLUP);
  pinMode(chanXung2,INPUT_PULLUP);
  pinMode(chanXung,INPUT_PULLUP);
  digitalWrite(Reset,HIGH);

  // chờ nhận thông tin order từ esp
  while(Serial2.available()==0);
  dataRecieve = Serial2.readString();
  data_order = dataRecieve;
  Serial.print(data_order);
  //---------------- xử lý thông tin order trước khi pha chế --------------------------------------------
  int j = 0;
  for (unsigned int i = 0; i <= data_order.length(); i++)
  {

    if ((data_order[i] == '0') | (data_order[i] == '1') | (data_order[i] == '2') | (data_order[i] == '3') | (data_order[i] == '4') | (data_order[i] == '5') | (data_order[i] == '6') | (data_order[i] == '7') | (data_order[i] == '8') | (data_order[i] == '9'))
    {
      switch (data_order[i])
      {
      case '1':
        mangOrder[j] = 1;
        break;
      case '2':
        mangOrder[j] = 2;
        break;
      case '3':
        mangOrder[j] = 3;
        break;
      case '4':
        mangOrder[j] = 4;
        break;
      case '5':
        mangOrder[j] = 5;
        break;
      case '6':
        mangOrder[j] = 6;
        break;
      case '7':
        mangOrder[j] = 7;
        break;
      case '8':
        mangOrder[j] = 8;
        break;
      case '9':
        mangOrder[j] = 9;
        break;
      default:
        mangOrder[j] = 0;
        break;
      }
      j++;
    }
  }
  //---------------------------------------------------------------------------------------------------------

  
  vTaskSuspend(taskHandle_dayly);
  vTaskSuspend(taskHandle_captrasua);
  ModbusRTUClient.setTimeout(100);
}
void loop()
{
}
void hamChinh(void *pvParameters)
{
  while (1)
  {
    soLy = mangOrder[0];
    vTaskDelay(pdMS_TO_TICKS(500));
    if((soLy>0)|(digitalRead(cbViTri_CapLy)==0)|(digitalRead(cbViTri_chietRot)==0)|(digitalRead(cbViTri_dapLy)==0)|(digitalRead(cbViTri_dayLy)==0)) 
     {
      xoayBan();
      vTaskDelay(pdMS_TO_TICKS(2000));    
     }
    
    if (soLy > 0)
    {
      capLy();
      while (digitalRead(cbViTri_CapLy) == 1)
        ;
      soLy--;
      mangOrder[0] = soLy;
    }
    vTaskResume(taskHandle_dayly);
    vTaskResume(taskHandle_captrasua);
    vTaskSuspend(taskHandle_hamChinh);



 
  }


 
}

void dieuKhienXilanhDay(void *pvParameters)
{
  for (;;)
  {
    if (digitalRead(cbViTri_dayLy) == 0)
    {
      dayLy_len();
      vTaskDelay(pdMS_TO_TICKS(10000));
      dayLy_xuong();
      vTaskDelay(pdMS_TO_TICKS(6000));
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      status += 1;
      if (status >= 2)
      {
        status = 0;
        vTaskResume(taskHandle_hamChinh);
      }
      vTaskSuspend(taskHandle_dayly);
    }
    else
    {
      status += 1;
      if (status >= 2)
      {
        status = 0;
        vTaskResume(taskHandle_hamChinh);
      }
      vTaskSuspend(taskHandle_dayly);
    }
  }
}

void capTraSua(void *pvParameters)
{
  for (;;)
  {

    // kiêm tra thông tin order tại đây

    if (digitalRead(cbViTri_chietRot) == 0) // nếu có ly tại vị trí chiết rót
    {
      Start_MotorCapDa;
      vTaskDelay(pdMS_TO_TICKS(4000));
      Stop_MotorCapDa;
      // kiểm tra loại trà sữa ở đây

      if (mangOrder[1] > 0)
      {
        vanNuoc1_on;
        int i = mangOrder[1];
          i=i-1;
          mangOrder[1]=i;
        while (1)
        {
          demxung();
          if (soXung >= soXungCanRot)  
          {
            soXung = 0;
            break;
          }
        }
        vanNuoc1_off;
      }
      else
      {
        if (mangOrder[2] > 0)
        {
          vanNuoc2_on;
          int j = mangOrder[2];
          j=j-1;
          mangOrder[2]=j;
          while (1)
          {
            demxung2();
            if (soXung2 >= soXungCanRot)
            {
              soXung2 = 0;
              break;
            }
          }
          vanNuoc2_off;
        }
      }

      if (digitalRead(cbViTri_dapLy) == 0)
      {

        chayNghich();
        while (1)
        {
          if (digitalRead(cbRobot) == 0)
          {
            dung();
            break;
          }
        }
        dapLy_xuong();
        vTaskDelay(pdMS_TO_TICKS(7000));
        StartMotorHut;
        vTaskDelay(pdMS_TO_TICKS(4000));
        StopMotorHut;
        ngatCamBien;
        dapLy_len();
        vTaskDelay(pdMS_TO_TICKS(7000));

        chayThuan();
        vTaskDelay(pdMS_TO_TICKS(350));
        dung();
        // dập nắp ly
        dapLy_xuong();
        vTaskDelay(pdMS_TO_TICKS(15000));
        dapLy_len();
        vTaskDelay(pdMS_TO_TICKS(10000));
        //dừng xilanh
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        batCamBien;
      }
      status += 1;
      if (status >= 2)
      {
        status = 0;
        vTaskResume(taskHandle_hamChinh);
      }
      vTaskSuspend(taskHandle_captrasua);
    }
    else
    {
      if (digitalRead(cbViTri_dapLy) == 0)
      {

        chayNghich();
        while (1)
        {
          if (digitalRead(cbRobot) == 0)
          {
            dung();
            break;
          }
        }
        dapLy_xuong();
        vTaskDelay(pdMS_TO_TICKS(7000));
        StartMotorHut;
        vTaskDelay(pdMS_TO_TICKS(4000));
        StopMotorHut;
        ngatCamBien;
        dapLy_len();
        vTaskDelay(pdMS_TO_TICKS(10000));
        chayThuan();
        vTaskDelay(pdMS_TO_TICKS(350));
        dung();
        
        dapLy_xuong();
        vTaskDelay(pdMS_TO_TICKS(15000));
        dapLy_len();
        vTaskDelay(pdMS_TO_TICKS(10000));
        //dừng xilanh
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        batCamBien;
      }
      status += 1;
      if (status >= 2)
      {
        status = 0;
        vTaskResume(taskHandle_hamChinh);
      }
      vTaskSuspend(taskHandle_captrasua);
    }
  }
}

void xoayBan()
{
  // có 2 trường hợp để điều khiển bàn xoay
  /*
    TH1:không có cảm biến nào sập rãnh thì lúc này hai đầu vào đưa về vdk đều sẽ ở mức cao
    => lúc này chỉ kích cho bàn xoay chạy đến khi nào 1 trong 2 cảm biến sập rãnh thì sẽ dừng lại
    (TH này chỉ xảy ra khi bắt đầu mở máy khi bàn xoay chưa vào đúng vị trí kể từ lần thứ 2 trở đi TH này sẽ không xảy ra nữa)
    TH2:khi 1 trong 2 cảm biến sập rãnh thì chúng ta cứ việc cho bàn xoay chạy đến khi nào cảm biến còn lại sập rãnh thì dừng bàn xoay

  XX - lưu ý khi sập rãnh tín hiệu cảm biến đưa về sẽ là mức 0

  */
  if ((digitalRead(cbBanXoay1) == 0) && (digitalRead(cbBanXoay2) == 0)) // khi 2 cảm biến không sập rãnh
  {
    ModbusRTUClient.coilWrite(1, 0x00, 0x01); // viết lên slave có id = 1,lên vùng nhớ M0 với value là 0x00
    // bật cho bàn xoay chạy

    while (1)
    {

      if ((digitalRead(cbBanXoay1) == 1) || (digitalRead(cbBanXoay2) == 1))
      {
        ModbusRTUClient.coilWrite(1, 0x00, 0x00); // dừng bàn xoay
        break;
      }
    }
  }
  else if (digitalRead(cbBanXoay1) == 1) // nếu cảm biến 1 đang sập rãnh
  {
    ModbusRTUClient.coilWrite(1, 0x00, 0x01);

    while (1)
    {

      if (digitalRead(cbBanXoay2) == 1)
      {
        ModbusRTUClient.coilWrite(1, 0x00, 0x00);
        break;
      }
    }
  }
  else
  {
    // cảm biến 2 đang sập rãnh
    ModbusRTUClient.coilWrite(1, 0x00, 0x01);

    while (1)
    {

      if (digitalRead(cbBanXoay1) == 1)
      {
        ModbusRTUClient.coilWrite(1, 0x00, 0x00);
        break;
      }
    }
  }
}

void capLy()
{
  vTaskDelay(pdMS_TO_TICKS(2000));
  Start_MotorCapLy;
  vTaskDelay(pdMS_TO_TICKS(500));
  while (1)
  {
    if (digitalRead(cbCapLy) == 1)
    {
      Stop_MotorCapLy;
      break;
    }
  }
  vTaskDelay(pdMS_TO_TICKS(2000));

  Start_MotorCapLy;
  vTaskDelay(pdMS_TO_TICKS(500));
  while (1)
  {
    if (digitalRead(cbCapLy) == 1)
    {
      Stop_MotorCapLy;
      break;
    }
  }
}
void dapLy_xuong()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void dapLy_len()
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void dayLy_len()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
void dayLy_xuong()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void demxung()
{
  while (digitalRead(chanXung) == 1)
    ;
  soXung++;
  while (digitalRead(chanXung) == 0)
    ;
}
void chayThuan()
{
  ModbusRTUClient.coilWrite(1, 0x06, 0x00);
  Start_MotorMangLy;
}
void chayNghich()
{
  motorXoay220_Nghich;
  Start_MotorMangLy;
}
void dung()
{
  Stop_MotorMangLy;
  ModbusRTUClient.coilWrite(1, 0x06, 0x00);
}
void demxung2()
{
  while (digitalRead(chanXung2) == 1)
    ;
  soXung2++;
  while (digitalRead(chanXung2) == 0)
    ;
}