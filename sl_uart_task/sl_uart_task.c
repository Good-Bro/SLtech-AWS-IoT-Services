/** 
 * @mainpage CANADA향 LCD형 ESP-12E 모듈 개발
 * @brief ESP8266 Arduino core환경 개발@n
 * @details 이 프로젝트 자세한 내용 정리를 위한 개발 문서입니다
 * @author Goodbro(CTO) 이우섭
 * @date 2018.08.06
 * 
 * @li Wifi Module > Wall Pad
 * FCU                                                                                                                                       | ERV
 * ----------------------------------------------------------------------------------------------------------------------------------------- | ----
 * Header@n init : 0xF5@n 상태응답 : 0xD5@n 제어응답 : 0XC5                                                                                    |Header@n 상태응답 : 0xD7@n 제어응답 : 0XC7
 * Wifi감도@n 중단(<-85) : 0x01@n 연결1(<-75) : 0x02@n 연결2(> -75) : 0x03                                                                     |Wifi감도@n FCU와 동일
 * Mode & Fan 속도@n MSB [냉방 : 0x00] [난방 : 0x10] [송풍 : 0x20] +Fan 자동 : 0x80@n LSB [Off : 0x00] [Low : 0x01] [Mid : 0x02] [High : 0x03] |Mode@n Vent : 0x00@n Eco : 0x01@n Timer : 0x02@n ByPass : 0x03
 * FLAG@n 7bit : Remote lock@n 6bit : Mode event@n 3bit : Timer 비/가동@n 1bit: 화씨/섭씨@n 0bit : 가동신호                                    |Fan 속도@n Off : 0x00@n Low : 0x01@n Mid : 0x02@n High : 0x03
 * 온도세팅@ 7bit : .5도@n 0~6bit : 0~127도                                                                                                   |FLAG@n 7bit : Remote lock@n 6bit : Mode event@n 3bit : Timer 비/가동@n 2bit : Heater off/on@n 1bit : 화씨/섭씨@n 0bit : 가동신호
 * Wifi Status@n AP & 중단 : 0x00@n Wifi & 중단 : 0x10@n Wifi & 연결 : 0x11                                                                   |Wifi Status@n FCU와 동일
 * ECODE@n 내부통신에러 : 0x01                                                                                                                |ECODE@n FCU와 동일
 * CheckSum                                                                                                                                  |CheckSum
 * 
 * @li Wifi Module < Wall Pad
 * FCU                                                                                                                                                                                               | ERV
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----
 * Header@n init : 0xE5@n 상태응답 : 0xB5@n 제어응답 : 0XA5                                                                                                                                            |Header@n 상태응답 : 0xB7@n 제어응답 : 0XA7
 * 현재온도 (소수점)                                                                                                                                                                                  |현재온도 (소수점)
 * Mode & Fan 속도@n MSB [냉방 : 0x00] [난방 : 0x10] [송풍 : 0x20] +Fan 자동 : 0x80@n LSB [Off : 0x00] [Low : 0x01] [Mid : 0x02] [High : 0x03]                                                         |Mode@n Vent : 0x00@n Eco : 0x01@n Timer : 0x02@n ByPass : 0x03
 * FLAG@n 7bit : Remote lock@n 6bit : 팬 가동상태@n 5bit : 실내온도 센서(RC센서/리턴제어)@n 4bit : Pipe 개수(2/4)@n 3bit : 현재온도(양수/음수)@n 2bit : 동파방지 비/가동@n 1bit: 화씨/섭씨@n 0bit : 가동신호 |Fan 속도@n Off : 0x00@n Low : 0x01@n Mid : 0x02@n High : 0x03
 * 온도세팅@ 7bit : .5도@n 0~6bit : 0~127도                                                                                                                                                           |FLAG@n 7bit : Remote lock@n 6bit : Mode event@n 4bit : Filter교체알람@n 3bit : 현재온도(양수/음수)@n 2bit : Heater off/on@n 1bit : 화씨/섭씨@n 0bit : 가동신호
 * 현재온도 (정수)                                                                                                                                                                                    |현재온도 (정수)
 * ECODE@n 내부통신에러 : 0x01@n R/C온도센서에러 : 0x02@n 동파방지센서에러 : 0x04@n Pipe1온도센서에러 : 0x08@n Air센서에러 : 0x10@n 수위센서에러 : 0x20@n Pipe2온도센서에러 :0x40@n Fan(BLDC)에러 : 0x80     |ECODE@n 내부통신에러 : 0x01@n 실내온도에러 : 0x08@n 외기온도에러 : 0x10@n Fan(BLDC)에러 : 0x80
 * CheckSum                                                                                                                                                                                          |CheckSum
 */

//Arduino core library
#include <Arduino.h>
#include <Stream.h>
#include <EEPROM.h>

//ESP8266 arduino core library
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

//AWS
#include "sha256.h"
#include "Utils.h"

//WEBSockets
#include <Hash.h>
#include <WebSocketsClient.h>

//MQTT PUBSUBCLIENT LIB 
#include <PubSubClient.h>

//Custom Header 첨부
#include "WallPadCommon.h"

//AWS MQTT Websocket
#include "Client.h"
#include "AWSWebSocketClient.h"
#include "CircularByteBuffer.h"

//esp8266 core library
#include <stdarg.h>  
#include <os_type.h>

extern "C" {
  #include "user_interface.h"
}

#include <osapi.h>

#define WallpadSerial Serial
#define DebugSerial Serial1

///////////////////////////////////////////////////////////////////////
//Wallpad global variable

static WallPad g_WallPad;
/**
 * @brief 장비 이름을 저장하는 변수
 */
static char g_DeviceName [nMAX_NAME] = "SLT_FCUR-WTEST2";
/**
 * @brief 장비 버전 정보를 담는 초기화 체크 변수
 */
static char g_sInitTitle  [ nMAX_INIT_CHK ] = "SLTech. WallPad. V2.1(2018.02.13)";
/**
 * @brief 와이파이 접속 정보중 SSID를 담는 변수
 */
static char g_sInitSsid   [ nMAX_NAME ] = "";
/**
 * @brief 와이파이 접속 정보중 비밀번호를 담는 변수
 */
static char g_sInitSspass [ nMAX_NAME ] = "";
static ApList g_ApList  [ nMAX_SEARCH_AP ];
static pApList  g_pApList [ nMAX_SEARCH_AP ];
/**
 * @brief 주변 AP리스트를 저장하는 변수
 */
static int    g_nMaxAp;
static LastSend g_LastSend;

/**
 * @brief mqtt 수신 정보를 기록하는 글로벌 변수
 */
char g_cmd[8] = {0x00,};
/**
 * @brief mqtt 수신 트리거 변수
 */
int g_msgSend = 0;
/**
 * @brief 와이파이 연결 정보를 관리하기 위한 글로벌 변수 
 */
bool g_bConnecting = false;
bool g_bAppctl = false; 

/** 
 * @brief 오류 확인 함수
 * @details 파라미터 data를 가지고 checksum 판단
 * @param char * data 
 * @return int 0 : OK / 1 : Fail
 */
int checksum(char* data){
  if(data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]^data[7])
    return 0;
  else
    return 1;
}

/** 
 * @brief 응답 데이터 형식지정 함수 
 * @details 파라미터 readcmd의 데이터를 가지고 응답 데이터 형식을 만든다
 * @see 이 함수의 코드분석 내용
 * 1. wifi가 연결되어있다면 wifi의 감도에 따라 데이터 지정@n
 * 2. wifi의 상태를 global 구조체 g_WallPad 의 emode에 따라 지정@n
 * 3. 파라미터 readcmd[0] (Header)에 따라 구조형식을 맞춘다 [참고 : 표]@n
 * 4. 완성된 응답 데이터 형식의 체크섬을 값을 맨 마지막에 넣는다@n
 * 5. 시리얼 통신으로 값을 보낸다@n
 * 6. global 변수(int) msgsend값을 하나 줄인다
 * @param char * readcmd : Wifi Module > Wall Pad 
 * @return -
 */
void responsecmd(char* readcmd){
  // 통신 포맷 변경
  if(readcmd[0]== 0xc7)
    DebugSerial.println("ERVcmd sended");
  else if(readcmd[0] == 0xc5){
    DebugSerial.println("FCUcmd sended");
  }

  // wifi 감도
  byte cmd[8];
    if(WiFi.isConnected()){
      if(WiFi.RSSI() < - 85){
        cmd[1] = 0x01;
      }
      else if(WiFi.RSSI() < -75){
        cmd[1] = 0x02;
      }
      else{
        cmd[1] = 0x03;
      }
    }

  // wifi 상태
    switch(g_WallPad.eMode){
        case eMODE_AP:default:
          cmd[5] = 0x00; //WIFI status AP mode disconnected
        break;
        case eMODE_CONNECT:
            cmd[5] = 0x10; //WIFI status stand-alone mode disconnected
        break;
        case eMODE_NORMAL:
            cmd[5] = 0x11;
        break;   
    }

    switch(readcmd[0]) {
      // FCU : Wall pad 수신 : 상태/제어 응답
      case 0xa5 : case 0xb5:
        cmd[0] = 0xd5;
        cmd[2] = (byte)readcmd[2];  //MODE&FAN
        cmd[3] = (byte)readcmd[3];  //FLAG
        cmd[4] = (byte)readcmd[4];  //TARGET TEMP
        cmd[6] = 0x00;
      break;

      // FCU : Wall pad 수신 : Wifi 초기화 응답
      case 0xe5:
        cmd[0] = 0xf5;
        cmd[2] = 0x00;
        cmd[3] = 0x00;
        cmd[4] = 0x00;
        cmd[6] = 0x00;
        g_WallPad.eMode = eMODE_AP;
        WiFi.mode(WIFI_OFF);
        strcpy( g_WallPad.sSsId     , g_sInitSsid   );
        strcpy( g_WallPad.sSsPass   , g_sInitSspass   );
      break;

      // ERV : Wall Pad 수신 : 상태/제어 응답
      case 0xa7: case 0xb7: case 0xb8:
        cmd[0] = 0xd7;
        cmd[2] = (byte)readcmd[2];
        cmd[3] = (byte)readcmd[3];
        cmd[4] = (byte)readcmd[4];
        cmd[6] = 0x00;
      break;

      case 0xc5: case 0xc7:
        cmd[0] = (byte)readcmd[0];
        cmd[2] = (byte)readcmd[2];
        cmd[3] = (byte)readcmd[3];
        cmd[4] = (byte)readcmd[4];
        cmd[6] = 0x00;
      break;

    }

    // CHECK SUM
    cmd[7] = cmd[0]^cmd[1]^cmd[2]^cmd[3]^cmd[4]^cmd[5]^cmd[6];

    for(int i = 0 ; i < 8 ; i++)
    {
      Serial.write(cmd[i]);
    }
    
    if(g_msgSend>0){
      g_msgSend--;
    }
}

///////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////// AWS MQTT 

//AWS IOT config, change these:
char wifi_ssid[]       = "godjaehyeok";
char wifi_password[]   = "gjwogur123";
char aws_endpoint[]    = "a2q36qubve1ukk.iot.ap-northeast-2.amazonaws.com";
char aws_key[]         = "AKIAJA4X53VAFTGEDQ3Q";
char aws_secret[]      = "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";
char aws_region[]      = "ap-northeast-2";
const char* aws_topic_update  = "$aws/things/test2/shadow/update";
const char* aws_topic  = "$aws/things/test2/shadow/update/accepted";

int port = 443;

//# of connections
long connection = 0;

//MQTT config
const int maxMQTTpackageSize = 512;
const int maxMQTTMessageHandlers = 1;

ESP8266WiFiMulti WiFiMulti;
AWSWebSocketClient awsWSclient(1024);
PubSubClient client(awsWSclient);

//generate random mqtt clientID

/** 
 * @brief 클라이언트 ID 자동생성 함수 
 * @details 랜덤함수를 가지고 1~256 사이 수의 숫자를 23개 만든다
 * @param -
 * @return char * : 클라이언트 ID
 */
char* generateClientID () {
  char* cID = new char[23]();
  for (int i=0; i<22; i+=1)
    cID[i]=(char)random(1, 256);
  return cID;
}

//connects to websocket layer and mqtt layer

/** 
 * @brief  websocket & MQTT 접속 함수
 * @details 클라이언트 ID를 가지고 서버(server)에 접속한다
 * @see 이 함수의 코드분석 내용
 * 1. 이미 접속되어있는지 판단하고 되어있으면 연결을 끊는다@n
 * 2. 딜레이 함수 실행 : 힙 공간을 안전하게 확보@n
 * 3. 클라이언트 ID 자동생성 함수를 가지고 ID 확보@n
 * 4. 서버 연결하기 위한 서버 주소와 포트 설정@n
 * 5. 클라이언트 ID로 접속되어있는지 판단
 * @param -
 * @return bool OK / Fail 
 */
bool connect () {
  
    if (client.connected()) {    
        client.disconnect ();
    }  
    //delay is not necessary... it just help us to get a "trustful" heap space value
    delay (1000);
    DebugSerial.print (millis ());
    DebugSerial.print (" - conn: ");
    DebugSerial.print (++connection);
    DebugSerial.print (" - (");
    DebugSerial.print (ESP.getFreeHeap ());
    DebugSerial.println (")");


    //creating random client id
    char* clientID = generateClientID ();
    
    client.setServer(aws_endpoint, port);
    if (client.connect(clientID)) {
      DebugSerial.println("connected");     
      return true;
    } else {
      DebugSerial.print("failed, rc=");
      DebugSerial.print(client.state());
      return false;
    }
    
}

//subscribe to a mqtt topic

/** 
 * @brief  MQTT subscribe 함수
 * @details client callback함수와 MQTT topic을 지정한다
 * @param -
 * @return -
 */
void subscribe () {
    client.setCallback(callback);
    client.subscribe(aws_topic);

   //subscript to a topic
    DebugSerial.println("MQTT subscribed");
}

//send a message to a mqtt topic
//AWS 사물 shadow에 해당 사물 메시지 전

/** 
 * @brief  publishing 함수
 * @details 파라미터 readcmd와 appstate 정보를 가지고 정보를 가공해 메세지를 보낸다
 * @see 이 함수의 코드분석 내용
 * 1. 파라미터 appstate 상태 확인@n
 * 2. 파라미터 readcmd[0] (header)정보와 global 구조체 LastSend의 정보를 비교한다@n
 * 3. 파라미터 readcmd[0] (header)정보가 FCU의 상태/제어 응답인 경우 : readcmd 정보를 가공해 버퍼에 저장하고 checksum값을 global 구조체 LastSend에 넣는다@n
 * 4. 파라미터 readcmd[0] (header)정보가 ERV의 상태/제어 응답인 경우 : readcmd 정보를 가공해 버퍼에 저장하고 checksum값을 global 구조체 LastSend에 넣는다@n
 * 5. 파라미터 readcmd[0] (header)정보가 그외 메시지인 경우 : readcmd 정보를 가공해 버퍼에 저장하고 checksum값을 global 구조체 LastSend에 넣는다@n
 * 6. 현재 온도 정수 값을 global 구조체 LastSend에 넣는다@n
 * 7. 가공된 버퍼를 지정된 topic에 publish한다@n
 * 8. global 변수(bool) g_bAppctl를 false로 대입한다
 * @param char * readcmd : Wifi Module < Wall Pad
 * @param bool 
 * appstate 
 * @return - 
 */
void sendAwsMsg (char* readcmd, bool appstate) {
    //send a message   
    char buf[100];

    if(!appstate){
      if(readcmd[0] == 0xb5 && (abs(g_LastSend.cTemp - readcmd[5]) < 1 ) || (g_LastSend.cFcu == readcmd[7]) ) return;
      if(readcmd[0] == 0xb7 && (abs(g_LastSend.cTemp - readcmd[5]) < 1 ) || (g_LastSend.cErv == readcmd[7]) )return;
      if(readcmd[0] == 0xb8 && (g_LastSend.cSensor == readcmd[7]) )return;
    }
    //FCU
    if(readcmd[0] == 0xb5 || readcmd[0] == 0xa5 ){
      sprintf(buf, "{\"state\":{\"reported\":{\"cmd\": [%d,%d,%d,%d,%d,%d,%d,%d]}}}", readcmd[0], readcmd[1],readcmd[2], readcmd[3],readcmd[4], readcmd[5],readcmd[6], readcmd[7]);
      g_LastSend.cFcu = readcmd[7];
    }
    //ERV
    else if(readcmd[0] == 0xb7 || readcmd[0] == 0xa7){ 
      sprintf(buf, "{\"state\":{\"reported\":{\"cmdERV\": [%d,%d,%d,%d,%d,%d,%d,%d]}}}", readcmd[0], readcmd[1],readcmd[2], readcmd[3],readcmd[4], readcmd[5],readcmd[6], readcmd[7]);
      g_LastSend.cErv = readcmd[7];
    }

    else if(readcmd[0] == 0xb8){
      sprintf(buf, "{\"state\":{\"reported\":{\"sensorState\": [%d,%d,%d,%d,%d,%d,%d,%d]}}}", readcmd[0], readcmd[1],readcmd[2], readcmd[3],readcmd[4], readcmd[5],readcmd[6], readcmd[7]);
      g_LastSend.cSensor = readcmd[7];
    }
  
    g_LastSend.cTemp = readcmd[5];
    int rc = client.publish(aws_topic_update, buf);
    g_bAppctl = false;
}



//callback to handle mqtt messages
//uart task timer에서 wallpad로 회신할 메세지 세트 생성

/** 
 * @brief  callback 함수 
 * @details 받은 메시지를 길이만큼 버퍼에 담아 가공한다
 * @see 이 함수의 코드분석 내용
 * 1. payload로 받은 메시지 길이 만큼 msg에 복사한다@n
 * 2. msg에서 "desired"문자열이 있는지 찾는다@n
 * 3. 위의 문자열이 존재한다면, global 변수(int) g_msgSend 값을 2로 한다@n
 * 4. 다시 msg에서 "cmpAPP"라는 문자열이 있는지 찾고 그 포인터를 str로 연결한다@n
 * 5. str에서 포인터를 특정 문자([,])를 만나도록 이동후 잘라서 그 값을 ASCII값을 token에 넣는다@n
 * 6. 그 token값이 1인 경우 : global 변수(bool) g_bAppctl를 true로 대입한다@n
 * 7. 다시 msg에서 "cmdERV"라는 문자열이 있는지 찾고 그 포인터를 str로 연결한다@n
 * 8. str에서 포인터를 특정 문자([,])를 만나도록 이동후 잘라서 그 값(ASCII)을 token에 넣는다@n
 * 9. 4번 반복하여 그 값(ASCII)을 erv[4]에 넣는다@n
 * 10. 다시 msg에서 "cmdFCU"라는 문자열이 있는지 찾고 그 포인터를 str로 연결한다@n
 * 11. str에서 포인터를 특정 문자([,])를 만나도록 이동후 잘라서 그 값(ASCII)을 token에 넣는다@n
 * 12. 6번 반복하여 그 값(ASCII)을 fcu[6]에 넣는다@n
 * 13. fcu[5]의 1인 경우(FCU) : fcu checksum을 확인@n 
 * 14. checksum이 문제 없는 경우 : global 변수(char) g_cmd[8]에 값을 세팅한다@n
 * 15. fcu[5]의 0인 경우(ERV) : lobal 변수(char) g_cmd[8]에 값을 세팅한다@n
 * 16. lobal 변수(char) g_cmd[8]에 값을 세팅한다@n
 * @param  topic:  subcriber가 정한 topic 위치
 * @param  payload: 받은 메시지
 * @param  length: 메시지 길이
 * @return -
 */
void callback(char* topic, byte* payload, unsigned int length) {
  char msg[length];
  memcpy(msg, payload, length);

  char* str;
  char* token;

  byte fcu[6];byte erv[4];

  if(strstr(msg,"desired")!=NULL){
    g_msgSend = 2;
    str = strstr(msg,"cmdAPP");
    token = strtok(str+8,"[,]");

    if(token - 48){
      DebugSerial.println("App is controlled");
      g_bAppctl = true;
    }
    
    DebugSerial.print("cmdAPP : " + (String)token + " cmdERV : ");
    
    
    str = strstr(msg,"cmdERV");
    token = strtok(str+8,"[,]");
    
    for(int i= 0; i < 4; i++){
      String tobyte(token);
      erv[i] = tobyte.toInt();
      DebugSerial.print((String)erv[i] +" ");
      token = strtok(NULL,",");
    }
    DebugSerial.print(" cmdFCU : ");
    str = strstr(msg,"cmdFCU");
    token = strtok(str+8,"[,]");
    
    for(int i = 0; i < 6 ; i++){
      String tobyte(token);
      fcu[i] = tobyte.toInt();
      DebugSerial.print((String)fcu[i]+ " ");
      token = strtok(NULL,",");
    }
    
    if(!fcu[5]){
      if(!(fcu[0] ^ fcu[1] ^ fcu[2] ^ fcu[3]^fcu[4])){
        DebugSerial.println("FCU received");
        g_cmd[0] = 0xc5;
        g_cmd[2] = fcu[0] | fcu[1];
        g_cmd[3] = fcu[2];
        g_cmd[4] = fcu[3];
      }
    }
    else{
      DebugSerial.println("ERV received");
      g_cmd[0] = 0xc7;
      g_cmd[2] = erv[0];
      g_cmd[3] = erv[1];
      g_cmd[4] = erv[2];
    }

    g_cmd[1] = 0x00;
    g_cmd[5] = 0x11;
    g_cmd[6] = 0x00;
    g_cmd[7] = g_cmd[0] ^ g_cmd[1] ^ g_cmd[2] ^ g_cmd[3] ^ g_cmd[4] ^ g_cmd[5] ^ g_cmd[6] ;
  }
}


///////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////// UART TASK 선언부

#define UART_RX_BUFFER_SIZE 1024
uint8_t UART_RX_BUFFER[UART_RX_BUFFER_SIZE];
long int UART_RX_RECEIVE_COUNT = 0;

#define UART_RX_TASK_PRIORITY 1
#define UART_TASK_PRIORITY 0

#define UART_RX_QUEUE_SIZE 8

static os_timer_t UART_RX_TASK_Timer;
static os_timer_t UART_TASK_Timer;

static os_event_t UART_RX_QUEUE[UART_RX_QUEUE_SIZE];

long int UART_RX_CYCLE_IN_MS = 100;

//os_timer timeout callback

/**
 * @brief  task 메세지를 보내는 함수
 * @details ESP8266 NOn-os SDK에서 제공되는 API를 호출하는 함수@n
 *         이 함수 내부의 system_os_post함수를 지정된 형식으로 호출된다 (ESP8266-UART-RX-Interrupt)
 * @see 이 함수의 코드분석 내용
 * system_os_post API함수만 살펴보면 된다@n
 * bool system_os_post (uint8 prio, os_signal_t sig, os_param_t par)@n
 * uint8 prio : task priority@n
 * os_signal_t sig : message type@n
 * os_param_t par : message parameters@n
 * 여기서는 system_os_post(UART_RX_TASK_PRIORITY, 0, 0)으로 사용했다@n
 * UART_RX_TASK_PRIORITY 값은 1이다
 * @param -
 * @return -
 */
void Temp_Run_UART_RX_Task(void*) {
  system_os_post(UART_RX_TASK_PRIORITY, 0, 0);
}

void sendAwsMsg (char* readcmd);

//thread entry point

/**
 * @brief  UART_RX_TASK_Handler함수 (트랩)
 * @details UART로 받은 데이터가 있는 확인 후 그 데이터를 경우따라 분류 후 타이머를 설정한다
 * @see 이 함수의 코드분석 내용
 * 1. 받은 데이터를 있는지 확인한다@n
 * 2. 받은 데이터를 버퍼에 저장한다@n
 * 3. global 변수(int) g_msgSend 값이 0일 경우, 오류 확인 함수(checksum)로 버퍼의 값를 확인한다@n
 * 4. 오류 시, 응답 데이터 형식지정 함수을 호출한다@n
 * 5. wifi와 서버에 둘다 연결이 되어 있는 경우, publishing 함수를 호출한다@n
 * 6. global 변수(int) g_msgSend 값이 0이 아닐 경우, 응답 데이터 형식지정 함수을 호출한다@n
 * 7. API함수인 os_timer_arm를 호출한다 (밀리초 타이머를 설정)@n
 *    void os_timer_arm (os_timer_t *ptimer, uint32_t milliseconds, bool repeat_flag)@n
 *    os_timer_t *ptimer : 타이머 구조체@n
 *    uinit32_t milliseconds : ms@n
 *    bool repeat_flag : 반복 유/무 설정
 * @param os_event_t*
 * @return -
 */
void UART_RX_TASK_Handler(os_event_t *) {

  /* Wallpad Serial RX buffer 점검
   * if Wallpad Serial RX buffer is empty ==> WallpadSerial.available() return 0
   * if something is filled in Wallpad Serial RX buffer  ==> WallpadSerial.available() return 1
   */
  if (WallpadSerial.available() > 0)
  {

    
    //Wallpad로부터 8byte cmd 수신
    byte buf[8];
    WallpadSerial.readBytes(buf,8);

    if(!g_msgSend){
      if(checksum((char*)buf))
        responsecmd((char*)buf);
        
      //if (WiFi.isConnected() && awsWSclient.connected ()) {      
      if (WiFi.isConnected() && awsWSclient.connected ()) {    
        sendAwsMsg((char*)buf , g_bAppctl);    
      }    
      
    }

    else{
      responsecmd(g_cmd);
    }
    
  }
  
  os_timer_arm(&UART_RX_TASK_Timer, UART_RX_CYCLE_IN_MS /*ms*/, 0 /*once*/);
}
///////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////// EEPROM

/**
 * @brief EEPROM 초기화 함수
 * @details global 구조체 g_WallPad 값을 정해진 값으로 세팅하고 그 구조체만큼 EEPROM를 0으로 세팅한다
 * @see 이 함수의 코드분석 내용
 * 1. global 구조체 g_WallPad eMode를 값을 eMode_NONE 대입한다@n
 * 2. global 구조체 g_WallPad sInitChk 값을 global 배열[char] g_sInitTitle 값으로 대입한다@n
 * 3. global 구조체 g_WallPad sSsId 값을 global 배열[char] g_sInitSsid 값으로 대입한다@n
 * 4. global 구조체 g_WallPad sSsPass 값을 global 배열[char] g_sInitSspass 값으로 대입한다@n
 * 5. global 구조체 g_WallPad 크기만큼 EEPROM를 0로 세팅한다
 * @param -
 * @return -
 */
void InitEeprom( void )
{

  g_WallPad.eMode   = eMODE_NONE;
  strcpy( g_WallPad.sInitChk    , g_sInitTitle    );
  strcpy( g_WallPad.sSsId     , g_sInitSsid   );
  strcpy( g_WallPad.sSsPass   , g_sInitSspass   );
  for( int i= 0; i < sizeof( WallPad ); i++ )
    EEPROM.write( i, 0 );
  EEPROM.commit();
}

/**
 * @brief EEPROM 쓰기 함수
 * @details global 구조체 g_WallPad 값을 EEPROM에 세팅한다
 * @see 이 함수의 코드분석 내용
 * 1. global 구조체 g_WallPad를 포인터로 연결한다@n
 * 2. global 구조체 g_WallPad 크기만큼 EEPROM를 global 구조체 g_WallPad 값으로 세팅한다
 * @param -
 * @return -
 */
void StoreEeprom( void )
{
   char* pSetData = ( char*) &g_WallPad;
  int i;
  for( i=0; i < sizeof( WallPad ); i++ )
    EEPROM.write( i, *pSetData++ );
  EEPROM.commit();
}

/**
 * @brief EEPROM 읽기 함수
 * @details EEPROM의 값을 global 구조체 g_WallPad로 불러들인다
 * @see 이 함수의 코드분석 내용
 * 1. global 구조체 g_WallPad를 포인터로 연결한다@n
 * 2. global 구조체 g_WallPad 크기만큼 global 구조체 g_WallPad를 EEPROM 값으로 세팅한다
 * @param -
 * @return -
 */
void LoadEeprom( void )
{
   char* pSetData = ( char*) &g_WallPad;
  int i;
  for( i=0; i < sizeof( WallPad ); i++ ){
    *pSetData++ = EEPROM.read( i );
  }
  DebugSerial.println((String)pSetData);
}

]/**
 * @brief EEPROM 체크 함수
 * @details global 구조체 g_WallPad sInitChk 값과 global 배열[char] g_sInitTitle 값을 비교한다
 * @see 이 함수의 코드분석 내용
 * 1. EEPROM 읽기 함수를 호출한다@n
 * 2. global 구조체 g_WallPad sInitChk 값과 global 배열[char] g_sInitTitle 값을 비교한다@n
 * 3. 두 값이 일치하지 않는다면, EEPROM 초기화 함수를 호출 후 EEPROM 쓰기 함수를 호출한다
 * @param -
 * @return -
 */
void CheckEeprom( void )
{
  DebugSerial.println("loading");

  LoadEeprom();

  DebugSerial.println(g_WallPad.sInitChk);
  if( strcmp( g_sInitTitle, g_WallPad.sInitChk ) )
  {
    DebugSerial.println("start init");
    InitEeprom();
    StoreEeprom();
  }
}

///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////// APmode 소켓서버 설정

IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
WiFiClient tcpclient;
WiFiServer server(30300);

/**
 * @brief wifi 신호 탐색 함수
 * @details wifi로 접속할 수 있는 네트워크 장비를 탐색하고 식별ID와 신호감도 값을 저장한 후 신호감도에 따라 내림차순으로 정리하여 탐색을 마친다
 * @see 이 함수의 코드분석 내용
 * 1. 파라미터 ap 값이 false인 경우, wifi모드를 station으로 바꾸고 연결을 끊는다@n
 * 2. 네트워크 장비를 탐색하고 탐색된 그 수를 global 변수(int) g_nMaxAp에 반환한다@n
 * 3. g_nMaxAp만큼 반복하여 global 구조체배열 g_ApList의 sSsid, nRssi, bEncry값을 채운다@n
 * 4. 신호 감도에 따라 내림차순으로 정리한다@n
 * 5. 식별ID를 버퍼에 저장하고 global 구조체 g_WallPad의 sSsId값과 비교한다@n
 * 6. (g_nMaxAp만큼 반복동안) 한번이라도 두 값이 동일하다면,@n
 *    global 배열[char] g_sInitSsid, g_sInitSspass에 global 구조체 g_WallPad의 sSsId과 sSsPass값을 복사하고 탐색을 마친다@n
 * 7. g_nMaxAp만큼 반복횟수동안 두 값이 불일치하다면 탐색을 마친다 
 * @param bool ap 
 * @return int g_nMaxAp : wifi로 접속할 수 있는 네트워크 장비 수
 */
int FindAp( bool ap ){
  
  DebugSerial.println("Find AP Scan Start"+(String)ap);

  if(!ap){
    WiFi.mode( WIFI_STA );
    WiFi.disconnect();
    delay( 10 );
  }

  g_nMaxAp = WiFi.scanNetworks();
  
  int i, j;
  
  for( i=0; i < g_nMaxAp; i++ )
  {
    g_ApList[ i ].sSsid   = WiFi.SSID( i );
    g_ApList[ i ].nRssi   = WiFi.RSSI( i );
    g_ApList[ i ].bEncry  = (WiFi.encryptionType( i ) == ENC_TYPE_NONE) ? false : true;

    g_pApList[ i ] = &g_ApList[ i ];
    delay( 10 );
  }
  DebugSerial.println("p1");
  //Rssi sorting HIGH->LOW
  pApList pTmp;
  for( i=0; i < g_nMaxAp - 1; i++ )
    for( j = i+1; j < g_nMaxAp; j++ )
      if( g_pApList[ i ]->nRssi < g_pApList[ j ]->nRssi )
      {
        pTmp = g_pApList[ i ];
        g_pApList[ i ] = g_pApList[ j ];
        g_pApList[ j ] = pTmp;
      }

  DebugSerial.println("p2");
    for( i=0; i < g_nMaxAp ; i++ ){
      char buf[30];
      g_pApList[i]->sSsid.toCharArray(buf, g_pApList[i]->sSsid.length()+1);
      
      if(ap)
        DebugSerial.println(buf);

      if(!strcmp(g_WallPad.sSsId,buf)){
        strcpy(g_sInitSsid,g_WallPad.sSsId);
        strcpy(g_sInitSspass,g_WallPad.sSsPass);
        WiFi.scanDelete();
        
        return 1;
      }

    }
    DebugSerial.println("p3");
 
  WiFi.scanDelete();
  return g_nMaxAp;
}

/**
 * @brief wifi 서비스 함수
 * @details wifi 서버로 역할을 하고 접속된 클라이언트의 정보에 따라 클라이언트에게 필요한 메세지를 보내준다
 * @see 이 함수의 코드분석 내용
 * 1. wifi가 접속되어 있는지 판단하고 연결되어있으면 연결을 끊는다@n
 * 2. AP mode에 대한 ip가 설정이 안되었다면, mode를 AP_STA로 바꾸고, IP, gateway, subnet를 설정 후@n
 *    장비 이름 설정하고 wifi서버를 실행한다@n
 * 3. 클라이언트가 서버와 연결 할 수 없다면, 서버에서 이용할 수 있도록 허용해준다@n
 * 4. 서버와 클라이언트가 연결 되었다면, 클라이언트의 데이터의 양을 확인한다@n
 * 5. 클라이언트 정보 18byte를 버퍼에 저장한다@n
 * 6. 버퍼에 내용 중에 "SPHIOT0010"이 있는지 확인한다@n
 * 7. 관련 문자열이 있다면, 버퍼의 내용 중 일부를 이용하여 경우따라 방침을 만든다@n
 * 8. 101 : findAp 함수를 호출한다
 * 9. 101 :  global 변수(int) g_nMaxAp 수가 10이상 경우, 10으로 고정하여 10*32의 값을 변수(int) bodysize에 대입한다@n
 * 10. 101 : 버퍼에 "SPHIOT0010"문자열과 201, 정해진 사이즈를 입력한다@n
 * 11. 101 : 클라이언트에게 버퍼를 내용을 전달한다@n
 * 12. 101 : (global 변수(int) g_nMaxAp 만큼 반복 <단, 10이상 도달 시 이 조건문을 나오게된다>)@n
 *                global 구조체배열 g_ApList sSsid를 ap버퍼에 저장하고, 클라이언트에게 전달한다@n
 * 13. 101 : 32 - sSsid길이 뺀 숫자만큼 반복하여 클라이언트에게 ' '을 보낸다@n
 * 14. 102 : 클라이언트 정보 128byte를 힙 버퍼에 저장한다@n
 * 15. 102 : 힙 버퍼를 " "문자열로 첫 분류에서 나온 문자열을 global 배열[char] g_sInitSsid에 대입한다@n
 * 16. 102 : 힙 버퍼를 " "문자열로 그다음 분류에서 나온 문자열을 global 배열[char] g_sInitSspass에 대입한다@n
 * 17. 102 : global 구조체 g_WallPad의 eMode를 eMODE_CONNECT으로 변환한다@n
 * 18. 104 : bodysize를 4로 대입하고, 버퍼에 "SPHIOT0010"문자열과 204, bodysize의 값, 0를 입력한다@n
 * 19. 104 : 클라이언트에게 버퍼를 내용을 전달한다@n
 * 20. 104 : 클라이언트와 서버 연결을 끊는다
 * @param -
 * @return -
 */
void Apmode(){
  unsigned long conn_prev_time;
  if(WiFi.isConnected())
    WiFi.disconnect();

  if(!WiFi.softAPIP()){
    
    WiFi.mode(WIFI_AP_STA);
    DebugSerial.print("Setting soft-AP configuration ... ");
    DebugSerial.println(WiFi.softAPConfig(local_IP,gateway, subnet) ? "Ready" : "Failed!");

    WiFi.softAP(g_DeviceName);

    DebugSerial.print("IP address : \t");
    DebugSerial.println(WiFi.softAPIP());

    server.begin();

  }


  if (!tcpclient.connected()) {
    // try to connect to a new client
    tcpclient = server.available();
  }

  else{

    if( tcpclient.available() > 0){
      char buf[18];
      tcpclient.readBytes(buf,18);

      if(strstr(buf, "SPHIOT0010")){
        unsigned int *pCode = (unsigned int*)&buf[10];
        unsigned int *pBodySize = (unsigned int*)&buf[14];
        DebugSerial.print(*pCode);
        DebugSerial.println(*pBodySize);

        char *bufbody = new char[*pBodySize];
        char* ptr;

        unsigned int bodysize = 0;
        unsigned int index = 0;

        switch(*pCode){
          case 101:
            FindAp(1);
            DebugSerial.println("body : "+ (String)tcpclient.readString().toInt() +" send available aplist");

            bodysize = (g_nMaxAp > 10) ? 10*32 : g_nMaxAp*32;

            index = sprintf(buf, "SPHIOT0010");
            index += sprintf(buf+index, "%04d%04d",201,bodysize);

            //send HEADER
            tcpclient.print(buf);
            DebugSerial.print(buf);

            //send BODY
            for(int i = 0; i < g_nMaxAp ; i++){
              if(i == 10) break;
              char ap_buf[32];
              g_pApList[i]->sSsid.toCharArray(ap_buf, g_pApList[i]->sSsid.length()+1);

              tcpclient.print(ap_buf);
              //DebugSerial.print(ap_buf);
              for(int j = 0; j < 32 - g_pApList[i]->sSsid.length(); j++){
                tcpclient.print(' ');
                //DebugSerial.print(' ');
              }
            }

            DebugSerial.println("sending done");

          break;

          case 102:
            DebugSerial.println(" save connecting info");
            tcpclient.readBytes(bufbody,128);
            DebugSerial.println(bufbody);
            ptr = strtok(bufbody, " ");

            strcpy(g_sInitSsid,ptr);
            DebugSerial.println(g_sInitSsid);
            ptr = strtok( NULL, " ");
            strcpy(g_sInitSspass,ptr);
            DebugSerial.println(g_sInitSspass);

            g_WallPad.eMode = eMODE_CONNECT;
          break;

          case 104:
            DebugSerial.println("closing socket");
            bodysize = 4;

            index = sprintf(buf, "SPHIOT0010");
            index += sprintf(buf+index, "%04d%04d%04d",204,bodysize,0);

            //send HEADER
            tcpclient.print(buf);
            DebugSerial.print(buf);

            tcpclient.stop();

          break;

          default:

          break;

        }

        delete bufbody;
      }

      else{
        DebugSerial.println("received from app coruppted");
        DebugSerial.println(tcpclient.readString());
        DebugSerial.println("flush remainder");

      }
    }
  }

}

/**
 * @brief wifi & aws 접속 함수
 * @details 신호가 가장 강력한 wifi를 잡아서 연결하고 AWS의 양식의 채운 후 aws와도 연결한다
 * @see 이 함수의 코드분석 내용
 * 1. wifi의 mode를 station으로 바꾼다@n
 * 2. 프로그램이 돌리기 시작한 후 지난 밀리 초 숫자 값을 변수(long) conn_prev_time에 대입한다@n
 * 3. AP에 접속하기 위해 global 배열[char] g_sInitSsid과 global 배열[char] g_sInitSspass를 사용한다@n
 * 4. wifi 상태가 WL_CONNECTED과 같지 않을때까지 계속 반복하고, 프로그램이 돌리기 시작한 후 지난 밀리 초 숫자 값이 conn_prev_time보다 20000보다 클 경우@n
 *    wifi 연결진행을 중단하고 global 변수(bool) g_bConnecting를 false로 변환한다@n
 * 5. 위의 반복 조건으로 인해 global 변수(bool) g_bConnecting를 true로 변환한다@n
 * 6. wifi 상태가 WL_CONNECTED이라면 global 구조체 g_WallPad의 sSsId, sSsPass값을 g_sInitSsid과 g_sInitSspass값으로 복사한다@n
 * 7. EEPROM 쓰기 함수를 호출한다@n
 * 8. 클라이언트에게 "SPHIOT0010020200040000" 문자열을 보낸다@n
 * 9. aws관한 정보를 채운다 (Region, domain, KeyID, Secretkey, ssl 사용유/무)@n
 * 10. websocket & MQTT 접속 함수를 호출하고 클라이언트 ID로 접속되었다면 MQTT subscribe 함수를 호출한다@n
 * 11. g_WallPad의 eMode값을 eMODE_NORMAL로 변환한다@n
 * 12. 클라이언트와 서버 연결을 끊는다@n
 * 13. wifi의 mode를 station으로 바꾼다@n
 * 14. wifi 상태가 WL_CONNECTED아니라면, 클라이언트에게 "SPHIOT0010020200040001" 문자열을 보낸다@n
 * 15. g_WallPad의 eMode값을 eMODE_AP로 변환한다@n
 * 16. 클라이언트와 서버 연결을 끊는다@n
 * 17. wifi의 mode를 WIFI_OFF으로 바꾼다@n 
 * @param -
 * @return -
 */
void Connect(){
  WiFi.mode(WIFI_STA);
  unsigned long conn_prev_time = millis();
  ESP8266WiFiMulti WiFiMulti;
  WiFiMulti.addAP(g_sInitSsid, g_sInitSspass);
  DebugSerial.println ("connecting to wifi");

  DebugSerial.println(g_sInitSsid);
  DebugSerial.println(g_sInitSspass);

  
  while(WiFiMulti.run() != WL_CONNECTED ) {
    delay(100);
    DebugSerial.print (".");
    if(millis() - conn_prev_time > 20000){
      DebugSerial.println ("\nconnecting time out");
      WiFi.disconnect();
      break;
    }
    g_bConnecting = true;
  }
  g_bConnecting = false;

  if(WiFiMulti.run() == WL_CONNECTED){
    strcpy( g_WallPad.sSsId     , g_sInitSsid   );
    strcpy( g_WallPad.sSsPass   , g_sInitSspass   );
    StoreEeprom();

    tcpclient.print("SPHIOT0010020200040000");
    DebugSerial.print("SPHIOT0010020200040000");
    DebugSerial.println ("\nconnected");

    //fill AWS parameters    
    awsWSclient.setAWSRegion(aws_region);
    awsWSclient.setAWSDomain(aws_endpoint);
    awsWSclient.setAWSKeyID(aws_key);
    awsWSclient.setAWSSecretKey(aws_secret);
    awsWSclient.setUseSSL(true);

    if (connect ()){
      subscribe ();
     }

     g_WallPad.eMode = eMODE_NORMAL;
    tcpclient.stop();
    WiFi.mode(WIFI_STA);
    return ;
  }

  else{
    tcpclient.print("SPHIOT0010020200040001");
    DebugSerial.print("SPHIOT0010020200040001");
    DebugSerial.println ("\nconnecting failed return to Ap mode");
    g_WallPad.eMode = eMODE_AP;
  }

  tcpclient.stop();
  WiFi.mode(WIFI_OFF);

}

///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/**
 * @brief task 설정 함수
 * @details UART 통신을 위한 Baud rate와 EEPROM 크기, system task관한 설정한다
 * @see 이 함수의 코드분석 내용
 * 1. wifi sleep 모드는 사용 안한다@n
 * 2. UART 통신을 위한 Baud rate는 9600으로 한다@n
 * 3. EEPROM 크기를 구조체 WallPad 만큼한다@n
 * 4. EEPROM 체크 함수 호출한다@n
 * 5. system_os_task API함수 : set up tasks@n
 *    bool system_os_task(os_task_t task, uint8 prio, os_event_t *queue, uint8 qlen)@n
 *    os_task_t task : task 함수@n
 *    uint8 prio : task 우선순위@n
 *    os_event_t *queue : 메세지 큐 포인터@n
 *    uint8 qlen : 메시지 큐 길이@n
 * 6. os_timer_setfn API함수 : set timer callback 함수@n
 *    oid os_timer_setfn (os_timer_t *ptimer, os_timer_func_t *pfunction, void *parg)@n
 *    os_timer_t *ptimer : 타이머 구조체@n
 *    os_timer_func_t *pfunction : timer callback 함수@n
 *    void *parg : callback 함수 파라미터@n
 * 7. Temp_Run_UART_RX_Task(0) 함수를 호출한다@n
 * 8. g_WallPad의 eMode값을 eMODE_NONE로 변환한다@n
 * @param -
 * @return -
 */
void setup() {
  wifi_set_sleep_type(NONE_SLEEP_T);

  
  WallpadSerial.begin(9600);
  DebugSerial.begin(115200);
  
  DebugSerial.setDebugOutput(true);
  DebugSerial.setTimeout(10);

  EEPROM.begin(sizeof(WallPad));
  CheckEeprom();

  //thread olusturma
  system_os_task(UART_RX_TASK_Handler,
    UART_RX_TASK_PRIORITY, UART_RX_QUEUE,
    UART_RX_QUEUE_SIZE);

  //timer olusturma
  os_timer_setfn(&UART_RX_TASK_Timer, 
      (os_timer_func_t*)&Temp_Run_UART_RX_Task, 0);

  //taski baslatma
  Temp_Run_UART_RX_Task(0);
    
    g_WallPad.eMode = eMODE_NONE;

}

// the loop function runs over and over again until power down or reset

/**
 * @brief eMode 함수
 * @details g_WallPad의 eMode값에 따라 분기점을 나누어서 원하는 조건에 실행하도록 한다
 * @see 이 함수의 코드분석 내용
 * 1. g_WallPad의 eMode값에 따라 스위치문으로 나눈다@n
 * 2. eMODE_NONE : g_WallPad의 eMode값을 eMODE_SEARCH_AP로 변환한다@n
 * 3. eMODE_SEARCH_AP : wifi 신호 탐색 함수를 호출하고 반환값이 1이면 eMODE_CONNECT값을 아니면 eMODE_AP값으로 변환한다@n
 * 4. eMODE_AP : wifi 서비스 함수를 호출한다@n
 * 5. eMODE_CONNECT : websocket & MQTT 접속 함수를 호출한다@n 
 * 6. eMODE_NORMAL : wifi가 연결되어있다면, 또 aws에도 연결이 되어있는지, 둘 다 되어있으면 eMode 함수를 호출한다@n
 * 7. eMODE_NORMAL : aws가 연결이 안되어있다면, websocket & MQTT 접속 함수를 호출하고 반환값이 true이면 MQTT subscribe 함수를 호출한다@n
 * 8. eMODE_NORMAL : wifi가 연결이 안되어 있다면, eMODE_CONNECT값으로 변환한다
 * @param -
 * @return -
 */
void loop() {

   switch(g_WallPad.eMode){
    
    case eMODE_NONE:
      g_WallPad.eMode = eMODE_SEARCH_AP;
      
    break;

    case eMODE_SEARCH_AP:
      if(FindAp(0) == 1){
        g_WallPad.eMode = eMODE_CONNECT;  
      }
      else{
        g_WallPad.eMode = eMODE_AP;
      }
    break;

    case eMODE_AP:
      Apmode();
      
    break;

    case eMODE_CONNECT:
      Connect();
    break;

    case eMODE_NORMAL:
      if(WiFi.isConnected()){
        if (awsWSclient.connected ()) {    
            client.loop ();
        } else {
          //handle reconnection
          if (connect ()){
            subscribe ();      
          }
        }
      }

      else
        g_WallPad.eMode = eMODE_CONNECT;
        
    break;
   }
 
  //keep the mqtt up and running

}
