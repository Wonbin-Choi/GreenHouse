/* 스마트 화분 관리기 */

#include <SoftwareSerial.h> // 블루투스 교신을 위한 라이브러리 가져오기
SoftwareSerial BTSerial(9, 8); //블루투스 교신 TX, RX
#include <Wire.h>
#include <LiquidCrystal_I2C.h>      // LCD 모듈 라이브러리
#include <DHT.h>                    // 온습도 센서 모듈 라이브러리
#include <DFRobot_DHT11.h>          // DHT11 센서 모듈
#include <Emotion_Farm.h>           // 특수 문자 및 이모티콘 라이브러리
#include <string.h>
#include <stdlib.h> 
// 센서 핀 설정
#define cdsPin A0                  // 조도센서 모듈 핀
#define DHTPIN 3                    // 온습도센서 모듈 핀
#define DHTTYPE DHT11               // 온습도 센서타입 설정
#define CMD_SIZE 10

// 객체 생성
LiquidCrystal_I2C lcd(0x27, 16, 2); //LCD 초기화 (LCD 주소값, x축, y축)
DHT dht(DHTPIN, DHTTYPE);           //온습도 센서 모듈

// 문자열을 출력하기 위한 변수
char sendBuf[CMD_SIZE];
char recvId[10] = "OHC_SQL";  
char str_1[10];
char str_2[10];
unsigned long setime = 10000;  // 프린트 타임 
unsigned long before =0;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  Serial.println("setup() start!");
  pinMode(cdsPin, INPUT);
  
  //선풍기 디폴트
  pinMode(6,OUTPUT);
  digitalWrite(6,LOW);



  //LCD에 인트로 출력
  lcd.begin();
  lcd.clear();
  lcd.noBacklight();
  delay(500);
  lcd.backlight();
  delay(500);
  lcd.setCursor(2,0);
  lcd.print("SMART");
  delay(1000);
  lcd.setCursor(8,0);
  lcd.print("GARDEN");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("GMO in a GARDEN");
  delay(1000);
  lcd.clear();


  // 라이브러리로 추가한 특수 문자 및 이모티콘 추가
  lcd.createChar(0, temp);
  lcd.createChar(1, C);
  lcd.createChar(2, humi);  
  lcd.createChar(3, Qmark);
  lcd.createChar(4, water);
  lcd.createChar(5, good);
  lcd.createChar(6, wind);
}

void loop() {

  //센서값 측정(num1,num2,num3)
  int num1 = dht.readTemperature(); //온도
  int num2 = dht.readHumidity();    //습도
  int num3 = analogRead(cdsPin);    //조도 측정: 0(밝음) ~ 1023(어두움)

  unsigned char h_Value = dht.readHumidity();                        // 공기 중 습도 값 측정
  unsigned char t_Value = dht.readTemperature();                     // 공기 중 온도 값 측정
  unsigned long now = millis(); // millis 시간을 적는다.

  // 값을 환산
  int num3b = map(num3, 0, 1023, 0, 100);

  // 변환된 조도 값 출력(num3)
  lcd.setCursor(1,0);
  lcd.print("LIGHT:");
  lcd.print(num3b);
  delay(500);
  lcd.setCursor(10,0);
  lcd.print("%");

  //LCD에 온도값(num1) 출력
  lcd.setCursor(1,1);
  lcd.write(0);
  sprintf(str_1, "%02d", t_Value);
  lcd.setCursor(3,1);
  lcd.print(str_1);
  lcd.write(1);

  //LCD에 습도값(num2) 출력
  lcd.setCursor(7,1);
  lcd.write(2);
  sprintf(str_2, "%02d", h_Value);
  lcd.setCursor(9,1);
  lcd.print(str_2);
  lcd.print("%");

  // 선풍기 동작 세팅
  if ( num3b < 60){
    pinMode(6,OUTPUT);
    digitalWrite(6,HIGH);
  }



   // 블루투스 버퍼에 저장한 다음에 서버로 보내기
  if ( now - before >= setime) {
    sprintf(sendBuf,"[%s]GMO@%d@%d@%d\n","KSH_BT",num1,num2,num3b);
    BTSerial.write(sendBuf,strlen(sendBuf)); //명령처리
    delay(1000);

    sprintf(sendBuf,"[%s]GMO@%d@%d@%d\n","OHC_SQL",num1,num2,num3b);
    BTSerial.write(sendBuf,strlen(sendBuf)); //DB전달
    before = now ;
    
  }
  Serial.print(sendBuf);
  Serial.println("setup() start!");
  
}
