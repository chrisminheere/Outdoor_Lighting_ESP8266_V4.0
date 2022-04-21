//                                 Libraries
//==================================================================================
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_PWMServoDriver.h>
#include <BH1750.h>
#include <SensorUtil.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <LiquidCrystal_I2C.h>
#include <dht11.h>
#include <SimpleTimer.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

#include <Arduino.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>





//                                 Definitions
//==================================================================================
#define lightSensorAdress 0x23     // I2C addres of Light sensor.                            Default =  0x23.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // I2C addres of PWM breakout.      Default =  0x40.
LiquidCrystal_I2C lcd(0x27, 20, 4); // Set the LCD I2C address   Default =  0x27.

WiFiUDP ntpUDP;
const long utcOffsetInSeconds = 7200;   //Winter (3600)  Zomer(7200).
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", utcOffsetInSeconds);
BH1750 lightMeter;
#define DHT11PIN D4
dht11 DHT11;
SimpleTimer timer;
WidgetLED led1(V9);
WidgetLED led2(V12);
RF24 radio(D0, D8); // CE, CSN
const byte address[6] = "45623";
int pirin = D3;   //input for Pir motion sensor
AsyncWebServer server(80);


//                                  Variables
//==================================================================================
int val = 0;
int pinValue = 0;
int pinData = 0;
int i = 0;
int lux = 0;
int autoMode = 1;
int currentHour;
int currentMinutes;
int currentDay;
String formattedDate;
String timeStamp;
String dayStamp;
int lichtsOnOff = 0;
int k = 0;
int tempWarning = 0;
int pir = 0;
bool inputState = false;
int t = 0;




//                                   Settings
//==================================================================================
// Blynk settings
char auth[] = "************************************";   //Auth key.
char ssid[] = "***************";                        //WIFI SSID it connects with.
char pass[] = "***************";                        //WIFI password.

//Min Lux, is when the light go ON.
int minLux = 1;
//Max Lux, is when the light go OFF.
int maxLux = 8;


//=============Vrijdag T/M Zaterdag==============
//Defining start day time:
int startHour = 1;  //Lights go out at 1:59.
//Defining end day time:
int endHour = 5; //Lights turn on at 5:59.
//===============================================


//=============Zondag T/M Donderdag=============
//Defining start day time:
int startHour1 = 0;  //Lights go out at 22:59.
//Defining end day time:
int endHour1 = 5; //Lights turn on at 5:59.
//===============================================



//                          Icons for Display (Bitmap IMG)
//==================================================================================

byte temp[8] = {
  B00100,
  B01010,
  B01010,
  B01110,
  B01110,
  B11111,
  B11111,
  B01110
};

byte vocht[8] = {
  B00100,
  B00100,
  B01010,
  B01010,
  B10001,
  B10001,
  B10001,
  B01110
};



//                                    Setup
//==================================================================================
void setup() {
  //  Serial.begin(115200); // For debugging.
  Blynk.begin(auth, ssid, pass); //ESP8266 connects to WiFi.

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "Go to > IP/update");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(0x66);
  radio.stopListening();
  printf_begin();
  radio.printDetails();


  timer.setInterval(1000L, secondTimer); //Time length of the 2nd timer.

  //The I2c bus starts on the indicated pins.
  Wire.begin(4, 5);   //Define the i2c pins on the ESP8266    SDA, GPIO4 = D2___SCL, GPIO5 = D1.

  lightMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE);  //Begins the sensor en sets it to the LowRes Mode.
  pwm.begin();
  pwm.setPWMFreq(1600); // This is the maximum PWM frequency.
  timeClient.begin(); //Start communication with the time server to retrieve the time.
  pinMode(pirin, INPUT);
  Blynk.notify("System Ready"); //Sends a notification to your phone that the system is active.

  lcd.begin(); //Start the I2c LCD screen.
  lcd.createChar(1, temp);
  lcd.createChar(2, vocht);
  lcd.clear();


  lcd.setCursor(0, 1);
  lcd.print("IP for OTA update:");
  lcd.setCursor(0, 2);
  lcd.print(WiFi.localIP());
  lcd.setCursor(0, 3);
  lcd.print("/update");


  delay(4000);
  lcd.clear();
  lcd.setCursor(7, 0);
  lcd.print("Made by");
  lcd.setCursor(3, 1);
  lcd.print("Chris");
  lcd.setCursor(5, 3);
  lcd.print("V4.0 - OTA");
  delay(4000);
  lcd.clear();
}




//                              Loop (NO Delay's!!)
//==================================================================================
void loop() {
  t++;
  Serial.print("Loop Count: ");
  Serial.println(t);


  Blynk.run(); //Communication with the app.
  timer.run(); //2nd timer to time stuff.
  pir = digitalRead(pirin);
  if (pir == 1) {
    led2.on();
  }
  else {
    led2.off();
  }
  //Serial.print("Status PIR: ");
  //Serial.println(pir);

  //Writes temperature and LUX data to the app.
  Blynk.virtualWrite(V0, DHT11.temperature);
  Blynk.virtualWrite(V3, lux);



  //____________Auto Mode____________
  if (autoMode == 1 ) {
    //Serial.println("AUTO AAN");
    //    digitalWrite(ledPin2, HIGH); //Turns ON the LED to see that Auto Mode is ON.
    Blynk.virtualWrite(V5, 1); //turns on the button in the app.


    //=============================Weekend=============================
    if (currentDay == 6 || currentDay == 0) {
      if (lux <= minLux ) {
        if (currentHour >= startHour && currentHour <= endHour ) {
          if (pir == 1) {
            turnAllOn();
          }
          else if (pir == 0) {
            turnAllOff();
            //Serial.println("Alles UIT in LOOP");
            pwm.setPin(10, 4096 );//setPWM(channel, on, off).
          }
        }
        else {
          turnAllOn();
          //Serial.println("Alles AAN in LOOP");
        }
      }
      else if (lux > maxLux) {
        turnAllOff();
        //Serial.println("Alles UIT in LOOP");
      }
    }

    //=========================During the week===========================
    else {
      if (lux <= minLux ) {
        if (currentHour >= startHour1 && currentHour <= endHour1 ) {
          if (pir == 1) {
            turnAllOn();
          }
          else if (pir == 0) {
            turnAllOff();
            //Serial.println("Alles UIT in LOOP");
            pwm.setPin(10, 4096 );//setPWM(channel, on, off).
          }
        }
        else {
          turnAllOn();
          //Serial.println("Alles AAN in LOOP");
        }
      }
      else if (lux > maxLux) {
        turnAllOff();
        //Serial.println("Alles UIT in LOOP");
      }
    }
  }
  //====================================================================

  else {
    //Serial.println("AUTO UIT");
    //    digitalWrite(ledPin2, LOW);//Turns OFF the LED to see that Auto Mode is OFF.
    Blynk.virtualWrite(V5, 0); //turns on the button in the app.
  }
  yield();

}






//                        Second Timer(one second interval)
//==================================================================================
void secondTimer() {
  getTime();                           //Get Time.
  tempWarn();
  automodeCheck();
  readSensor();
  radio.write(&inputState, sizeof(inputState));
  DrawLcd();                           //Draws the LCD.
}


//                          Get Data From Sensors
//==================================================================================
void readSensor() {
  int chk = DHT11.read(DHT11PIN);      //Get the Temperature from DHT11.
  lux = lightMeter.readLightLevel();   //Get the Lux from BH1750.
}



//                Check if auto mode is off in the morning.
//==================================================================================
void automodeCheck() {
  if (autoMode == 0 ) {
    if (currentHour == endHour) {
      autoMode = 1;
    }
  }
}


//           If the temperature becomes too high, a message is sent.
//==================================================================================
void tempWarn() {
  if  (tempWarning == 0) {
    if (DHT11.temperature > 45) {
      Blynk.notify("WAARSCHUWING!   Tempratuur boven 45°C, Kontroleer systeem.");
      tempWarning = 1;
    }
  }
  if (DHT11.temperature <= 42) {
    tempWarning = 0;
  }
}



//                                 Draw the LCD.
//==================================================================================
void DrawLcd() {


  //writes the time to the screen.
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  Serial.println(timeStamp);
  lcd.setCursor(0, 0);
  lcd.print(timeStamp);



  //LCD Draw Day of the week.
  lcd.setCursor(0, 1);
  if (currentDay == 0) {
    lcd.print("Zondag   ");
  }
  else if (currentDay == 1) {
    lcd.print("Maandag  ");
  }
  else if (currentDay == 2) {
    lcd.print("Dinsdag  ");
  }
  else if (currentDay == 3) {
    lcd.print("Woensdag ");
  }
  else if (currentDay == 4) {
    lcd.print("Donderdag  ");
  }
  else if (currentDay == 5) {
    lcd.print("Vrijdag  ");
  }
  else if (currentDay == 6) {
    lcd.print("Zaterdag ");
  }


  lcd.setCursor(10, 1);
  lcd.print("LUX");
  lcd.setCursor(13, 1);
  lcd.print("       ");
  lcd.setCursor(13, 1);
  lcd.print(lux);

  if (lichtsOnOff == 1) {
    lcd.setCursor(0, 3);
    lcd.print("Lights:  ON");
  }
  else {
    lcd.setCursor(0, 3);
    lcd.print("Lights: OFF");
  }

  if (autoMode == 1 ) {
    lcd.setCursor(0, 2);
    lcd.print("Auto:    ON");
  }
  else {
    lcd.setCursor(0, 2);
    lcd.print("Auto:   OFF");
  }
  lcd.setCursor(11, 0);
  lcd.write(byte(1));
  lcd.setCursor(12, 0);
  lcd.print((float)DHT11.temperature, 0);
  lcd.print((char)223);
  lcd.setCursor(16, 0);
  lcd.write(byte(2));
  lcd.print((float)DHT11.humidity, 0);
  lcd.print("%");
}




//retrieved time from time server.
//-------------------------------------------------------------
void getTime() {
  formattedDate = timeClient.getFormattedDate();
  timeClient.update();
  currentHour = timeClient.getHours();
  currentMinutes = timeClient.getMinutes();
  currentDay = timeClient.getDay();

}
//-------------------------------------------------------------




//turn everything on.
//-------------------------------------------------------------
void turnAllOn() {
  for (int i = 0; i <= 7; i++) {
    pwm.setPin(i, 4096 );//setPWM(channel, on, off).
  }
  for (int j = 8; j <= 11; j++) {
    pwm.setPin(j, 0 );//setPWM(channel, on, off).
  }
  lichtsOnOff = 1;
  Blynk.virtualWrite(V1, 1);
  Blynk.virtualWrite(V13, 1);
  Blynk.virtualWrite(V2, 4096);
  Blynk.virtualWrite(V4, 0);
  Blynk.virtualWrite(V6, 0);
  Blynk.virtualWrite(V7, 4096);
  Blynk.virtualWrite(V8, 4096);
  Blynk.virtualWrite(V10, 4096);
  Blynk.virtualWrite(V11, 4096);
  led1.on();
  //  digitalWrite(nRF24, HIGH);
  //Serial.println("Alles aan");
  inputState = true;
}
//-------------------------------------------------------------




//turn everything off.
//-------------------------------------------------------------
void turnAllOff() {
  for (int i = 0; i <= 7; i++) {
    pwm.setPin(i, 0 );//setPWM(channel, on, off)
  }
  for (int j = 8; j <= 11; j++) {
    pwm.setPin(j, 4096 );//setPWM(channel, on, off)
  }
  lichtsOnOff = 0;
  Blynk.virtualWrite(V1, 0);
  Blynk.virtualWrite(V13, 0);
  Blynk.virtualWrite(V2, 0);
  Blynk.virtualWrite(V4, 4096);
  Blynk.virtualWrite(V6, 4096);
  Blynk.virtualWrite(V7, 0);
  Blynk.virtualWrite(V8, 0);
  Blynk.virtualWrite(V10, 0);
  Blynk.virtualWrite(V11, 0);
  led1.off();
  //  digitalWrite(nRF24, LOW);
  //Serial.println("Alles uit");
  inputState = false;
}
//-------------------------------------------------------------








//blynk app stuff.
//-------------------------------------------------------------
BLYNK_WRITE(V1) {
  int pinValue = param.asInt();

  if (pinValue == 1) {
    lichtsOnOff = 1;
    turnAllOn();

  }
  else {
    lichtsOnOff = 0;
    turnAllOff();
    autoMode = 0;
  }
  yield();
}

BLYNK_WRITE(V2)
{
  int pinValue = param.asInt();
  for (int i = 0; i <= 4; i++) {
    pwm.setPin(i, pinValue );//setPWM(channel, on, off)
  }
  if (pinValue < 4090) {
    autoMode = 0;
  }

  Blynk.virtualWrite(V7, pinValue);
  Blynk.virtualWrite(V8, pinValue);

  yield();
}

BLYNK_WRITE(V4) {
  int pinValue = param.asInt();
  pwm.setPin(10, pinValue );//setPWM(channel, on, off)

  if (pinValue < 1) {
    autoMode = 0;
  }
}

BLYNK_WRITE(V5) {
  autoMode = param.asInt();
}

BLYNK_WRITE(V6) {
  int pinValue = param.asInt();
  pwm.setPin(11, pinValue );//setPWM(channel, on, off)

  if (pinValue < 1) {
    autoMode = 0;
  }
}

BLYNK_WRITE(V10) {
  int pinValue = param.asInt();
  pwm.setPin(5, pinValue );//setPWM(channel, on, off)

  if (pinValue < 4090) {
    autoMode = 0;
  }
}

BLYNK_WRITE(V11) {
  int pinValue = param.asInt();
  pwm.setPin(6, pinValue );//setPWM(channel, on, off)

  if (pinValue < 4090) {
    autoMode = 0;
  }
}

BLYNK_WRITE(V13) {
  int pinValue = param.asInt();
  if (pinValue == 0) {
    inputState = false;

  }

  else {
    inputState = true;

  }
  if (pinValue == 1) {
    autoMode = 0;
  }
}


BLYNK_WRITE(V7)
{
  int pinValue = param.asInt();
  for (int i = 0; i <= 2; i++) {
    pwm.setPin(i, pinValue );//setPWM(channel, on, off)
  }
  if (pinValue < 4090) {
    autoMode = 0;
  }
  yield();
}

BLYNK_WRITE(V8)
{
  int pinValue = param.asInt();
  for (int i = 3; i <= 5; i++) {
    pwm.setPin(i, pinValue );//setPWM(channel, on, off)
  }
  if (pinValue < 4090) {
    autoMode = 0;
  }
  yield();
}
