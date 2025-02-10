#include <ESP8266WiFi.h>
#include <ThingESP.h>
// #include <Servo.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include "PCF8574.h"

ThingESP8266 thing("deatsamara", "Myincubator", "suhu dan kelembapan");
PCF8574 io(0x38);
LiquidCrystal_I2C lcd(0x27,16,2);  
// #define SERVO_PIN D3
#define WATER_SENSOR_POWER_PIN D8
#define WATER_SENSOR_SIGNAL_PIN A0
#define DHT_PIN D4
#define FAN_PIN_ENA D5
#define FAN_PIN_1 D6
#define FAN_PIN_2 D7
#define RELAY_PIN 7
#define MODE_BUTTON_PIN 5
#define UP_BUTTON_PIN 4
#define DOWN_BUTTON_PIN 3
#define MOTOR_PIN D3

#define DHTTYPE DHT22
DHT dht = DHT(DHT_PIN, DHTTYPE);
// Servo myservo;
int servoState = 0;

const long tempWaterCheckingPeriod = 1000;
const long rotateServoPeriod = 1000 * 3600 * 6;

unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;

const int minWaterLevel = 0;
const int minHumidity = 60;
int minTemperature = 37; //default 36
int maxTemperature = 38; //default 40
float temperature = 0.0;
float motordc = 0.0;
int humidity = 0;
int pwmMamdani;
int fanFixSpeed = 500;

int buttonModeState = 0;
int buttonUpState = 0;
int buttonDownState = 0;
int state = 1;
int oldState=0;

String fanStrSpeed = "Lambat";

const char* ssid     = "Fauji Feriyaman"; // Your WiFi ssid
const char* password = "Ujianhidup123"; // Your Wifi password;

void setup() {
  Serial.begin(115200);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.print("Attempting to connect to SSID");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    // Connect to WPA/WPA2 network. Change this line if using open or WEP  network:
    // Wait 1 seconds for connection:
    delay(1000);
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); 

  digitalWrite(D0, HIGH);  
  digitalWrite(MOTOR_PIN, LOW);
  
  dht.begin();

  digitalWrite(WATER_SENSOR_POWER_PIN, LOW); // turn the sensor OFF

  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(WATER_SENSOR_POWER_PIN, OUTPUT);
  pinMode(FAN_PIN_1, OUTPUT);
  pinMode(FAN_PIN_2, OUTPUT);
  pinMode(FAN_PIN_ENA, OUTPUT);

  digitalWrite(FAN_PIN_1, HIGH);
  digitalWrite(FAN_PIN_2, LOW);
  
  lcd.init();
  lcd.clear();         
  lcd.backlight();

  io.begin();

  thing.SetWiFi("Fauji Feriyaman", "Ujianhidup123");
  thing.initDevice();  
}

void loop() {

  thing.Handle();
  unsigned long currentMillis = millis(); // store the current time

  if(state != oldState) {
    lcd.clear();
  }

  if(state == 1) {
    lcd.setCursor(0,0);   //Set cursor to character 2 on line 0
    lcd.print("Humidity");   

    lcd.setCursor(0,1);   //Move cursor to character 2 on line 1
    lcd.print("Temperature");   
    lcd.setCursor(12,0); 
    lcd.print(humidity );
    lcd.setCursor(12,1); 
    lcd.print(temperature);
  } else if(state == 2) {
    lcd.setCursor(0,0);
    lcd.print("Min Temp");
    lcd.setCursor(12,0);   
    lcd.print(minTemperature);   
  } else {
    lcd.setCursor(0,0);
    lcd.print("Max Temp");  
    lcd.setCursor(12,0);   
    lcd.print(maxTemperature);       
  }
  oldState = state;

  if(io.readButton(MODE_BUTTON_PIN) == LOW && buttonModeState == 0) {
    if(state >= 3){
      state=1;
    } else {
      state += 1;      
    }
    buttonModeState=1;    
  } else if(io.readButton(MODE_BUTTON_PIN) == HIGH) {
    buttonModeState=0;
  }

 if(io.readButton(UP_BUTTON_PIN) == LOW && buttonUpState == 0) {
   if(state == 3){
     maxTemperature += 1;
   } else if(state == 2) {
     minTemperature += 1;      
   }
   buttonUpState=1;    
 } else if(io.readButton(UP_BUTTON_PIN) == HIGH) {
   buttonUpState=0;
 }

 if(io.readButton(DOWN_BUTTON_PIN) == LOW && buttonDownState == 0) {
   if(state == 3){
     maxTemperature -= 1;
   } else if(state == 2) {
     minTemperature -= 1;      
   }
   buttonDownState=1;    
 } else if(io.readButton(DOWN_BUTTON_PIN) == HIGH) {
   buttonDownState=0;
 }

  if (currentMillis - previousMillis1 >= tempWaterCheckingPeriod) {  
    previousMillis1 = currentMillis;
    
    int waterValue = waterCheck()  ;
    Serial.print("Water Sensor Value: ");
    Serial.println(waterValue);
        
    int* humAndTemp=temperatureCheck();
    humidity = humAndTemp[0];
    temperature = humAndTemp[1];
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println("% ");
    
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("\xC2\xB0");
    Serial.println("C");

    mamdaniDisplay();
    analogWrite(FAN_PIN_ENA, fanFixSpeed);
    
    if(temperature > maxTemperature) {
      io.write(RELAY_PIN, LOW);
    } else if(temperature <= minTemperature) {
      io.write(RELAY_PIN, HIGH);   
    }
  }
  
  if (currentMillis - previousMillis2 >= rotateServoPeriod || servoState == 1) {  
    previousMillis2 = currentMillis;
    digitalWrite(MOTOR_PIN, HIGH);
    delay(8000);
    digitalWrite(MOTOR_PIN, LOW);
    // rotateServo();
    servoState = 0;
  }
}

int waterCheck() {
  digitalWrite(WATER_SENSOR_POWER_PIN, HIGH);  
  delay(10);                      
  int value = analogRead(WATER_SENSOR_SIGNAL_PIN); 
  digitalWrite(WATER_SENSOR_POWER_PIN, LOW); 
  
  return value;
}

int* temperatureCheck(){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  // Read the humidity in %:
  float h = dht.readHumidity();
  // Read the temperature as Celsius:
  float t = dht.readTemperature();
  // Read the temperature as Fahrenheit:
  float f = dht.readTemperature(true);


  // Compute heat index in Fahrenheit (default):
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius:
  float hic = dht.computeHeatIndex(t, h, false);
  
  int* value = new int[2];
  
  value[0]= h;
  value[1]= t;
  
  return value;
}

String HandleResponse(String query)
{
  //  dht22 
  awal:
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    goto awal;
  }

  else if (query == "status") {
    int w = waterCheck();
    int height = w * 100 / 570;
    String lamps;
 
    if (io.read(RELAY_PIN) == HIGH) {
      lamps = "The light is on";
    } else {
      lamps = "The light is off";
    }

    return "Temperature: " + String(t,2) + "°C \n"
    "Humidity: " + String(h,2) + "% \n"
    "Water level: " + String(height) + " \n"
    "Fan Speed: " + String(deffuzzyfikasi()) + " (" + fanStrSpeed + "PWM) \n"
    "Lamps: " + lamps;
  }

  else if (query == "servo") {
    servoState = 1;
    return "The servo is being rotated.";
  }

  else if (query && temperature > maxTemperature) {
    return "The temperature has exceeded the max limit";
  }

  else return "Query not available..";
}

#include<math.h>
float A, B;
int sel_;

float a1, b1a, b1b, c1;

float L1, L2, L3, L4, L5, L6, L7;
float M1, M2, M3, M4, M5, M6, M7;

float FiN() {
  if (temperature <= 35) {
    return 1;
  }
  else if (temperature >= 35 && temperature <= 37) {
    return (37 - temperature) / (2);
  }
  else if (temperature >= 37) {
    return 0;
  }
}

float FiZ() {
  if (temperature <= 35) {
    return 0;
  }
  else if (temperature >= 35 && temperature <= 37) {
    return (temperature - 35) / (2);
  }
  else if (temperature >= 40 && temperature <= 42) {
    return (42 - temperature) / (2);
  }
  else if (temperature >= 37 && temperature <= 40) {
    return 1;
  }
}

float FiP() {
  if (temperature <= 40 ) {
    return 0;
  }
  else if (temperature >= 40 && temperature <= 42) {
    return (temperature - 40) / (2);
  }
  else if (temperature >= 42) {
    return 1;
  }
}

float FoN() {
  if (motordc <= 16) {
    return 1;
  }
  else if (motordc >= 16 && motordc <= 33) {
    return (33 - motordc) / (17);
  }
  else if (motordc >= 37) {
    return 0;
  }
}

// parameter motor dc
float FoZ() {
  if (motordc <= 16) {
    return 0;
  }
  else if (motordc >= 16 && motordc <= 33) {
    return (motordc - 16) / (17);
  }
  else if (motordc >= 49 && motordc <= 66) {
    return (66 - motordc) / (17);
  }
  else if (motordc >= 33 && motordc <= 49) {
    return 1;
  }
}

float FoP() {
  if (motordc <= 49) {
    return 0;
  }
  else if (motordc <= 49) {
    return (motordc - 49) / (17);
  }
  else if (motordc >= 66) {
    return 1;
  }
}

void implikasi () {
  //sesuai dengan rule
  // if dingin then lambat
  a1 = 85 - (FiN() * (43));
  // if hangat then sedang
  b1a = 42 + (FiZ() * (43));
  b1b = 170 - (FiZ() * (43));
  // if panas then cepat
  c1 = 127 + (FiP() * (43));
}

void luas_deffuzzy() {
  implikasi ();
  A1 = ((85 - a1) * FiN()) / 2;
  A2 = (a1 - 43) * FiN();
  A3 = ((b1a - 42) * FiZ()) / 2;
  A4 = ((170 - b1b) * FiZ()) / 2;
  A5 = (b1b - b1a) * FiZ();
  A6 = ((c1 - 42) * FiP()) / 2;
  A7 = (127 - c1) * FiP();
}

float f(float x) {
  if (B > 0 && sel_ == 0) {
    return ((x - A) / B) * x;
  }
  else if (B > 0 && sel_ == 1) {
    return ((A - x) / B) * x;
  }
  else {
    return A * x;
  }
}

/*Function definition to perform integration by Simpson's 1/3rd Rule */
float simpsons(float f(float x), float a, float b, float n) {
  float h, integral, x, sum = 0;
  int i;
  h = fabs(b - a) / n;
  for (i = 1; i < n; i++) {
    x = a + i * h;
    if (i % 2 == 0) {
      sum = sum + 2 * f(x);
    }
    else {
      sum = sum + 4 * f(x);
    }
  }
  integral = (h / 3) * (f(a) + f(b) + sum);
  return integral;
}

float fx(float limd, float limu, float a, float b, int sel) {
  int n, i = 2;
  float h, x, integral, eps = 0.1, integral_new;
  A = a;
  B = b;
  sel_ = sel;
  integral_new = simpsons(f, limd, limu, i);
  do {
    integral = integral_new;
    i = i + 2;
    integral_new = simpsons(f, limd, limu, i);
  } while (fabs(integral_new - integral) >= eps);
  /*Print the answer */
  return integral_new;
}

void moment() {
  luas_deffuzzy();
  //M1 = ∫ ((85-x)/43)x dx ==================> limd a1 dan limup 100
  M1 = fx(a1, 85, 85, (43), 1);
  //M2 = ∫ 0.555556x dx ==================> limd 0 dan limup a1
  M2 = fx(42, a1, FiN(), 0, 0);
  //M3 = ∫ ((x-42)/43)x dx ==================> limd 20 dan limup b1a
  M3 = fx(42, b1a, 42, (43), 0);
  //M4 = ∫ ((170-x)/43)x dx ==================> limd b1b dan limup 235
  M4 = fx (b1b, 170, 170, (43), 1);
  //M5 = ∫ 0 dx ==================> limd b1a dan limup b1b
  M5 = fx (b1a, b1b, FiZ(), 0, 0);
  //M6 = ∫ ((x-127)/43)x dx ==================> limd 127  dan limup c1
  M6 = fx(127, c1, 127, (43), 0);
  //M7 = ∫ 0 dx ==================> limd c1 dan limu p170
  M7 = fx(c1, 170, FiP(), 0, 0);
}

float deffuzzyfikasi() {
  return (M1 + M2 + M3 + M4 + M5 + M6 + M7) / (A1 + A2 + A3 + A4 + A5 + A6 + A7);
}

void mamdaniDisplay() {
  Serial.print("DHT Temperature : ");
  Serial.println(temperature);
  Serial.print("Keanggotaan Suhu Dingin : ");
  Serial.println(FiN());
  Serial.print("Keanggotaan Suhu Hangat : ");
  Serial.println(FiZ());
  Serial.print("Keanggotaan Suhu Panas : ");
  Serial.println(FiP());
  Serial.print("a1 : ");
  Serial.println(a1);
  Serial.print("b1a : ");
  Serial.println(b1a);
  Serial.print("b1b : ");
  Serial.println(b1b);
  Serial.print("C1 : ");
  Serial.println(c1);
  Serial.print("A1 : ");
  Serial.println(A1);
  Serial.print("A2 : ");
  Serial.println(A2);
  Serial.print("A3 : ");
  Serial.println(A3);
  Serial.print("A4 : ");
  Serial.println(A4);
  Serial.print("A5 : ");
  Serial.println(A5);
  Serial.print("A6 : ");
  Serial.println(A6);
  Serial.print("A7 : ");
  Serial.println(A7);
  Serial.print("M1 : ");
  Serial.println(M1);
  Serial.print("M2 : ");
  Serial.println(M2);
  Serial.print("M3 : ");
  Serial.println(M3);
  Serial.print("M4 : ");
  Serial.println(M4);
  Serial.print("M5: ");
  Serial.println(M5);
  Serial.print("M6 : ");
  Serial.println(M6);
  Serial.print("M7 : ");
  Serial.println(M7);
  pwmMamdani = int(deffuzzyfikasi());
  Serial.print("OutDefuzzyfikasi : ");
  Serial.print(pwmMamdani);
  Serial.println(" PWM");
  fanFixSpeed = map(pwmMamdani,0,255,500,1023);
  if(fanFixSpeed >= 500 && fanFixSpeed <= 700){
    fanStrSpeed = "Lambat";
  }else if(fanFixSpeed >= 701 && fanFixSpeed <= 900){
    fanStrSpeed = "Sedang";
  }else if(fanFixSpeed >= 901 && fanFixSpeed <= 1023){
    fanStrSpeed = "Cepat";
  }
  
}
