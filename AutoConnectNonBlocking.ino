#include <ESPEssentials.h>
#include <WebSockets2_Generic.h>
//#include <SoftwareSerial.h>
#include <ArduinoJson.h>
//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 1;
uint16_t WiFi_DELAY_MS = 100;
uint32_t t1,tWiFi = 0;
static uint32_t t0 = 4000000000;//initialiserer ved at simulere taloverløb

/*Globale variable************************************************ */
float mx,my,mz = 0.0;
float gx,gy,gz,dt = 0.0;
float rotx,roty,rotz = 0.0;
float ax,ay,az = 0.0;
float dx,dy,dz = 0.0;
float kurs,kursRaw,roll,rollRaw,pitch,pitchRaw = 0.0;
float K = 0.99;
//uint8_t systemC, gyroC, accelC, magC = 0;
//float bnoData[10];

boolean LostConnection_Flag = false;
int NrOfLostPongs = 0;

using namespace websockets2_generic;
WebsocketsClient client;
// Check I2C device (id,address) and correct line below (by default address is 0x29 or 0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);



void setup(){
    pinMode(2, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  //1. OTA
  initESPEssentials("MARNAV Autonomiprojekt");
  Wifi.setWiFiAutoReconnect(true);
  
  //2. Seriel forbindelse oprettes
  Serial.begin(115200);
  while (!Serial); 
   
  // 3.WebSocket connect to server
  client.connect("192.168.137.1", 8000, "/ws");
   // run callback when messages are received
  client.onMessage(onMessageCallback);
  // run callback when events are occuring
  client.onEvent(onEventsCallback); 

  // 4. initialiserer BNO055
  if (!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1500);
  bno.setExtCrystalUse(true); /* Bruger microprocessorens  clock (jeg bruger en esp88266)- frem for BNO055'ens. Anbefales af alle */ 
}


void loop(){
  client.poll();
  handleESPEssentials();
    
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on by making the voltage LOW
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on by making the voltage LOW
    client.ping("SKIB1");
    Serial.println("ping");
    NrOfLostPongs++;
    
  if(LostConnection_Flag||NrOfLostPongs>10){
    NrOfLostPongs=0;
    LostConnection_Flag = false;
   //initESPEssentials("MARNAV Autonomiprojekt");
    Wifi.setWiFiAutoReconnect(true);
    client.connect("192.168.137.1", 8000, "/ws");
    Serial.println("prøver at forbinde");
  
 }
}

/* ********************************************************** */
/* FUNKTIONER WEBSOCKET                                       */
/* ********************************************************** */
void onMessageCallback(WebsocketsMessage message){
  Serial.print("Got Message: ");
  Serial.println(message.data());
}
void onEventsCallback(WebsocketsEvent event, String data){
    (void) data;//????
    if (event == WebsocketsEvent::ConnectionOpened){
        delay(1000);
        Serial.println("ConnectionOpened");
    }
    else if (event == WebsocketsEvent::ConnectionClosed){
        Serial.println("ConnectionClosed");
        LostConnection_Flag=true;
    } 
    else if (event == WebsocketsEvent::GotPing){
        Serial.println("Got a Ping!");
 
    } 
    else if (event == WebsocketsEvent::GotPong){
        Serial.println("Got a Pong!");
        NrOfLostPongs--; 
    }
}
/* ********************************************************** */
/* FUNKTIONER BNO055                                          */
/* ********************************************************** */
void sendBNO055val(){
  /* Get a new sensor vector*/
  imu::Vector<3> g = bno.getVector( Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> m = bno.getVector( Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> a = bno.getVector( Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
 
  /* Beregner tidsdifferencen dt mellem læsninger - til brug for gyro (jf.: kurs,efter = kurs,før + ROT*dt) */
  t1 = micros();//fryser tiden  
  if(t0>t1){t0 = t1;}//initialiserer og håndtere taloverløb
  dt =(t1-t0)
  dt =dt/1000000.0; /* i sek. */;
  t0 = t1;//Husker tidsstempel til næste tick 

/* Henter data fra BNO055 */
/* NB! akser fra 'euler'-koordinater til 'fly-koordinater' (dvs. x-akse frem & z-akse NEDAD!) dvs y- og z- akser skifter fortegn) 
 * Årsag: for at få pos ROT om z-akse i samme retn. som kompassets pos. retn)
 * NB! acc er neg da BNO055 måler reaktionskraft.
 */
  rotx = g[0];
  roty = -g[1];
  rotz = -g[2];
  
  ax = -a[0];
  ay = -(-a[1]);
  az = -(-a[2]); 

  mx = m[0];
  my = -m[1];
  mz = -m[2];

  //Beregner gyroens kurs fra Rate Of Turn
  gx = gx + rotx*dt;
  gy = gy + roty*dt;
  gz = gz + rotz*dt;

  // Roll, Pitch og Yaw (kurs) beregnes - bare trigonometri
  float RollRaw = atan2(ay,az); //rollRaw i radianer vinkelrum +-180
  rollRaw = RollRaw*180/PI;
  pitchRaw = atan2(-ax,(ay*sin(RollRaw)+az*cos(RollRaw)))*180/PI; // vinkelrum +-90
  kursRaw = atan2(-my,mx)*180/PI;
  
  //Comperatorfilter på roll, og pitch, 99% gyro, 1% acc
  float k=0.99;//procent gyro
  roll = (roll + rotx*dt)*k + rollRaw*(1-k); 
  pitch = (pitch + roty*dt)*k + pitchRaw*(1-k);

  //'Gyrostabiliserede' værdier
  //Serial.print(roll); //Serial.print(", ");
  //Serial.print(pitch); //Serial.print(", ");

  
  //roll & pitch i radianer
  float Roll = roll*PI/180;
  float Pitch =pitch*PI/180;

  //tilt kompenseret kurs.(Jeg kan vise dig beregningen hvis du er interesseret Jørgen - fås direkte ud fra rotationsmatriserne for roll og pitch)  
  //(Findes også mange steder på nettet! Pas dog på wikipiedia - der har byttet om på roll, pitch og yaw... Hmmm det ligner dem ellers ikke...) 
  // NB! her anvendes de gode! værdier for roll og pitch i radianer!
  float X = mx*cos(Pitch) + mz*sin(Pitch);
  float Y = mx*sin(Roll)*sin(Pitch) + my*cos(Roll) - mz * sin(Roll)*cos(Pitch);
  float kursGyroStabiliseret = (atan2(-Y,X)*180/PI);
  float gyrokurs = kurs +rotz*dt;
  
  // løser et fjollet diskontinuitetsproblem mellem gyro og mag. (Første del)
  // får mag til at være kontinuært over 180
  if(gyrokurs - kursGyroStabiliseret < -180){
    while (gyrokurs - kursGyroStabiliseret < -180){
      kursGyroStabiliseret = kursGyroStabiliseret -360.0;
    }
  }
  else if(gyrokurs - kursGyroStabiliseret > 180){
    while (gyrokurs - kursGyroStabiliseret > 180){
      kursGyroStabiliseret = kursGyroStabiliseret +360.0;
    }
  }
  kurs = (K*gyrokurs + (1-K)*kursGyroStabiliseret);
  
  //Serial.print(kursGyroStabiliseret); //Serial.print(", ");
  //Serial.print(kurs); //Serial.print(", ");
    //Auto kalibreringens status: (integer) 0=lavest niveau (forkast data), 3=højste niveau (fuldt kalibreret data)
  //uint8_t systemC, gyroC, accelC, magC = 0;
  bno.getCalibration(&systemC, &gyroC, &accelC, &magC);
 
  tWiFi = tWiFi + dt*1000;
  if(tWiFi > WiFi_DELAY_MS){
    tWiFi = 0;
    const int capacity = JSON_OBJECT_SIZE(7+2);
          StaticJsonDocument<capacity> doc;
          doc["name"] = "bno";
          doc["roll"] = roll;
          doc["pitch"] = pitch;
          doc["dt"]= dt;
          doc["kurs"] = kurs;
          doc["rawkurs"] = kursGyroStabiliseret;
          doc["kal"]= gyroC*1000+accelC*100+magC*10+systemC;
          String output = "";
          serializeJson(doc, output);
          client.send( output);
   
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
  //return kurs;
}
