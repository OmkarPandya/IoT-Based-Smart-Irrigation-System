/*************************************************************

  This is a simple demo of sending and receiving some data.
  Be sure to check out other examples!
 *************************************************************/

// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
//The all library (.h)
 
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID           "TMPLNqXy_QRF"
#define BLYNK_DEVICE_NAME           "Quickstart Device"
#define BLYNK_AUTH_TOKEN            "JVAGSXzhuikrVYAVqAsd0_XRKylXRtUq"

#include <Servo.h> 
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>
#include <SimpleDHT.h>

char auth[] = BLYNK_AUTH_TOKEN ;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Omkar M31s";
char pass[] = "Omk@180202";

// Variable(PINs) Initialize & Declaration
        #define value D3 
        #define Buzzer D4         
        #define Sensor D1              // PIN descriptions of Components (Sensors).
        int relayInput = D0;          // Relay pin D0        
        const int buzzerPin = D4;    // Buzzer pin D4
        const int flamePin = D1;    // flame sensor pin D1
        int soilsensor = D3;       // Soil sensor pin D3
        int rainsensor = A0;      // Rain sensor pin A0
        int pinDHT11 = D5;       // DHT sensor pin D5
        int sensorState = 0;
        int lastState = 0;
        int pinValue = 0;
        int Flame = HIGH;
        int relay=1; 
        int sensor=0;
        void sendTemps(); 
        SimpleDHT11 dht11(pinDHT11);
        Servo myservo;
        DHT dht(D5, DHT11);   //(sensor pin,sensor type)        
        BlynkTimer timer;


//Code for Live Moisture Data (Soil Sensor) on Blynk app. 
void moisture() 
  {
        //Soil sensor Data on Blynk App.
        Serial.println("    Soil Sensor Data    ");
        int value = digitalRead(D3);
        value = map(value, 0, 1023, 0, 1000);
        Blynk.virtualWrite(V2, value);    //V2 pin is use for moisture (Gauge on Blynk).
        Serial.println(value); 

        //Rain sensor Data on Blynk App.
        Serial.println("    Rain Sensor Data    ");
        int rainSensor = analogRead(A0);
        rainSensor = map(rainSensor, 0, 1023, 0, 100);
        Blynk.virtualWrite(V4, rainSensor);
        Serial.println(rainSensor);

  }


//Code for Live Temperature/Humidity Data (DHT11 Sensor) on Blynk app.
void sendSensor() 
  {
        //DHT11 sensor Data on Blynk App.
        Serial.begin(9600);
        float h = dht.readHumidity();
        float t = dht.readTemperature();
          
        if (isnan(h) || isnan(t)) 
          {
            Serial.println("Failed to read from DHT sensor!");
            return;
          }
            Blynk.virtualWrite(V0, t);      //V0 pin is use for Temperature (Gauge on Blynk).
            Blynk.virtualWrite(V1, h);     //V1 pin is use for Humidity (Gauge on Blynk).
  }
void setup()
{
  // Debug console
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  dht.begin();
  timer.setInterval(100L, sendSensor);

  //Soil Sensor & Relay code.
  Serial.begin(115200);
  pinMode(D3, INPUT_PULLUP); 
  pinMode(D0, OUTPUT);
  digitalWrite(D0, HIGH);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, moisture);

  //Rain sensor code.
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(100L, moisture);
  
  //Flame sensor & Buzzer code.
  Serial.begin(115200);
  pinMode(Buzzer, OUTPUT);
  pinMode(Sensor, INPUT);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, notification);
 }
// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V3)
{
  // Set incoming value from pin V0 to a variable
  pinValue = param.asInt();
}


//Code for Notification of Flame Warning on Blynk App.
void notification()
{
  int sensor = digitalRead(Sensor);
  if (pinValue == 1) 
  {
    Serial.println("System is ON");  //when Button is ON then the system gets ON.
    if (sensor == 1)
    {
      digitalWrite(Buzzer, LOW);  //No flame is detected.
    } 
    else if (sensor == 0) 
    {
      Blynk.notify("WARNING! A fire was detected");  //Flame is detected.
      digitalWrite(Buzzer, HIGH);
    }
  } 
  else if (pinValue == 0) 
  {
    Serial.println("System is OFF");  //when Button is OFF then the system gets OFF.
  } 
}


// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
}



void loop()
{
  Blynk.run();
  timer.run();
  //Soil sensor code with Relay 
            sensorState = digitalRead(value);
            Serial.println(sensorState);
            Serial.println("    Soil Sensor Data    ");   //print Soil Sensor
            
            if (sensorState == 1 && lastState == 0)  //Comparison of Last & Current State.
                                                    //If it is 1:0 then it send notification for Watering Plants.  
            {
                  Serial.println("    needs water, send notification    ");
                  Serial.println();
                  digitalWrite(relayInput,LOW);                //Relay is in OFF state.
                  Blynk.notify("    Water your plants   ");   //Get notification on Blynk App.
                  Serial.println();
                  lastState = 1;
                  delay(100);
            } 
              
            else if (sensorState == 1 && lastState == 1)  //Comparison of Last & Current State.
                                                         //If it is 1:1 then it shows no Water is given to the Plants.   
            {
                  //do nothing, has not been watered yet
                  digitalWrite(relayInput,LOW);
                  Serial.println("    Has not been watered yet    ");
                  Serial.println();
                  delay(100);
            }
            
            else    //Comparison of Last & Current State.
                   //If it is 0:0 then it shows that the Plants get water and stop the supply.
            {
                  digitalWrite(relayInput,HIGH);    //Relay is ON state
                  Serial.println("    Does not need water   ");
                  Serial.println();
                  Blynk.notify("    Does not need water   ");   //Get notification on Blynk App.
                  lastState = 0;
                  delay(100);
            }
                  delay(100);
                  
//Flame code with Buzzer.
            Serial.begin(9600);
            Flame = digitalRead(flamePin);
            Serial.println("    Flame Sensor Data   ");    // Print Flame sensor Data.
            
            if (Flame == LOW) 
            //If flame sensor(point) detecte any flame(Light) then it start/HIGH the buzzer.
            {
                 Serial.println("    Fire is Detected    ");    // Print Fire is Detected.  
                 Serial.println(); 
                 digitalWrite(buzzerPin, HIGH);               // Buzzer ON. 
            }           
            
            else
            //If flame sensor(point) detecte no flame(Light) then it stop/LOW the buzzer. 
            {
                 Serial.println("    No Fire is Detected   ");  // Print No Fire is Detected
                 digitalWrite(buzzerPin, LOW);                 // Buzzer OFF.
                 Serial.println();                            // Space betN next program reading.         
            } 
                 delay(500);
                 Serial.begin(9600);
            
//DHT11 sensor code (Temperature & Humidity Data).
            byte t = 0;  
            byte h = 0;  
            int err = SimpleDHTErrSuccess;   
            
            //starting of Dht11 which sense wheather the sensor is connected or not if not it shows the error.
            
            if ((err = dht11.read(  &t,  &h, NULL)) != SimpleDHTErrSuccess) 
            {
                  Serial.print(""); 
                  Serial.print(SimpleDHTErrCode(err));     // Sensor FAIL statment
                  Serial.println();
                  Serial.print(""); 
                  Serial.println(SimpleDHTErrDuration(err));
                  delay(1500); 
                  Serial.println();  
                  return;
            }  
            //code to print the value of Dht11 sensor 
                  Serial.println("    Temperature & Humidity Data   ");      // Print T & H Sensor.
                  Serial.print(( int)t); 
                  Serial.println("  Celsius  ");     // Celsius reading.
                  Serial.print(( int)h); 
                  Serial.println("  Humidity  ");   // Humidity reading.
                  Serial.println(); 
                 
                  delay(500);  
            
  } //END of the Void loop code.
