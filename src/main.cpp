/*
* Project Heartguard
* Author: Aaron Benedek, Emily Rawson, Jadynn Zane
* Date: 
* For comprehensive documentation and examples, please visit:
* https://docs.particle.io/firmware/best-practices/firmware-template/
*/


// Include Particle Device OS APIs
#include "Particle.h"
#include <Arduino.h>
// Let Device OS manage the connection to the Particle Cloud
//SYSTEM_MODE(AUTOMATIC);


// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);


// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler;


//Dummy Variables

int MAXHR;
int MAXRR;



//
//  Demo code for the MAX30003 breakout board
//
//  Arduino connections:
//
//  |MAX30003 pin label| Pin Function         |Arduino Connection|         Particle Connection
//  |----------------- |:--------------------:|-----------------:|
//  | MISO             | Slave Out            |  D12             |         MI
//  | MOSI             | Slave In             |  D11             |         MO
//  | SCLK             | Serial Clock         |  D13             |         SCK
//  | CS               | Chip Select          |  D7              |         D18 (S3)
//  | VCC              | Digital VDD          |  +5V             |         VUSB
//  | GND              | Digital Gnd          |  Gnd             |         GND
//  | FCLK             | 32K CLOCK            |  -               |       
//  | INT1             | Interrupt1           |  02              |        D7
//  | INT2             | Interrupt2           |  -               |         
//
//  This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  For information on how to use, visit https://github.com/Protocentral/protocentral-max30003-arduino
//
/////////////////////////////////////////////////////////////////////////////////////////



/*
#define MAX30003_SPI_SPEED 2000000


#define INT_PIN D7


#define MAX30003_CS_PIN D18
*/
#include<SPI.h>
#include "MAXdef.h"
#include "ThingSpeak.h"




#define INT_PIN D7

#define MAX30003_CS_PIN S3

MAX30003 max30003(MAX30003_CS_PIN);

bool rtorIntrFlag = false;
uint8_t statusReg[3];

// PPG Sensor connected to Analog Pin A0
const int ppgPin = A5;

int threshold = 1000; // Adjust this based on your sensor's output : keep working on this threshold value, workshopping never hurt any
int sensorValue = 0;
bool ppgpulseDetected = false;
unsigned long lastPulseTime = 0;
unsigned long interval = 0; // Time between beats in milliseconds
float bpm = 0;

// Variables for averaging BPM
const int ppgsampleCount = 10; // Number of samples for averaging
float bpmSamples[ppgsampleCount];
int sampleIndex = 0;


void rtorInterruptHndlr(){
  rtorIntrFlag = true;
}

void enableInterruptPin(){

  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), rtorInterruptHndlr, CHANGE);
}

//ADXL335 pin definitions
const int xPin = A0;
const int yPin = A1;
const int zPin = A2;
//Offsets
const float xOffset = 0.007;
const float yOffset = -0.002;
const float zOffset = 0.054;


//Sensitivity and Zero-g Reference
const float zeroG = 1.65;
const float sensitivity = 0.3;


//Fall Detection Thresholds
const float freeFallThreshold = 0.3;
const float impactThreshold = 2;


//Timing For Monitoring Impact After Free Fall
const unsigned long monitoringDuration = 2000; // 2 seconds

//ThingSpeak API setup
TCPClient client;

unsigned long myChannelNumber = 2772475;    // change this to your channel number
const char * myWriteAPIKey = "V6FNUKOQBLA93DF6"; // change this to your channel write API key

//const char* apiKey = "V6FNUKOQBLA93DF6";  // Replace with your ThingSpeak Write API Key
//const char* serverName = "api.thingspeak.com";
//const int port = 80;

//Timings
system_tick_t lastPublishecg = 0;
std::chrono::milliseconds ecgInterval = 1ms;


system_tick_t lastPublishaccel = 0;
std::chrono::milliseconds accelInterval = 100ms;


system_tick_t lastPublishppg = 0;
std::chrono::milliseconds ppgInterval = 50ms;

system_tick_t lastPublish = 0;
std::chrono::milliseconds publishInterval = 1100ms;


//Initialize fallstatus variable
int fallStatus;


//Calibration Function
 float getCalibratedAxis(int pin, float offset) {
  // Read raw value and convert to voltage
  float voltage = analogRead(pin) * (3.3 / 4095);
   
   // Apply calibration offset and convert to acceleration in g's
  return (voltage - zeroG - offset) / sensitivity;
 }


// setup() runs once, when the device is first turned on
void setup()
{
   Serial.begin(115200); //Serial begin
       WiFi.connect();
    while (WiFi.connecting()) {
        delay(100);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi.");

  ThingSpeak.begin(client);
   


   pinMode(MAX30003_CS_PIN,OUTPUT);
   digitalWrite(MAX30003_CS_PIN,HIGH); //disable device


   SPI.begin();


   bool ret = max30003.max30003ReadInfo();
   if(ret){
     Serial.println("Max30003 ID Success");
   }else{


     while(!ret){
       //stay here untill the issue is fixed.
       ret = max30003.max30003ReadInfo();
       Serial.println("Failed to read ID, please make sure all the pins are connected");
       delay(5000);
     }
   }


   Serial.println("Initialising the chip ...");
   max30003.max30003BeginRtorMode();   // initialize MAX30003
   enableInterruptPin();
   max30003.max30003RegRead(STATUS, statusReg);

     for (int i = 0; i < ppgsampleCount; i++) {
    bpmSamples[i] = 0;
  }
}


// loop() runs over and over again, as quickly as it can execute.
void loop()
{
  sensorValue = analogRead(ppgPin); //Setting up ppg

   if (millis() - lastPublishaccel >= accelInterval.count())
   {
    lastPublishaccel = millis();
    //Accelerometer Read Calibrated Acceleration Values
    float accelX = getCalibratedAxis(xPin, xOffset);
    float accelY = getCalibratedAxis(yPin, yOffset);
    float accelZ = getCalibratedAxis(zPin, zOffset);
    //Calculation Combined Magnitude
    float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    ThingSpeak.setField(4,(float)accelMagnitude/10);
    if (accelMagnitude/10 < freeFallThreshold)
   {
     fallStatus = 0;
     Serial.println("Free fall detected!");
     //Monitoring for impact
     unsigned long startTime = millis();
     bool impactDetected = false;
     while (millis() - startTime < monitoringDuration)
     {
       accelX = getCalibratedAxis(xPin, xOffset);
       accelY = getCalibratedAxis(yPin, yOffset);
       accelZ = getCalibratedAxis(zPin, zOffset);
       accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);


       if (accelMagnitude > impactThreshold)
       {
         //Impact Detected During Monitoring Period
         impactDetected = true;
         fallStatus = 1;
         //Particle.publish("fallDetection", String(fallStatus), PRIVATE);
         Serial.println("Impact detected - Fall likely!");
        ThingSpeak.setField(5,fallStatus);
         
         break;
       }
     }
     if (!impactDetected)
     {
       fallStatus = 0;
     }
     //int accel = accelMagnitude
     //int fallLikely = fallStatus;
   }
   }
   if (millis() - lastPublishecg >= ecgInterval.count())
   { lastPublishecg = millis();
     if(rtorIntrFlag){
        rtorIntrFlag = false;
        max30003.max30003RegRead(STATUS, statusReg);


        if(statusReg[1] & RTOR_INTR_MASK){


            max30003.getHRandRR();   //It will store HR to max30003.heartRate and rr to max30003.RRinterval.

            //Use the Serial.print functions to see if data is being collected if it is not being published to the web
            //Serial.print("Heart Rate  = ");
            //Serial.println(max30003.heartRate);
            //int MAXHR = max30003.heartRate;


            //Serial.print("RR interval  = ");
            //Serial.println(max30003.RRinterval);
            //int MAXRR = max30003.RRinterval;
        }
    }
    max30003.getEcgSamples();   //It reads the ecg sample and stores it to max30003.ecgdata .
    //Serial.println(max30003.ecgdata);
   }
   //PPG section:
   
   if (millis() - lastPublishppg >= ppgInterval.count())
   {
      // Detect the rising edge of the PPG signal (heartbeat)
  if (sensorValue > threshold && !ppgpulseDetected) {
    ppgpulseDetected = true; // Mark the pulse as detected
    unsigned long currentTime = millis(); // Get current time in milliseconds
    interval = currentTime - lastPulseTime; // Calculate time since last pulse

    // Avoid division by zero and ignore unreasonable intervals
    if (interval > 300 && interval < 2000) {
      bpm = 60000.0 / interval; // Calculate BPM (60000 ms in a minute)
      bpmSamples[sampleIndex] = bpm; // Store the sample
      sampleIndex = (sampleIndex + 1) % ppgsampleCount; // Increment index cyclically
    }

    lastPulseTime = currentTime; // Update last pulse time
  }

  // Detect falling edge to reset pulseDetected
  if (sensorValue < threshold && ppgpulseDetected) {
    ppgpulseDetected = false;
  }

  // Calculate average BPM from samples
  float averageBPM = 0;
  for (int i = 0; i < ppgsampleCount; i++) {
    averageBPM += bpmSamples[i];
  }
  averageBPM /= ppgsampleCount;

  // Output BPM
  ThingSpeak.setField(6,(float)bpm);
  ThingSpeak.setField(7,(float)averageBPM);
/*
  Serial.print("Current BPM: ");
  Serial.print(bpm);
  Serial.print(" | Average BPM: ");
  Serial.println(averageBPM);
*/
   }
   if (millis() - lastPublish >= publishInterval.count())
   {
    lastPublish = millis();

    //I Set the ECG data to be set on this interval to avoid unecessary processing/commands, since there is such a short sampling interval
   ThingSpeak.setField(1,(long)max30003.ecgdata);
   Serial.println(max30003.ecgdata);
   ThingSpeak.setField(2,(long)max30003.heartRate);
   Serial.println(max30003.heartRate);
   ThingSpeak.setField(3,(long)max30003.RRinterval);
   Serial.println(max30003.RRinterval);
   int writeStatus = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

   if (writeStatus == 200) {
    Serial.println("Channel update successful.");
   } else {
    Serial.print("Problem updating channel. HTTP error code ");
    Serial.println(writeStatus);
   }
   
}
delay(5);

}

 // The core of your code will likely live here.


 // Example: Publish event to cloud every 10 seconds. Uncomment the next 3 lines to try it!
 // Log.info("Sending Hello World to the cloud!");
 // Particle.publish("Hello world!");
 // delay( 10 * 1000 ); // milliseconds and blocking - see docs for more info!



/*
if (client.connect(serverName, port)) {
        String postStr = String::format(
            "api_key=%s&field1=%.2f&field2=%.2f",
            apiKey, value1, value2
        );

        client.println("POST /update HTTP/1.1");
        client.println("Host: api.thingspeak.com");
        client.println("Connection: close");
        client.println("Content-Type: application/x-www-form-urlencoded");
        client.print("Content-Length: ");
        client.println(postStr.length());
        client.println();
        client.print(postStr);

        Serial.println("Data sent successfully!");

        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                Serial.print(c);
            }
        }

        client.stop();
    } else {
        Serial.println("Connection failed");
    }
    */