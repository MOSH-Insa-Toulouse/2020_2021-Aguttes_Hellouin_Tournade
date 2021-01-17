#include "rn2xx3.h"
#include <SoftwareSerial.h>

//#define DEBUG
#define LEDPIN 13
//Sur le shield => // TX = 10 // RX = 11 // RST = 12
#define TX  10
#define RX  11
#define RST  12
#define redPin 5
#define greenPin 3
#define bluePin 6

SoftwareSerial mySerial(TX, RX); // RX, TX

//CONFIGURATION EN VERSION ABP
const char *devAddr = "260116C2";
const char *nwkSKey = "F0B72CF81F926876006F560C7EFD0EBE";
const char *appSKey = "D3E7F5C440497228980E525E1A8A1029";

rn2xx3 myLora(mySerial);

void led_on() {digitalWrite(LEDPIN, HIGH);}
void led_off() {digitalWrite(LEDPIN, LOW);}

float R0;
//int sensorPin = A0;    // select the input pin for the potentiometer   
//float gazValue = 0;  // variable to store the value coming from the sensor

void setup() {
  pinMode(LEDPIN, OUTPUT);
  led_on();
  // read the value from the sensor:

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  

  // Initialisation de la led RGB en rouge le temps d'acquerir les premieres valeurs de gaz
  setColor(255, 0, 0);  // red
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("Startup");

  // Reset rn2483
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);
  digitalWrite(RST, LOW);
  delay(500);
  digitalWrite(RST, HIGH);

  // Initialise the rn2483 module
  myLora.autobaud();

  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(myLora.hweui());
  Serial.print("RN2483 version number: ");
  Serial.println(myLora.sysver());

  myLora.initABP(devAddr, appSKey, nwkSKey);

  led_off();
  delay(2000);

  // Calibration du capteur de gaz dans l'initialisation
  R0 = calibration_sensor();
  
  led_off();
}

void loop() {
  led_on();
  delay(1000);
    
    // Acquisition des mesures de gaz moyennées 
    float sensor_volt=0;
    float RS_gas=0; // Get value of RS in a GAS
    float ratio=0; // Get ratio RS_GAS/RS_air
    int sensorValue = analogRead(A0); // tension en sortie de capteurs de gaz connecté en A0
    sensor_volt=(float)sensorValue/1024*5.0; // Convertion de la valeur mesurée en tension
    RS_gas = (5.0-sensor_volt)/sensor_volt; // 

          /*-Replace the name "R0" with the value of R0 in the demo of First Test -*/
    ratio = RS_gas/R0;  // ratio = RS/R0 avec R0 calculer en initialisation 

    // Affichage couleur led RGB en fonction du ration de gaz obtenu
    if (ratio>=5){
        Serial.println("green");
        setColor(0, 255, 0);  // green 
    }
    else if (ratio< 5 && ratio> 2){ 
        Serial.println("orange");  
        setColor(255, 255, 0);  // orange 
    }
    else{
        Serial.println("red");
        setColor(255, 0, 0); //red
    }
    
    Serial.print("sensor_volt = ");
    Serial.println(sensor_volt);
    Serial.print("RS_ratio = ");
    Serial.println(RS_gas);
    Serial.print("Rs/R0 = ");
    Serial.println(ratio);

   //formatage de la valeur de gaz pour envoie
   uint32_t gaz = ratio *100 ;

  Serial.println("TXing");
  //FORMATAGE POUR TTN
  byte payload[2];
  payload[0]=highByte(gaz);
  payload[1]=lowByte(gaz);

  Serial.println("Message Hex envoyé:");
  Serial.println(gaz, HEX);

  #ifdef DEBUG
    for (int i=0; i < sizeof(payload); i++) {Serial.println(payload[i]);}
  #endif 

  //Envoie des données via LORA sur TTN
  myLora.txBytes(payload, sizeof(payload)); 
  
  led_off();
  delay(10000);
}

// Programme pour parametrer la led RGB en fonction du gaz sensor
void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

float calibration_sensor(){
    float sensor_volt=0;
    float RS_air=0; //  Get the value of RS via in a clear air
    float R0=0;  // Get the value of R0 via in H2
    float sensorValue=0;
 
        /*--- Get a average data by testing 100 times ---*/
    for(int x = 0 ; x < 100 ; x++)
    {
        sensorValue = sensorValue + analogRead(A0);
    }
    sensorValue = sensorValue/100.0;
        /*-----------------------------------------------*/
 
    sensor_volt = sensorValue/1024*5.0;
    RS_air = (5.0-sensor_volt)/sensor_volt; // omit *RL
    R0 = RS_air/6.5; // The ratio of RS/R0 is 6.5 in a clear air from Graph (Found using WebPlotDigitizer)
 
    Serial.print("sensor_volt = ");
    Serial.print(sensor_volt);
    Serial.println("V");
 
    Serial.print("R0 = ");
    Serial.println(R0);

    //on retourne la valeur de la resistance obtenue avec une air "pur" considérée comme notre référence d'initialisation
    return R0;
}
