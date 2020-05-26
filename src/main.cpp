#include <Arduino.h>
#include <U8x8lib.h>
#include "device_drivers/src/ads111x.h"

////////////////////////////////////////////////////////////////
// Programme simple de webserver
////////////////////////////////////////////////////////////////
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>


//These values are in the datasheet
#define VCC 3.23    //Supply voltage

/**
 * @brief 
 * 
 */
typedef struct t_NTCsensor{
  struct s_ntc_setting{
      int RThbeta=3000;  
      int RTh0=10000;
      int RRef=10000;

  }settings;
  struct s_ntc_data{
      float RThValue=-1;  
      float Temp=-1;
  }measure;
}NTCsensor;

float Humidity=-1;
float HrOffset=0.0;


NTCsensor myNTC, myNTC1, myNTC2;
float Kp=0.8, Ki=0.8, Kd=1.0;

char SetPWM(int pcPWM);
float calcNTCTemp(int UR10K, NTCsensor * NTC);
int PID_PowerControl(float currentTemp, float setPoint);
int timer1Sec, timer30Sec;

int State=0;
int Fan=0;
int Temp_setpoint=25;
int Cooler_power=0;
int power;
char myText[25];
//U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

const char* ssid = "cac-39837";
const char* password = "2xok-1f62-wply-kpur";

device_ads111x dev_ads111x;

// ATTENTION GPIO13 correspond à la PIN 07
//WiFiServer server(80);
ESP8266WebServer webserver(80);

//------------------------------------------------
static const char PROGMEM singleLine[] = R"rawliteral(  
<!DOCTYPE html>
<html>
<head>
<title>Single Line Input</title>
</head>
<body>
<form method="post" action="/save" >
<label for='name'>Serial Printer (Put Text here: ): </label>
<br><br>
<input type='text' id='name' name='name' required minlength='0' maxlength='800' size='100'>
<input type="submit" name="clk_action" value="accept">
</form>
</body>
</html>
)rawliteral";

//------------------------------------------------
static const char PROGMEM INDEX_HTML[] = R"rawliteral(  
<!DOCTYPE html>
<html>
<head>
<title>Clock Settings</title>
</head>
<body>
<form method="post" action="/save" >
State:<br><input name="State" type="text" size="16" value="0" ><br><br>
Ventilation:<br><input name="Ventilation" type="text" size="16" value="0" ><br><br> 
Temp Setpoint:<br><input name="TempSet" type="text" size="16" value="" ><br><br> 
<input type="submit" name="clk_action" value="Set param">
</form>

<form method="post" action="/getSettings" >
<input type="submit" name="clk_action" value="Get Settings">
</form>

<form method="post" action="/saveSettings" >
Kp:<br><input name="Kp" type="text" size="16" value="" ><br><br>
Ki:<br><input name="Ki" type="text" size="16" value="" ><br><br> 
Kd:<br><input name="Kd" type="text" size="16" value="" ><br><br> 
<input type="submit" name="clk_action" value="Set param">
</form>

</body>
</html>
)rawliteral";

//------------------------------------------------
void handleRoot() {
  webserver.send(200, "text/html", INDEX_HTML);
}

//------------------------------------------------
void handleNotFound() {
  webserver.send(404, "text/plain", "Page not found ...");
}

//------------------------------------------------
void handleGetSettings() {
  char dataSettings[1024];

  static const char PROGMEM JSONREPLY[] = R"rawliteral(  
{
  "MsgID": 0,
  "MsgFrom": "Cooler",
  "MsgData": {
    "State" : %d,
    "Fan" : %d,
    "TSetPoint" : %d,
    "CoolerPower" : %d,
    "Humidity" : %d,
    "NTC": [
      {
        "MsgSettings": {
          "RThBeta": %d
        },
        "MsgValue": {
          "Temp": %.2f
        }
      },
      {
        "MsgSettings": {
          "RThBeta": %d
        },
        "MsgValue": {
          "Temp": %.2f
        }
      },
      {
        "MsgSettings": {
          "RThBeta": %d
        },
        "MsgValue": {
          "Temp": %.2f
        }
      }      
    ]
  }
}
)rawliteral";

  sprintf(dataSettings, JSONREPLY, State, Fan, Temp_setpoint, power, (int)Humidity, myNTC.settings.RThbeta, myNTC.measure.Temp, myNTC1.settings.RThbeta, myNTC1.measure.Temp, myNTC2.settings.RThbeta, myNTC2.measure.Temp);
  webserver.send(200, "text/plain", dataSettings);
}

//------------------------------------------------
void handleSave() {
  String str = "Settings Saved ...\r\n";

  Serial.print("number of arguments "); 
  Serial.println(webserver.args());                    // number of arguments

  if (webserver.args() > 0 ) {
    for ( uint8_t i = 0; i < webserver.args(); i++ ) {
      str += webserver.argName(i) + " = " + webserver.arg(i) + "\r\n";

      Serial.println("Arg "+ String(i)+"="+ webserver.argName(i));     
      Serial.println("Arg "+ String(i)+"="+ webserver.arg(i));
      
    }
    webserver.arg(0).toCharArray(myText, 25);
    if(atoi(myText)>=0)
      State = atoi(myText);
    //u8x8.drawString(6, 0, myText);

    webserver.arg(1).toCharArray(myText, 25);
   // u8x8.drawString(6, 2, myText);
    if(atoi(myText)>=0)
      Fan=atoi(myText);

    webserver.arg(2).toCharArray(myText, 25);
   // u8x8.drawString(6, 2, myText);
    if(atoi(myText)>=0)
      Temp_setpoint=atoi(myText);

//    SetPWM(Cooler_power);
  }

  webserver.send(200, "text/plain", str.c_str());
}

//------------------------------------------------
void handleSaveSettings() {
  String str = "Settings PID Saved ...\r\n";

  Serial.print("number of arguments "); 
  Serial.println(webserver.args());                    // number of arguments

  if (webserver.args() > 0 ) {
    for ( uint8_t i = 0; i < webserver.args(); i++ ) {
      str += webserver.argName(i) + " = " + webserver.arg(i) + "\r\n";

      Serial.println("Arg "+ String(i)+"="+ webserver.argName(i));     
      Serial.println("Arg "+ String(i)+"="+ webserver.arg(i));
      
    }
    webserver.arg(0).toCharArray(myText, 25);
    Kp = atoi(myText);

    webserver.arg(1).toCharArray(myText, 25);
    Ki=atoi(myText);

    webserver.arg(2).toCharArray(myText, 25);
    Kd=atoi(myText);
  }

  webserver.send(200, "text/plain", str.c_str());
}

//------------------------------------------------
void handleSingleLine() {
  webserver.send(200, "text/html", singleLine);
}

void setup() {

  Serial.begin(9600);
  delay(10);
  
  pinMode(13, OUTPUT);
  dev_ads111x.deviceAddress = 0x49;
  ads111x_init(&dev_ads111x);

  u8x8.begin();
  //u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_7x14B_1x2_f);
  u8x8.drawString(0, 0, " -- COOLER -- ");
  u8x8.drawString(0, 2, "RTH 07.05.2020");

  delay(3000);
  u8x8.setFont(u8x8_font_7x14B_1x2_f);
  u8x8.clearDisplay();

  u8x8.drawString(0, 0, "Set:    C    %");
  //u8x8.drawString(0, 2, "TEMP:    /   C");
  u8x8.drawString(0, 2, "12.99C /   %Hr");
  u8x8.drawString(0, 4, "COOL:    /   C");
  u8x8.drawString(0, 6, "OUT:     /   C");
  u8x8.setCursor(8, 6);

  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  // Start the server
  //server.begin();
  Serial.println("Server started");
 
  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.println(WiFi.localIP());

  webserver.on("/", handleRoot);
  webserver.on("/save", handleSave);
  webserver.on("/saveSettings", handleSaveSettings);
  webserver.onNotFound(handleNotFound);
  webserver.on("/single", handleSingleLine);
  webserver.on("/getSettings", handleGetSettings);
  

  webserver.begin();
  Serial.println("Web server has started");
}
 
void loop() {
  float temp;

  webserver.handleClient();

    if(Fan)
      digitalWrite(13, 1);
    else digitalWrite(13, 0); // Turn off FAN

    if(timer1Sec>=10){
      if(State){
        power=PID_PowerControl(myNTC2.measure.Temp, Temp_setpoint);
        SetPWM(power);

        u8x8.drawString(11, 0, "   %");
        u8x8.setCursor(11, 0);
        u8x8.print(power);
    
        }else{
          SetPWM(0);
          u8x8.drawString(0, 0, "Set:    C  OFF ");
          u8x8.setCursor(5, 0);
          u8x8.print(Temp_setpoint, DEC); 
      }
      u8x8.setCursor(5, 0);
      u8x8.print(Temp_setpoint, DEC);
      timer1Sec=0;
    }

  //u8x8.drawString(0, 0, "Set: xxºC yyy%");
  if(timer30Sec>=50){
    Humidity=(((float)ads111x_getVoltage_mv(&dev_ads111x, 3)/500)-0.826)/0.0315;
    Humidity+=HrOffset;
    //Humidity=((float)ads111x_getVoltage_mv(&dev_ads111x, 3)/500);  // /500 beacause voltage dividor
    u8x8.setCursor(9, 2);
    //Serial.print(Humidity);
    u8x8.print((int)Humidity);
    timer30Sec=0;
  }

  temp=calcNTCTemp(ads111x_getVoltage_mv(&dev_ads111x, 0), &myNTC);
  u8x8.setCursor(7, 6);
  u8x8.print(temp,2);
//u8x8.print(11.11,2);

  temp=calcNTCTemp(ads111x_getVoltage_mv(&dev_ads111x, 1), &myNTC1);
  //Serial.print(ads111x_getVoltage_mv(&dev_ads111x, 1));
  //Serial.print("   ");
  u8x8.setCursor(7, 4);
  u8x8.print(temp, 2);
//u8x8.print(22.22,2);
  temp=calcNTCTemp(ads111x_getVoltage_mv(&dev_ads111x, 2), &myNTC2);
  u8x8.setCursor(0, 2);
  u8x8.print(temp, 2);
//u8x8.print(33.33,2);

  timer1Sec++;
  timer30Sec++;
  delay(100);
}

/**
 * @brief 
 * 
 * @param pcPWM 
 * @return char 
 */

char SetPWM(int pcPWM){
  int PWM_reg_setpoint;
  if (pcPWM<0){
      PWM_reg_setpoint = map(pcPWM*-1, 0, 100, 0, 1024);
      analogWrite(14, PWM_reg_setpoint);
      analogWrite(12, 0);
  }else{
      if (pcPWM>0){
        PWM_reg_setpoint = map(pcPWM, 0, 100, 0, 1024);
        analogWrite(14, 0);
        analogWrite(12, PWM_reg_setpoint);
      }else{
        analogWrite(14, 0);
        analogWrite(12, 0);
      }
  }
}


/**
 * @brief Calculation of NTC température
 * 
 * @param UR10K Ref. Resistor input voltage
 * @param NTC NTC circuit settings (RTh beta, RTh@0degree, RRef value)
 * @return float Temperature in degree C
 */
float calcNTCTemp(int UR10K, NTCsensor * NTC){
  //Variables
  float RT, ln, TX, T0, Rvoltage;
  T0 = 25 + 273.15;                 //Temperature T0 from datasheet, conversion from Celsius to kelvin

  Rvoltage = (float)UR10K/1000;

  RT = (VCC - Rvoltage) / (Rvoltage/NTC->settings.RRef);
  NTC->measure.RThValue=RT;

  ln = log(RT / NTC->settings.RTh0);
  TX = (1 / ((ln / NTC->settings.RThbeta) + (1 / T0))); //Temperature from thermistor

  TX = TX - 273.15;                 //Conversion to Celsius

  NTC->measure.Temp=TX;
  return (TX);
}

int PID_PowerControl(float currentTemp, float setPoint){

    float loopTimeDT = 1; 
    
    static int lastSpeed;
    static float sumError;
        
    float output;
    float outputMin=0;
    float outputMax=100;
    float error;
    float newSum;
    float dErrorLoopTime; 
     
    //error = setPoint - currentTemp;
    error = currentTemp - setPoint;
    newSum = (sumError + error) * loopTimeDT;
    dErrorLoopTime = (lastSpeed - currentTemp) / loopTimeDT;
    lastSpeed = currentTemp;
    
    output = Kp * error + Ki * sumError + Kd * dErrorLoopTime;
    
    if(output >= outputMax)
        output = outputMax;
    else
        if(output <= outputMin)
            output = outputMin;
        else 
            sumError =  newSum;
    
    return output;
}
