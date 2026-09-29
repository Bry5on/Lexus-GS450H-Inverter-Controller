/*Basic software to run the Lexus GS450H hybrid transmission and inverter using the open source V1 or V2 controller
 * Take an analog throttle signal and converts to a torque command to MG1 and MG2
 * Feedback provided over USB serial
 * V3.01 simple menu system via usb uart added.
 *
 * Copyright 2019 T.Darby , D.Maguire
 * openinverter.org
 * evbmw.com
 *
 */


#include <Metro.h>
#include "variant.h"
#include <due_can.h>  //https://github.com/collin80/due_can
#include <due_wire.h>
//#include <DueTimer.h>  //https://github.com/collin80/DueTimer
#include <Wire_EEPROM.h>
#include <ISA.h>  //isa can shunt library
#include <stdio.h>

#define MG2MAXSPEED 10000
#define pin_inv_req 22

#define PARK 0
#define REVERSE 1
#define NEUTRAL 2
#define DRIVE 3

#define OilPumpPower  33
#define OilPumpPWM  2
#define InvPower    34
#define Out1  50
#define TransSL1  47
#define TransSL2  44
#define TransSP   45

#define IN1   6
#define IN2   7
#define Low_In   62

#define TransPB1    40
#define TransPB2    43
#define TransPB3    42

#define OilpumpTemp A7
#define TransTemp A4
#define MG1Temp A5
#define MG2Temp A6

#define EEPROM_VERSION      11


#define SerialDEBUG SerialUSB
 template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Allow streaming




byte get_gear()
{
  if(digitalRead(IN1))
  {
  return(DRIVE);
  }
  else if(digitalRead(IN2))
  {
  return(REVERSE);
  }
  else
  {
  return(NEUTRAL);
  }
}

////////// CAN variables////////////////
word RPM;
CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.

Metro timer_htm=Metro(10);
Metro timer_Frames200 = Metro(200);
Metro timer_Frames100 = Metro(100);

byte mth_data[100];
byte htm_data_setup[80]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,128,0,0,0,128,0,0,0,37,1};
byte htm_data[80]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,0,0,0};

unsigned short htm_checksum=0,
               mth_checksum=0,
               since_last_packet=4000;

unsigned long  last_packet=0;

volatile byte mth_byte=0;

float dc_bus_voltage=0,temp_inv_water=0,temp_inv_inductor=0; //just used for diagnostic output

short mg1_torque=0,
      mg2_torque=0,
      mg1_speed=-1,
      mg2_speed=-1,
      mg2_speed_temp=-1;
int vehicle_doublespeed = 0;

byte inv_status=1,
     gear=get_gear(); //get this from your transmission

bool htm_sent=0,
     mth_good=0;
uint32_t last_mth_valid_us=0;
uint8_t consecutive_mth_valid=0;
uint32_t startupDogReadyMs=0;
uint32_t startupDogTimeoutMs=0;
bool shiftFault=false;
bool dogPositionKnown=false;

int oil_power=120; //oil pump pwm value

float Version=3.01;
char incomingByte;
int Throt1Pin = A0; //throttle pedal analog inputs
int Throt2Pin = A1;
int ThrotVal=0; //value read from throttle pedal analog input
int ThrotRange=0; //total range between min throttle and max throttle
int maxDtorque=0, maxRtorque=0; //max torque values in variable form for math later
int16_t torque = 0, smoothtorque = 0; //torque command mapped from -3500 to 3500 for inverter control

/////////////throttle input smoothing variables///////////////
const int numReadings = 20;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total

/////////////temp sensor data////////////////////
struct ThermistorProfile
{
  float resistanceAt25C;
  float beta;
  float pullupResistance;
};

bool readThermistor(int adc, float resistanceAt25C, float beta,
                    float pullupResistance, float& celsius);
bool invalidThermistorReading(float& celsius);

const float thermistorDividerVoltage = 5.0f;
const float thermistorAdcReferenceVoltage = 3.3f;
const int thermistorAdcMax = 1023;
const float kelvinAt25C = 298.15f;
const float minimumThermistorCelsius = -40.0f;
const float maximumThermistorCelsius = 200.0f;

// Provisional resistance/Beta models; each channel has its own pull-up value.
const ThermistorProfile mgThermistorProfile = {47000.0f, 3500.0f, 33000.0f};
const ThermistorProfile oilPumpThermistorProfile = {100000.0f, 3950.0f, 66000.0f};
// Anchored to 3.36 kOhm at about 26 C; this is an estimate, not a calibration.
const ThermistorProfile transmissionThermistorProfile = {3495.0f, 3500.0f, 1800.0f};

// The pull-ups remain tied to +5 V. An open sensor can drive the ADC input
// toward +5 V; software clipping checks do not make that electrically safe.
float mg1_stat=0;
float mg2_stat=0;
float high_stat=0;
bool mg1_temp_valid=false;
bool mg2_temp_valid=false;
uint8_t StatorCAN=0;
uint8_t CoolantCAN=0;

unsigned long delayStart = 0; // the time the delay started, oil press light
bool delayRunning = false; // true if still waiting for delay to finish
////////////////////////////////////////////////////

/////Pedal Map - Drive - High/////
int16_t pedalmap_drive[11][6] = {     //torque 0-3500 (full scale for MG2)
{350,   700,  1050,   1575,   2450,   3500},
{175,   525,  1050,   1575,   2450,   3500},
{0,   350,  875,  1575,   2450,   3500},
{-390,  0,  656,  1400,   2362,   3500},
{-390,  0,  656,  1400,   2362,   3500},
{-360,  -35,  546,  1312,   2318,   3500},
{-350,  -70,  437,  1225,   2275,   3500},
{-312,  -105,   328,  1137,   2231,   3500},
{-259,  -140,   218,  1050,   2187,   3500},
{-221,  -175,   109,  962,  2143,   3500},
{-193,  -140,   0,  875,  2100,   3500}};

/////Pedal Map - Reverse/////
int16_t pedalmap_reverse[5][6] = { //torque 0-3500 (full scale for MG2)
{1400,  1050,  700,  350,  174,  0},
{700,   140,  -420,  -980,  -1540,  -2100},
{0,   -700,  -1400,  -2100,  -2800,  -3500},
{0,   -700,  -1400,  -2100,  -2800,  -3500},
{0,   -700,  -1400,  -2100,  -2800,  -3500}};

int16_t speedrange_drive[11] = //rpm
{-3500, 	-1750, 	0, 	900, 	3500, 	5250, 	7000, 	8750, 	10500, 	12250, 	14000}; // speed index 3 being different works with forcing speed index to 3 below

int16_t speedrange_reverse[5] = //rpm
{-3500, 	-1750, 	0, 	1750, 	3500};

////////////////////////////////////////////////////

typedef struct
{
  uint8_t  version; //eeprom version stored
  int Max_Drive_Torque=0;
  int Max_Reverse_Torque=0;
  unsigned int Min_throttleVal=0;
  unsigned int Max_throttleVal=0;
  unsigned int PumpPWM=0;
  bool  selGear=HIGH;
}ControlParams;

ControlParams parameters;

// MG2 regen scaled by the 2-speed, and a latched dog shift.
void applyDrivetrainTorque(int16_t mapTorque);
void updateBrakeLight();

short get_torque()
{
    ThrotVal = analogRead(Throt1Pin); // read from the throttle sensor
    ThrotRange = parameters.Max_throttleVal - parameters.Min_throttleVal; //full range of min-max throttle
    int16_t map_x, map_y;
    uint8_t pedal_index, speed_index;
    int16_t pedal_range[6] = {parameters.Min_throttleVal, 	parameters.Min_throttleVal+ThrotRange/5, 	parameters.Min_throttleVal+2*ThrotRange/5, 	parameters.Min_throttleVal+3*ThrotRange/5, 	parameters.Min_throttleVal+4*ThrotRange/5, 	parameters.Max_throttleVal};
    mg2_speed_temp = mg2_speed;
    if(gear==DRIVE) {
        if (mg2_speed_temp < speedrange_drive[0]) {
          mg2_speed_temp = speedrange_drive[0]; // force min speed if speed below expected range
          //SerialDEBUG.print("Below speed map range");
        }
        pedal_index = map(ThrotVal, pedal_range[0], pedal_range[5], 0, 5);
        speed_index = map(mg2_speed_temp, speedrange_drive[0], speedrange_drive[10], 0, 10);
        if(mg2_speed_temp < 1750 && mg2_speed_temp > 900) speed_index = 3; //force speed index to 3 instead of 2 so that interpolation works from 900rpm to 0 for regen instead of 1750-0
        //SerialDEBUG.print("Pedal map - Pedal/Speed "); SerialDEBUG.print(pedal_index); SerialDEBUG.print("/"); SerialDEBUG.println(speed_index);

        if (pedal_index >= 5 && speed_index >= 10) {
          return (pedalmap_drive[10][5]); // pedal and speed maxed out
          //SerialDEBUG.println("Pedal map - Pedal & Speed maxed out");
        }
        if (pedal_index >= 5) {
          return (map(mg2_speed_temp, speedrange_drive[speed_index], speedrange_drive[speed_index + 1], pedalmap_drive[speed_index][5], pedalmap_drive[speed_index + 1][5])); // pedal maxed out
          //SerialDEBUG.println("Pedal map - Pedal maxed out");;
        }
        if (speed_index >= 10) {
          return (map(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_drive[10][pedal_index], pedalmap_drive[10][pedal_index + 1])); // speed maxed out
          //SerialDEBUG.println("Pedal map - Speed maxed out");
        }

        map_x = map(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_drive[speed_index][pedal_index], pedalmap_drive[speed_index][pedal_index + 1]);
        map_y = map(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_drive[speed_index + 1][pedal_index], pedalmap_drive[speed_index + 1][pedal_index + 1]);
        //SerialDEBUG.print("Pedal map - Interp x/y "); SerialDEBUG.print(map_x); SerialDEBUG.print("/"); SerialDEBUG.println(map_y);

        torque = map(mg2_speed_temp, speedrange_drive[speed_index], speedrange_drive[speed_index + 1], map_x, map_y);
        //SerialDEBUG.print("Torque "); SerialDEBUG.print(torque), SerialDEBUG.print(", Throttle "); SerialDEBUG.print(ThrotVal), SerialDEBUG.print(", Speed "); SerialDEBUG.println(mg2_speed);

        torque = (long)torque * parameters.Max_Drive_Torque / 3500;
      }

  if(gear==REVERSE) {
      if (mg2_speed_temp < speedrange_reverse[0]) mg2_speed_temp = speedrange_reverse[0]; // force min speed if speed below expected range
      pedal_index = map(ThrotVal, pedal_range[0], pedal_range[5], 0, 5);
      speed_index = map(mg2_speed_temp, speedrange_reverse[0], speedrange_reverse[4], 0, 4);

      if (pedal_index >= 5 && speed_index >= 4) return (pedalmap_drive[10][4]); // pedal and speed maxed out
      if (pedal_index >= 5) return (map(mg2_speed_temp, speedrange_reverse[speed_index], speedrange_reverse[speed_index + 1], pedalmap_reverse[speed_index][5], pedalmap_reverse[speed_index + 1][5])); // pedal maxed out
      if (speed_index >= 4) return (map(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_reverse[4][pedal_index], pedalmap_reverse[4][pedal_index + 1])); // speed maxed out

      map_x = map(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_reverse[speed_index][pedal_index], pedalmap_reverse[speed_index][pedal_index + 1]);
      map_y = map(ThrotVal, pedal_range[pedal_index], pedal_range[pedal_index + 1], pedalmap_reverse[speed_index + 1][pedal_index], pedalmap_reverse[speed_index + 1][pedal_index + 1]);

      torque = map(mg2_speed_temp, speedrange_reverse[speed_index], speedrange_reverse[speed_index + 1], map_x, map_y);

      torque = (long)torque * parameters.Max_Reverse_Torque / 1750;
    }

    if(gear==NEUTRAL) torque = 0;//no torque in neutral

    total = total - readings[readIndex]; // subtract the last torque command
    readings[readIndex] = torque; // pull in the latest torque command
    total = total + readings[readIndex]; // add the torque command to the total
    readIndex = readIndex + 1; // advance to the next position in the array
    if (readIndex >= numReadings) readIndex = 0; // if we're at the end of the array...wrap around to the beginning
    
    smoothtorque = total / numReadings; // calculate the average of the torque commands
    return smoothtorque; //return torque
}

ISA Sensor;  //Instantiate ISA Module Sensor object to measure current and voltage 

void setup() {

  Can0.begin(CAN_BPS_500K);  //CAN bus for V2. Use for isa shunt comms etc
  Sensor.begin(0,500);  //Start ISA object on CAN 0 at 500 kbps
  Can1.begin(CAN_BPS_500K);  //CAN bus for V2. Use for gauges
  //Sensor.begin(1,500);  //Start ISA object on CAN 1 at 500 kbps
  
  pinMode(pin_inv_req, OUTPUT);
  digitalWrite(pin_inv_req, 1);
  pinMode(13, OUTPUT);  //led
  pinMode(OilPumpPower, OUTPUT);  //Oil pump control relay
  digitalWrite(OilPumpPower,HIGH);  //turn on oil pump 12v power supply. Use for oil light, flash on at startup
  analogWrite(OilPumpPWM,125);  //set 50% pwm to oil pump at 1khz for testing

  pinMode(InvPower, OUTPUT);  //Inverter Relay
  pinMode(Out1, OUTPUT);  //GP output one
  pinMode(TransSL1,OUTPUT); //Trans solenoids
  pinMode(TransSL2,OUTPUT); //Trans solenoids
  pinMode(TransSP,OUTPUT); //Trans solenoids

  digitalWrite(Out1,LOW);  //turn off at startup
  digitalWrite(TransSL1,LOW);  //turn off at startup
  digitalWrite(TransSL2,LOW);  //turn off at startup
  digitalWrite(TransSP,LOW);  //turn off at startup
  startupDogReadyMs=millis()+250;
  startupDogTimeoutMs=millis()+1500;
  digitalWrite(InvPower,HIGH);  //turn on after selecting the default high dog state

  pinMode(IN1,INPUT); //Input 1
  pinMode(IN2,INPUT); //Input 2
  pinMode(Low_In,INPUT); //Low gear selection input

  pinMode(TransPB1,INPUT); //Trans inputs
  pinMode(TransPB2,INPUT); //Trans inputs
  pinMode(TransPB3,INPUT); //Trans inputs

  Serial1.begin(250000);

  PIOA->PIO_ABSR |= 1<<17;
  PIOA->PIO_PDR |= 1<<17;
  USART0->US_MR |= 1<<4 | 1<<8 | 1<<18;

  htm_data[63]=(-5000)&0xFF;  // regen ability of battery
  htm_data[64]=((-5000)>>8);

  htm_data[65]=(27500)&0xFF;  // discharge ability of battery
  htm_data[66]=((27500)>>8);

  SerialDEBUG.begin(115200);
  Serial2.begin(19200); //setup serial 2 for wifi access

   Wire.begin();
  EEPROM.read(0, parameters);
  if (parameters.version != EEPROM_VERSION)
  {
    parameters.version = EEPROM_VERSION;
    parameters.Max_Drive_Torque=0;
    parameters.Max_Reverse_Torque=0;
    parameters.Min_throttleVal=0;
    parameters.Max_throttleVal=0;
    parameters.PumpPWM=0;
    EEPROM.write(0, parameters);
  }
  for (int thisReading = 0; thisReading < numReadings; thisReading++) { //smoothing filter setup for torque command
    readings[thisReading] = 0;
  }
  maxDtorque = parameters.Max_Drive_Torque; //values for calculating a torque map
  maxRtorque = parameters.Max_Reverse_Torque; //values for calculating a torque map
  delayStart = millis();   // start delay
  delayRunning = true; // not finished yet

}

static char wifiTxBuffer[512];
static size_t wifiTxLength = 0;
static size_t wifiTxOffset = 0;

void appendWifiText(const char* text)
{
  size_t remaining = sizeof(wifiTxBuffer) - wifiTxLength;
  if (remaining == 0) return;
  int written = snprintf(wifiTxBuffer + wifiTxLength, remaining, "%s", text);
  if (written > 0) {
    wifiTxLength += (size_t)written < remaining ? (size_t)written : remaining - 1;
  }
}

void appendWifiInt(long value)
{
  char text[16];
  snprintf(text, sizeof(text), "%ld", value);
  appendWifiText(text);
}

void appendWifiFloat(float value, uint8_t decimalPlaces)
{
  long scale = decimalPlaces == 1 ? 10L : 100L;
  long scaled = (long)(value * (float)scale + (value >= 0.0f ? 0.5f : -0.5f));
  long whole = scaled / scale;
  long fraction = scaled % scale;
  if (scaled < 0) appendWifiText("-");
  if (whole < 0) whole = -whole;
  if (fraction < 0) fraction = -fraction;
  appendWifiInt(whole);
  appendWifiText(".");
  char text[4];
  snprintf(text, sizeof(text), decimalPlaces == 1 ? "%01ld" : "%02ld", fraction);
  appendWifiText(text);
}

void service_wifi_tx()
{
  if (wifiTxOffset >= wifiTxLength) return;
  int available = Serial2.availableForWrite();
  if (available <= 0) return;
  size_t remaining = wifiTxLength - wifiTxOffset;
  size_t count = remaining < (size_t)available ? remaining : (size_t)available;
  size_t written = Serial2.write(
    (const uint8_t*)wifiTxBuffer + wifiTxOffset, count);
  wifiTxOffset += written;
  if (wifiTxOffset >= wifiTxLength) {
    wifiTxOffset = 0;
    wifiTxLength = 0;
  }
}

void handle_wifi(){
/*
 *
 * Routine to send data to wifi on serial 2
The information is provided over serial to the ESP8266 at 19200 baud 8n1:
@v=...;i=...;p=...;m=...;n=...;o=...;r=...;q=...;...*

The first eight fields retain the original protocol meanings:

v=pack voltage (0-700Volts)
i=current (0-1000Amps)
p=power (0-300kw)
m=mg1 rpm (0-10000rpm)
n=mg2 rpm (0-10000rpm)
o=mg1 temp (-20 to 120C)
r=mg2 temp (-20 to 120C)
q=oil pressure (0-100%)
*=end of string
xxx=three digit integer for each parameter eg p100 = 100kw.
The current v3 user loop sends approximately once per second.

The remaining fields are read-only diagnostics. A newline after the frame
lets the ESP8266 reject incomplete records.
*/

if (wifiTxOffset < wifiTxLength) return;
wifiTxLength = 0;
wifiTxOffset = 0;
digitalWrite(13,!digitalRead(13));//blink led every time we fire this interrrupt.

int throttle_percent = 0;
if (ThrotRange > 0) {
  throttle_percent = constrain(
    map(ThrotVal, parameters.Min_throttleVal, parameters.Max_throttleVal, 0, 100),
    0, 100);
}

appendWifiText("@v="); appendWifiFloat(Sensor.Voltage, 2);
appendWifiText(";i="); appendWifiFloat(Sensor.Amperes, 2);
appendWifiText(";p="); appendWifiFloat(Sensor.KW, 2);
appendWifiText(";m="); appendWifiInt(abs(mg1_speed));
appendWifiText(";n="); appendWifiInt(abs(mg2_speed));
appendWifiText(";o="); appendWifiFloat(mg1_stat, 1);
appendWifiText(";r="); appendWifiFloat(mg2_stat, 1);
appendWifiText(";q="); appendWifiInt(parameters.PumpPWM); // legacy field: oil-pump PWM command (%)
appendWifiText(";iw="); appendWifiFloat(temp_inv_water, 2);
appendWifiText(";il="); appendWifiFloat(temp_inv_inductor, 2);
float transmissionTemp = 0.0f;
readThermistor(
  analogRead(TransTemp), transmissionThermistorProfile.resistanceAt25C,
  transmissionThermistorProfile.beta, transmissionThermistorProfile.pullupResistance,
  transmissionTemp);
appendWifiText(";tt="); appendWifiFloat(transmissionTemp, 1);
float oilPumpTemp = 0.0f;
readThermistor(
  analogRead(OilpumpTemp), oilPumpThermistorProfile.resistanceAt25C,
  oilPumpThermistorProfile.beta, oilPumpThermistorProfile.pullupResistance,
  oilPumpTemp);
appendWifiText(";ot="); appendWifiFloat(oilPumpTemp, 1);
appendWifiText(";th="); appendWifiInt(throttle_percent);
appendWifiText(";brakeOut="); appendWifiInt(digitalRead(Out1)); // Out1 brake-light output, not a pedal input
appendWifiText(";gear="); appendWifiInt(gear);
appendWifiText(";sel="); appendWifiInt(parameters.selGear ? 1 : 0);
appendWifiText(";in1="); appendWifiInt(digitalRead(IN1));
appendWifiText(";in2="); appendWifiInt(digitalRead(IN2));
appendWifiText(";low="); appendWifiInt(digitalRead(Low_In));
appendWifiText(";sl1="); appendWifiInt(digitalRead(TransSL1));
appendWifiText(";sl2="); appendWifiInt(digitalRead(TransSL2));
appendWifiText(";sp="); appendWifiInt(digitalRead(TransSP));
appendWifiText(";pb1="); appendWifiInt(digitalRead(TransPB1));
appendWifiText(";pb2="); appendWifiInt(digitalRead(TransPB2));
appendWifiText(";pb3="); appendWifiInt(digitalRead(TransPB3));
appendWifiText(";inverterPower="); appendWifiInt(digitalRead(InvPower));
appendWifiText(";inverterRequest="); appendWifiInt(digitalRead(pin_inv_req));
appendWifiText(";oilPumpPower="); appendWifiInt(digitalRead(OilPumpPower));
appendWifiText(";md="); appendWifiInt(mth_good ? 1 : 0);
appendWifiText(";sf="); appendWifiInt(shiftFault ? 1 : 0);
appendWifiText(";is="); appendWifiInt(inv_status);
appendWifiText("*\r\n");

}




void control_inverter() {

  int speedSum=0;

  if(timer_htm.check()) //prepare htm data
  {
    if(mth_good)
    {
      dc_bus_voltage=(((mth_data[82]|mth_data[83]<<8)-5)/2);
      temp_inv_water=(mth_data[42]|mth_data[43]<<8);
      temp_inv_inductor=(mth_data[86]|mth_data[87]<<8);
      mg1_speed=mth_data[6]|mth_data[7]<<8;
      mg2_speed=mth_data[31]|mth_data[32]<<8;
    }
    gear=get_gear();
    applyDrivetrainTorque(get_torque());

    //speed feedback
    speedSum=mg2_speed+mg1_speed;
    speedSum/=113;
    htm_data[0]=(byte)speedSum;
    htm_data[75]=(mg1_torque*4)&0xFF;
    htm_data[76]=((mg1_torque*4)>>8);

    //mg1
    htm_data[5]=(mg1_torque*-1)&0xFF;  //negative is forward
    htm_data[6]=((mg1_torque*-1)>>8);
    htm_data[11]=htm_data[5];
    htm_data[12]=htm_data[6];

    //mg2
    htm_data[26]=(mg2_torque)&0xFF; //positive is forward
    htm_data[27]=((mg2_torque)>>8);
    htm_data[32]=htm_data[26];
    htm_data[33]=htm_data[27];

    //checksum
    htm_checksum=0;
    for(byte i=0;i<78;i++)htm_checksum+=htm_data[i];
    htm_data[78]=htm_checksum&0xFF;
    htm_data[79]=htm_checksum>>8;
  }

  since_last_packet=micros()-last_packet;

  if(since_last_packet>=4000) //read mth
  {
    htm_sent=0;
    mth_byte=0;
    mth_checksum=0;
    bool mth_frame_overflow=false;

    for(int i=0;i<100;i++)mth_data[i]=0;
    while(Serial1.available()) {
      byte incoming=Serial1.read();
      if(mth_byte<sizeof(mth_data))mth_data[mth_byte++]=incoming;
      else mth_frame_overflow=true;
    }

    for(int i=0;i<98;i++)mth_checksum+=mth_data[i];
    if(!mth_frame_overflow && mth_byte==sizeof(mth_data) &&
       mth_checksum==(mth_data[98]|(mth_data[99]<<8))) {
      mth_good=1;
      last_mth_valid_us=micros();
      if (consecutive_mth_valid < 3) consecutive_mth_valid++;
    } else {
      mth_good=0;
      consecutive_mth_valid=0;
    }
    last_packet=micros();
    digitalWrite(pin_inv_req,0);
  }

  since_last_packet=micros()-last_packet;

  if(since_last_packet>=10)digitalWrite(pin_inv_req,1);

  if(since_last_packet>=1000)
  {
    if(!htm_sent&&inv_status==0){for(int i=0;i<80;i++)Serial1.write(htm_data[i]);}
    else if(!htm_sent&&inv_status!=0){for(int i=0;i<80;i++)Serial1.write(htm_data_setup[i]);if(mth_data[1]!=0) inv_status--;}
    htm_sent=1;
  }
}




void diag_mth()
{
  ///mask just hides any MTH data byte which is represented here with a 0. Useful for debug/discovering.
  bool mth_mask[100] = {
    0,0,0,0,0,0,0,0,1,1,
    1,1,0,0,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,0,0,1,1,1,1,0,0,1,
    1,1,0,0,1,1,1,1,1,1,
    1,1,1,1,1,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    1,1,0,0,1,1,0,0,1,1,
    1,1,1,1,1,1,1,1,0,0,};

  SerialDEBUG.print("\n");
  SerialDEBUG.println("\t0\t1\t2\t3\t4\t5\t6\t7\t8\t9");
  SerialDEBUG.println("   ------------------------------------------------------------------------------");
  for (int j=0;j<10;j++)
  {
    SerialDEBUG.print(j*10);if(j==0)SerialDEBUG.print("0");SerialDEBUG.print(" |\t");
    for (int k=0;k<10;k++)
    {
      if(mth_mask[j*10+k])SerialDEBUG.print(mth_data[j*10+k]);else SerialDEBUG.print (" ");
      SerialDEBUG.print("\t");
    }
    SerialDEBUG.print("\n");
  }
  SerialDEBUG.print("\n");

  SerialDEBUG.print("MTH Valid: ");if(mth_good)SerialDEBUG.print("Yes"); else SerialDEBUG.print("No");SerialDEBUG.print("\tChecksum: ");SerialDEBUG.print(mth_checksum);
  SerialDEBUG.print("\n");

  SerialDEBUG.print("DC Bus: ");if(dc_bus_voltage>=0)SerialDEBUG.print(dc_bus_voltage);else SerialDEBUG.print("----");
  SerialDEBUG.print("v\n");

  SerialDEBUG.print("MG1 - Speed: ");SerialDEBUG.print(mg1_speed);
  SerialDEBUG.print("rpm\tPosition: ");SerialDEBUG.print(mth_data[12]|mth_data[13]<<8);
  SerialDEBUG.print("\n");

  SerialDEBUG.print("MG2 - Speed: ");SerialDEBUG.print(mg2_speed);
  SerialDEBUG.print("rpm\tPosition: ");SerialDEBUG.print(mth_data[37]|mth_data[38]<<8);
  SerialDEBUG.print("\n");

  SerialDEBUG.print("Water Temp:\t");SerialDEBUG.print(temp_inv_water);
  SerialDEBUG.print("c\nInductor Temp:\t" );SerialDEBUG.print(temp_inv_inductor);
  SerialDEBUG.print("c\nAnother Temp:\t");SerialDEBUG.print(mth_data[88]|mth_data[89]<<8);
  SerialDEBUG.print("c\nAnother Temp:\t");SerialDEBUG.print(mth_data[41]|mth_data[40]<<8);
  SerialDEBUG.print("c\n");

  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
}

/////////////////////////////////////////////////////////////////////////////////
//Serial menu system
////////////////////////////////////////////////////////////////////////////////



void printMenu()
{
   SerialDEBUG<<"\f\n=========== EVBMW GS450H VCU Version "<<Version<<" ==============\n************ List of Available Commands ************\n\n";
   SerialDEBUG<<"  ?  - Print this menu\n ";
   SerialDEBUG<<"  d - Print recieved data from inverter\n";
   SerialDEBUG<<"  D - Print configuration data\n";
   SerialDEBUG<<"  f  - Calibrate minimum throttle.\n ";
   SerialDEBUG<<"  g  - Calibrate maximum throttle.\n ";
   SerialDEBUG<<"  i  - Set max drive torque (0-3500) e.g. typing i200 followed by enter sets max drive torque to 200\n ";
   SerialDEBUG<<"  q  - Set max reverse torque (0-3500) e.g. typing q200 followed by enter sets max reverse torque to 200\n ";
   SerialDEBUG<<"  v  - Set gearbox oil pump speed (0-100%) e.g. typing v50 followed by enter sets oil pump to 50% speed\n ";
   SerialDEBUG<<"  a  - Select LOW gear.\n ";
   SerialDEBUG<<"  s  - Select HIGH gear.\n ";
   SerialDEBUG<<"  z  - Save configuration data to EEPROM memory\n ";

   SerialDEBUG<<"**************************************************************\n==============================================================\n\n";

}

void checkforinput()
{
  //Checks for keyboard input from Native port
   if (SerialDEBUG.available())
     {
      int inByte = SerialDEBUG.read();
      switch (inByte)
         {
          case 'z':
          EEPROM.write(0, parameters);
           SerialDEBUG.print("Parameters stored to EEPROM");
            break;

          case 'f':
            Cal_minthrottle();
            break;

          case 'g':
            Cal_maxthrottle();
            break;


          case 'd':     //Print data received from inverter
             diag_mth();
            break;

          case 'D':     //Print out the raw ADC throttle value
                PrintRawData();
            break;

          case 'i':
              Cal_torque_D();
            break;
          case 'q':
              Cal_torque_R();
            break;
          case 'v':
              SetPumpSpeed();
            break;

          case '?':     //Print a menu describing these functions
              printMenu();
            break;

          case 'a':
          parameters.selGear=0;
      SerialDEBUG.println("LOW Gear Selected");
            break;

         case 's':
          parameters.selGear=1;
     SerialDEBUG.println("HIGH Gear Selected");
            break;

          }
      }
}



//////////////////////////////////////////////////////////////////////////////

void PrintRawData()
{
  SerialDEBUG.println("");
  SerialDEBUG.println("***************************************************************************************************");
  SerialDEBUG.print("Throttle Channel 1: ");
  SerialDEBUG.println(analogRead(Throt1Pin));
  SerialDEBUG.print("Throttle Channel 2: ");
  SerialDEBUG.println(analogRead(Throt2Pin));
  SerialDEBUG.print("Commanded Torque: ");
  SerialDEBUG.println(torque);
  SerialDEBUG.print("Selected Direction: ");
  if (get_gear()==1) SerialDEBUG.println("REVERSE");
  if (get_gear()==2) SerialDEBUG.println("NEUTRAL");
  if (get_gear()==3) SerialDEBUG.println("DRIVE");
  SerialDEBUG.print("Selected Gear: ");
  if(parameters.selGear) SerialDEBUG.println("HIGH");
  if(!parameters.selGear || digitalRead(Low_In)) SerialDEBUG.println("LOW");
  SerialDEBUG.print("Configured Max Drive Torque: ");
  SerialDEBUG.println(parameters.Max_Drive_Torque);
  SerialDEBUG.print("Configured Max Reverse Torque: ");
  SerialDEBUG.println(parameters.Max_Reverse_Torque);
  SerialDEBUG.print("Configured gearbox oil pump speed: ");
  SerialDEBUG.println(parameters.PumpPWM);
  SerialDEBUG.println("Current valve positions: ");
  if(digitalRead(TransPB1))
  {
  SerialDEBUG.println("PB1:ON");
  }
  else
  {
  SerialDEBUG.println("PB1:OFF");
  }

  if(digitalRead(TransPB2))
  {
  SerialDEBUG.println("PB2:ON");
  }
  else
  {
  SerialDEBUG.println("PB2:OFF");
  }

   if(digitalRead(TransPB3))
  {
  SerialDEBUG.println("PB3:ON");
  }
  else
  {
  SerialDEBUG.println("PB3:OFF");
  }
  SerialDEBUG.print("MG1 Stator temp: ");
  SerialDEBUG.println(mg1_stat);
  SerialDEBUG.print("MG2 Stator temp: ");
  SerialDEBUG.println(mg2_stat);
  SerialDEBUG.println("***************************************************************************************************");
}

///////////////////Throttle pedal calibration//////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void Cal_minthrottle()
{
  SerialDEBUG.println("");
  SerialDEBUG.print("Configured min throttle value: ");
  parameters.Min_throttleVal=(analogRead(Throt1Pin));
  if(parameters.Min_throttleVal<0) parameters.Min_throttleVal=0;//noting lower than 0 for min.
  SerialDEBUG.println(parameters.Min_throttleVal);
}

void Cal_maxthrottle()
{
  SerialDEBUG.println("");
  SerialDEBUG.print("Configured max throttle value: ");
  parameters.Max_throttleVal=(analogRead(Throt1Pin));
  if (parameters.Max_throttleVal>1000) parameters.Max_throttleVal=1000;//limit on max value
  SerialDEBUG.println(parameters.Max_throttleVal);

}
//////////////////////////////////////////////////////////////////////////////////////////


//////////Torque calibration//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void Cal_torque_D()
{
  SerialDEBUG.println("");
  SerialDEBUG.print("Configured drive torque: ");
  if (SerialDEBUG.available()) {
    parameters.Max_Drive_Torque = SerialDEBUG.parseInt();
  }
  if(parameters.Max_Drive_Torque>3500) parameters.Max_Drive_Torque=3500;//limit max drive torque to within range
  SerialDEBUG.println(parameters.Max_Drive_Torque);
  maxDtorque = parameters.Max_Drive_Torque;
}

void Cal_torque_R()
{
  SerialDEBUG.println("");
  SerialDEBUG.print("Configured reverse torque: ");
  if (SerialDEBUG.available()) {
    parameters.Max_Reverse_Torque = SerialDEBUG.parseInt();
  }
  if(parameters.Max_Reverse_Torque>3500) parameters.Max_Reverse_Torque=3500;//limit max reverse torque to within range
  SerialDEBUG.println(parameters.Max_Reverse_Torque);
  maxRtorque = parameters.Max_Reverse_Torque;
}

/////////////////////////////////////////////////////////////////////////////////////////


void SetPumpSpeed()
{
  SerialDEBUG.println("");
  SerialDEBUG.print("Configured gearbox oil pump speed: ");
  if (SerialDEBUG.available()) {
    parameters.PumpPWM = SerialDEBUG.parseInt();
    if(parameters.PumpPWM>100) parameters.PumpPWM=100; //limit to max 100%
    if(parameters.PumpPWM<0) parameters.PumpPWM=0;//limit to 0
  }
  SerialDEBUG.println(parameters.PumpPWM);
}

void applyDog(bool lowGear)
{
  if (lowGear) {
    digitalWrite(TransSL1, HIGH);
    digitalWrite(TransSL2, HIGH);
    digitalWrite(TransSP, LOW);
  } else {
    digitalWrite(TransSL1, LOW);
    digitalWrite(TransSL2, LOW);
    digitalWrite(TransSP, LOW);
  }
}

// MG2 reduction. Low puts 2.05x the wheel torque on the same MG2 count.
static const float MG2_RATIO_LOW = 3.9f;
static const float MG2_RATIO_HIGH = 1.9f;
// Draft equivalence: 1 high-gear MG2 count ≈ this many MG1 counts at the wheels.
static const float MG1_PER_OUT = 1.25f;

static const int16_t SLEW_STEP = 40;           // counts per 10 ms while shifting
static const int16_t SLEW_STEP_ZERO = 20;      // counts per 10 ms across the lash
static const int16_t LASH_BAND = 80;           // only this band is rate-limited
static const uint16_t LASH_DWELL_MS = 50;
static const uint16_t UNLOAD_DWELL_MS = 100;
static const uint16_t SHIFT_HANDOFF_TIMEOUT_MS = 1200;
static const uint16_t SHIFT_CONFIRM_TIMEOUT_MS = 4000;
static const uint16_t DOG_POSITION_CONFIRM_MS = 100;
static const uint16_t MTH_FRESH_TIMEOUT_US = 40000;
static const int16_t MG2_UPSHIFT_START = 6500; // begin so the dog is home by 7000
static const int16_t MG2_UPSHIFT_HARD = 7000;
static const int16_t MG2_DOWNSHIFT_RESULT = 3000;

enum ShiftPhase {
  PHASE_IDLE = 0,
  PHASE_HANDOFF,
  PHASE_DWELL,
  PHASE_ACTUATE,
  PHASE_CONFIRM,
  PHASE_RELOAD,
  PHASE_FAULT
};

static bool ratioIsLow = false;       // updated only after dog-position feedback confirms a shift
static bool pendingLow = false;
static int8_t startupDogCandidate = -1;
static ShiftPhase shiftPhase = PHASE_IDLE;
static uint32_t shiftStartedMs = 0;
static uint32_t phaseMs = 0;
static uint32_t dogPositionSinceMs = 0;
static bool shiftFast = false;
static bool awaitNeutralRelease = false;
static bool faultNeutralSeen = false;
static int16_t slewMg1 = 0;
static bool slewMg1Dwelling = false;
static uint32_t slewMg1Until = 0;
static int16_t slewMg2 = 0;
static bool slewMg2Dwelling = false;
static uint32_t slewMg2Until = 0;
bool mthDataFresh()
{
  return consecutive_mth_valid >= 3 && last_mth_valid_us != 0 &&
         (uint32_t)(micros() - last_mth_valid_us) <= MTH_FRESH_TIMEOUT_US;
}

int16_t iabs16(int16_t v)
{
  if (v == (-32767 - 1)) return 32767;
  return v < 0 ? (int16_t)-v : v;
}

int16_t clamp16(int32_t v, int16_t lo, int16_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

int16_t outEquiv(int16_t mg2Cmd, bool lowGear)
{
  if (!lowGear) return mg2Cmd;
  return (int16_t)((float)mg2Cmd * (MG2_RATIO_LOW / MG2_RATIO_HIGH));
}

int16_t mg1FromOut(int32_t outCounts)
{
  return clamp16((int32_t)((float)outCounts * MG1_PER_OUT), -4375, 4375);
}

bool driverWantsLow()
{
  if (digitalRead(Low_In)) return true;
  if (!parameters.selGear) return true;
  return false;
}

// Low dog only when the driver asked for it and a downshift would land at <= 3000 MG2 rpm.
// In low, leave by 6500 so the shift is finished before 7000.
bool wantLowDog()
{
  int16_t spd = iabs16(mg2_speed);
  if (!driverWantsLow()) return false;
  if (ratioIsLow) return spd < MG2_UPSHIFT_START;
  float predicted = (float)spd * (MG2_RATIO_LOW / MG2_RATIO_HIGH);
  return predicted <= (float)MG2_DOWNSHIFT_RESULT;
}

void splitMapTorque(int16_t mapTorque, int16_t &mg1, int16_t &mg2)
{
  mg2 = mapTorque;
  mg1 = 0;
  if (mg2 > 0) {
    if (mg2 < 700) {
      mg2 = (int16_t)(mg2 * 5 / 2);
      mg1 = 0;
    } else if (mg2 < 1750) {
      mg1 = (int16_t)((mg2 - 700) * 25 / 12);
      mg2 = 1750;
    } else {
      mg1 = (int16_t)((mg2 * 5) / 4);
    }
  } else if (mg2 < 0 && gear == DRIVE) {
    if (mg2 < -2100) {
      mg1 = (int16_t)((mg2 * 5) / 4);
    } else {
      mg1 = (int16_t)((mg2 * 25) / 12);
      mg2 = 0;
    }
  } else {
    mg1 = (int16_t)((mg2 * 5) / 4);
  }
  // Same MG2 regen count is 2.05x harsher in low. Scale the command, not the drive torque.
  if (ratioIsLow && mg2 < 0) {
    mg2 = (int16_t)((float)mg2 * (MG2_RATIO_HIGH / MG2_RATIO_LOW));
  }
  mg1 = clamp16(mg1, -4375, 4375);
  mg2 = clamp16(mg2, -3500, 3500);
}

int16_t slewToward(int16_t &value, bool &dwelling, uint32_t &dwellUntil, int16_t target, uint32_t now, int16_t step)
{
  if (dwelling) {
    if ((int32_t)(now - dwellUntil) < 0) return value;
    dwelling = false;
  }
  int16_t cur = value;
  if (cur == target) return cur;
  bool cross = (cur > 0 && target < 0) || (cur < 0 && target > 0);
  if (cross) {
    int16_t zstep = shiftFast ? (int16_t)(SLEW_STEP_ZERO * 3) : SLEW_STEP_ZERO;
    if (cur > LASH_BAND || cur < (int16_t)-LASH_BAND) {
      int16_t edge = cur > 0 ? LASH_BAND : (int16_t)-LASH_BAND;
      if (cur > edge) {
        int32_t next = (int32_t)cur - step;
        cur = next < edge ? edge : (int16_t)next;
      } else {
        int32_t next = (int32_t)cur + step;
        cur = next > edge ? edge : (int16_t)next;
      }
    } else {
      if (cur > 0) cur = cur > zstep ? (int16_t)(cur - zstep) : 0;
      else cur = cur < (int16_t)-zstep ? (int16_t)(cur + zstep) : 0;
      if (cur == 0) {
        dwelling = true;
        dwellUntil = now + LASH_DWELL_MS;
      }
    }
  } else if (target > cur) {
    int32_t next = (int32_t)cur + step;
    cur = next > target ? target : (int16_t)next;
  } else {
    int32_t next = (int32_t)cur - step;
    cur = next < target ? target : (int16_t)next;
  }
  value = cur;
  return cur;
}

int8_t dogPositionState()
{
  // WiFi UI shows high=OFF/ON/OFF and low=ON/OFF/OFF for PB1/PB2/PB3.
  bool pb1 = digitalRead(TransPB1);
  bool pb2 = digitalRead(TransPB2);
  bool pb3 = digitalRead(TransPB3);
  if (pb1 && !pb2 && !pb3) return 1;
  if (!pb1 && pb2 && !pb3) return 0;
  return -1;
}

bool dogPositionConfirmed(bool lowGear, uint32_t now)
{
  if (dogPositionState() != (lowGear ? 1 : 0)) {
    dogPositionSinceMs = 0;
    return false;
  }
  if (dogPositionSinceMs == 0) {
    dogPositionSinceMs = now;
    return false;
  }
  return (now - dogPositionSinceMs) >= DOG_POSITION_CONFIRM_MS;
}

void serviceShift(int16_t &tgt1, int16_t &tgt2, uint32_t now)
{
  if (shiftPhase == PHASE_FAULT) {
    if (gear == NEUTRAL) {
      if (!faultNeutralSeen) {
        faultNeutralSeen = true;
        dogPositionSinceMs = 0;
      }
      int8_t position = dogPositionState();
      if (position < 0) dogPositionSinceMs = 0;
      else if (dogPositionConfirmed(position == 1, now)) {
        ratioIsLow = position == 1;
        pendingLow = ratioIsLow;
        shiftFault = false;
        shiftPhase = PHASE_IDLE;
        shiftFast = false;
        dogPositionSinceMs = 0;
        faultNeutralSeen = false;
        awaitNeutralRelease = true;
        applyDog(ratioIsLow);
      } else {
        applyDog(iabs16(mg2_speed) >= MG2MAXSPEED ? false : ratioIsLow);
      }
    } else {
      faultNeutralSeen = false;
      dogPositionSinceMs = 0;
      applyDog(iabs16(mg2_speed) >= MG2MAXSPEED ? false : ratioIsLow);
    }
    tgt1 = 0;
    tgt2 = 0;
    return;
  }

  if (awaitNeutralRelease) {
    if (gear == NEUTRAL) {
      applyDog(ratioIsLow);
      tgt1 = 0;
      tgt2 = 0;
      return;
    }
    awaitNeutralRelease = false;
  }

  if (shiftPhase == PHASE_IDLE) {
    shiftFast = false;
    bool overspeed = iabs16(mg2_speed) >= MG2MAXSPEED;
    bool requestLow = overspeed ? false : wantLowDog();
    if (requestLow != ratioIsLow) {
      pendingLow = requestLow;
      shiftFast = ratioIsLow && iabs16(mg2_speed) >= MG2_UPSHIFT_HARD;
      shiftPhase = PHASE_HANDOFF;
      shiftStartedMs = now;
    }
  }

  if (shiftPhase == PHASE_IDLE) {
    applyDog(ratioIsLow);
    return;
  }

  int16_t driver1 = tgt1;
  int16_t driver2 = tgt2;
  int16_t appliedOut = outEquiv(slewMg2, ratioIsLow);
  int16_t driverOut = outEquiv(driver2, ratioIsLow);

  if (shiftPhase == PHASE_HANDOFF || shiftPhase == PHASE_DWELL ||
      shiftPhase == PHASE_ACTUATE || shiftPhase == PHASE_CONFIRM) {
    tgt2 = 0;
    tgt1 = clamp16((int32_t)driver1 + (int32_t)mg1FromOut((int32_t)driverOut - (int32_t)appliedOut), -4375, 4375);
    applyDog(ratioIsLow);
  }

  if (shiftPhase == PHASE_HANDOFF) {
    if (iabs16(slewMg2) < 25) {
      shiftPhase = PHASE_DWELL;
      phaseMs = now;
    } else if ((now - shiftStartedMs) > SHIFT_HANDOFF_TIMEOUT_MS) {
      shiftFault = true;
      shiftPhase = PHASE_FAULT;
      applyDog(ratioIsLow);
      tgt1 = 0;
      tgt2 = 0;
    }
  } else if (shiftPhase == PHASE_DWELL) {
    tgt2 = 0;
    if ((now - phaseMs) >= UNLOAD_DWELL_MS) {
      shiftPhase = PHASE_ACTUATE;
      phaseMs = now;
      dogPositionSinceMs = 0;
      applyDog(pendingLow);
    }
  } else if (shiftPhase == PHASE_ACTUATE) {
    applyDog(pendingLow);
    if ((now - phaseMs) > 40) shiftPhase = PHASE_CONFIRM;
  } else if (shiftPhase == PHASE_CONFIRM) {
    applyDog(pendingLow);
    if (dogPositionConfirmed(pendingLow, now)) {
      ratioIsLow = pendingLow;
      shiftPhase = PHASE_RELOAD;
      phaseMs = now;
    } else if ((now - phaseMs) > SHIFT_CONFIRM_TIMEOUT_MS) {
      shiftFault = true;
      shiftPhase = PHASE_FAULT;
      applyDog(ratioIsLow);
      tgt1 = 0;
      tgt2 = 0;
    }
  } else if (shiftPhase == PHASE_RELOAD) {
    int16_t reloadOut = outEquiv(driver2, ratioIsLow);
    int16_t haveOut = outEquiv(slewMg2, ratioIsLow);
    tgt2 = driver2;
    tgt1 = clamp16((int32_t)driver1 + (int32_t)mg1FromOut((int32_t)reloadOut - (int32_t)haveOut), -4375, 4375);
    applyDog(ratioIsLow);
    if ((iabs16((int16_t)(slewMg2 - driver2)) < 30 &&
         iabs16((int16_t)(slewMg1 - driver1)) < 40) ||
        (now - phaseMs) > 1200) {
      shiftPhase = PHASE_IDLE;
    }
  }
}

void applyDrivetrainTorque(int16_t mapTorque)
{
  uint32_t now = millis();
  if (!dogPositionKnown && (int32_t)(now - startupDogReadyMs) >= 0) {
    int8_t position = dogPositionState();
    if (position < 0) {
      startupDogCandidate = -1;
      dogPositionSinceMs = 0;
    } else if (startupDogCandidate != position) {
      startupDogCandidate = position;
      dogPositionSinceMs = now;
    } else if ((now - dogPositionSinceMs) >= DOG_POSITION_CONFIRM_MS) {
      ratioIsLow = position == 1;
      dogPositionKnown = true;
      dogPositionSinceMs = 0;
    } else if ((int32_t)(now - startupDogTimeoutMs) >= 0) {
      shiftFault = true;
      shiftPhase = PHASE_FAULT;
    }
  }
  if (!mthDataFresh() || !dogPositionKnown ||
      (int32_t)(now - startupDogReadyMs) < 0) {
    mg1_torque = 0;
    mg2_torque = 0;
    slewMg1 = 0;
    slewMg2 = 0;
    slewMg1Dwelling = false;
    slewMg2Dwelling = false;
    updateBrakeLight();
    return;
  }

  int16_t tgt1 = 0;
  int16_t tgt2 = 0;
  splitMapTorque(mapTorque, tgt1, tgt2);
  serviceShift(tgt1, tgt2, now);

  int16_t step = shiftFast ? (int16_t)(SLEW_STEP * 3) : SLEW_STEP;
  if (shiftPhase == PHASE_IDLE) step = 4500; // follow the pedal; lash is the only pause
  mg1_torque = slewToward(slewMg1, slewMg1Dwelling, slewMg1Until, tgt1, now, step);
  mg2_torque = slewToward(slewMg2, slewMg2Dwelling, slewMg2Until, tgt2, now, step);
  if (iabs16(mg2_speed) >= MG2MAXSPEED) {
    mg2_torque = 0;
    slewMg2 = 0;
    slewMg2Dwelling = false;
  }
  updateBrakeLight();
}

void updateBrakeLight()
{
  bool regen = false;
  if (gear == DRIVE && mthDataFresh()) {
    regen = (mg1_torque < -70 && mg1_speed > 0) ||
            (mg1_torque > 70 && mg1_speed < 0) ||
            (mg2_torque < -70 && mg2_speed > 0) ||
            (mg2_torque > 70 && mg2_speed < 0);
  }
  bool closedPedal = ThrotVal <= (parameters.Min_throttleVal + ThrotRange / 64);
  digitalWrite(Out1, (regen || closedPedal) ? HIGH : LOW);
}

void changeGear()
{
  // Kept so older call sites still link. The dog is driven from applyDrivetrainTorque().
  applyDog(ratioIsLow);
}

void processTemps()
{
  mg1_temp_valid = readThermistor(
    analogRead(MG1Temp), mgThermistorProfile.resistanceAt25C,
    mgThermistorProfile.beta, mgThermistorProfile.pullupResistance, mg1_stat);
  mg2_temp_valid = readThermistor(
    analogRead(MG2Temp), mgThermistorProfile.resistanceAt25C,
    mgThermistorProfile.beta, mgThermistorProfile.pullupResistance, mg2_stat);
  if(!mg1_temp_valid || !mg2_temp_valid || mg1_stat > 120 || mg2_stat > 120 || Sensor.Voltage < 286)
    digitalWrite(OilPumpPower,HIGH);  // signal high temperature, invalid stator input, or low battery
  else if (delayRunning && ((millis() - delayStart) <= 200)) digitalWrite(OilPumpPower, HIGH); // oil pressure light on during startup
  else 
  {
    delayRunning = false; // prevent delay code being run more then once
    digitalWrite(OilPumpPower,LOW);
  }
  if(mg1_temp_valid && (!mg2_temp_valid || mg1_stat > mg2_stat)) high_stat = mg1_stat;
  else if(mg2_temp_valid) high_stat = mg2_stat;
  else high_stat = 0.0f;
}


//////////////Dilbert's temp sensor routine////////////////////////////
bool readThermistor(int adc, float resistanceAt25C, float beta,
                    float pullupResistance, float& celsius)
{
  if(adc <= 0 || adc >= thermistorAdcMax ||
     resistanceAt25C <= 0.0f || beta <= 0.0f || pullupResistance <= 0.0f)
  {
    return invalidThermistorReading(celsius);
  }

  float voltage = ((float)adc * thermistorAdcReferenceVoltage) / thermistorAdcMax;
  float dividerRemainder = thermistorDividerVoltage - voltage;
  if(voltage <= 0.0f || dividerRemainder <= 0.0f)
  {
    return invalidThermistorReading(celsius);
  }

  float sensorResistance = (voltage * pullupResistance) / dividerRemainder;
  if(sensorResistance <= 0.0f)
  {
    return invalidThermistorReading(celsius);
  }

  float inverseKelvin = (1.0f / kelvinAt25C) +
                        (log(sensorResistance / resistanceAt25C) / beta);
  if(inverseKelvin <= 0.0f)
  {
    return invalidThermistorReading(celsius);
  }

  float convertedCelsius = (1.0f / inverseKelvin) - 273.15f;
  if(convertedCelsius != convertedCelsius ||
     convertedCelsius < minimumThermistorCelsius ||
     convertedCelsius > maximumThermistorCelsius)
    return invalidThermistorReading(celsius);

  celsius = convertedCelsius;
  return true;
}

bool invalidThermistorReading(float& celsius)
{
  celsius = 0.0f;
  return false;
}
///////////////////////////////////////////////////////////////////////

void Frames100MS() // gauge + OBD frames; period set by timer_Frames100 (100 ms)
{
  if(timer_Frames100.check())
  {
    RPM=abs(mg1_speed) / 2.28; // absolute shaft rpm, still used by the OBD speed
    vehicle_doublespeed = abs(mg1_speed) / 52; //mg1_speed is 1.2*mg2_speed, mg2_speed is 1.9*output shaft speed, mg1=2.28*output shaft, 4000rpm output shaft is 88mph. mg1*.009649 = ground speed, 1/.009649 = 103.63 (~104)
    int16_t shaft_rpm = (int16_t)constrain((long)(mg1_speed / 2.28f), -32768L, 32767L);
    CoolantCAN = temp_inv_water;
    StatorCAN = high_stat;
    outframe.id = 0x0AA;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 // data frame (not a remote request)
    outframe.data.bytes[0] = (uint8_t)(int8_t)constrain(map(torque, -3500, 3500, -100, 100), -128, 127); // signed torque %
    outframe.data.bytes[1] = vehicle_doublespeed; //Two times the car's ground speed in mph * 2
    outframe.data.bytes[2] = StatorCAN; //higher of both stator temps in C. Gauge range 80 - 150C
    outframe.data.bytes[3] = CoolantCAN; //coolant temp in C. Gauge range 0 - 100C
    outframe.data.bytes[4] = lowByte((uint16_t)shaft_rpm);
    outframe.data.bytes[5] = highByte((uint16_t)shaft_rpm);
    outframe.data.bytes[6] = (uint8_t)(fabsf(Sensor.Amperes) / 2.0f); // |A|/2, divide before uint8 (avoids wrap at 256A)
    outframe.data.bytes[7] = 0x00;

    Can0.sendFrame(outframe);
    Can1.sendFrame(outframe);

    // 0x0AB — analog Serial2 fields not already packed in 0x0AA
    // b0-1 Voltage*10 (V), b2-3 kW*10 signed, b4-5 mg2 rpm signed,
    // b6 throttle %, b7 oil-pump PWM %
    int throttle_percent = 0;
    if (ThrotRange > 0) {
      throttle_percent = constrain(
        map(ThrotVal, parameters.Min_throttleVal, parameters.Max_throttleVal, 0, 100),
        0, 100);
    }
    float transmissionTemp = 0.0f;
    readThermistor(
      analogRead(TransTemp), transmissionThermistorProfile.resistanceAt25C,
      transmissionThermistorProfile.beta, transmissionThermistorProfile.pullupResistance,
      transmissionTemp);
    float oilPumpTemp = 0.0f;
    readThermistor(
      analogRead(OilpumpTemp), oilPumpThermistorProfile.resistanceAt25C,
      oilPumpThermistorProfile.beta, oilPumpThermistorProfile.pullupResistance,
      oilPumpTemp);

    {
      uint16_t v10 = (uint16_t)constrain((long)(Sensor.Voltage * 10.0f), 0L, 65535L);
      int16_t  p10 = (int16_t)constrain((long)(Sensor.KW * 10.0f), -32768L, 32767L);
      int16_t n_rpm = (int16_t)constrain((long)mg2_speed, -32768L, 32767L);
      outframe.id = 0x0AB;
      outframe.length = 8;
      outframe.extended = 0;
      outframe.rtr = 0;
      outframe.data.bytes[0] = lowByte(v10);
      outframe.data.bytes[1] = highByte(v10);
      outframe.data.bytes[2] = lowByte((uint16_t)p10);
      outframe.data.bytes[3] = highByte((uint16_t)p10);
      outframe.data.bytes[4] = lowByte((uint16_t)n_rpm);
      outframe.data.bytes[5] = highByte((uint16_t)n_rpm);
      outframe.data.bytes[6] = (uint8_t)constrain(throttle_percent, 0, 255);
      outframe.data.bytes[7] = (uint8_t)constrain(parameters.PumpPWM, 0, 255);
      Can0.sendFrame(outframe);
      Can1.sendFrame(outframe);
    }

    // 0x0AC — remaining analog temps + full MG1 rpm (no inv_status)
    // b0 mg1 C, b1 mg2 C, b2 inductor C, b3 trans C, b4 oil-pump C,
    // b5 gear, b6-7 mg1 rpm signed LE
    {
      int16_t m_rpm = (int16_t)constrain((long)mg1_speed, -32768L, 32767L);
      outframe.id = 0x0AC;
      outframe.length = 8;
      outframe.extended = 0;
      outframe.rtr = 0;
      outframe.data.bytes[0] = (uint8_t)constrain((int)mg1_stat, 0, 255);
      outframe.data.bytes[1] = (uint8_t)constrain((int)mg2_stat, 0, 255);
      outframe.data.bytes[2] = (uint8_t)constrain((int)temp_inv_inductor, 0, 255);
      outframe.data.bytes[3] = (uint8_t)constrain((int)(transmissionTemp + 0.5f), 0, 255);
      outframe.data.bytes[4] = (uint8_t)constrain((int)(oilPumpTemp + 0.5f), 0, 255);
      outframe.data.bytes[5] = (uint8_t)gear;
      outframe.data.bytes[6] = lowByte((uint16_t)m_rpm);
      outframe.data.bytes[7] = highByte((uint16_t)m_rpm);
      Can0.sendFrame(outframe);
      Can1.sendFrame(outframe);
    }

    outframe.id = 0x05C;            // Set our transmission address ID, OBD2 standard oil temp: https://en.wikipedia.org/wiki/OBD-II_PIDs
    outframe.length = 1;            // Data payload 1 byte
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 // data frame (not a remote request)
    outframe.data.bytes[0] = StatorCAN+40; //higher of both stator temps in C+40. Gauge range 80 - 150C

    Can0.sendFrame(outframe);
    Can1.sendFrame(outframe);

    outframe.id = 0x005;            // Set our transmission address ID, OBD2 standard coolant temp: https://en.wikipedia.org/wiki/OBD-II_PIDs
    outframe.length = 1;            // Data payload 1 byte
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 // data frame (not a remote request)
    outframe.data.bytes[0] = CoolantCAN+40; //higher of both stator temps in C+40. Gauge range 0 - 100C

    Can0.sendFrame(outframe);
    Can1.sendFrame(outframe);

    // Unsolicited OBD-II Mode 01 responses for the CAN2-002 (Can1 only)
    uint8_t vss_kph = (uint8_t)constrain((int)(RPM / 29.6f), 0, 255);
    outframe.id = 0x7E8;
    outframe.length = 8;
    outframe.extended = 0;
    outframe.rtr = 0;
    outframe.data.bytes[0] = 0x03;
    outframe.data.bytes[1] = 0x41;
    outframe.data.bytes[4] = 0;
    outframe.data.bytes[5] = 0;
    outframe.data.bytes[6] = 0;
    outframe.data.bytes[7] = 0;

    outframe.data.bytes[2] = 0x05;              // coolant
    outframe.data.bytes[3] = CoolantCAN + 40;
    Can1.sendFrame(outframe);

    outframe.data.bytes[2] = 0x5C;              // oil / stator
    outframe.data.bytes[3] = StatorCAN + 40;
    Can1.sendFrame(outframe);

    outframe.data.bytes[2] = 0x0D;              // VSS km/h, 205/70-15 + 3.73
    outframe.data.bytes[3] = vss_kph;
    Can1.sendFrame(outframe);

    // PID 0C: report |amps|*10 as engine RPM so an OBD tach reads like an ammeter
    // OBD rpm = (256*A + B) / 4  →  encode (amps*10)*4 = amps*40
    uint16_t tach_rpm_enc = (uint16_t)constrain((int)(fabsf(Sensor.Amperes) * 40.0f), 0, 65535);
    outframe.data.bytes[0] = 0x04;
    outframe.data.bytes[2] = 0x0C;
    outframe.data.bytes[3] = highByte(tach_rpm_enc);
    outframe.data.bytes[4] = lowByte(tach_rpm_enc);
    Can1.sendFrame(outframe);
  }    
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////Send these frames every 200ms /////////////////////////////////////////
/*void Frames200MS()
{
  if(timer_Frames200.check())
  {
digitalWrite(13,!digitalRead(13));//blink led every time we fire this interrrupt.

  if(can_status)
  {

///////////////////////////////////////////////////////////////////////////////////////////////////
        outframe.id = 0x1D2;            // current selected gear message
        outframe.length = 5;            // Data payload 5 bytes
        outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outframe.rtr=0;                 // data frame
        outframe.data.bytes[0]=shiftPos;  //e1=P  78=D  d2=R  b4=N
        outframe.data.bytes[1]=0x0c;  
        outframe.data.bytes[2]=0x8f;
        outframe.data.bytes[3]=Gcount;
        outframe.data.bytes[4]=0xf0;
        Can0.sendFrame(outframe);
        Can1.sendFrame(outframe);
        ///////////////////////////
        //Byte 3 is a counter running from 0D through to ED and then back to 0D///
        //////////////////////////////////////////////
         
///////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  }
}*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////


Metro timer_diag = Metro(1100);

void loop() {

  control_inverter();
  service_wifi_tx();
  Frames100MS();
  //Frames200MS();

  if(timer_diag.check())
  {
    processTemps();
    handle_wifi();
    analogWrite(OilPumpPWM,map(parameters.PumpPWM, 0, 100, 0, 255)); //set oil pump pwm
  }
  checkforinput(); //Check keyboard for user input
}
