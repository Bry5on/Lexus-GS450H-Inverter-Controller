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
#include <DueTimer.h>  //https://github.com/collin80/DueTimer
#include <Wire_EEPROM.h>
#include <ISA.h>  //isa can shunt library

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


Metro timer_htm=Metro(10);

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

byte inv_status=1,
     gear=get_gear(); //get this from your transmission

bool htm_sent=0,
     mth_good=0;

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
float vcc = 5.0;
float adc_step = 3.3/1023.0;
float Rtop = 1800.0;
float Ro = 47000;
float To = 25+273;
float B = 3500;
float mg1_stat=0;
float mg2_stat=0;
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
{700,   525,  350,  175,  87,   0},
{350,   70,   -210,   -490,   -770,   -1050},
{0,   -350,   -700,  -1050,  -1400,  -1750},
{0,   -350,   -700,  -1050,  -1400,  -1750},
{0,   -350,   -700,  -1050,  -1400,  -1750}};

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
    
    if(smoothtorque < -70 && gear==DRIVE) digitalWrite(Out1,HIGH); //Set Out1 as brake light output during regen greater than normal engine coast, down to 3.75mph in jag
    else digitalWrite(Out1,LOW); //turn off Out1 as brake light when not regenerating
    return smoothtorque; //return torque
}

ISA Sensor;  //Instantiate ISA Module Sensor object to measure current and voltage 

void setup() {

  Can0.begin(CAN_BPS_500K);  //CAN bus for V2. Use for isa shunt comms etc
  Sensor.begin(0,500);  //Start ISA object on CAN 0 at 500 kbps
  //Can1.begin(CAN_BPS_500K);  //CAN bus for V2. Use for isa shunt comms etc
  //Sensor.begin(1,500);  //Start ISA object on CAN 1 at 500 kbps
  
  pinMode(pin_inv_req, OUTPUT);
  digitalWrite(pin_inv_req, 1);
  pinMode(13, OUTPUT);  //led
  pinMode(OilPumpPower, OUTPUT);  //Oil pump control relay
  digitalWrite(OilPumpPower,HIGH);  //turn on oil pump 12v power supply.
  analogWrite(OilPumpPWM,125);  //set 50% pwm to oil pump at 1khz for testing

  pinMode(InvPower, OUTPUT);  //Inverter Relay
  pinMode(Out1, OUTPUT);  //GP output one
  pinMode(TransSL1,OUTPUT); //Trans solenoids
  pinMode(TransSL2,OUTPUT); //Trans solenoids
  pinMode(TransSP,OUTPUT); //Trans solenoids

  digitalWrite(InvPower,HIGH);  //turn on at startup
  digitalWrite(Out1,LOW);  //turn off at startup
  digitalWrite(TransSL1,LOW);  //turn off at startup
  digitalWrite(TransSL2,LOW);  //turn off at startup
  digitalWrite(TransSP,LOW);  //turn off at startup

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

}

void handle_wifi(){
/*
 *
 * Routine to send data to wifi on serial 2
The information will be provided over serial to the esp8266 at 19200 baud 8n1 in the form :
vxxx,ixxx,pxxx,mxxxx,nxxxx,oxxx,rxxx,qxxx* where :

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
updates will be every 100ms approx.

v100,i200,p35,m3000,n4000,o20,r100,q50*
*/

//Serial2.print("v100,i200,p35,m3000,n4000,o20,r30,q50*"); //test string

digitalWrite(13,!digitalRead(13));//blink led every time we fire this interrrupt.

Serial2.print("v");//dc bus voltage
//Serial2.print(dc_bus_voltage);//voltage derived from Lexus inverter
Serial2.print(Sensor.Voltage);//voltage derived from ISA shunt
Serial2.print(",i");//dc current
Serial2.print(Sensor.Amperes);//current derived from ISA shunt
//Serial2.print(0);
Serial2.print(",p");//total motor power
Serial2.print(Sensor.KW);//Power value derived from ISA Shunt
//Serial2.print(0);
Serial2.print(",m");//mg1 rpm
Serial2.print(abs(mg1_speed));
Serial2.print(",n");//mg2 rpm
Serial2.print(abs(mg2_speed));
Serial2.print(",o");//mg1 temp. Using inductor temp
Serial2.print(temp_inv_inductor);
Serial2.print(",r");//mg2 temp. Using water temp for now
Serial2.print(temp_inv_water);
Serial2.print(",q");// pwm percent on oil pump
Serial2.print(parameters.PumpPWM);// disply oil pump speed in %.
Serial2.print("*");// end of data indicator

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
    mg2_torque=get_torque(); // -3500 (reverse) to 3500 (forward)
    mg1_torque=((mg2_torque*5)/4);
    if((mg2_speed>MG2MAXSPEED)||(mg2_speed<-MG2MAXSPEED)) mg2_torque=0;
    //if(gear==REVERSE)mg1_torque=0;

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

    for(int i=0;i<100;i++)mth_data[i]=0;
    while(Serial1.available()){mth_data[mth_byte]=Serial1.read();mth_byte++;}

    for(int i=0;i<98;i++)mth_checksum+=mth_data[i];
    if(mth_checksum==(mth_data[98]|(mth_data[99]<<8)))mth_good=1;else mth_good=0;
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

void changeGear()
{
  if (mg2_speed<100 && mg1_speed<100) //only shift at very low rpm (ideally 0 but leave a little play)
  {
  
    if(digitalRead(Low_In)) //shift to low gear on command at ~0 speed regardless of above setting
    {
      digitalWrite(TransSL1,HIGH);
      digitalWrite(TransSL2,HIGH);
      digitalWrite(TransSP,LOW);
    }
    else if(parameters.selGear)    //high gear
    {
      digitalWrite(TransSL1,LOW);
      digitalWrite(TransSL2,LOW);
      digitalWrite(TransSP,LOW);    //yes we are leaving them all off for initial proof of this version.
    }
  
    if(!parameters.selGear)   //low gear
    {
      digitalWrite(TransSL1,HIGH);
      digitalWrite(TransSL2,HIGH);
      digitalWrite(TransSP,LOW);
    }
  }
}


void processTemps()
{
  mg1_stat=readThermistor(analogRead(MG1Temp));
  mg2_stat=readThermistor(analogRead(MG2Temp));
}



//////////////Dilbert's temp sensor routine////////////////////////////
float readThermistor(int adc)
{
  float raw = adc;
  float voltage = raw*adc_step;
  
  float Rt = (voltage * Rtop)/(vcc-voltage);
  
  float temp = (1/(1.0/To + (1.0/B)*log(Rt/Ro)))-273;
  
  return temp;
}
///////////////////////////////////////////////////////////////////////

Metro timer_diag = Metro(1100);

void loop() {

  control_inverter();

  if(timer_diag.check())
  {
    changeGear();
    processTemps();
    handle_wifi();
    analogWrite(OilPumpPWM,map(parameters.PumpPWM, 0, 100, 0, 255)); //set oil pump pwm
  }
  checkforinput(); //Check keyboard for user input
}
