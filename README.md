# Lexus-GS450H-Inverter-Controller
Opensource controller for the Lexus GS450H Hybrid inverter and gearbox.
<br>
Controls the inverter using it's own OEM logic board. No modification required.
<br>
Connects to the gearbox and controls oil pump, gear shifting and temperature monitoring.
<br>
Only requires throttle and brake inputs from the vehicle but also has CAN capabilty for communicating with modern vehicles.
<br>
Big thanks to Tom Darby for figuring out the control protocol.
<br>
Thread on the openinverter.org forum : <br>
https://openinverter.org/forum/viewtopic.php?f=14&t=205
<br>
PCB files in Designspark PCB 8.1 format.
<br>
Copyright 2019 D.Maguire and T.Darby
<br>
<br>
29/09/19 : Initial commit.PCB design in progress. 
<br>
<br>
27/10/19 : First successful test of the prototype running on the bench : https://youtu.be/F5XM6blwMlw
<br>
24/11/19 : Uploaded V3 software with reverse function. When IN1 is low motors run forward. when IN1 is pulled to +12v MG2 reverses.

21/01/2020 : Due to popular demand and a return to a no risk no reward culture, I have released the V2 design including the JLCPCB files so you can all take an arrow in back for me if I screwed up the design. In case anyone for some weird reason would like to support me why not buy a board from the EVBMW webshop : https://www.evbmw.com/index.php/evbmw-webshop

17/02/20 : WiFi interface files uploaded. Designed to run on the Olimex ESP8266 WiFi Module : <br>
https://www.olimex.com/Products/IoT/ESP8266/MOD-WIFI-ESP8266/open-source-hardware
<br>
<br>
02/06/20 : It works : https://www.youtube.com/watch?v=9eRWR5xXItc
<br>
<br>
18/01/21 : A more user friendly firmware (gs450h_v3_user.ino) with a basic serial menu interface has been tested and uploaded to aid those not familiar with programming.<br> A full description of the menu system may be found on the openinverter wiki page :
https://openinverter.org/wiki/Lexus_GS450h_Inverter

## Wi-Fi telemetry

The active Wi-Fi integration is `Software/gs450h_v3_user/gs450h_v3_user.ino` together
with `WiFi/GS450H_WiFi_V1.ino` and the files in `WiFi/Data`. The controller sends
one newline-terminated framed record approximately once per second over Serial2:

### Wi-Fi sketch setup

The Wi-Fi sketch supports ESP32 and ESP8266 builds. For ESP8266, LittleFS is
aliased to the existing `SPIFFS` name so the deployed route and file layout
remain compatible. Copy `WiFi/secrets.example.h` to `WiFi/secrets.h` and set
local values for `WIFI_AP_PASSWORD` and `OTA_PASSWORD`; the real file is
ignored by Git and must not be committed. Upload the contents of `WiFi/Data`
to the board filesystem.

At boot, `/ssid.txt` and `/password.txt` are read from the filesystem and used
for station mode. If either is missing or the station does not connect within
10 seconds, the sketch starts the `GS450H-Inverter` fallback access point using
`WIFI_AP_PASSWORD`. ArduinoOTA uses hostname `GS450H-Inverter` and
`OTA_PASSWORD`. The existing `/admin`, `/setBGcolor`, `/getSsid`,
`/getPassword`, and legacy scalar telemetry routes remain available.

```text
@v=630;i=12.4;p=7.8;m=1200;n=2400;o=42.1;r=44.0;q=50;iw=38;il=41;tt=45;ot=40;th=12;brakeOut=0;gear=3;sel=1;in1=1;in2=0;low=0;sl1=0;sl2=0;sp=0;pb1=0;pb2=1;pb3=0;inverterPower=1;inverterRequest=0;oilPumpPower=1;md=1;is=0*
```

The original `v,i,p,m,n,o,r,q` fields retain their meanings; `q` is the
configured oil-pump PWM command, not a pressure measurement. The legacy
`vxxx,ixxx,pxxx,mxxxx,nxxxx,oxxx,rxxx,qxxx*` records are still accepted by
the ESP8266. The read-only `/telemetry` endpoint exposes the latest complete
frame as JSON and includes `ageMs`; the mobile UI marks data stale after three
seconds. Temperatures are degrees Celsius, speeds are RPM, pump/throttle values
are percentages, and `0`/`1` state fields represent the observed hardware pin
level. `brakeOut`, `inverterPower`, `inverterRequest`, and `oilPumpPower` are
outputs; `pb1`-`pb3`, `in1`, `in2`, and `low` are inputs. No web endpoint
actuates hardware.
