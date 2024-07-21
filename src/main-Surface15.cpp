#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <cstddef>
#include <cstring>

HardwareSerial dwin(1);
unsigned char Buffer[9];
unsigned char Pic_now[7] = {0x5A, 0xA5, 0x04, 0x83, 0x00, 0x14, 0x01};                    // vraagt naar actieve pagina
unsigned char topage0[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x00}; // start page
unsigned char topage1[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x01}; // choose A B or C
unsigned char topage2[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x02}; // schoose type of vacuuming
unsigned char topage3[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x03}; // peu de grains
unsigned char topage4[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x04}; // plus de rgains
unsigned char topage5[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x05}; // boue
unsigned char topage6[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x06}; // gravier
unsigned char topage7[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x07}; // menu manuel
unsigned char topage8[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x08};
unsigned char topage9[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x09};
unsigned char topage10[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x0A};
unsigned char topage11[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x0B};
unsigned char topage12[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x0C};
unsigned char topage13[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x0D};
unsigned char topage14[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x0E};
unsigned char topage15[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x0F};
unsigned char topage16[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x10};
unsigned char topage17[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x11};
unsigned char topage18[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x12};
unsigned char topage19[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x13};
unsigned char topage20[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x14};
unsigned char topage21[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x15};
unsigned char topage22[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x16};
unsigned char topage23[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x17};
unsigned char topage24[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x18};
unsigned char topage25[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x19};
unsigned char topage26[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x1A};
unsigned char topage27[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x1B};
unsigned char topage28[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x1C};
unsigned char topage29[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x1D};
unsigned char topage30[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x1E};
unsigned char topage31[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x1F};
unsigned char topage32[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x20};// pass assez de pression d'eau
unsigned char topage33[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x21};//pas asssez de vide (refer help page)
unsigned char topage34[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x22};// trop d'air comprimé
unsigned char topage35[10] = {0x5a, 0xa5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x23};//pas assez d'air comprimé
unsigned char ToWrite[8] = {0x5a, 0xa5, 0x05, 0x82, 0x00, 0x00, 0x00, 0x00};

bool busy = false;

String recv;
int fld = 0;
String str = "";

#define AC 0x60
#define VD 0x61
#define WP 0x62
#define ST 0x63
// actions
// ACtions numb ers
#define Pg 1
#define Plg 2
#define Bou 3
#define Grav 4
#define Rinc 5

// Pins
#define i36PV 36
#define i39PP 39
#define i34PE 34
#define i35AU 35
#define o32RP 32
#define o33RD 33
#define o25RE1 25
#define o26RS 26
#define o27RE2 27
#define o14VVI 14
#define o12VVSS 12
#define o2AC 2
#define o15VID 15
const byte rxPin = 16; // rx2
const byte txPin = 17; // tx2
// timer numbers
int MaxWork = 1;
int IntervalClean = 2;
int MaxPlaf = 3;
int MaxDev = 4;
int MaxEntr1 = 5;
int MaxSort = 6;
int MaxEntr2 = 7;
int MaxClapOuv = 8;
int TimeoutVac = 9;
int TimeoutClapClose = 10;

int Time_MaxWork = 0;
int Time_IntervalClean = 0;
int Time_MaxPlaf = 0;
int Time_MaxDev = 0;
int Time_MaxEntr1 = 0;
int Time_MaxSort = 0;
int Time_MaxEntr2 = 0;
int Time_MaxClapOuv = 0;
int Time_TimeoutVac = 0;
int Time_TimeoutClapClose = 0;
unsigned long tim;
unsigned long tims[10];
int timstrt[10]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int timact[10]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int j = 0;

// readouts
double pv = 0; // pressure vacuum
double pp = 0; // pressure compressed air
double pe = 0; // pressure water
double au = 0; // emergency stop

void setup()
{
    // Initialize serial communication with the HMI
    Serial.begin(9600);
    dwin.begin(9600, SERIAL_8N1, rxPin, txPin); // Replace RX_PIN and TX_PIN with appropriate pin numbers

    // All outputs HIGH for start

    digitalWrite(o32RP, HIGH);
    digitalWrite(o33RD, HIGH);
    digitalWrite(o25RE1, HIGH);
    digitalWrite(o26RS, HIGH);
    digitalWrite(o27RE2, HIGH);
    digitalWrite(o14VVI, HIGH);
    digitalWrite(o12VVSS, HIGH);
    digitalWrite(o2AC, HIGH);
    digitalWrite(o15VID, HIGH);
}
// lopende timers
void timers()
{
    // timer
    // 0= MaxTime work|1= Interval Clean |2 = MaxTime Plafond | 3 = devant | 4= entrée 1 |5 = sortie |6= entrée2 | 7 = Clapet ouvert | 8 = Timeout Vacuum| 9= clapet close
    for (int i = 1; i < 10; i++)
    {
        if (timstrt[i] == 1)
        {
            tims[i] = millis();
            timstrt[i] = 0;
            timact[i] = 1;
        }
    }
}
void dw_Write_Str() {}

    


void setinact()
{
    for (int i = 1; i <= 5; i++)
    {
        timact[i] = 0;
        timstrt[i] = 0;
    }
}

void StopAll()
{
    digitalWrite(o32RP, HIGH);
    digitalWrite(o33RD, HIGH);
    digitalWrite(o25RE1, HIGH);
    digitalWrite(o26RS, HIGH);
    digitalWrite(o27RE2, HIGH);
    digitalWrite(o14VVI, HIGH);
    digitalWrite(o12VVSS, HIGH);
    digitalWrite(o2AC, HIGH);
    digitalWrite(o15VID, HIGH);
}
void readall()
{
    pv = analogRead(i36PV); // 36 correct with division!! Val to start > 0.5
    pp = analogRead(i39PP); // 39 Val to start > 6 < 7
    au = analogRead(i35AU); // 35
    pe = analogRead(i34PE); //34
}
void startAll()
{
    digitalWrite(o12VVSS, LOW);// close clapet
    digitalWrite(o15VID, HIGH);// close vacuum
    delay(3000);
    digitalWrite(o2AC, LOW);// open compressed air
    delay(3000);
    readall();
delay (1000);
//check compressed air pressure
if(pp <6 || pp> 7){
    if (pp < 6 ) {dwin.write(topage35,10);} // pas assez d'air comprimé
    StopAll();return;
     if (pp > 7) {dwin.write(topage34,10);} // trop d'air comprimé
    StopAll();return;
}
// check waterpressure
if (pe < 1.5 ) {dwin.write(topage32,10); StopAll(); return;} // pas assez de pression d'eau
   
readall();
// check vacuum if after 10 seconds no vacuum then stop
 for (int i = 0; i <= 8; i++) {
    if (pv<0.5) goto getout;
    delay(1000);
 }

 getout:
 if (pv > 0.5) {dwin.write(topage33,10);} // pas assez de vide
    StopAll();
 return;
}

void StopWrk()
{ // Stop all actions if busy
    if (busy == true)
    {

        dwin.write(topage20, 10);
    }
}
void CloseVV()
{
}
void startPG() // start Peu de grains
{
    setinact();
    Time_IntervalClean = 300000;
    timstrt[IntervalClean] = 1;
    Time_MaxWork = 3600000;
    timers();
}
void startPLG() // start Plus de grains
{
    setinact();
    Time_IntervalClean = 60000;
    timstrt[IntervalClean] = 1;
    Time_MaxWork = 1800000;
    timers();
}

void startBOUE()
{
    setinact();
    Time_IntervalClean = 30000;
    timstrt[IntervalClean] = 1;
    Time_MaxWork = 1800000;
    timers();
}

void startGRAV()
{
    setinact();
    {
        Time_IntervalClean = 20000;
        timstrt[IntervalClean] = 1;
        Time_MaxWork = 600000;
        timers();
    }
}
void startRinc_cpl()
{
    setinact();
    {
        Time_IntervalClean = 10000;
        timstrt[IntervalClean] = 1;
        Time_MaxWork = 30000;
        timers();
    }
}
void Clean_interval()
{

    // reset timer
}

void loop()
{
    StopWrk();
    readall();
   /* Serial.print ("pv: ");
    Serial.print (pv);
    Serial.print ("   |   ");
     Serial.print ("pp: ");
    Serial.print (pp);
    Serial.print ("   |   ");
     Serial.print ("au: ");
    Serial.print (au);
    Serial.print ("   |   ");
     Serial.print ("pe: ");
    Serial.print (pe);  */
   
    // set time
    tim = millis();
    // check if a work is busy and stop if timeout
    for (int i = 1; i <= 10; i++)
    {
        switch (i)
        case 1 ... 5:
            if (timact[i] == 1)
            {
                if (tim > tims[IntervalClean])
                {
                    Clean_interval();
                    if (tim > tims[MaxWork])
                    {
                        StopWrk();
                    }
                }
                break;
            }}

        // Check for input from HMI screen

        if (dwin.available())
        {
            for (int j = 0; j <= 8; j++) // this loop will store whole frame in buffer array.
            {
                Buffer[j] = dwin.read();
                if (Buffer[j] < 0x10)
                {
                    Serial.print("0"); // Print leading zero for single-digit hexadecimal values
                }
                Serial.print(Buffer[j], HEX);
                Serial.print(" ");
            }
                {
      Buffer[j] = dwin.read();
    }

            if (Buffer[0] == 0x5A)
            {
                switch (Buffer[8])
                {
                case 0x00: // startpage
                    StopAll();
                case 0xA1: // peu de grains
                    startPG;
                case 0xA2: // peu de grains
                    startPLG;
                case 0xA3: // peu de grains
                    startBOUE;
                case 0xA4: // peu de grains
                    startGRAV;
                case 0xFF: // stop
                    StopAll();
                case 0xF0: // home
                    break;
                case 0x07: // help en info
                    break;
                }

                delay(1000);
            }
       else {Serial.println("No dwin");} }
    
}
