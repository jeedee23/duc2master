#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>

/* Important comments:
INPUTS DIGITAL:
DI17Start = pushbutton: when pushed the input is pulled to high, when not pushed input is pulled to low
DI16Man = toggle button:when turned to ON the input is pulled to high, when turned to OFF input is pulled to low
DI33Stop = pushbutton: when pushed the input is pulled to high, when not pushed input is pulled to low
DI32E_Stop = emergency button: when toggled to HIGH, the ESP needs to pull all outputs high and flash the led.
INPUTS ANALOG:
AI35P_Water = sensor for water pressure between 0 and 5V, 6 V being 10 bar, therefore 4.8K connected from pin 39 to GND and 6.8K connected from pin 39 to output of sensor
AI34P_vacuum = sensor set to vacuum giving a value between 0 and 3.3 volt, 0 being appr atmosphere, and 1000 being appr 0bara
AI36P_Air = same as AI36P but for air
AI39L_Tank = analog sensor 0 to 190 Ohm, 0 = lowest level, 190 = highest level. Pin 35 connected with 220 ohm to gnd and one side of sensor, other side of sensor connected to 3.3 V
OUTPUTS DIGITAL:
O2Vid = open or close Vacuum valve : LOW = CLOSED| HIGH = OPEN
O13Ac = open or close compressed air Valve: LOW = OPEN | HIGH = CLOSED
O19Vv_cd = direction of emptying valve : LOW = OPENING | HIGH = CLOSING
O18Vv_On_Off = activation of emptying valve: LOW = RUNNING | HIGH = STOPPED
O15LED = LED : LOW = ON| HIGH = OFF
Cleaning valves: O27Clean_e1_2 = 27, O14Clean_sortie = 14, O5Clean_Ceil = 5, O25Clean_Vv = 25 : LOW = OPEN| HIGH = CLOSED
*/

enum Pins {
    DI17Start = 17, DI16Man = 16, DI33Stop = 33, DI32E_Stop = 32, AI35P_Water = 35, AI34P_Vacuum = 34, AI36P_Air = 36, AI39L_Tank = 39,
    O2Vid = 2, O13Ac = 13, O19Vv_cd = 19, O18Vv_On_Off = 18, O15LED = 15, O27Clean_e1_2 = 27, O14Clean_sortie = 14, O5Clean_Ceil = 5, O25Clean_Vv = 25
};

enum Task { Stop = 0, Man = 1, PG = 2, PLG = 3, Bou = 4, GVrav = 5, Clean = 6 };
enum State { none = 0, started = 1, busy = 2, ended = 3, error = 4 };

// State variables
int stat_OpenVv = none;
int stat_CloseVv = none;
int stat_OpenAc = none;
int stat_CloseAc = none;
int stat_Clean_Vv = none;
int stat_Check_Ac = none;
int stat_Check_Water = none;
int stat_Check_Vid = none;
int Act_Vacuum = none;
int Act_Stop = none;

// Timers
unsigned long Timer_Vv = 0;
unsigned long Timer_Ac = 0;
unsigned long Timer_Clean_Vv = 0;
unsigned long Timer_Check_Ac = 0;
unsigned long Timer_Check_Water = 0;
unsigned long Timer_Check_Vid = 0;

// Timeouts
const unsigned long timeout_Vv = 5000;        // 5 seconds
const unsigned long timeout_Ac = 3000;        // 3 seconds
const unsigned long Timeout_Clean_Vv = 3000;  // 3 seconds
const unsigned long Timeout_Check_Ac = 2000;  // 2 seconds
const unsigned long Timeout_Check_Water = 2000;  // 2 seconds
const unsigned long Timeout_Check_Vid = 7000;  // 7 seconds

// Sensor values
int Val_Vacuum = 0;
int Val_Level = 0;
int Val_Water = 0;
int Val_Air = 0;

// HMI message
String Hmi_msg = "";

// Mac address of the slave
uint8_t slaveAddress[] = {0x0C, 0xB8, 0x15, 0xF4, 0x5D, 0x34};

// Data structure for receiving data
typedef struct struct_message {
    char command[32];
    // Add more fields as needed
} struct_message;

struct_message myData;

// Callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.println("Received command: " + String(myData.command));

    // Handle received command
    if (strcmp(myData.command, "start") == 0) {
        // Start operation
        Serial.println("Starting operation...");
    } else if (strcmp(myData.command, "stop") == 0) {
        // Stop operation
        Serial.println("Stopping operation...");
    }
    // Add more command handlers as needed
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Add slave as peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, slaveAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Initialize the pins
    pinMode(DI17Start, INPUT_PULLDOWN);
    pinMode(DI16Man, INPUT_PULLDOWN);
    pinMode(DI33Stop, INPUT_PULLDOWN);
    pinMode(DI32E_Stop, INPUT_PULLDOWN);
    pinMode(AI35P_Water, INPUT);
    pinMode(AI34P_Vacuum, INPUT);
    pinMode(AI36P_Air, INPUT);
    pinMode(AI39L_Tank, INPUT);
    pinMode(O2Vid, OUTPUT);
    pinMode(O13Ac, OUTPUT);
    pinMode(O19Vv_cd, OUTPUT);
    pinMode(O18Vv_On_Off, OUTPUT);
    pinMode(O15LED, OUTPUT);
    pinMode(O27Clean_e1_2, OUTPUT);
    pinMode(O14Clean_sortie, OUTPUT);
    pinMode(O5Clean_Ceil, OUTPUT);
    pinMode(O25Clean_Vv, OUTPUT);

    digitalWrite(O2Vid, HIGH);
    digitalWrite(O13Ac, HIGH);
    digitalWrite(O19Vv_cd, HIGH);
    digitalWrite(O18Vv_On_Off, HIGH);
    digitalWrite(O27Clean_e1_2, HIGH);
    digitalWrite(O14Clean_sortie, HIGH);
    digitalWrite(O5Clean_Ceil, HIGH);
    digitalWrite(O25Clean_Vv, HIGH);
    digitalWrite(O15LED, LOW);
}

void Open_Vv() {
    Clean_Vv();
    if (stat_OpenVv == busy && (millis() - Timer_Vv) > timeout_Vv) {
        digitalWrite(O19Vv_cd, HIGH);
        digitalWrite(O18Vv_On_Off, HIGH);
        stat_OpenVv = ended;
        Timer_Vv = 0;
    }
    if (stat_OpenVv == none) {
        Timer_Vv = millis();
        digitalWrite(O18Vv_On_Off, HIGH);
        digitalWrite(O18Vv_On_Off, LOW);
        stat_OpenVv = busy;
    }
}

void Close_Vv() {
    Clean_Vv();
    if (stat_CloseVv == busy && (millis() - Timer_Vv) > timeout_Vv) {
        digitalWrite(O19Vv_cd, HIGH);
        digitalWrite(O18Vv_On_Off, HIGH);
        stat_CloseVv = ended;
        Timer_Vv = 0;
    }
    if (stat_CloseVv == none) {
        Timer_Vv = millis();
        digitalWrite(O18Vv_On_Off, HIGH);
        digitalWrite(O18Vv_On_Off, LOW);
        stat_CloseVv = busy;
    }
}

void Close_Ac() {
    if (stat_CloseAc == busy && (millis() - Timer_Ac) > timeout_Ac) {
        digitalWrite(O13Ac, HIGH);
        stat_CloseAc = ended;
        Timer_Ac = 0;
    }
    if (stat_CloseAc == none) {
        Timer_Ac = millis();
        digitalWrite(O13Ac, HIGH);
        stat_CloseAc = busy;
    }
}

void Open_Ac() {
    if (stat_OpenAc == busy && (millis() - Timer_Ac) > timeout_Ac) {
        digitalWrite(O13Ac, LOW);
        stat_OpenAc = ended;
        Timer_Ac = 0;
    }
    if (stat_OpenAc == none) {
        Timer_Ac = millis();
        digitalWrite(O13Ac, LOW);
        stat_OpenAc = busy;
    }
}

// Clean all open all valves
void Clean_All() {
    digitalWrite(O14Clean_sortie, LOW);
    digitalWrite(O27Clean_e1_2, LOW);
    digitalWrite(O25Clean_Vv, LOW);
    digitalWrite(O5Clean_Ceil, LOW);
}

void Clean_Vv() {
    if (stat_Clean_Vv == busy && (millis() - Timer_Clean_Vv) > Timeout_Clean_Vv) {
        digitalWrite(O14Clean_sortie, HIGH);
        stat_Clean_Vv = ended;
        Timer_Clean_Vv = 0;
    }
    if (stat_Clean_Vv == none) {
        Timer_Clean_Vv = millis();
        digitalWrite(O14Clean_sortie, LOW);
        stat_Clean_Vv = busy;
    }
}

void Stop_Clean() {
    digitalWrite(O14Clean_sortie, HIGH);
    digitalWrite(O27Clean_e1_2, HIGH);
    digitalWrite(O25Clean_Vv, HIGH);
    digitalWrite(O5Clean_Ceil, HIGH);
}

void Check_Ac() {
    if (stat_Check_Ac == busy && (millis() - Timer_Check_Ac) > Timeout_Check_Ac) {
        if (Val_Air > 600 && Val_Air < 700) {
            stat_Check_Ac = ended;
        } else {
            stat_Check_Ac = error;
        }
        Timer_Check_Ac = 0;
    }
    if (stat_Check_Ac == none) {
        Timer_Check_Ac = millis();
        stat_Check_Ac = busy;
    }
}

void Check_Water() {
    if (stat_Check_Water == busy && (millis() - Timer_Check_Water) > Timeout_Check_Water) {
        if (Val_Water > 200) {
            stat_Check_Water = ended;
        } else {
            stat_Check_Water = error;
        }
        Timer_Check_Water = 0;
    }
    if (stat_Check_Water == none) {
        Timer_Check_Water = millis();
        stat_Check_Water = busy;
    }
}

void Check_Vid() {
    if (stat_Check_Vid == busy && (millis() - Timer_Check_Vid) > Timeout_Check_Vid) {
        if (Val_Vacuum >= 600) {
            stat_Check_Vid = ended;
        } else {
            stat_Check_Vid = error;
        }
        Timer_Check_Vid = 0;
    }
    if (stat_Check_Vid == none) {
        Timer_Check_Vid = millis();
        stat_Check_Vid = busy;
    }
}

void Start_Vac() {
    if (Act_Vacuum == none) {
        if ((Timer_Clean_Vv == 0 && stat_Clean_Vv == ended) || stat_Clean_Vv == none) {
            Clean_Vv();
        }
        if ((Timer_Vv == 0 && stat_CloseVv == ended) || stat_CloseVv == none) {
            Close_Vv();
        }
    }
}

void Stop_All() {
    if (Act_Stop == none || Act_Stop == busy) {
        if (stat_CloseAc != ended) {
            Close_Ac();
        }
        if (stat_Check_Ac != ended && stat_CloseAc == ended) {
            Check_Ac();
        }
        if (stat_Check_Water != ended && stat_Check_Ac == ended) {
            Check_Water();
        }
        if (stat_Check_Vid != ended && stat_Check_Ac == ended && stat_Check_Water == ended) {
            Check_Vid();
        }
        if (stat_OpenVv != ended && stat_Check_Ac == ended && stat_Check_Water == ended && stat_Check_Vid == ended) {
            Open_Vv();
        }
        if (stat_OpenVv == ended) {
            Act_Stop = ended;
            Hmi_msg = "STOPPED";
            delay(30);
            Act_Vacuum = none;
        }
    }
}

void Empty_between() {
    // Placeholder function
}

int mostFrequent(int *arr, int n) {
    std::sort(arr, arr + n);
    int maxCount = 1, res = arr[0], currCount = 1;
    for (int i = 1; i < n; i++) {
        if (arr[i] == arr[i - 1]) {
            currCount++;
        } else {
            if (currCount > maxCount) {
                maxCount = currCount;
                res = arr[i - 1];
            }
            currCount = 1;
        }
    }
    if (currCount > maxCount) {
        maxCount = currCount;
        res = arr[n - 1];
    }
    return res;
}

int processDigitalReading(int readings[], int numReadings) {
    return mostFrequent(readings, numReadings);
}

int median(int *arr, int n) {
    std::sort(arr, arr + n);
    return arr[n / 2];
}

void processRealSensorData() {
    const int numReadings = 5;
    const int delayInterval = 10;
    int Val_StartReadings[numReadings];
    int Val_ManReadings[numReadings];
    int Val_StopReadings[numReadings];
    int Val_VacuumReadings[numReadings];
    int Val_WaterReadings[numReadings];
    int Val_AirReadings[numReadings];
    int Val_EstopReadings[numReadings];
    int Val_LevelReadings[numReadings];

    for (int i = 0; i < numReadings; i++) {
        Val_StartReadings[i] = digitalRead(DI17Start);
        Val_ManReadings[i] = digitalRead(DI16Man);
        Val_StopReadings[i] = digitalRead(DI33Stop);
        Val_VacuumReadings[i] = analogRead(AI34P_Vacuum);
        Val_WaterReadings[i] = analogRead(AI35P_Water);
        Val_AirReadings[i] = analogRead(AI36P_Air);
        Val_LevelReadings[i] = analogRead(AI39L_Tank);
        Val_EstopReadings[i] = digitalRead(DI32E_Stop);

        delay(delayInterval);
    }

    Val_Start = processDigitalReading(Val_StartReadings, numReadings);
    Val_Man = processDigitalReading(Val_ManReadings, numReadings);
    Val_Stop = processDigitalReading(Val_StopReadings, numReadings);
    Val_Estop = processDigitalReading(Val_EstopReadings, numReadings);
    Val_Vacuum = median(Val_VacuumReadings, numReadings);
    Val_Water = median(Val_WaterReadings, numReadings);
    Val_Air = median(Val_AirReadings, numReadings);
    Val_Level = median(Val_LevelReadings, numReadings);
}

void sendData() {
    if (PreviousOutputString != OutputString) {
        PreviousOutputString = OutputString;
        Serial.println(OutputString);
    }
    Hmi_data = OutputString;

    const size_t maxDataLength = 250;
    if (Hmi_data.length() > maxDataLength) {
        size_t pos = 0;
        while (pos < Hmi_data.length()) {
            String chunk = Hmi_data.substring(pos, pos + maxDataLength);
            esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)chunk.c_str(), chunk.length());
            if (result != ESP_OK) {
                Serial.printf("Error sending ESP-NOW data chunk: %d\n", result);
                Hmi_connected = false;
            }
            pos += maxDataLength;
        }
    } else {
        esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)Hmi_data.c_str(), Hmi_data.length());
        if (result != ESP_OK) {
            Serial.printf("Error sending ESP-NOW data: %d\n", result);
            Hmi_connected = false;
        }
    }

    responseReceived = false;
    unsigned long startTime = millis();
    unsigned long timeout = 500;

    while (!responseReceived && (millis() - startTime) < timeout) {
        delay(1);
    }

    if (!responseReceived) {
        Stop_All();
        Hmi_connected = false;
    } else {
        Hmi_connected = true;
    }
    delay(1);
}

void buildAndSendOutputString() {
    String OutputString = "||Val_SimOn:" + String(Val_SimOn ? 1 : 0) + "|";
    OutputString += "Start:" + String(Val_Start) + "|";
    OutputString += "Man:" + String(Val_Man) + "|";
    OutputString += "Stop:" + String(Stop) + "|";
    OutputString += "Estop:" + String(Val_Estop) + "|";
    OutputString += "Water:" + String(Val_Water) + "|";
    OutputString += "Vacuum:" + String(Val_Vacuum) + "|";
    OutputString += "Air:" + String(Val_Air) + "|";
    OutputString += "L:" + String(Val_Level) + "|";
    OutputString += "Vid:" + String(digitalRead(O2Vid) ? "OPEN" : "CLOSED") + "|";
    OutputString += "Ac:" + String(digitalRead(O13Ac) ? "CLOSED" : "OPEN") + "|";
    OutputString += "Vv_cd:" + String(digitalRead(O19Vv_cd) ? "CLOSING" : "OPENING") + "|";
    OutputString += "Vv_On_Off:" + String(digitalRead(O18Vv_On_Off) ? "OFF" : "ON") + "|";
    OutputString += "LED:" + String(digitalRead(O15LED) ? "OFF" : "ON") + "|";
    OutputString += "Clean_e1_2:" + String(digitalRead(O27Clean_e1_2) ? "CLOSED" : "OPEN") + "|";
    OutputString += "Rs:" + String(digitalRead(O14Clean_sortie) ? "CLOSED" : "OPEN") + "|";
    OutputString += "Clean_Ceil:" + String(digitalRead(O5Clean_Ceil) ? "CLOSED" : "OPEN") + "|";
    OutputString += "Clean_Vv:" + String(digitalRead(O25Clean_Vv) ? "CLOSED" : "OPEN") + "||";

    Serial.println(OutputString);
    sendData();
}

void read_all() {
    processRealSensorData();
    buildAndSendOutputString();
}

void loop() {
    read_all();
    // Manual working
    if (Val_Man != 0) {
        // start button pressed?
        if (Val_Start != 0) {
        }
    }
}
