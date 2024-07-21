#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include <algorithm>

/* #region// Define constants */

#define I22Start 22             // touch button normally LOW
#define I21Man 21               // toggle button normally LOW
#define I33Stop 33              // touch button normally LOW
#define I35P_Water 35           // analog sensor water pressure
#define I32E_Stop 32            // toggle emergency button normally HIGH
#define I34P_Vacuum 34          // analog sensor vacuum
#define I39P_Air 39             // analog sensor compressed air
#define I36L_Tank 36            // analog sensor level low by 200 ohm resistor
#define O17Vid 17               // actuator valve
#define O16Ac 16                // actuator valve compressed air
#define O19Outlet_change_dir 19 // relays change direction outlet
#define O18Outlet_On_Off 18     // relay start open or close outlet
#define O23Led 23               // relay for led
#define O25Clean_e1 25          // clean E1
#define O26Clean_sortie 26      // clean S
#define O27Clean_e2 27          // clean E2
#define O5Rp 5                  // clean P
#define O21Rd 21                // clean D
char debugBuffer[512];
/* #endregion */

// Enum definitions
enum Clean
{
    e1,
    sortie,
    e2,
    Rp,
    Rd
};
enum Task
{
    Standby,
    Manual,
    Less_Grain,
    More_Grain,
    Sludge,
    Gravel,
    Clean
};
enum Action
{
    Vac_stopping,
    Vac_stopping_Opening_Outlet,
    Vac_stopping_Emptying_Tank,
    Vac_stopping_Cleaning_Tank,
    Vac_starting,
    Vac_starting_Closing_Outlet,
    Vac_starting_Cleaning_Tank,
    Vac_starting_wait_vacuum,
    Vac_busy,
    Clean_busy,
    None
};
enum Status
{
    Vac_stopped,
    Vid_closed,
    Vid_opened,
    Ac_closed,
    Ac_opened,
    Outlet_closed,
    Outlet_opened,
    Clean_E1_opened,
    Clean_E1_closed,
    Clean_E2_opened,
    Clean_E2_closed,
    Clean_sortie_opened,
    Clean_sortie_closed,
    Led_On,
    Led_Off,
    Rp_opened,
    Rp_closed,
    Rd_opened,
    Rd_closed
};
// Boolean array to store status flags
bool statusFlags[20];
// Function to set a status
void setStatus(Status status, bool value)
{
    statusFlags[status] = value;
}
// Function to get a status
bool getStatus(Status status)
{
    return statusFlags[status];
}
// Min/Max value parameters
enum Min_Max
{
    Vac_starting_Max_Vacuum,
    Vac_starting_Pressure_Air,
    Vac_starting_Pressure_Water,
    Vac_busy_Max_Vacuum,
    Vac_busy_Max_Level,
    Vac_busy_Max_Time
};
/* #region a bunch of other variables and settings */
String receivedData = "empty";
String debugString = "empty";
// Define task states
Task currentTask = Standby;
// Define action states
Action currentAction = None;

// Maximum times for actions (in milliseconds)
#define MAX_TIME_VAC_stopping 30000
#define MAX_TIME_VAC_STARTING 60000

// Define variables
int Simulation_On = 0;

// Data reading
int P_Vacuum = 0; // vacuum
int L_Tank = 0;   // level
int P_Air = 0;    // compressed air
int P_Water = 0;  // water
int E_Stop = 0;   // emergency stop coming from I32E_Stop
int Start = 0;    // Start button or data from HMI
int Stop = 0;     // Normal stop, no emergency coming from I33Stop or inside code (to simplify code)
int Man = 0;      // Manual mode coming from I21Man

unsigned long Time_Empty = 0;
unsigned long previousMillis = 0;
unsigned long previousBlinkMillis = 0;
int blinkCount = 0;
int totalBlinks = 0;
int timesToBlink = 0;
float timeBetweenBlink = 0;
float durationBlink = 0;

String Hmi_msg = "This is the start message";
String Hmi_P_Vacuum = "0.00";
String Hmi_P_Water = "0.00";
String Hmi_P_Air = "0.00";
String Hmi_Start = "0";
String Hmi_man = "0";
String Hmi_stop = "1";
String Hmi_data = "";
String outputString = "";
unsigned long tim;

unsigned long actionTimers[14] = {0}; // To track start time of each action

bool standby = true;          // system stand-by
bool stopping = false;        // loop intercepts stopping Priority 1
bool emptying = false;        // void stopping intercepts emptying, so this is a sub-priority 1.1
bool Vacuum_starting = false; // loop intercepts Vac_starting 2
bool Man_started = false;     // loop intercepts if Man_started Priority 2

uint8_t masterAddress[] = {0x30, 0xC6, 0xF7, 0x30, 0x1C, 0x0C}; // test board = 30:C6:F7:30:1C:0C on site : {0xA4, 0xCF, 0x12, 0x9A, 0xCC, 0x68}
uint8_t slaveAddress[] = {0x0C, 0xB8, 0x15, 0xF4, 0xC3, 0x48};  // 0C:B8:15:F4:C3:48
bool responseReceived = false;

bool actions[14] = {false}; // To track which actions are active
unsigned long maxTimes[14] = {
    0,                     // Dummy value for index 0
    MAX_TIME_VAC_stopping, // Vac_stopping
    10000,                 // Vac_stopping_Opening_Outlet
    20000,                 // Vac_stopping_Emptying_Tank
    15000,                 // Vac_stopping_Cleaning_Tank
    MAX_TIME_VAC_STARTING, // Vac_starting
    5000,                  // Vac_starting_Closing_Outlet
    10000,                 // Vac_starting_Cleaning
    20000,                 // Vac_starting_Max_Vacuum
    10000,                 // Vac_starting_Pressure_Air
    10000,                 // Vac_starting_Pressure_Water
    20000,                 // Vac_busy_Max_Vacuum
    15000,                 // Vac_busy_Max_Level
    30000                  // Vac_busy_Max_Time
};

// Air and Water pressure monitoring
const unsigned long airTimeout = 30000;   // Timeout duration for air pressure below threshold
const unsigned long waterTimeout = 30000; // Timeout duration for water pressure below threshold

unsigned long airPressureTimer = 0;
unsigned long waterPressureTimer = 0;
/* #endregion */
void StopAll()
{
    if (!standby)
    {
        actionTimers[Vac_stopping] = millis();
        if (Ac_opened && (millis() - actionTimers[Vac_stopping]) > 1000)
        {
            digitalWrite(O16Ac, HIGH);
            setStatus(Ac_closed, true);
            setStatus(Ac_opened, false);
        }
    }

    digitalWrite(O17Vid, HIGH);
    digitalWrite(O19Outlet_change_dir, LOW); // clapet open
    digitalWrite(O18Outlet_On_Off, LOW);
    digitalWrite(O23Led, LOW);
    digitalWrite(O25Clean_e1, HIGH);
    digitalWrite(O26Clean_sortie, HIGH);
    digitalWrite(O27Clean_e2, HIGH);
    digitalWrite(O5Rp, HIGH);
    digitalWrite(O21Rd, HIGH);
    for (int i = 0; i < 14; i++)
    {
        actionTimers[i] = 0;
    }
    Hmi_msg = "STOPPED";
    currentTask = Standby;
    delay(30);
    Vacuum_starting = 0;
    Man_started = 0;
    standby = true;
}
void CloseOutlet()
{
    digitalWrite(O19Outlet_change_dir, LOW);
    digitalWrite(O18Outlet_On_Off, LOW);
}
void OpenOutlet()
{
    digitalWrite(O19Outlet_change_dir, HIGH);
    digitalWrite(O18Outlet_On_Off, LOW);
}
void CleanEntry() {}
void OpenVid() {}
void CloseVid()
{
    digitalWrite(O17Vid, LOW);
    setStatus(Vid_closed, true);
    setStatus(Vid_opened, false);
}
void Openac()
{
    digitalWrite(O16Ac, LOW);
    setStatus(Ac_closed, false);
    setStatus(Ac_opened, true);
}
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // Serial.print("\r\nLast Packet Send Status: ");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    // Serial.print("Bytes received: ");
    // Serial.println(len);
    responseReceived = true; // Set the response flag to true
}
int median(int *arr, int n)
{
    std::sort(arr, arr + n); // Using standard sort function
    return arr[n / 2];
}
int mostFrequent(int *arr, int n)
{
    std::sort(arr, arr + n); // Using standard sort function
    int maxCount = 1, res = arr[0], currCount = 1;
    for (int i = 1; i < n; i++)
    {
        if (arr[i] == arr[i - 1])
        {
            currCount++;
        }
        else
        {
            if (currCount > maxCount)
            {
                maxCount = currCount;
                res = arr[i - 1];
            }
            currCount = 1;
        }
    }
    if (currCount > maxCount)
    {
        maxCount = currCount;
        res = arr[n - 1];
    }
    return res;
}
void StopBlink()
{
    // Stop blink functionality
}
void blinkf(int aant, int draan, int druit, int wacht)
{
    druit = draan;
    delay(wacht);
}
void resetInputsToDefault() {
    Start = 0;
    Man = 0;
    Stop = 0;
    P_Water = 0;
    E_Stop = 1;
    P_Vacuum = 0;
    P_Air = 0;
    L_Tank = 4095; // Assuming this is the default for the analog sensor
}
unsigned long lastDebugSendTime = 0;
const unsigned long debugSendInterval = 500; // Interval between debug messages in milliseconds
int debugSendCount = 0;
const int maxDebugSendCount = 100; // Maximum number of debug messages to send
bool debugStringReady;
String debugStringToSend;
void sendDebugString() {
    unsigned long currentTime = millis();

    if (debugSendCount < maxDebugSendCount && currentTime - lastDebugSendTime >= debugSendInterval) {
        if (debugStringReady) {
            esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)debugStringToSend.c_str(), debugStringToSend.length());
            if (result == ESP_OK) {
                Serial.println(debugStringToSend);
            } else {
                Serial.println("Error sending debug string");
            }
            debugStringReady = false;
            lastDebugSendTime = currentTime;
            debugSendCount++;
        }
    }
}
void readall() {
    const int numReadings = 5;
    const int delayInterval = 10;
    char serialBuffer[512]; // Increased buffer size to 512
    bool dataReceived = false;
    unsigned long startTime = millis();
    // E-stop override
    E_Stop = 1;

    // Check for incoming serial data within a 100ms window
    while (millis() - startTime < 100) {
        if (Serial.available() > 0) {
            Serial.readBytesUntil('\n', serialBuffer, sizeof(serialBuffer) - 1);
            dataReceived = true;
            break;
        }
    }

    // Process received data if available
    if (dataReceived) {
        String receivedData = String(serialBuffer);
        if (receivedData == "REBOOT") {
            ESP.restart();
        }
        if (receivedData.startsWith("||") && receivedData.endsWith("||")) {
            receivedData = receivedData.substring(2, receivedData.length() - 4);
            debugStringToSend = "||Debugbuffer:|" + receivedData + "||";
            debugStringReady = true;
        }

        // Process the rest of the received data
        int numKeyValuePairs = 0;
        String keyValuePairs[20];
        int startIndex = 0;
        int endIndex = receivedData.indexOf('|');

        while (endIndex != -1) {
            keyValuePairs[numKeyValuePairs++] = receivedData.substring(startIndex, endIndex);
            startIndex = endIndex + 1;
            endIndex = receivedData.indexOf('|', startIndex);
        }

        for (int i = 0; i < numKeyValuePairs; i++) {
            int colonIndex = keyValuePairs[i].indexOf(':');
            if (colonIndex != -1) {
                String key = keyValuePairs[i].substring(0, colonIndex);
                String value = keyValuePairs[i].substring(colonIndex + 1);

                if (key == "Simulation_On") {
                    Simulation_On = value.toInt();
                } else if (key == "I22Start") {
                    Start = value.toInt();
                } else if (key == "I21Man") {
                    Man = value.toInt();
                } else if (key == "I33Stop") {
                    Stop = value.toInt();
                } else if (key == "I35P_Water") {
                    P_Water = value.toInt();
                } else if (key == "I32E_Stop") {
                    E_Stop = value.toInt();
                } else if (key == "I34P_Vacuum") {
                    P_Vacuum = value.toInt();
                } else if (key == "I39P_Air") {
                    P_Air = value.toInt();
                } else if (key == "I36L_Tank") {
                    L_Tank = value.toInt();
                }
            }
        }
    }
    E_Stop = 1;
    if (!Simulation_On) {
        // Actual reading from sensors
        int startReadings[numReadings];
        int manReadings[numReadings];
        int stopReadings[numReadings];
        int P_VacuumReadings[numReadings];
        int P_WaterReadings[numReadings];
        int P_AirReadings[numReadings];
        int E_StopReadings[numReadings];
        int L_TankReadings[numReadings];

        for (int i = 0; i < numReadings; i++) {
            startReadings[i] = digitalRead(I22Start);
            manReadings[i] = digitalRead(I21Man);
            stopReadings[i] = digitalRead(I33Stop);
            P_VacuumReadings[i] = analogRead(I34P_Vacuum);
            P_WaterReadings[i] = analogRead(I35P_Water);
            P_AirReadings[i] = analogRead(I39P_Air);
            L_TankReadings[i] = analogRead(I36L_Tank);
            E_StopReadings[i] = digitalRead(I32E_Stop);
            delay(delayInterval);
        }

        Start = mostFrequent(startReadings, numReadings);
        Man = mostFrequent(manReadings, numReadings);
        Stop = mostFrequent(stopReadings, numReadings);
        P_Vacuum = median(P_VacuumReadings, numReadings);
        P_Water = median(P_WaterReadings, numReadings);
        P_Air = median(P_AirReadings, numReadings);
        E_Stop = mostFrequent(E_StopReadings, numReadings);
        L_Tank = median(L_TankReadings, numReadings);
    } else {
        // Simulation mode handling
        /* if (Start == 1) {
            Start = 0; // Touch button, goes LOW after HIGH
        }
        if (Stop == 1) {
            Stop = 0; // Touch button, goes LOW after HIGH
        } */
        if (E_Stop == 0 || E_Stop == 1) {
            E_Stop = 1; // Emergency button stays LOW
            // StopAll(); // Invoke emergency interrupt
        }
    }

    // Update states based on outputs
    if (digitalRead(O17Vid) == LOW) {
        setStatus(Vid_closed, true);
        setStatus(Vid_opened, false);
    } else {
        setStatus(Vid_closed, false);
        setStatus(Vid_opened, true);
    }

    if (digitalRead(O16Ac) == HIGH) {
        setStatus(Ac_closed, true);
        setStatus(Ac_opened, false);
    } else {
        setStatus(Ac_closed, false);
        setStatus(Ac_opened, true);
    }

    if ((digitalRead(O18Outlet_On_Off) == LOW) && (digitalRead(O19Outlet_change_dir) == LOW)) {
        setStatus(Outlet_closed, true);
        setStatus(Outlet_opened, false);
    } else if ((digitalRead(O18Outlet_On_Off) == LOW) && (digitalRead(O19Outlet_change_dir) == HIGH)) {
        setStatus(Outlet_closed, false);
        setStatus(Outlet_opened, true);
    }

    if (digitalRead(O25Clean_e1) == HIGH) {
        setStatus(Clean_E1_closed, true);
        setStatus(Clean_E1_opened, false);
    } else {
        setStatus(Clean_E1_closed, false);
        setStatus(Clean_E1_opened, true);
    }

    if (digitalRead(O27Clean_e2) == HIGH) {
        setStatus(Clean_E2_closed, true);
        setStatus(Clean_E2_opened, false);
    } else {
        setStatus(Clean_E2_closed, false);
        setStatus(Clean_E2_opened, true);
    }

    if (digitalRead(O26Clean_sortie) == HIGH) {
        setStatus(Clean_sortie_closed, true);
        setStatus(Clean_sortie_opened, false);
    } else {
        setStatus(Clean_sortie_closed, false);
        setStatus(Clean_sortie_opened, true);
    }

    if (digitalRead(O23Led) == HIGH) {
        setStatus(Led_Off, true);
        setStatus(Led_On, false);
    } else {
        setStatus(Led_Off, false);
        setStatus(Led_On, true);
    }

    if (digitalRead(O5Rp) == HIGH) {
        setStatus(Rp_closed, true);
        setStatus(Rp_opened, false);
    } else {
        setStatus(Rp_closed, false);
        setStatus(Rp_opened, true);
    }

    if (digitalRead(O21Rd) == HIGH) {
        setStatus(Rd_closed, true);
        setStatus(Rd_opened, false);
    } else {
        setStatus(Rd_closed, false);
        setStatus(Rd_opened, true);
    }

    // Build the output string
    outputString = "||Simulation_On:" + String(Simulation_On ? 1 : 0) + "|";
    outputString += "I22Start:" + String(Start) + "|";
    outputString += "I21Man:" + String(Man) + "|";
    outputString += "I33Stop:" + String(Stop) + "|";
    outputString += "I35P_Water:" + String(P_Water) + "|";
    outputString += "I32E_Stop:" + String(E_Stop) + "|";
    outputString += "I34P_Vacuum:" + String(P_Vacuum) + "|";
    outputString += "I39P_Air:" + String(P_Air) + "|";
    outputString += "I36L_Tank:" + String(L_Tank) + "|";
    outputString += "O17Vid:" + String(digitalRead(O17Vid) ? "OPEN" : "CLOSED") + "|";
    outputString += "O16Ac:" + String(digitalRead(O16Ac) ? "CLOSED" : "OPEN") + "|";
    outputString += "O19Outlet_change_dir:" + String(digitalRead(O19Outlet_change_dir) ? "CLOSING_DIR" : "OPENING_DIR") + "|";
    outputString += "O18Outlet_On_Off:" + String(digitalRead(O18Outlet_On_Off) ? "OFF" : "ON") + "|";
    outputString += "O23Led:" + String(digitalRead(O23Led) ? "OFF" : "ON") + "|";
    outputString += "O25Clean_e1:" + String(digitalRead(O25Clean_e1) ? "CLOSED" : "OPEN") + "|";
    outputString += "O26Clean_sortie:" + String(digitalRead(O26Clean_sortie) ? "CLOSED" : "OPEN") + "|";
    outputString += "O27Clean_e2:" + String(digitalRead(O27Clean_e2) ? "CLOSED" : "OPEN") + "|";
    outputString += "O5Rp:" + String(digitalRead(O5Rp) ? "CLOSED" : "OPEN") + "|";
    outputString += "O21Rd:" + String(digitalRead(O21Rd) ? "CLOSED" : "OPEN") + "|";
    outputString += Hmi_msg + "|";
    outputString += "RequestUpdate||";

    Serial.println(outputString);

    // Delay to ensure PC processes the output string
    delay(100);

    // Now send the debug string
    //sendDebugString();
}
void sendData() {
    // Print data to be sent for debugging
    Hmi_data = outputString;

    // Ensure data length does not exceed ESP-NOW limits
    const size_t maxDataLength = 250; // Maximum payload size for ESP-NOW
    if (Hmi_data.length() > maxDataLength) {
        // Split the data into smaller chunks
        size_t pos = 0;
        while (pos < Hmi_data.length()) {
            String chunk = Hmi_data.substring(pos, pos + maxDataLength);
            esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)chunk.c_str(), chunk.length());
            if (result != ESP_OK) {
                Serial.printf("Error sending ESP-NOW data chunk: %d\n", result);
            }
            pos += maxDataLength;
        }
    } else {
        esp_err_t result = esp_now_send(slaveAddress, (uint8_t *)Hmi_data.c_str(), Hmi_data.length());
        if (result != ESP_OK) {
            Serial.printf("Error sending ESP-NOW data: %d\n", result);
        }
    }

    // Wait for response with timeout
    responseReceived = false; // Reset the response flag
    unsigned long startTime = millis();
    unsigned long timeout = 5000; // 5 seconds timeout

    while (!responseReceived && (millis() - startTime) < timeout) {
        // Do nothing, just wait for the response or timeout
        delay(10); // Small delay to avoid busy waiting
    }

    if (!responseReceived) {
        // If no response is received within the timeout period, invoke StopAll
        StopAll();
    }
    delay(10);
}

void CleanEntr()
{
    digitalWrite(O25Clean_e1, LOW);
    digitalWrite(O27Clean_e2, LOW);
}
void Stopclean()
{
    digitalWrite(O25Clean_e1, HIGH);
    digitalWrite(O27Clean_e2, HIGH);
    digitalWrite(O26Clean_sortie, HIGH);
    digitalWrite(O21Rd, HIGH);
    digitalWrite(O5Rp, HIGH);
}
void StopNormal()
{
    Vacuum_starting = false;
    Man_started = false;
    stopping = true;
    actionTimers[Vac_stopping_Emptying_Tank] = millis();
    stopping = true;
    if (L_Tank <= 20)
    {
        Time_Empty = 17000;
    }
    if (L_Tank >= 200)
    {
        Time_Empty = 60000;
    }
    if (L_Tank > 20 && L_Tank < 200)
    {
        Time_Empty = 60000 * (L_Tank / 200);
    }
    digitalWrite(O27Clean_e2, LOW);
    digitalWrite(O25Clean_e1, LOW);

    digitalWrite(O16Ac, HIGH);
    digitalWrite(O17Vid, HIGH);
    digitalWrite(O19Outlet_change_dir, LOW); // clapet open
    digitalWrite(O18Outlet_On_Off, LOW);
    digitalWrite(O23Led, LOW);
    digitalWrite(O25Clean_e1, HIGH);
    digitalWrite(O26Clean_sortie, HIGH);
    digitalWrite(O27Clean_e2, HIGH);
    digitalWrite(O5Rp, HIGH);
    digitalWrite(O21Rd, HIGH);
    delay(1000);
}
void IRAM_ATTR normalStop()
{
    StopAll(); // Stop all operations
}
void IRAM_ATTR emergencyStop()
{
    StopAll(); // Stop all operations
}
void handleVac_stopping_Opening_Outlet()
{
    Hmi_msg = "Vac_stopping:Opening_Outlet";
    // Handle Vac_stopping_Opening_Outlet logic
}
void handleVac_stopping_Emptying_Tank()
{
    Hmi_msg = "Vac_stopping:Emptying_Tank";
    // Handle Vac_stopping_Emptying_Tank logic
}
void handleVac_stopping_Cleaning_Tank()
{
    Hmi_msg = "Vac_stopping:Cleaning_Tank";
    // Handle Vac_stopping_Cleaning_Tank logic
}
void handleVac_starting_Cleaning_Tank()
{
    Hmi_msg = "Vac_starting:Cleaning_Tank";
    // Handle Vac_starting_Cleaning_Tank logic
}
void handleVac_busy()
{
    Hmi_msg = "Vac_busy";
    // Handle Vac_busy logic
}
void handleClean_busy()
{
    Hmi_msg = "Clean_busy";
    // Handle Clean_busy logic
}
void handleVac_stopping()
{
    Hmi_msg = "Vac_stopping";
    if (actions[Vac_stopping_Opening_Outlet])
    {
        if (millis() - actionTimers[Vac_stopping_Opening_Outlet] > maxTimes[Vac_stopping_Opening_Outlet])
        {
            // Complete action and move to next
            actions[Vac_stopping_Opening_Outlet] = false;
            actions[Vac_stopping_Emptying_Tank] = true;
            actionTimers[Vac_stopping_Emptying_Tank] = millis();
        }
        handleVac_stopping_Opening_Outlet();
    }
    else if (actions[Vac_stopping_Emptying_Tank])
    {
        if (millis() - actionTimers[Vac_stopping_Emptying_Tank] > maxTimes[Vac_stopping_Emptying_Tank])
        {
            // Complete action and move to next
            actions[Vac_stopping_Emptying_Tank] = false;
            actions[Vac_stopping_Cleaning_Tank] = true;
            actionTimers[Vac_stopping_Cleaning_Tank] = millis();
        }
        handleVac_stopping_Emptying_Tank();
    }
    else if (actions[Vac_stopping_Cleaning_Tank])
    {
        if (millis() - actionTimers[Vac_stopping_Cleaning_Tank] > maxTimes[Vac_stopping_Cleaning_Tank])
        {
            // Complete action and move to next
            actions[Vac_stopping_Cleaning_Tank] = false;
        }
        handleVac_stopping_Cleaning_Tank();
    }
}
void handleVac_starting()
{
    Hmi_msg = "Vac_starting";
    if (getStatus(Outlet_closed) == false)
    {
        currentAction = Vac_starting_Closing_Outlet;
        actionTimers[Vac_starting_Closing_Outlet] = millis();
        maxTimes[Vac_starting_Closing_Outlet] = 2000;
        return;
    }
}
void handleVac_starting_Closing_Outlet()
{
    Hmi_msg = "Vac_starting:Closing_Outlet";
    digitalWrite(O25Clean_e1, LOW);
    digitalWrite(O26Clean_sortie, LOW);
    digitalWrite(O27Clean_e2, LOW);

    CloseOutlet();
    if ((millis() - actionTimers[Vac_starting_Closing_Outlet]) > maxTimes[Vac_starting_Closing_Outlet])
    {
        actions[Vac_starting_Closing_Outlet] = false;
        CloseVid();
        Openac();
        actions[Vac_starting_wait_vacuum] = true;
        actionTimers[Vac_starting_wait_vacuum] = millis();
        currentAction = Vac_starting_wait_vacuum;
    }
}
void HandleVacTimeout()
{
    // Handle vacuum timeout logic
    // For now, this function is empty
}
void handleVac_starting_wait_vacuum()
{
    Hmi_msg = "Vac_starting:wait_vacuum";
    if (analogRead(I34P_Vacuum) >= 700)
    {
        actions[Vac_starting_wait_vacuum] = false;
        actions[Vac_busy] = true;
        actionTimers[Vac_busy] = millis();
        currentAction = Vac_busy;
    }
    else if ((millis() - actionTimers[Vac_starting_wait_vacuum]) > maxTimes[Vac_starting_wait_vacuum])
    {
        HandleVacTimeout();
    }
}
void handleManual()
{
    Hmi_msg = "Manual action asked";
    actionTimers[Manual] = millis();
    maxTimes[Vac_busy_Max_Time] = 180000;

    if (Start == 1 && !Vac_starting)
    {
        actions[Vac_starting] = true;
        currentAction = Vac_starting;
        actionTimers[Vac_starting] = millis();
        // Ensure that Start is handled properly
        Start = 0;
    }

    if ((millis() - actionTimers[Manual]) > maxTimes[Vac_busy_Max_Time])
    {
        StopNormal();
    }

    if ((P_Vacuum <= 200 && actions[Vac_busy]) || L_Tank >= 200)
    {
        Man_started = false;
        StopNormal();
    }

    if (P_Vacuum <= 300 && !Man_started)
    {
        digitalWrite(O17Vid, HIGH);
        Man_started = true;
        Hmi_msg = "Manual action started";
    }
}
void handleLess_Grain()
{
    Hmi_msg = "Less_Grain";
    // Code to start Less_Grain
}
void handleMore_Grain()
{
    Hmi_msg = "More_Grain";
    // Code to start More_Grain
}
void handleSludge()
{
    Hmi_msg = "Sludge";
    // Code to start Sludge
}
void handleGRAV()
{
    Hmi_msg = "GRAV";
    // Code to start GRAV
}
void StopWrk()
{
    // Code to stop work
}

void checkNewActions()
{
    // Check and start new actions if needed

    if (!actions[Vac_starting])
    {
        actions[Vac_starting] = true;
        actionTimers[Vac_starting] = millis();
    }
    // Check other conditions and start corresponding actions
}
void checkNewTasks()
{
    // Check and start new tasks based on input conditions

    // Task: Manual Mode
    if (Man==1)
    {
        currentTask = Manual;
    }

    // Task: Standby Mode
    if (Stop == 1)
    {
        currentTask = Standby;
        StopNormal();
    }

    // Add conditions for other tasks as needed
}
// Function to handle task switching
void handleTasks()
{
    switch (currentTask)
    {
    case Standby:
        // Handle Standby logic
        Hmi_msg = "Standby";
        break;
    case Manual:
        handleManual();
        break;
    case Less_Grain:
        handleLess_Grain();
        break;
    case More_Grain:
        handleMore_Grain();
        break;
    case Sludge:
        handleSludge();
        break;
    case Gravel:
        handleGRAV();
        break;
    case Clean:
        // Handle Clean logic
        Hmi_msg = "Clean";
        break;
    }
}
void handleActions()
{
    switch (currentAction)
    {
    case Vac_stopping:
        handleVac_stopping();
        break;
    case Vac_stopping_Opening_Outlet:
        handleVac_stopping_Opening_Outlet();
        break;
    case Vac_stopping_Emptying_Tank:
        handleVac_stopping_Emptying_Tank();
        break;
    case Vac_stopping_Cleaning_Tank:
        handleVac_stopping_Cleaning_Tank();
        break;
    case Vac_starting:
        handleVac_starting();
        break;
    case Vac_starting_Closing_Outlet:
        handleVac_starting_Closing_Outlet();
        break;
    case Vac_starting_Cleaning_Tank:
        handleVac_starting_Cleaning_Tank();
        break;
    case Vac_starting_wait_vacuum:
        handleVac_starting_wait_vacuum();
        break;
    case Vac_busy:
        handleVac_busy();
        break;
    case Clean_busy:
        handleClean_busy();
        break;
    case None:
        // Handle None or idle logic if needed
        Hmi_msg = "None";
        break;
    }
}
void Handle_Error_not_enough_water()
{
    Hmi_msg = "NOT ENOUGH WATER";
    esp_now_send(slaveAddress, (uint8_t *)Hmi_msg.c_str(), Hmi_msg.length());
    // Additional logic for handling the error if needed
}
void Handle_Error_not_enough_air()
{
    Hmi_msg = "NOT ENOUGH AIR";
    esp_now_send(slaveAddress, (uint8_t *)Hmi_msg.c_str(), Hmi_msg.length());
    // Additional logic for handling the error if needed
}
void setup()
{
    // Initialize pins
    pinMode(I22Start, INPUT);
    pinMode(I21Man, INPUT);
    pinMode(I33Stop, INPUT);
    pinMode(I35P_Water, INPUT);
    pinMode(I32E_Stop, INPUT);
    pinMode(I34P_Vacuum, INPUT);
    pinMode(I39P_Air, INPUT);
    pinMode(I36L_Tank, INPUT);
    pinMode(O17Vid, OUTPUT);
    pinMode(O16Ac, OUTPUT);
    pinMode(O19Outlet_change_dir, OUTPUT);
    pinMode(O18Outlet_On_Off, OUTPUT);
    pinMode(O23Led, OUTPUT);
    pinMode(O25Clean_e1, OUTPUT);
    pinMode(O26Clean_sortie, OUTPUT);
    pinMode(O27Clean_e2, OUTPUT);
    pinMode(O5Rp, OUTPUT);
    pinMode(O21Rd, OUTPUT);

    // Initialize serial communication
    Serial.begin(115200);

    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        return;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t masterInfo = {};
    memcpy(masterInfo.peer_addr, masterAddress, 6);
    masterInfo.channel = 0;
    masterInfo.encrypt = false;

    if (esp_now_add_peer(&masterInfo) != ESP_OK)
    {
        return;
    }

    esp_now_peer_info_t slaveInfo = {};
    memcpy(slaveInfo.peer_addr, slaveAddress, 6);
    slaveInfo.channel = 0;
    slaveInfo.encrypt = false;

    if (esp_now_add_peer(&slaveInfo) != ESP_OK)
    {
        return;
    }

    // attachInterrupt(digitalPinToInterrupt(I33Stop), normalStop, RISING);
    //attachInterrupt(digitalPinToInterrupt(I32E_Stop), emergencyStop, RISING);

    setStatus(Vac_stopped, true);
    setStatus(Vid_closed, false);
    setStatus(Vid_opened, true);
    setStatus(Ac_closed, true);
    setStatus(Ac_opened, false);
    setStatus(Outlet_closed, false);
    setStatus(Outlet_opened, true);
    setStatus(Clean_E1_opened, false);
    setStatus(Clean_E1_closed, true);
    setStatus(Clean_E2_opened, false);
    setStatus(Clean_E2_closed, true);
    setStatus(Clean_sortie_opened, false);
    setStatus(Clean_sortie_closed, true);
    setStatus(Led_On, false);
    setStatus(Led_Off, true);
    setStatus(Rp_opened, false);
    setStatus(Rp_closed, true);
    setStatus(Rd_opened, false);
    setStatus(Rd_closed, true);
}
void loop() {
    readall();

    checkNewTasks(); // this has priority since through the readall(), it could be that a new task is called, halting eventual actions which are busy.
    // Check if any new actions need to be started
    checkNewActions();
    // Handle actions and tasks based on priority
    if (actions[Vac_stopping] && (millis() - actionTimers[Vac_stopping] < maxTimes[Vac_stopping])) {
        // Handle Vac_stopping
        handleVac_stopping();
    } else if (actions[Vac_starting] && (millis() - actionTimers[Vac_starting] < maxTimes[Vac_starting])) {
        // Handle Vac_starting
        handleVac_starting();
    } else {
        // Handle other tasks and actions based on priority
        if (stopping) {
            handleVac_stopping();
        } else if (actions[Vac_starting]) {
            handleVac_starting();
        } else {
            // Handle active tasks (Less_Grain, More_Grain, Sludge, etc.)
            handleTasks();
        }
    }

    sendData();

    if (E_Stop == 0) {
        StopAll();
    
    }

    // Continuous monitoring of I39P_Air and I35P_Water
if (Stop==1){StopAll();}
    if (P_Air < 600) {
        if (airPressureTimer == 0) {
            airPressureTimer = millis();
        } else if ((millis() - airPressureTimer) > airTimeout) {
            Handle_Error_not_enough_air();
            return;
        }
    } else {
        airPressureTimer = 0; // Reset the timer if the air pressure is above the threshold
    }

    if (P_Water < 200) {
        if (waterPressureTimer == 0) {
            waterPressureTimer = millis();
        } else if ((millis() - waterPressureTimer) > waterTimeout) {
            Handle_Error_not_enough_water();
            return;
        }
    } else {
        waterPressureTimer = 0; // Reset the timer if the water pressure is above the threshold
    }
}
