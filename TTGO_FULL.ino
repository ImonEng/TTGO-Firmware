#include <SoftwareSerial.h>
#include <ModbusRTU.h>
#define TINY_GSM_MODEM_SIM800
#include <Wire.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

#define MODEM_RST 5
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define LED_GPIO 13
#define LED_ON HIGH
#define LED_OFF LOW

#define I2C_SDA 21
#define I2C_SCL 22

// Power management settings for IP5306
#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

#define SerialMon Serial
#define SerialAT Serial1
// GPRS credentials
const char apn[] = "blweb";  // APN for your network
const char gprsUser[] = "";
const char gprsPass[] = "";
// Server details for your data
const char* serverName = "http://nodesapi.nodesdigitalbd.com:8083/api/data/pipeline/v1/kafka/post/task/devices";
float sensorValue3;
float temperature;
float phSensorValue;
float ionConcentrationValue;

// Setup TinyGSM and HttpClient
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
HttpClient http(client, "nodesapi.nodesdigitalbd.com", 8083);

bool setupPMU() {
  bool en = true;
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37);  // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35);  // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}
void setupModem() {
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, HIGH);

  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  // Turn on the modem power first
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Pull down PWRKEY for more than 1 second according to manual requirements
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(100);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, HIGH);

  // Initialize the indicator as an output
  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, LED_OFF);
}
// Modbus setup
ModbusRTU mb;
// #define RS485_DE_RE 4  // Control pin for DE/RE
#define RS485_DE_RE 32  // Control pin for DE/RE
float ecSensorValue;

// Define Slave IDs for different sensors
#define SLAVE_ID_DO      03      // Example Slave ID for DO Sensor
#define SLAVE_ID_EC      02       // Example Slave ID for EC Sensor
#define SLAVE_ID_PH      01       // Example Slave ID for pH Sensor
#define SLAVE_ID_AMMONIA 04  // Example Slave ID for Ion Concentration and Electrode Signal

// Use UART2 (GPIO16 as RX, GPIO17 as TX)
HardwareSerial rs485(2);

void preTransmission() {
  digitalWrite(RS485_DE_RE, HIGH);  // Enable transmit mode
}

void postTransmission() {
  digitalWrite(RS485_DE_RE, LOW);  // Enable receive mode
}

void setup() {
  Serial.begin(115200);  // Initialize Serial Monitor
 SerialMon.println("Wait...");
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  // Set up the modem
  setupModem();
  delay(6000);

   SerialMon.println("Initializing modem...");
  modem.restart();

  // Get modem info
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (modem.getSimStatus() != 3) {
    modem.simUnlock("");
  }
  // Initialize the RS485 serial port
  // rs485.begin(9600, SERIAL_8N1, 16, 17);
  rs485.begin(9600, SERIAL_8N1, 18, 19);
  mb.begin(&rs485, RS485_DE_RE);

  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);  // Set to receive mode by default
  mb.master();                     // Set ESP32 as Modbus master
}

void loop() {
  static unsigned long lastUpdateTime = 0;  // Stores the last update time
  // Check if 1 minute has passed since the last update
  if (millis() - lastUpdateTime >= 30000 || lastUpdateTime == 0) {
    
    readTemperature();
    readDOSensor();
    readECSensor();
    readPHSensor();
    readAmmoniaSensor();
    
    lastUpdateTime = millis();  // Update the last update time to current millis()
    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork()) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    SerialMon.println(" success");

    if (modem.isNetworkConnected()) {
      SerialMon.println("Network connected");
    }

    SerialMon.print(F("Connecting to "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    SerialMon.println(" success");

    if (modem.isGprsConnected()) {
      SerialMon.println("GPRS connected");

      // Get current time from the time API
      String timeHour = getTimeFromAPI();
      SerialMon.print("timeHour: ");
      SerialMon.println(timeHour);
      if (timeHour == "") {
        SerialMon.println("Failed to get time from API.");
        return;
      }
      // Create the POST data string
 String payload = "{\"device_id\":\"ImonNode #00000008F09TST06\","
                 "\"device_token\":\"00000008F09TST06\","
                 "\"ts\":" + String(timeHour) + ","
                 "\"temperature_scale\": null,"
                 "\"do_temperature\":" + String(temperature, 2) + ","
                 "\"ec\": null,"  // If you want to send null for ec
                 "\"ph\":" + String(phSensorValue, 2) + ","
                 "\"electrode_signal\": null,"  // Replace with actual electrode signal value if needed
                 "\"do_value\":" + String(sensorValue3, 2) + ","
                 "\"ph_temperature\": null,"
                 "\"lon_concentration\":" + String(ionConcentrationValue, 2) + ","
                 "\"battery_percentage\": null}";

// Debugging: Print the payload to check its correctness
SerialMon.println(payload);

      http.beginRequest();
      http.post("/api/data/pipeline/v1/kafka/post/task/devices");
      http.sendHeader("Content-Type", "application/json");
      http.sendHeader("Content-Length", payload.length());
      http.beginBody();
      http.print(payload);
      http.endRequest();

      // Get the HTTP response
      int statusCode = http.responseStatusCode();
      String response = http.responseBody();

      // Check the response status code
      if (statusCode == 200) {
        SerialMon.println("Data sent successfully.");
      } else {
        SerialMon.print("Failed to send data. HTTP error code: ");
        SerialMon.println(statusCode);
        SerialMon.println(response);
      }

      lastUpdateTime = millis();  // Update the last update time to current millis()
    }

    // Disconnect GPRS
    modem.gprsDisconnect();
    SerialMon.println(F("GPRS disconnected"));
  }
}

void readTemperature() {
  // Local swapBytes function
  auto swapBytes = [](uint16_t value) -> uint16_t {
    return (value >> 8) | (value << 8);
  };

  // Buffer to hold register values
  uint16_t resultBuffer[2];
  // Sending Modbus request
  uint16_t register_address = 0x2000;  // Start address for temperature data

  preTransmission();
  delay(100);
  postTransmission();

  // Read 2 registers (4 bytes) from the DO sensor
  if (mb.readHreg(SLAVE_ID_DO, register_address, resultBuffer, 2)) {
    // Wait for response
    while (mb.slave()) {
      mb.task();
    }

    // Swap the bytes within each 16-bit register and then combine
    uint32_t rawValue = ((uint32_t)swapBytes(resultBuffer[1]) << 16) | swapBytes(resultBuffer[0]);

    // Convert raw data to float
    float temperature;
    memcpy(&temperature, &rawValue, sizeof(temperature));

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
  } else {
    Serial.println("Error reading sensor: Timeout");
  }
}

void readDOSensor() {
  // Buffer to hold register values
  uint16_t resultBuffer[2];
  // Sending Modbus request
  uint16_t register_address = 0x2100;  // Start address DO
  // uint16_t register_address = 0x0000; // Start address NH4

  preTransmission();
  delay(100);
  postTransmission();

  // Read 2 registers (4 bytes) from the device
  if (mb.readHreg(SLAVE_ID_DO, register_address, resultBuffer, 2)) {
    // Wait for response
    while (mb.slave()) {
      mb.task();
    }

    // Attempt 3: Swap the bytes within each 16-bit register and then combine
    uint32_t rawData3 = ((uint32_t)((resultBuffer[1] >> 8) | (resultBuffer[1] << 8)) << 16) | ((resultBuffer[0] >> 8) | (resultBuffer[0] << 8));

    // Convert raw data to floats
    float sensorValue1, sensorValue2, sensorValue3;
    memcpy(&sensorValue3, &rawData3, sizeof(sensorValue3));

    Serial.print("DO Sensor Value: ");
    Serial.print(sensorValue3, 6);
    Serial.println(" mg/L");
  } else {
    Serial.println("Modbus read failed");
  }

  delay(2000);  // Delay before the next read
}

void readECSensor() {
  // Buffer to hold register values
  uint16_t resultBuffer[2];
  // Sending Modbus request
  uint16_t register_address = 0x2004;  // EC sensor start address (adjust if different)

  preTransmission();
  delay(100);
  postTransmission();

  // Read 2 registers (4 bytes) from the device
  if (mb.readHreg(SLAVE_ID_EC, register_address, resultBuffer, 6)) {
    // Wait for response
    while (mb.slave()) {
      mb.task();
    }

    // Swap the bytes within each 16-bit register and then combine
    uint32_t rawData = ((uint32_t)((resultBuffer[3] >> 8) | (resultBuffer[3] << 8)) << 16) | ((resultBuffer[5] >> 8) | (resultBuffer[5] << 8));

    // Convert raw data to float for EC sensor
    float ecSensorValue;
    memcpy(&ecSensorValue, &rawData, sizeof(ecSensorValue));

    Serial.print("EC Sensor Value: ");
    Serial.print(ecSensorValue, 6);
    Serial.println(" mS/cm");  // EC value in mS/cm
  } else {
    Serial.println("Modbus read failed");
  }

  delay(2000);  // Delay before the next read
}

void readPHSensor() {
  // Buffer to hold register values
  uint16_t resultBuffer[2];
  // Sending Modbus request
  uint16_t register_address = 0x0601;  // pH sensor start address (adjust if different) 0x0600

  preTransmission();
  delay(100);
  postTransmission();

  // Read 2 registers (4 bytes) from the device
  if (mb.readHreg(SLAVE_ID_PH, register_address, resultBuffer, 8)) {
    // Wait for response
    while (mb.slave()) {
      mb.task();
    }

    // Swap the bytes within each 16-bit register and then combine
    uint32_t rawData = ((uint32_t)((resultBuffer[1] >> 8) | (resultBuffer[1] << 8)) << 16) | ((resultBuffer[1] >> 8) | (resultBuffer[1] << 8));

    // Convert raw data to float for pH sensor
    float phSensorValue;
    memcpy(&phSensorValue, &rawData, sizeof(phSensorValue));

    Serial.print("pH Sensor Value: ");
    Serial.print(phSensorValue, 6);
    Serial.println(" pH");  // pH typically ranges from 0 to 14
  } else {
    Serial.println("Modbus read failed");
  }

  delay(2000);  // Delay before the next read
}


void readAmmoniaSensor() {
  // Buffer to hold register values
  uint16_t resultBuffer[4];
  uint16_t ion_concentration_address = 0x0000;  // Start address for Ion Concentration Value
                                                // uint16_t electrode_signal_address = 0x0002;   // Start address for Electrode Signal Value

  preTransmission();
  delay(100);
  postTransmission();

  if (mb.readHreg(SLAVE_ID_AMMONIA, ion_concentration_address, resultBuffer, 2)) {
    while (mb.slave()) {
      mb.task();
    }
    // Combine without swapping
    uint32_t ionConcentrationRaw = ((uint32_t)resultBuffer[1] << 16) | resultBuffer[0];

    // Convert to float
    float ionConcentrationValue;
    memcpy(&ionConcentrationValue, &ionConcentrationRaw, sizeof(ionConcentrationValue));

    Serial.print("Ammonia Sensor Value: ");
    Serial.print(ionConcentrationValue, 2);
    Serial.println(" ppm");
  } else {
    Serial.println("Failed to read Ion Concentration");
  }
  delay(2000);  
}

String getTimeFromAPI() {
  HttpClient timeClient(client, "worldtimeapi.org", 80);
  timeClient.get("/api/timezone/Asia/Dhaka"); // Bangladesh timezone
  int statusCode = timeClient.responseStatusCode();
  String response = timeClient.responseBody();

  if (statusCode == 200) {
  return parseTimeFromResponse(response);
  } else {
    SerialMon.print("Failed to get time. HTTP error code: ");
    SerialMon.println(statusCode);
    return "";
  }
  delay(2000);  // Delay before the next read
}
// Function to parse time from the API response
String parseTimeFromResponse(String response) {
  int unixTimeIndex = response.indexOf("\"unixtime\":") + 11;
  String unixTime = response.substring(unixTimeIndex, response.indexOf(',', unixTimeIndex));
  return unixTime;
}