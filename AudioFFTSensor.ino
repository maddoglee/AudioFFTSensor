#include <Arduino.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include "secrets.h"
//#include "mqtt_discovery.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>  //Used for HTTP callback to enable/disable discovery

//#define MQTT_BROKER     "homeassistant.local"
//#define MQTT_PORT       1883
//#define MQTT_USERNAME   "mqtt_user"
//#define MQTT_PASSWORD   "mqttPassword"

#define MQTT_TOPIC    "AudioSensor"

// size of noise sample
#define SAMPLES 1024

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = SAMPLES;

#define OCTAVES 9

// our FFT data
static float real[SAMPLES];
static float imag[SAMPLES];
static arduinoFFT fft(real, imag, SAMPLES, SAMPLES);
static float energy[OCTAVES];
// A-weighting curve from 31.5 Hz ... 8000 Hz
static const float aweighting[] = {-39.4, -26.2, -16.1, -8.6, -3.2, 0.0, 1.2, 1.0, -1.1};

static unsigned int washingMachine = 0;
static unsigned int dishWasher = 0;
static unsigned int fireAlarm = 0;
static unsigned long ts = millis();
static unsigned long last = micros();
static unsigned int sum = 0;
static unsigned int mn = 9999;
static unsigned int mx = 0;
static unsigned int cnt = 0;
static unsigned long lastTrigger[2] = {0, 0};

//Auto-discover enable/disable option
bool auto_discovery = false;      //default to false and provide end-user interface to allow toggling 

//Variables for creating unique entity IDs and topics
byte macAddr[6];                 //Device MAC address
char uidPrefix[] = "Audio";      //Prefix for unique ID generation (limit to 20 chars)
char devUniqueID[30];             //Generated Unique ID for this device (uidPrefix + last 6 MAC characters) 

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
WebServer server(80);


// Handle root URL
void handleRoot() {
  server.send(200, "text/html", "<h1>Welcome to the ESP32 Web Server!</h1>");
}

static void integerToFloat(int32_t *integer, float *vReal, float *vImag, uint16_t samples)
{
    for (uint16_t i = 0; i < samples; i++)
    {
        vReal[i] = (integer[i] >> 16) / 10.0;
        vImag[i] = 0.0;
    }
}

// calculates energy from Re and Im parts and places it back in the Re part (Im part is zeroed)
static void calculateEnergy(float *vReal, float *vImag, uint16_t samples)
{
    for (uint16_t i = 0; i < samples; i++)
    {
        vReal[i] = sq(vReal[i]) + sq(vImag[i]);
        vImag[i] = 0.0;
    }
}

// sums up energy in bins per octave
static void sumEnergy(const float *bins, float *energies, int bin_size, int num_octaves)
{
    // skip the first bin
    int bin = bin_size;
    for (int octave = 0; octave < num_octaves; octave++)
    {
        float sum = 0.0;
        for (int i = 0; i < bin_size; i++)
        {
            sum += real[bin++];
        }
        energies[octave] = sum;
        bin_size *= 2;
    }
}

static float decibel(float v)
{
    return 10.0 * log(v) / log(10);
}

// converts energy to logaritmic, returns A-weighted sum
static float calculateLoudness(float *energies, const float *weights, int num_octaves, float scale)
{
    float sum = 0.0;
    for (int i = 0; i < num_octaves; i++)
    {
        float energy = scale * energies[i];
        sum += energy * pow(10, weights[i] / 10.0);
        energies[i] = decibel(energy);
    }
    return decibel(sum);
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Configuring I2S...");
    esp_err_t err;

    // The I2S config as per the example
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
        .sample_rate = 22627,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // for old esp-idf versions use RIGHT
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
        .dma_buf_count = 8,                       // number of buffers
        .dma_buf_len = BLOCK_SIZE,                // samples per buffer
        .use_apll = true};

    // The pin config as per the setup
    const i2s_pin_config_t pin_config = {
        .bck_io_num = 14,   // BCKL
        .ws_io_num = 15,    // LRCL
        .data_out_num = -1, // not used (only for speakers)
        .data_in_num = 32   // DOUTESP32-INMP441 wiring
    };

    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK)
    {
        Serial.printf("Failed installing driver: %d\n", err);
        while (true)
            ;
    }
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK)
    {
        Serial.printf("Failed setting pin: %d\n", err);
        while (true)
            ;
    }
    Serial.println("I2S driver installed.");

     // Connect to Wi-Fi
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait for the connection to establish
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }

    // Print the IP address
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Connect to MQTT 

    mqttClient.setId("test123");

    mqttClient.setUsernamePassword(MQTT_USERNAME, MQTT_PASSWORD);

    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(MQTT_BROKER);

    while (!mqttClient.connect(MQTT_BROKER, MQTT_PORT)) {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
      delay(100);
    }

    Serial.println("You're connected to the MQTT broker!");
    Serial.println();

    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);

    Serial.print("Subscribing to topic: ");
    Serial.println(MQTT_TOPIC);
    Serial.println();

    // subscribe to a topic
    mqttClient.subscribe(MQTT_TOPIC);

    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(topic);

    Serial.print("Waiting for messages on topic: ");
    Serial.println(MQTT_TOPIC);
    Serial.println();

      // The following should be included in (or called from) the main setup routine

    //Get MAC address when joining wifi and place into char array
    WiFi.macAddress(macAddr);  
    
    //Call routing (or embed here) to create initial Unique ID
    createDiscoveryUniqueID();

    // Define routing
    server.on("/", handleRoot);
  
    //Handle web callbacks for enabling or disabling discovery (using this method is just one of many ways to do this)
    server.on("/discovery_on",[]() {
      server.send(200, "text/html", "<h1>Discovery ON...<h1><h3>Home Assistant MQTT Discovery enabled</h3>");
      delay(200);
      auto_discovery = true;
      haDiscovery();
      Serial.print("Discovery_on");
    });
    server.on("/discovery_off",[]() {
      server.send(200, "text/html", "<h1>Discovery OFF...<h1><h3>Home Assistant MQTT Discovery disabled. Previous entities removed.</h3>");
      delay(200);
      auto_discovery = false;
      haDiscovery();
      Serial.print("Discovery_off");
    });
    server.begin();
    Serial.print("WebServer Started");
}

unsigned int countSetBits(unsigned int n)
{
    unsigned int count = 0;
    while (n)
    {
        count += n & 1;
        n >>= 1;
    }
    return count;
}

//detecting 2 frequencies. Set wide to true to match the previous and next bin as well
bool detectFrequency(unsigned int *mem, unsigned int minMatch, double peak, unsigned int bin1, unsigned int bin2, bool wide)
{
    *mem = *mem << 1;
    if (peak == bin1 || peak == bin2 || (wide && (peak == bin1 + 1 || peak == bin1 - 1 || peak == bin2 + 1 || peak == bin2 - 1)))
    {
        *mem |= 1;
    }

    if (countSetBits(*mem) >= minMatch)
    {
        return true;
    }
    //print crap to screen
    //Serial.println(countSetBits(*mem));

    return false;
}

void sendAlarm(unsigned int index, char *topic, unsigned int timeout)
{
    
    
    // do not publish if last trigger was earlier than timeout ms
    if (labs(millis() - lastTrigger[index]) < timeout)
    {
        return;
    }

    lastTrigger[index] = millis();
    //publish to mqtt
    //publish(topic, "1");
    Serial.println(topic);
    mqttClient.beginMessage(topic);
    //mqttClient.print(topic);
    mqttClient.print("ON");
    mqttClient.endMessage();
    Serial.println("MQTT message sent");
}

void sendMetrics(char * topic, unsigned int mn, unsigned int mx, unsigned int avg)
{
    String payload = "{\"min\": ";
    payload += mn;
    payload += ", \"max\":";
    payload += mx;
    payload += ", \"value\":";
    payload += avg;
    payload += "}";

    Serial.println(payload);
    //publish to mqtt
    //publish(topic, (char *)payload.c_str());
    
}

void calculateMetrics(int val) 
{
  cnt++;
  sum += val;

  if (val > mx)
  {
      mx = val;
  }

  if (val < mn)
  {
      mn = val;
  }
}

// MQTT messaging function
void onMqttMessage(int messageSize) 
{
    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.println("'");
    Serial.print("Message length: ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    // Read the message contents into a String
    String message = mqttClient.readString();
    
    Serial.print("Message to my Arduino: ");
    Serial.println(message);
}

// Variable for MQTT loop in main loop
int g_loop = 0;

void loop(void)
{
    if (micros() - last < 45200) {
      // send mqtt metrics every 10s while waiting and no trigger is detected
      if (millis() - ts >= 10000 && washingMachine == 0 && fireAlarm == 0 && dishWasher == 0)
      {
          //Serial.println(cnt[0]);
          sendMetrics("home/noise", mn, mx, sum / cnt);
          cnt = 0;
          sum = 0;
          mn = 9999;
          mx = 0;
          ts = millis();
      }
      return;
    }

    last = micros();
    
    static int32_t samples[BLOCK_SIZE];

    // Read multiple samples at once and calculate the sound pressure
    size_t num_bytes_read;
    esp_err_t err = i2s_read(I2S_PORT,
                             (char *)samples,
                             BLOCK_SIZE, // the doc says bytes, but its elements.
                             &num_bytes_read,
                             portMAX_DELAY); // no timeout
    int samples_read = num_bytes_read / 8;

    // integer to float
    integerToFloat(samples, real, imag, SAMPLES);

    // apply flat top window, optimal for energy calculations
    fft.Windowing(FFT_WIN_TYP_FLT_TOP, FFT_FORWARD);
    fft.Compute(FFT_FORWARD);

    // calculate energy in each bin
    calculateEnergy(real, imag, SAMPLES);

    // sum up energy in bin for each octave
    sumEnergy(real, energy, 1, OCTAVES);

    // calculate loudness per octave + A weighted loudness
    float loudness = calculateLoudness(energy, aweighting, OCTAVES, 1.0);

    unsigned int peak = (int)floor(fft.MajorPeak());
    //Serial.println(peak);
    
    // detecting frequencies around 2550Hz, 1628Hz (Washine Machine)
    if (detectFrequency(&washingMachine, 7, peak, 116, 175, true))
    {
        Serial.println("Detected Washing Machine");
        sendAlarm(0, "home/alarm/washingmachine", 10000);
    }

    //detecting frequencies around 3775Hz, 6767Hz (Smoke Alarm)
    if (detectFrequency(&fireAlarm, 5, peak, 153, 164, true))
    {
        Serial.println("Detected fire alarm");
        sendAlarm(1, "home/alarm/fire", 10000);
    }

        //detecting frequencies around 2kHz, 3kHz (Dishwasher)
    if (detectFrequency(&dishWasher, 5, peak, 229, 91, true))
    {
        Serial.println("Detected Dishwasher");
        sendAlarm(2, "home/alarm/dishwasher", 10000);
    }

    calculateMetrics(loudness);

    // call poll() regularly to allow the library to receive MQTT messages and
    // send MQTT keep alives which avoids being disconnected by the broker
    mqttClient.poll();

      // Handle client requests
    server.handleClient();

   // if(g_loop % 10 == 0) {
   //    mqttClient.beginMessage("AudioSensor");
   //     mqttClient.print("Time=");
   //     mqttClient.print(millis());
   //     mqttClient.endMessage();
   //     //Serial.println("MQTT Loop working");
   // }
   // delay(100);
   // g_loop++;
}




// =====================================
//  Create Unique ID for topics/entities
// =====================================
void createDiscoveryUniqueID() {
  //Generate UniqueID from uidPrefix + last 6 chars of device MAC address
  //This should insure that even multiple devices installed in same HA instance are unique
  
  strcpy(devUniqueID, uidPrefix);
  int preSizeBytes = sizeof(uidPrefix);
  int preSizeElements = (sizeof(uidPrefix) / sizeof(uidPrefix[0]));
  //Now add last 6 chars from MAC address (these are 'reversed' in the array)
  int j = 0;
  for (int i = 2; i >= 0; i--) {
    sprintf(&devUniqueID[(preSizeBytes - 1) + (j)], "%02X", macAddr[i]);   //preSizeBytes indicates num of bytes in prefix - null terminator, plus 2 (j) bytes for each hex segment of MAC
    j = j + 2;
  }
  // End result is a unique ID for this device (e.g. rctdevE350CA) 
  Serial.print("Unique ID: ");
  Serial.println(devUniqueID);  
}
// ===============================
//  Main HA MQTT Discover Function
//    This creates a single fictional device with four entities:
//      - A dimmer switch with light level, temperature and IP address
// ===============================
void haDiscovery() {
  char topic[128];
  if (auto_discovery) {
    char buffer1[1024];
    char buffer2[1024];
    char buffer3[1024];
    char buffer4[1024];
    char uid[128];
    DynamicJsonDocument doc(1024);
    doc.clear();
    Serial.println("Discovering new devices...");
    Serial.println("Adding Audio Alarm Sensors...");

    //Create unique topic based on devUniqueID
    strcpy(topic, "homeassistant/switch/");
    strcat(topic, devUniqueID);
    strcat(topic, "S/config");

    //Create unique_id based on devUniqueID
    strcpy(uid, devUniqueID);
    strcat(uid, "S");

    //Create JSON payload per HA documentation
    doc["name"] = "Audio Alarm Switch";
    doc["obj_id"] = "mqtt_switch";
    doc["uniq_id"] = uid;
    doc["stat_t"] = "homeassistant/mydevice/state";
    doc["cmd_t"] = "homeassistant/mydevice/set";
//    doc["brightness"] = "true";
//    doc["bri_scl"] = "100";
//    doc["bri_stat_t"] = "stat/mydevice/brightness";
//    doc["bri_cmd_t"] = "cmnd/mydevice/brightness";
    JsonObject device = doc.createNestedObject("device");
    device["ids"] = "mymqttdevice01";
    device["name"] = "Audio Alarm Sensor";
    device["mf"] = "Xymed";
    device["mdl"] = "ESP32";
    //device["sw"] = "1.00";
    //device["hw"] = "1.00";
    //device["cu"] = "http://192.168.1.187/config";  //web interface for device, with discovery toggle
    serializeJson(doc, buffer1);
    //size_t n = serializeJson(doc, buffer1, sizeof(buffer1));
    //Publish discovery topic and payload (with retained flag)
    //client.publish(topic, buffer1, true); 
    //if (n > sizeof(buffer1)) {
    //  Serial.println("Warning: Buffer size is too small for the JSON payload.");
    //}

    // Print the serialized JSON for debugging
    //Serial.println("Serialized JSON:");
    //Serial.println(n);
    
    Serial.println(topic);
    Serial.println(buffer1);
    Serial.println(true);
    mqttClient.beginMessage(topic, true);
    mqttClient.print(buffer1);
    mqttClient.endMessage();

    //Washing Machine Sensor
    Serial.println("Adding Washing Machine sensor...");
    //Create unique Topic based on devUniqueID
    strcpy(topic, "homeassistant/binary_sensor/");
    strcat(topic, devUniqueID);
    strcat(topic, "L/config");
    //Create unique_id based on decUniqueID
    strcpy(uid, devUniqueID);
    strcat(uid, "L");
    //Create JSON payload per HA documentation
    doc.clear();
    doc["name"] = "Washing Machine Tone Detected";
    doc["obj_id"] = "mqtt_wm_sound";
    doc["dev_cla"] = "sound";
    doc["uniq_id"] = uid;
    doc["stat_t"] = "home/alarm/washingmachine";
	  doc["pl_on"] = "ON";
	  doc["pl_off"] = "OFF";
    JsonObject deviceS = doc.createNestedObject("device");
    deviceS["ids"] = "mymqttdevice01";
    deviceS["name"] = "Audio Alarm Sensor";
    serializeJson(doc, buffer2);
    //Publish discovery topic and payload (with retained flag)
    Serial.println(topic), buffer2;  
    mqttClient.beginMessage(topic, true);
    mqttClient.print(buffer2);
    mqttClient.endMessage();  
    
    //Temperature Sensor
    Serial.println("Adding Temp Sensor...");
    //Create unique Topic based on devUniqueID
    strcpy(topic, "homeassistant/sensor/");
    strcat(topic, devUniqueID);
    strcat(topic, "T/config");
    //Create unique_id based on decUniqueID
    strcpy(uid, devUniqueID);
    strcat(uid, "T");
    //Create JSON payload per HA documentation
    doc.clear();
    doc["name"] = "Temperature";
    doc["obj_id"] = "mqtt_temperature";
    doc["dev_cla"] = "temperature";
    doc["uniq_id"] = uid;
    doc["stat_t"] = "stat/mydevice/temperature";
    doc["unit_of_meas"] = "°F";
    JsonObject deviceT = doc.createNestedObject("device");
    deviceT["ids"] = "mymqttdevice01";
    deviceT["name"] = "Audio Alarm Sensor";
    serializeJson(doc, buffer3);
    //Publish discovery topic and payload (with retained flag)
    //client.publish(topic, buffer3, true);
    mqttClient.beginMessage(topic, true);
    //mqttClient.print(topic);
    mqttClient.print(buffer3);
    //mqttClient.print(true);
    mqttClient.endMessage();
    
    
    
    //IP Address Diagnostic
    Serial.println("Adding IP Diagnostic Sensor...");
    //Create unique Topic based on devUniqueID
    strcpy(topic, "homeassistant/sensor/");
    strcat(topic, devUniqueID);
    strcat(topic, "I/config");
    //Create unique_id based on decUniqueID
    strcpy(uid, devUniqueID);
    strcat(uid, "I");
    //Create JSON payload per HA documentation
    doc.clear();
    doc["name"] = "IP Address";
    doc["uniq_id"] = uid;
    doc["ent_cat"] = "diagnostic";
    doc["stat_t"] = "stat/mydevice/ipaddress";
    JsonObject deviceI = doc.createNestedObject("device");
    deviceI = doc.createNestedObject("device");
    deviceI["ids"] = "mymqttdevice01";
    deviceI["name"] = "Audio Alarm Sensor";
    serializeJson(doc, buffer4);
    //Publish discovery topic and payload (with retained flag)
    //client.publish(topic, buffer4, true);
    mqttClient.beginMessage(topic, true);
    //mqttClient.print(topic);
    mqttClient.print(buffer4);
    //mqttClient.print(true);
    mqttClient.endMessage();
    Serial.println("All devices added!");

  } else {

    //Remove all entities by publishing empty payloads
    //Must use original topic, so recreate from original Unique ID
    //This will immediately remove/delete the device/entities from HA
    Serial.println("Removing discovered devices...");

    //Lux Sensor
    strcpy(topic, "homeassistant/binary_sensor/");
    strcat(topic, devUniqueID);
    strcat(topic, "L/config");
    //client.publish(topic, "");
    mqttClient.beginMessage(topic);
    //mqttClient.print(topic);
    mqttClient.print("");
    mqttClient.endMessage();

    //Temperature Sensor
    strcpy(topic, "homeassistant/sensor/");
    strcat(topic, devUniqueID);
    strcat(topic, "T/config");
    //Client.publish(topic, "");
    mqttClient.beginMessage(topic);
    //mqttClient.print(topic);
    mqttClient.print("");
    mqttClient.endMessage();

    //IP Address Sensor
    strcpy(topic, "homeassistant/sensor/");
    strcat(topic, devUniqueID);
    strcat(topic, "I/config");
    //Client.publish(topic, "");
    mqttClient.beginMessage(topic);
    //mqttClient.print(topic);
    mqttClient.print("");
    mqttClient.endMessage();

    //Light (switch)
    strcpy(topic, "homeassistant/switch/");
    strcat(topic, devUniqueID);
    strcat(topic, "S/config");
    //client.publish(topic, "");
    mqttClient.beginMessage(topic);
    //mqttClient.print(topic);
    mqttClient.print("");
    mqttClient.endMessage();

    Serial.println("Devices Removed...");
  }
}