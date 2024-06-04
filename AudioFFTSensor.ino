#include <Arduino.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include "secrets.h"

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

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

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
    mqttClient.beginMessage("AudioSensor");
    mqttClient.print(topic);
    mqttClient.print("1");
    mqttClient.endMessage();
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
    if (detectFrequency(&washingMachine, 7, peak, 116, 74, true))
    {
        Serial.println("Detected Washing Machine");
        sendAlarm(0, "home/alarm/washingmachine", 2000);
    }

    //detecting frequencies around 3775Hz, 6767Hz (Smoke Alarm)
    if (detectFrequency(&fireAlarm, 5, peak, 154, 308, true))
    {
        Serial.println("Detected fire alarm");
        sendAlarm(1, "home/alarm/fire", 10000);
    }

        //detecting frequencies around 2kHz, 3kHz (Dishwasher)
    if (detectFrequency(&dishWasher, 5, peak, 227, 136, true))
    {
        Serial.println("Detected Dishwasher");
        sendAlarm(2, "home/alarm/dishwasher", 2000);
    }

    calculateMetrics(loudness);

    // call poll() regularly to allow the library to receive MQTT messages and
    // send MQTT keep alives which avoids being disconnected by the broker
    mqttClient.poll();

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