#include <time.h>

// MPU - MUX -FFT
#include <arduinoFFT.h>
#include <Wire.h>
#include <MPU6050_light.h>

#define MUXADD 0x70
#define AMOSTRAS 512
#define FREQ_AMOSTRAGEM 50  // Hz
#define NUM_SENSORES 3

MPU6050 sensores[NUM_SENSORES] = { MPU6050(Wire), MPU6050(Wire), MPU6050(Wire) };

long timer = 0;

float acel[NUM_SENSORES][AMOSTRAS];
float acel_filtrada[NUM_SENSORES][AMOSTRAS];
float maiorPicoAceleracao[NUM_SENSORES] = { 0.0 };
double dados_reais[NUM_SENSORES][AMOSTRAS], dados_imag[NUM_SENSORES][AMOSTRAS], dados_reais_f[NUM_SENSORES][AMOSTRAS], dados_imag_f[NUM_SENSORES][AMOSTRAS];
float freq_fund;
float periodo = 1000.0 / FREQ_AMOSTRAGEM;
unsigned int periodo_amostragem_us, ind_freq_pico;

arduinoFFT FFT[NUM_SENSORES];
arduinoFFT FFT_F[NUM_SENSORES];

//strain gauge
#define amostras 2000
#define sensor 5 // Porta do microcontrolador a ser usada
#define E 3400 // Módulo de elasticidade do material em MPa
#define L 300 // Comprimento do braço em mm
#define b1 94 // Largura da barra em mm
#define h2 2 // Espessura da barra em mm

float zero = 0, strain = 0, def = 0, delta_def;
float calib = 0, w, m, p, e, sigma;

//Json
#include <ArduinoJson.h>

// C99 libraries
#include <cstdlib>
#include <string.h>
#include <time.h>

// Libraries for MQTT client and WiFi connection
#include <WiFi.h>
#include <mqtt_client.h>

// Azure IoT SDK for C includes
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>

// Additional sample headers
#include "AzIoTSasToken.h"
#include "SerialLogger.h"
#include "iot_configs.h"

// When developing for your own Arduino-based platform,
// please follow the format '(ard;<platform>)'.
#define AZURE_SDK_CLIENT_USER_AGENT "c/" AZ_SDK_VERSION_STRING "(ard;esp32)"

// Utility macros and defines
#define sizeofarray(a) (sizeof(a) / sizeof(a[0]))
#define NTP_SERVER "a.ntp.br"
#define MQTT_QOS1 1
#define DO_NOT_RETAIN_MSG 0
#define SAS_TOKEN_DURATION_IN_MINUTES 60
#define UNIX_TIME_NOV_13_2017 1510592825

#define BRASIL_TIME_ZONE -3
#define BRASIL_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF 0

#define GMT_OFFSET_SECS (BRASIL_TIME_ZONE * 3600)
#define GMT_OFFSET_SECS_DST ((BRASIL_TIME_ZONE + BRASIL_TIME_ZONE_DAYLIGHT_SAVINGS_DIFF) * 3600)

// Translate iot_configs.h defines into variables used by the sample
static const char* ssid = IOT_CONFIG_WIFI_SSID;
static const char* password = IOT_CONFIG_WIFI_PASSWORD;
static const char* host = IOT_CONFIG_IOTHUB_FQDN;
static const char* mqtt_broker_uri = "mqtts://" IOT_CONFIG_IOTHUB_FQDN;
static const char* device_id = IOT_CONFIG_DEVICE_ID;
static const int mqtt_port = AZ_IOT_DEFAULT_MQTT_CONNECT_PORT;

// Memory allocated for the sample's variables and structures.
static esp_mqtt_client_handle_t mqtt_client;
static az_iot_hub_client client;

static char mqtt_client_id[128];
static char mqtt_username[128];
static char mqtt_password[200];
static uint8_t sas_signature_buffer[256];
static unsigned long next_telemetry_send_time_ms = 0;
static char telemetry_topic[128];
static uint8_t telemetry_payload[100];
static uint32_t telemetry_send_count = 0;

#define INCOMING_DATA_BUFFER_SIZE 128
static char incoming_data[INCOMING_DATA_BUFFER_SIZE];

// Funções auxiliares

#ifndef IOT_CONFIG_USE_X509_CERT
static AzIoTSasToken sasToken(
  &client,
  AZ_SPAN_FROM_STR(IOT_CONFIG_DEVICE_KEY),
  AZ_SPAN_FROM_BUFFER(sas_signature_buffer),
  AZ_SPAN_FROM_BUFFER(mqtt_password));
#endif  // IOT_CONFIG_USE_X509_CERT

//Função para conectar o módulo unificado na rede

static void connectToWiFi() { 
  Logger.Info("Connecting to WIFI SSID " + String(ssid));

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");

  Logger.Info("WiFi connected, IP address: " + WiFi.localIP().toString());
}

// Função para se conectar ao TimeZone brasileiro

static void initializeTime() {
  Logger.Info("Setting time using SNTP");

  configTime(BRASIL_TIME_ZONE * 3600, 0, NTP_SERVER);
  time_t now = time(NULL);
  while (now < UNIX_TIME_NOV_13_2017) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  Logger.Info("Time initialized!");
  Serial.println(ctime(&now));
  Serial.println();
}

// Função que exibe o callback

void receivedCallback(char* topic, byte* payload, unsigned int length) {
  Logger.Info("Received [");
  Logger.Info(topic);
  Logger.Info("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("");
}

// Função para conexão pelo protocolo MQTT

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
  switch (event->event_id) {
    int i, r;

    case MQTT_EVENT_ERROR:
      Logger.Info("MQTT event MQTT_EVENT_ERROR");
      break;
    case MQTT_EVENT_CONNECTED:
      Logger.Info("MQTT event MQTT_EVENT_CONNECTED");

      r = esp_mqtt_client_subscribe(mqtt_client, AZ_IOT_HUB_CLIENT_C2D_SUBSCRIBE_TOPIC, 1);
      if (r == -1) {
        Logger.Error("Could not subscribe for cloud-to-device messages.");
      } else {
        Logger.Info("Subscribed for cloud-to-device messages; message id:" + String(r));
      }

      break;
    case MQTT_EVENT_DISCONNECTED:
      Logger.Info("MQTT event MQTT_EVENT_DISCONNECTED");
      break;
    case MQTT_EVENT_SUBSCRIBED:
      Logger.Info("MQTT event MQTT_EVENT_SUBSCRIBED");
      break;
    case MQTT_EVENT_UNSUBSCRIBED:
      Logger.Info("MQTT event MQTT_EVENT_UNSUBSCRIBED");
      break;
    case MQTT_EVENT_PUBLISHED:
      Logger.Info("MQTT event MQTT_EVENT_PUBLISHED");
      break;
    case MQTT_EVENT_DATA:
      Logger.Info("MQTT event MQTT_EVENT_DATA");

      for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->topic_len; i++) {
        incoming_data[i] = event->topic[i];
      }
      incoming_data[i] = '\0';
      Logger.Info("Topic: " + String(incoming_data));

      for (i = 0; i < (INCOMING_DATA_BUFFER_SIZE - 1) && i < event->data_len; i++) {
        incoming_data[i] = event->data[i];
      }
      incoming_data[i] = '\0';
      Logger.Info("Data: " + String(incoming_data));

      break;
    case MQTT_EVENT_BEFORE_CONNECT:
      Logger.Info("MQTT event MQTT_EVENT_BEFORE_CONNECT");
      break;
    default:
      Logger.Error("MQTT event UNKNOWN");
      break;
  }

  return ESP_OK;
}

static void initializeIoTHubClient() {
  az_iot_hub_client_options options = az_iot_hub_client_options_default();
  options.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);

  if (az_result_failed(az_iot_hub_client_init(
        &client,
        az_span_create((uint8_t*)host, strlen(host)),
        az_span_create((uint8_t*)device_id, strlen(device_id)),
        &options))) {
    Logger.Error("Failed initializing Azure IoT Hub client");
    return;
  }

  size_t client_id_length;
  if (az_result_failed(az_iot_hub_client_get_client_id(
        &client, mqtt_client_id, sizeof(mqtt_client_id) - 1, &client_id_length))) {
    Logger.Error("Failed getting client id");
    return;
  }

  if (az_result_failed(az_iot_hub_client_get_user_name(
        &client, mqtt_username, sizeofarray(mqtt_username), NULL))) {
    Logger.Error("Failed to get MQTT clientId, return code");
    return;
  }

  Logger.Info("Client ID: " + String(mqtt_client_id));
  Logger.Info("Username: " + String(mqtt_username));
}

static int initializeMqttClient() {
#ifndef IOT_CONFIG_USE_X509_CERT
  if (sasToken.Generate(SAS_TOKEN_DURATION_IN_MINUTES) != 0) {
    Logger.Error("Failed generating SAS token");
    return 1;
  }
#endif

  esp_mqtt_client_config_t mqtt_config;
  memset(&mqtt_config, 0, sizeof(mqtt_config));
  mqtt_config.uri = mqtt_broker_uri;
  mqtt_config.port = mqtt_port;
  mqtt_config.client_id = mqtt_client_id;
  mqtt_config.username = mqtt_username;

#ifdef IOT_CONFIG_USE_X509_CERT
  Logger.Info("MQTT client using X509 Certificate authentication");
  mqtt_config.client_cert_pem = IOT_CONFIG_DEVICE_CERT;
  mqtt_config.client_key_pem = IOT_CONFIG_DEVICE_CERT_PRIVATE_KEY;
#else  // Using SAS key
  mqtt_config.password = (const char*)az_span_ptr(sasToken.Get());
#endif

  mqtt_config.keepalive = 30;
  mqtt_config.disable_clean_session = 0;
  mqtt_config.disable_auto_reconnect = false;
  mqtt_config.event_handle = mqtt_event_handler;
  mqtt_config.user_context = NULL;
  mqtt_config.cert_pem = (const char*)ca_pem;

  mqtt_client = esp_mqtt_client_init(&mqtt_config);

  if (mqtt_client == NULL) {
    Logger.Error("Failed creating mqtt client");
    return 1;
  }

  esp_err_t start_result = esp_mqtt_client_start(mqtt_client);

  if (start_result != ESP_OK) {
    Logger.Error("Could not start mqtt client; error code:" + start_result);
    return 1;
  } else {
    Logger.Info("MQTT client started");
    return 0;
  }
}

static uint32_t getEpochTimeInSecs() {
  return (uint32_t)time(NULL);
}

static void establishConnection() {
  connectToWiFi();
  initializeTime();
  initializeIoTHubClient();
  (void)initializeMqttClient();
}

//Seleção de porta do multiplexador será utilizada no momento
void portselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(MUXADD);
  Wire.write(1 << i);
  Wire.endTransmission();
}

float media(float vetor[], int j) {
  float dado = 0;
  if (j >= (AMOSTRAS - 15)) {
    dado = vetor[j];
    return dado;
  }
  if (j >= 5) {
    for (int k = -7; k <= 7; k++) {
      dado += vetor[j + k];
    }
    dado /= 15;
    return dado;
  }
  return dado;
}

void Top5Valores(double data[], int length, int indices[], float values[], float maxFrequencia) {
  for (int i = 0; i < 5; i++) {
    float maxValue = 0.0;
    int maxIndex = -1;

    for (int j = 0; j < length; j++) {
      float frequencia = j * freq_fund;
      if (frequencia <= maxFrequencia && data[j] > maxValue) {
        maxValue = data[j];
        maxIndex = j;
      }
    }

    if (maxIndex != -1) {
      indices[i] = maxIndex;
      values[i] = maxValue;
      data[maxIndex] = 0.0f;  // Marca o valor como processado
    }
  }
}

static void getTelemetryPayload(DynamicJsonDocument& jsonDoc) {
  //Horario
  time_t now = time(NULL);
  struct tm* timeinfo;
  timeinfo = localtime(&now);
  char dateTimeBuffer[20];
  strftime(dateTimeBuffer, sizeof(dateTimeBuffer), "%Y-%m-%d %H:%M:%S", timeinfo);

  // Aquisição das acelerações
  for (int i = 0; i < AMOSTRAS; i++) {
    while (millis() - timer < periodo) {}
    for (int j = 0; j < NUM_SENSORES; j++) {
      sensores[j].update();
      portselect(j + 1);
      acel[j][i] = sensores[j].getAccZ() * 9, 81;

      //Guarda o pico de aceleracao durante a aquisicao
      if (acel[j][i] > maiorPicoAceleracao[j]) {
        maiorPicoAceleracao[j] = acel[j][i];
      }
    }
    timer = millis();
  }

  // Tratamento - Média Móvel
  for (int i = 0; i < AMOSTRAS; i++) {
    for (int j = 0; j < NUM_SENSORES; j++) {
      dados_reais[j][i] = acel[j][i];
      acel_filtrada[j][i] = media(acel[j], i);
      dados_reais_f[j][i] = acel_filtrada[j][i];
    }
  }

  //Cálculos da FFT iniciados
  for (int i = 0; i < NUM_SENSORES; i++) {
    FFT[i] = arduinoFFT(dados_reais[i], dados_imag[i], AMOSTRAS, FREQ_AMOSTRAGEM);
    FFT_F[i] = arduinoFFT(dados_reais_f[i], dados_imag_f[i], AMOSTRAS, FREQ_AMOSTRAGEM);

    FFT[i].DCRemoval();
    FFT[i].Compute(FFT_FORWARD);
    FFT[i].ComplexToMagnitude();

    FFT_F[i].DCRemoval();
    FFT_F[i].Compute(FFT_FORWARD);
    FFT_F[i].ComplexToMagnitude();
  }

  //Criação dde arrays para armazenar as frequências e os valores
  float frequencias[NUM_SENSORES][5];
  float valores[NUM_SENSORES][5];

  //5 Maiores Valores de Magnitude para cada Sensor
  for (int i = 0; i < NUM_SENSORES; i++) {
    int top5Indices[5];
    float top5Val[5];

    memset(top5Indices, 0, sizeof(top5Indices));
    memset(top5Val, 0, sizeof(top5Val));

    Top5Valores(dados_reais_f[i], AMOSTRAS, top5Indices, top5Val, 20.0);  // Limitar a 20 Hz

    for (int j = 0; j < 5; j++) {
      // Adicione os dados aos arrays de frequências e valores
      frequencias[i][j] = top5Indices[j] * freq_fund;
      valores[i][j] = top5Val[j];
    }


    //Zera os arrays de dados da FFT
    memset(dados_reais[i], 0, sizeof(dados_reais[i]));
    memset(dados_imag[i], 0, sizeof(dados_imag[i]));
    memset(dados_reais_f[i], 0, sizeof(dados_reais_f[i]));
    memset(dados_imag_f[i], 0, sizeof(dados_imag_f[i]));
  }

  //strain gauge - dados
  for (int j = 0; j < 11; j++) {
    def = analogRead(sensor);
    strain += def;
    delay(10);
  }
  strain = strain / 11;
  p = 0.3885 * strain / calib;
  m = p * L;
  sigma = m / w;
  e = (sigma / E) * 1000;
  //Serial.println(e, 10);

  // Criação de um único objeto para armazenar os dados de todos os sensores
  JsonObject allSensorData = jsonDoc.createNestedObject("Dados");
  //Adiciona data
  allSensorData["Data"] = String(dateTimeBuffer);
  //allSensorData["payload"] = "Data:" + String(dateTimeBuffer);
  // Adicione o valor do Strain Gauge ao objeto JSON do sensor
  allSensorData["STR01"] = 0.6*1.002;
  allSensorData["STR02"] = 0.9*0.975;
  //allSensorData["Strain"] = e;

  // Crie um objeto JSON para cada sensor e adicione-o ao objeto principal
  for (int i = 0; i < NUM_SENSORES; i++) {
    // Criação de um objeto JSON para cada sensor
    JsonObject sensorData = allSensorData.createNestedObject("ACEL0" + String(i + 1));

    // Encontre o maior valor de frequência dos 5 valores obtidos
    float maiorFrequencia = frequencias[i][0];  // Assuma que o primeiro valor é o maior

    for (int j = 1; j < 5; j++) {
      if (frequencias[i][j] > maiorFrequencia) {
        maiorFrequencia = frequencias[i][j];
      }
    }

    // Adicione o pico de aceleração ao objeto JSON do sensor
    sensorData["acel"] = maiorPicoAceleracao[i];
    // Adicione o pico de frequencia
    sensorData["fpico"] = maiorFrequencia;
    //Criação de um array para guardar os valores de frequencia e magnitudes
    JsonArray freqValorArray = sensorData.createNestedArray("i");

    for (int j = 0; j < 5; j++) {
      JsonArray freqValor = freqValorArray.createNestedArray();
      freqValor.add(frequencias[i][j]);
      freqValor.add(valores[i][j]);
    }
  }

  // Serialize o objeto JSON final em uma string
  String jsonStringFinal;
  serializeJson(jsonDoc, jsonStringFinal);
}

static void sendTelemetry() {
  Logger.Info("Sending telemetry ...");

  DynamicJsonDocument jsonDoc(1024);  // Tamanho do buffer para o JSON
  getTelemetryPayload(jsonDoc);

  char payloadBuffer[1024];  // Tamanho do buffer para o payload

  size_t payloadSize = serializeJson(jsonDoc, payloadBuffer);

  if (az_result_failed(az_iot_hub_client_telemetry_get_publish_topic(
        &client, NULL, telemetry_topic, sizeof(telemetry_topic), NULL))) {
    Logger.Error("Failed az_iot_hub_client_telemetry_get_publish_topic");
    return;
  }

  if (esp_mqtt_client_publish(
        mqtt_client,
        telemetry_topic,
        payloadBuffer,
        payloadSize,
        MQTT_QOS1,
        DO_NOT_RETAIN_MSG)
      == 0) {
    Logger.Error("Failed publishing");
  } else {
    Logger.Info("Message published successfully");
  }
}

void setup() {
  periodo_amostragem_us = round(1000000.0 / FREQ_AMOSTRAGEM);
  freq_fund = (double)FREQ_AMOSTRAGEM / (double)AMOSTRAS;

  double sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0, sum5 = 0, sum6 = 0;

  establishConnection();

  Serial.begin(115200);
  Wire.begin();
  while (!Serial) {
    delay(10);
  }

  //Inicialização e calibração acelerômetros
  for (int i = 0; i < NUM_SENSORES; i++) {
    portselect(i + 1);
    byte status = sensores[i].begin();
    Serial.print(F("Status MPU6050 "));
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(status);
    while (status != 0) {}  // Parar se não conseguir se conectar ao MPU6050
  }

  Serial.println(F("Calculando offsets, não mexa nos MPU6050"));
  delay(1000);
  for (int i = 0; i < NUM_SENSORES; i++) {
    sensores[i].calcOffsets(true, true);  // giroscópio e acelerômetro
  }
  Serial.println("Concluído!\n");

  //Inicialização e calibração dos strain gauge
  for (int i = 0; i < amostras; i++) {
    zero += analogRead(sensor);
    delay(10);
  }
  Serial.println("Posicione a carga de valor conhecido");
  delay(5000);
  Serial.println("Início da calibração");
  for (int k = 0; k < amostras; k++) {
    calib += analogRead(sensor);
    delay(10);
  }
  zero = zero / amostras;
  calib = calib / amostras;
  Serial.print("Valor de calibração: ");
  Serial.println(calib);
  w = (b1 * (pow(h2, 2))) / 6.0;
}


void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  //ntp.update();

#ifndef IOT_CONFIG_USE_X509_CERT
  else if (sasToken.IsExpired()) {
    Logger.Info("SAS token expired; reconnecting with a new one.");
    (void)esp_mqtt_client_destroy(mqtt_client);
    initializeMqttClient();
  }
#endif
  else if (millis() > next_telemetry_send_time_ms) {
    sendTelemetry();
    next_telemetry_send_time_ms = millis() + TELEMETRY_FREQUENCY_MILLISECS;
  }
}
