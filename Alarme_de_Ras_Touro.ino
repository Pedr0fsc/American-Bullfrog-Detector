#include <Arduino.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include "time.h"

// ===== CONFIG Wi-Fi (use se quiser data/hora real) =====
const char* ssid     = "SUA_REDE";
const char* password = "SENHA_REDE";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3 * 3600; // Brasil (UTC-3)
const int   daylightOffset_sec = 0;

// ===== CONFIG SOM =====
#define SAMPLES 256
#define SAMPLING_FREQUENCY 4000
#define MIC_PIN 34
#define BUZZER_PIN 25
#define CONSECUTIVE_NEEDED 3

double vReal[SAMPLES];
double vImag[SAMPLES];

arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

double targetFrequency = 418.4;
double toleranceFreq = 120.0;
double magnitudeThreshold = 150.0;
double bandLow = 300.0;
double bandHigh = 600.0;
double bandEnergyThreshold = 400.0;

int consecutiveHits = 0;
unsigned long windowsAnalyzed = 0;
unsigned long startMillis;

bool timeSynced = false;

// ===== FUNÇÕES =====
void printTimestamp() {
  if (timeSynced) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      Serial.printf("%02d/%02d/%04d %02d:%02d:%02d",
        timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900,
        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
      Serial.print("Erro ao ler hora");
    }
  } else {
    unsigned long elapsedSec = (millis() - startMillis) / 1000;
    unsigned int hours = elapsedSec / 3600;
    unsigned int minutes = (elapsedSec % 3600) / 60;
    unsigned int seconds = elapsedSec % 60;
    Serial.printf("%02u:%02u:%02u desde o início", hours, minutes, seconds);
  }
}

void connectWiFiAndTime() {
  Serial.printf("Conectando ao Wi-Fi %s...\n", ssid);
  WiFi.begin(ssid, password);

  unsigned long attemptStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - attemptStart < 8000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi conectado, sincronizando hora...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      timeSynced = true;
      Serial.println("Hora sincronizada com sucesso!");
    } else {
      Serial.println("Falha ao sincronizar hora via NTP.");
    }
  } else {
    Serial.println("Não foi possível conectar ao Wi-Fi, usando relógio interno.");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  startMillis = millis();

  connectWiFiAndTime();

  Serial.println("Iniciando captura de som e monitoramento...");
}

void loop() {
  // Captura amostras
  for (int i = 0; i < SAMPLES; i++) {
    int raw = analogRead(MIC_PIN);
    vReal[i] = (double)raw;
    vImag[i] = 0.0;
    delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
  }

  // FFT
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  // Pico
  double maxMag = 0.0;
  int indexOfMax = 0;
  for (int i = 1; i < SAMPLES / 2; i++) {
    if (vReal[i] > maxMag) {
      maxMag = vReal[i];
      indexOfMax = i;
    }
  }
  double freqOfMax = (indexOfMax * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;

  // Energia na faixa
  int binLow = max(1, (int)floor((bandLow * SAMPLES) / (double)SAMPLING_FREQUENCY));
  int binHigh = min(SAMPLES / 2 - 1, (int)ceil((bandHigh * SAMPLES) / (double)SAMPLING_FREQUENCY));
  double bandEnergy = 0.0;
  for (int b = binLow; b <= binHigh; b++) bandEnergy += vReal[b];

  windowsAnalyzed++;

  bool ruleA = (fabs(freqOfMax - targetFrequency) <= toleranceFreq) && (maxMag >= magnitudeThreshold);
  bool ruleB = (bandEnergy >= bandEnergyThreshold);

  if (ruleA || ruleB) {
    consecutiveHits++;
  } else {
    if (consecutiveHits > 0) consecutiveHits = 0;
  }

  Serial.print("Janela: "); Serial.print(windowsAnalyzed);
  Serial.print(" | FreqMax: "); Serial.print(freqOfMax, 1);
  Serial.print("Hz | Mag: "); Serial.print(maxMag, 1);
  Serial.print(" | BandEnergy: "); Serial.print(bandEnergy, 1);
  Serial.print(" | Hits: "); Serial.print(consecutiveHits);
  Serial.print(" | RegA: "); Serial.print(ruleA);
  Serial.print(" | RegB: "); Serial.println(ruleB);

  if (consecutiveHits >= CONSECUTIVE_NEEDED) {
    Serial.print("=== ALERTA: coaxar detectado -> buzzer ON ===  |  ");
    printTimestamp();
    Serial.println();
    digitalWrite(BUZZER_PIN, HIGH);
    delay(800);
    digitalWrite(BUZZER_PIN, LOW);
    consecutiveHits = 0;
    delay(300);
  }

  delay(50);
}