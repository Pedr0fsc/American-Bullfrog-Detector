#include <Arduino.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include "time.h"

// ===== CONFIG Wi-Fi (opcional p/ NTP) =====
const char* ssid = "TBRCorvinal";
const char* password = "lunalovegood";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -3 * 3600;  // Brasil (UTC-3)
const int daylightOffset_sec = 0;

// ===== CONFIG SOM / FFT =====
#define SAMPLES 256
#define SAMPLING_FREQUENCY 4000
#define MIC_PIN 34
#define BUZZER_PIN 25
#define CONSECUTIVE_NEEDED 2

double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, (double)SAMPLING_FREQUENCY);

// ===== SUB-FAIXAS DO COAXAR (tons diferentes) =====
struct Band { double low; double high; };
Band bullfrogBands[] = {
  {200.0, 250.0},
  {250.0, 300.0},
  {300.0, 350.0},
  {350.0, 400.0},
  {400.0, 470.0},
  {470.0, 520.0},
  {520.0, 580.0},
};

// Limiar iguais ao seu código-base
double magnitudeThreshold    = 100.0;
double bandEnergyThreshold   = 200.0;
double totalEnergyThreshold  = 1500.0;

int consecutiveHits = 0;
unsigned long windowsAnalyzed = 0;
unsigned long startMillis;
bool timeSynced = false;

// ===== FILTRO PASSA-BAIXA SIMPLES =====
void lowPassFilter(double* data, int size, int window) {
  double temp[SAMPLES];
  for (int i = 0; i < size; i++) {
    double sum = 0;
    int count = 0;
    for (int j = i - window; j <= i + window; j++) {
      if (j >= 0 && j < size) { sum += data[j]; count++; }
    }
    temp[i] = sum / count;
  }
  for (int i = 0; i < size; i++) data[i] = temp[i];
}

// ===== HORA =====
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
    delay(500); Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi conectado, sincronizando hora...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    delay(2000);
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

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  startMillis = millis();
  connectWiFiAndTime();

  Serial.println("=== Iniciando captura de som (rã-touro, multibanda) ===");
}

// ===== LOOP =====
void loop() {
  // Captura amostras
  double sum = 0, sumSq = 0;
  for (int i = 0; i < SAMPLES; i++) {
    int raw = analogRead(MIC_PIN);
    vReal[i] = (double)raw;
    vImag[i] = 0.0;
    sum += raw;
    sumSq += raw * raw;
    delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
  }

  // Estatísticas e remoção do DC (MAX9814 tem bias ~Vcc/2)
  double mean = sum / SAMPLES;
  double variance = (sumSq / SAMPLES) - (mean * mean);
  double stddev = sqrt(variance);

  for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;

  if (stddev < 2) {  // silêncio/baixo sinal
    Serial.println("⚠ Silêncio detectado (pouca variação).");
    delay(300);
    return;
  }

  // FFT
  lowPassFilter(vReal, SAMPLES, 2);
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  // Pico global
  double maxMag = 0.0;
  int indexOfMax = 0;
  for (int i = 1; i < SAMPLES / 2; i++) {
    if (vReal[i] > maxMag) { maxMag = vReal[i]; indexOfMax = i; }
  }
  double freqOfMax = (indexOfMax * SAMPLING_FREQUENCY) / (double)SAMPLES;

  // Energia total (para evitar falsos positivos em silêncio)
  double totalEnergy = 0.0;
  for (int b = 1; b < SAMPLES / 2; b++) totalEnergy += vReal[b];

  // Energia nas sub-faixas (e teste de “pico dentro de alguma faixa”)
  bool inAnyBand = false;
  double unionBandEnergy = 0.0;
  double bestBandEnergy = 0.0;
  int bestBandIdx = -1;

  for (int k = 0; k < (int)(sizeof(bullfrogBands)/sizeof(bullfrogBands[0])); k++) {
    double low = bullfrogBands[k].low;
    double high = bullfrogBands[k].high;
    int binLow  = max(1, (int)floor((low  * SAMPLES) / (double)SAMPLING_FREQUENCY));
    int binHigh = min(SAMPLES/2 - 1, (int)ceil ((high * SAMPLES) / (double)SAMPLING_FREQUENCY));

    double e = 0.0;
    for (int b = binLow; b <= binHigh; b++) e += vReal[b];

    unionBandEnergy += e;
    if (e > bestBandEnergy) { bestBandEnergy = e; bestBandIdx = k; }

    if (freqOfMax >= low && freqOfMax <= high) inAnyBand = true;
  }

  windowsAnalyzed++;

  // Regras (iguais ao seu fluxo)
  bool ruleA = (inAnyBand) && (maxMag >= magnitudeThreshold);
  bool ruleB = (unionBandEnergy >= bandEnergyThreshold);
  bool energyOk = (totalEnergy >= totalEnergyThreshold);

  if (ruleA && ruleB && energyOk) consecutiveHits++;
  else                            consecutiveHits = 0;

  // Log
  Serial.print("Captura N°: "); Serial.print(windowsAnalyzed);
  Serial.print(" | FreqMax: "); Serial.print(freqOfMax, 1); Serial.print("Hz");
  Serial.print(" | Detecções do coaxar da rã-touro: "); Serial.println(consecutiveHits);

  // Alerta
  if (consecutiveHits >= CONSECUTIVE_NEEDED) {
    Serial.print("=== ALERTA: Coaxar de RÃ-TOURO detectado ===  |  ");
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
