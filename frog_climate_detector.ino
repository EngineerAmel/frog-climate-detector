/*
  Realistic Frog Vocal Rain Detector - Final
  ESP32 + DHT11 + SSD1306 (SPI 128x64) + Speaker (PWM via ledc) + LED alert

  Features:
  - Reads temperature & humidity with smoothing (EMA).
  - Calculates climate trends (linear regression).
  - Computes rain probability (0â€“100%) using weighted factors.
  - Selects realistic frog croak patterns depending on weather.
  - Plays croaks with random pitch/rhythm variations (natural sound).
  - Non-blocking croak playback using ESP32 ledc PWM.
  - OLED displays weather details neatly.
  - LED alert lights up for imminent rain.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ------------------ OLED ------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_MOSI 23
#define OLED_CLK 18
#define OLED_CS   5
#define OLED_DC   16
#define OLED_RST  17
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RST, OLED_CS);

// ------------------ DHT11 ------------------
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
xx
// ------------------ Speaker & LED ------------------
#define SPEAKER_PIN 25
#define LEDC_CHANNEL 0
#define LEDC_FREQ 2000
#define LEDC_RES 8
#define LED_ALERT 26

// ------------------ Climate Thresholds ------------------
const float TEMP_MIN = 28.0f;
const float TEMP_MAX = 32.0f;
const float HUM_MIN  = 80.0f;
const float HUM_MAX  = 95.0f;

const unsigned long DHT_READ_INTERVAL = 2000UL;
const int TREND_BUF = 8;
const float EMA_ALPHA = 0.25f;
const unsigned long PATTERN_HOLD_MS = 4000UL;

// ------------------ Buffers ------------------
float tempBuf[TREND_BUF];
float humBuf[TREND_BUF];
unsigned long timeBuf[TREND_BUF];
int bufPos = 0;
bool bufFilled = false;

float tempEMA = NAN;
float humEMA  = NAN;
unsigned long lastDHT = 0;

int stablePattern = 1;
int currentTargetPattern = 1;
unsigned long patternSince = 0;

// ------------------ Frog Croak Patterns ------------------
struct Tone { uint16_t freq; uint16_t dur; uint16_t gap; };
struct Pattern { const Tone *tones; uint8_t count; };

static const Tone PAT1_TONES[] = { {300,180,1000}, {320,160,1400} };                     // Sunny/Dry
static const Tone PAT2_TONES[] = { {440,140,200}, {480,140,300}, {460,120,300} };        // Cloudy/Humid
static const Tone PAT3_TONES[] = { {600,120,100}, {640,120,120}, {620,120,180} };        // Pre-Rain
static const Tone PAT4_TONES[] = { {760,110,80}, {820,100,80}, {780,120,70}, {840,100,90} }; // Imminent Rain

Pattern patterns[5];

// Croak Player State
uint8_t playingPattern = 1;
uint8_t playIndex = 0;
unsigned long nextChange = 0;
bool toneOn = false;

// ------------------ Function Declarations ------------------
bool readDHTRetry(float &t, float &h);
void addSample(float t, float h, unsigned long tms);
void updateEMA(float t, float h);
void computeTrend(float &tempSlopePerMin, float &humSlopePerMin);
int computeRainProbability(float tEMA, float hEMA, float humTrendPerMin, float tempTrendPerMin);
void setTargetPatternWithHysteresis(int target);
void startPlayingPattern(uint8_t p);
void updatePlayer();
String patternLabel(int p);
void oledUpdate(int rainProb, float tEMA, float hEMA, float tTrend, float hTrend, int pattern);

// ------------------ Setup ------------------
void setup() {
  Serial.begin(115200);
  dht.begin();

  ledcSetup(LEDC_CHANNEL, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(SPEAKER_PIN, LEDC_CHANNEL);
  pinMode(LED_ALERT, OUTPUT);

  patterns[1] = { PAT1_TONES, (uint8_t)(sizeof(PAT1_TONES)/sizeof(Tone)) };
  patterns[2] = { PAT2_TONES, (uint8_t)(sizeof(PAT2_TONES)/sizeof(Tone)) };
  patterns[3] = { PAT3_TONES, (uint8_t)(sizeof(PAT3_TONES)/sizeof(Tone)) };
  patterns[4] = { PAT4_TONES, (uint8_t)(sizeof(PAT4_TONES)/sizeof(Tone)) };

  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("SSD1306 allocation failed");
    while(true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(18,20);
  display.println("Frog Rain");
  display.setCursor(20,36);
  display.println("Detector");
  display.display();
  delay(1200);

  for(int i=0;i<TREND_BUF;i++){ tempBuf[i]=humBuf[i]=0.0f; timeBuf[i]=0; }
  bufPos = 0; bufFilled = false;

  stablePattern = 1;
  currentTargetPattern = 1;
  patternSince = millis();

  startPlayingPattern(stablePattern);
  randomSeed(analogRead(0));
}

// ------------------ Loop ------------------
void loop() {
  unsigned long now = millis();

  if(now - lastDHT >= DHT_READ_INTERVAL) {
    lastDHT = now;
    float t, h;
    if(readDHTRetry(t,h)) {
      addSample(t,h,now);
      updateEMA(t,h);
    } else {
      Serial.println("DHT read failed");
    }
  }

  int sampleCount = bufFilled ? TREND_BUF : bufPos;
  if(sampleCount < 3) {
    updatePlayer();
    delay(12);
    return;
  }

  float tTrendPerMin = 0.0f, hTrendPerMin = 0.0f;
  computeTrend(tTrendPerMin, hTrendPerMin);

  int rainProb = computeRainProbability(tempEMA, humEMA, hTrendPerMin, tTrendPerMin);

  int targetPattern = 1;
  if(rainProb <= 30) targetPattern = 1;
  else if(rainProb <= 60) targetPattern = 2;
  else if(rainProb <= 80) targetPattern = 3;
  else targetPattern = 4;
  setTargetPatternWithHysteresis(targetPattern);

  digitalWrite(LED_ALERT, (stablePattern == 4) ? HIGH : LOW);

  if(playingPattern != stablePattern) startPlayingPattern(stablePattern);
  updatePlayer();

  oledUpdate(rainProb, tempEMA, humEMA, tTrendPerMin, hTrendPerMin, stablePattern);

  Serial.print("Temp: "); Serial.print(tempEMA,1);
  Serial.print(" C, Hum: "); Serial.print(humEMA,1);
  Serial.print(" %, RainProb: "); Serial.print(rainProb);
  Serial.print("%, Pattern: "); Serial.println(patternLabel(stablePattern));

  delay(20);
}

// ------------------ Functions ------------------
bool readDHTRetry(float &outT, float &outH) {
  for(int i=0;i<3;i++){
    float hh = dht.readHumidity();
    float tt = dht.readTemperature();
    if(!isnan(hh) && !isnan(tt)) {
      outT = tt;
      outH = hh;
      return true;
    }
    delay(80);
  }
  return false;
}

void addSample(float t, float h, unsigned long tms) {
  tempBuf[bufPos] = t;
  humBuf[bufPos]  = h;
  timeBuf[bufPos] = tms;
  bufPos++;
  if(bufPos >= TREND_BUF) { bufPos = 0; bufFilled = true; }
}

void updateEMA(float t, float h) {
  if(isnan(tempEMA)) tempEMA = t;
  else tempEMA = EMA_ALPHA * t + (1.0f - EMA_ALPHA) * tempEMA;
  if(isnan(humEMA)) humEMA = h;
  else humEMA = EMA_ALPHA * h + (1.0f - EMA_ALPHA) * humEMA;
}

void computeTrend(float &tempSlopePerMin, float &humSlopePerMin) {
  int n = bufFilled ? TREND_BUF : bufPos;
  if(n < 3) { tempSlopePerMin = 0.0f; humSlopePerMin = 0.0f; return; }

  double sumX=0.0, sumT=0.0, sumH=0.0;
  for(int i=0;i<n;i++){
    double x = (double)timeBuf[i] / 1000.0;
    sumX += x; sumT += tempBuf[i]; sumH += humBuf[i];
  }
  double meanX = sumX / n;
  double meanT = sumT / n;
  double meanH = sumH / n;

  double covXT=0.0, covXH=0.0, varX=0.0;
  for(int i=0;i<n;i++){
    double x = ((double)timeBuf[i] / 1000.0) - meanX;
    covXT += x * (tempBuf[i] - meanT);
    covXH += x * (humBuf[i] - meanH);
    varX  += x * x;
  }
  double slopeT = (varX == 0.0) ? 0.0 : covXT / varX;
  double slopeH = (varX == 0.0) ? 0.0 : covXH / varX;

  tempSlopePerMin = (float)(slopeT * 60.0);
  humSlopePerMin  = (float)(slopeH * 60.0);
}

int computeRainProbability(float tEMA, float hEMA, float humTrendPerMin, float tempTrendPerMin) {
  float score = 0.0f;

  if(tEMA >= TEMP_MIN && tEMA <= TEMP_MAX) score += 30.0f;
  if(hEMA >= HUM_MIN && hEMA <= HUM_MAX) score += 40.0f;
  if(humTrendPerMin > 1.0f) score += 20.0f;
  if(humTrendPerMin > 0.4f) score += 10.0f;
  if(tempTrendPerMin < -0.2f) score += 10.0f;

  if(score < 0.0f) score = 0.0f;
  if(score > 100.0f) score = 100.0f;
  return (int)(score + 0.5f);
}

void setTargetPatternWithHysteresis(int target) {
  unsigned long now = millis();
  if(target != stablePattern) {
    if(target == currentTargetPattern) {
      if(now - patternSince >= PATTERN_HOLD_MS) {
        stablePattern = target;
      }
    } else {
      currentTargetPattern = target;
      patternSince = now;
    }
  } else {
    currentTargetPattern = stablePattern;
    patternSince = now;
  }
}

void startPlayingPattern(uint8_t p) {
  playingPattern = p;
  playIndex = 0;
  nextChange = millis();
  toneOn = false;
}

void updatePlayer() {
  unsigned long now = millis();
  Pattern *P = (playingPattern >= 1 && playingPattern <= 4) ? &patterns[playingPattern] : nullptr;
  if(!P) return;
  if(playIndex >= P->count) playIndex = 0;
  Tone cur = P->tones[playIndex];

  if(!toneOn) {
    if(now >= nextChange) {
      if(cur.freq == 0 || cur.dur == 0) {
        nextChange = now + cur.gap;
        playIndex++;
        return;
      }
      int freqJit = (int)cur.freq + random(-10, 11);
      int durJit  = (int)cur.dur  + random(-20, 21);
      ledcWriteTone(LEDC_CHANNEL, max(50, freqJit));
      toneOn = true;
      nextChange = now + (unsigned long)max(20, durJit);
    }
  } else {
    if(now >= nextChange) {
      ledcWriteTone(LEDC_CHANNEL, 0);
      toneOn = false;
      int gapJit = (int)cur.gap + random(-20, 21);
      nextChange = now + (unsigned long)max(10, gapJit);
      playIndex++;
      if(playIndex >= P->count) {
        nextChange += 300;
        playIndex = 0;
      }
    }
  }
}

String patternLabel(int p) {
  switch(p) {
    case 1: return "Sunny / Dry";
    case 2: return "Cloudy / Humid";
    case 3: return "Pre-Rain";
    case 4: return "Imminent Rain";
    default: return "Unknown";
  }
}

void oledUpdate(int rainProb, float tEMA, float hEMA, float tTrend, float hTrend, int pattern) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.println("Frog Rain Detector");
  display.println("------------------");

  display.print("Temp : "); display.print(tEMA,1); display.println(" C");
  display.print("Hum  : "); display.print(hEMA,1); display.println(" %");
  display.print("Rain : "); display.print(rainProb); display.println(" %");

  display.print("Ttrnd: ");
  if(tTrend >= 0.0f) display.print('+');
  display.print(tTrend,2); display.println(" C/m");

  display.print("Htrnd: ");
  if(hTrend >= 0.0f) display.print('+');
  display.print(hTrend,2); display.println(" %/m");

  display.print("Weather: "); display.println(patternLabel(pattern));

  display.display();
}