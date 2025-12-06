#include <Wire.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <RTClib.h>
#include <Stepper.h>

#define DHTPIN 7
#define DHTTYPE DHT11

#define FAN_PIN 6

#define WATER_CH 0
#define VENT_CH 1

#define START_BTN_PIN 2
#define STOP_BTN_PIN 3
#define RESET_BTN_PIN 4

#define LED_YELLOW_PIN 30
#define LED_GREEN_PIN 31
#define LED_RED_PIN 32
#define LED_BLUE_PIN 33

#define LCD_RS 22
#define LCD_EN 23
#define LCD_D4 24
#define LCD_D5 25
#define LCD_D6 26
#define LCD_D7 27

#define STEPS_PER_REV 2048
#define STEP_PIN1 8
#define STEP_PIN2 9
#define STEP_PIN3 10
#define STEP_PIN4 11

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
RTC_DS1307 rtc;
Stepper ventStepper(STEPS_PER_REV, STEP_PIN1, STEP_PIN3, STEP_PIN2, STEP_PIN4);

enum CoolerState {
  STATE_DISABLED,
  STATE_IDLE,
  STATE_RUNNING,
  STATE_ERROR
};

volatile bool startRequested = false;
CoolerState currentState = STATE_DISABLED;

unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_UPDATE_INTERVAL = 60000UL;

int currentVentPosition = 0;
int lastReportedVentPosition = 0;

const float TEMP_ON_C = 28.0;
const float TEMP_OFF_C = 25.0;
const uint16_t WATER_THRESHOLD = 400;

void initLEDsAndButtons() {
  DDRC |= (1 << DDC7) | (1 << DDC6) | (1 << DDC5) | (1 << DDC4);
  DDRE &= ~((1 << DDE4) | (1 << DDE5));
  PORTE |= (1 << PORTE4) | (1 << PORTE5);
  DDRG &= ~(1 << DDG5);
  PORTG |= (1 << PORTG5);
}

bool readStopButton() {
  return !(PINE & (1 << PINE5));
}

bool readResetButton() {
  return !(PING & (1 << PING5));
}

void setLEDs(bool y, bool g, bool r, bool b) {
  if (y) PORTC |= (1 << PORTC7); else PORTC &= ~(1 << PORTC7);
  if (g) PORTC |= (1 << PORTC6); else PORTC &= ~(1 << PORTC6);
  if (r) PORTC |= (1 << PORTC5); else PORTC &= ~(1 << PORTC5);
  if (b) PORTC |= (1 << PORTC4); else PORTC &= ~(1 << PORTC4);
}

void adc_init() {
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC)) { }
  return ADC;
}

void usart0_init(unsigned long baud) {
  uint16_t ubrr = (F_CPU / 16 / baud) - 1;
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)ubrr;
  UCSR0A = 0;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void usart0_send_char(char c) {
  while (!(UCSR0A & (1 << UDRE0))) { }
  UDR0 = c;
}

void usart0_send_str(const char *s) {
  while (*s) usart0_send_char(*s++);
}

void usart0_send_int(long v) {
  char buf[16];
  ltoa(v, buf, 10);
  usart0_send_str(buf);
}

void logEvent(const char *label) {
  DateTime now = rtc.now();
  usart0_send_str("[");
  usart0_send_int(now.year());
  usart0_send_char('-');
  usart0_send_int(now.month());
  usart0_send_char('-');
  usart0_send_int(now.day());
  usart0_send_char(' ');
  usart0_send_int(now.hour());
  usart0_send_char(':');
  usart0_send_int(now.minute());
  usart0_send_char(':');
  usart0_send_int(now.second());
  usart0_send_str("] ");
  usart0_send_str(label);
  usart0_send_char('\n');
}

bool fanOn = false;

void fan_start() {
  analogWrite(FAN_PIN, 200);
  if (!fanOn) {
    fanOn = true;
    logEvent("MOTOR ON");
  }
}

void fan_stop() {
  analogWrite(FAN_PIN, 0);
  if (fanOn) {
    fanOn = false;
    logEvent("MOTOR OFF");
  }
}

void startButtonISR() {
  startRequested = true;
}

void updateVentPosition() {
  uint16_t val = adc_read(VENT_CH);
  int target = (int)((val / 1023.0) * 512.0);
  int delta = target - currentVentPosition;
  if (delta > 20) delta = 20;
  if (delta < -20) delta = -20;

  if (delta != 0) {
    ventStepper.step(delta);
    currentVentPosition += delta;

    if (abs(currentVentPosition - lastReportedVentPosition) >= 50) {
      lastReportedVentPosition = currentVentPosition;
      usart0_send_str("VENT POS ");
      usart0_send_int(currentVentPosition);
      usart0_send_char('\n');
    }
  }
}

bool isWaterLow() {
  uint16_t level = adc_read(WATER_CH);
  return (level < WATER_THRESHOLD);
}

float currentTempC = 0;
float currentHum = 0;
bool sensorOK = false;

void updateDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    sensorOK = false;
    return;
  }
  sensorOK = true;
  currentHum = h;
  currentTempC = t;
}

void updateLCD() {
  lcd.clear();
  if (!sensorOK) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor error");
    return;
  }
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(currentTempC, 1);
  lcd.print("C H:");
  lcd.print(currentHum, 0);
  lcd.print("%");
  lcd.setCursor(0, 1);
  switch (currentState) {
    case STATE_DISABLED: lcd.print("State: DISABLED"); break;
    case STATE_IDLE: lcd.print("State: IDLE    "); break;
    case STATE_RUNNING: lcd.print("State: RUNNING "); break;
    case STATE_ERROR: lcd.print("State: ERROR   "); break;
  }
}

void setup() {
  initLEDsAndButtons();
  adc_init();
  usart0_init(9600);
  DDRH |= (1 << DDH6);
  dht.begin();
  lcd.begin(16, 2);
  ventStepper.setSpeed(10);

  if (!rtc.begin()) {
    usart0_send_str("RTC not found!\n");
  }

  attachInterrupt(digitalPinToInterrupt(START_BTN_PIN), startButtonISR, FALLING);
  currentState = STATE_DISABLED;
  setLEDs(true, false, false, false);
  fan_stop();
  lastSensorUpdate = millis();
  logEvent("STATE -> DISABLED");
}

void handleDisabledState() {
  setLEDs(true, false, false, false);
  fan_stop();

  if (startRequested) {
    startRequested = false;
    currentState = STATE_IDLE;
    logEvent("STATE -> IDLE");
  }
}

void handleIdleState() {
  setLEDs(false, true, false, false);
  fan_stop();

  if (isWaterLow()) {
    currentState = STATE_ERROR;
    logEvent("STATE -> ERROR (water low from IDLE)");
    return;
  }

  unsigned long now = millis();
  if (now - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdate = now;
    updateDHT();
    updateLCD();
  }

  if (sensorOK && currentTempC >= TEMP_ON_C) {
    currentState = STATE_RUNNING;
    logEvent("STATE -> RUNNING (temp high)");
    return;
  }

  if (readStopButton()) {
    currentState = STATE_DISABLED;
    logEvent("STATE -> DISABLED (stop btn)");
  }
}

void handleRunningState() {
  setLEDs(false, false, false, true);
  fan_start();

  if (isWaterLow()) {
    currentState = STATE_ERROR;
    logEvent("STATE -> ERROR (water low from RUNNING)");
    fan_stop();
    return;
  }

  unsigned long now = millis();
  if (now - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdate = now;
    updateDHT();
    updateLCD();
  }

  if (sensorOK && currentTempC <= TEMP_OFF_C) {
    currentState = STATE_IDLE;
    fan_stop();
    logEvent("STATE -> IDLE (temp low)");
    return;
  }

  if (readStopButton()) {
    currentState = STATE_DISABLED;
    fan_stop();
    logEvent("STATE -> DISABLED (stop btn)");
  }
}

void handleErrorState() {
  setLEDs(false, false, true, false);
  fan_stop();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR: WATER LOW");
  lcd.setCursor(0, 1);
  lcd.print("Fix & press RESET");

  if (readResetButton() && !isWaterLow()) {
    currentState = STATE_IDLE;
    logEvent("STATE -> IDLE (reset)");
  }

  if (readStopButton()) {
    currentState = STATE_DISABLED;
    logEvent("STATE -> DISABLED (stop btn)");
  }
}

void loop() {
  if (currentState != STATE_DISABLED) {
    updateVentPosition();
  }

  switch (currentState) {
    case STATE_DISABLED: handleDisabledState(); break;
    case STATE_IDLE: handleIdleState(); break;
    case STATE_RUNNING: handleRunningState(); break;
    case STATE_ERROR: handleErrorState(); break;
  }
}