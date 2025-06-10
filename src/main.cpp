#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>


#define EN_PIN     12   // Enable (LOW = enabled)
#define DIR_PIN    13   // Direction
#define STEP_PIN   14   // STEP pin
#define CS_PIN     2    // SPI CS
#define MOSI_PIN   23
#define MISO_PIN   19
#define CLK_PIN    18
#define STOP_PIN   27   // Optional STOP input
#define POT_PIN    4    // Potentiometer analog input

#define R_SENSE 0.075f// Sense resistor used on your board, adjust if needed


TMC5160Stepper driver = TMC5160Stepper(CS_PIN, R_SENSE);

// Optional: limit range of STEP PWM (Hz)
const int MIN_STEP_FREQ = 10;
const int MAX_STEP_FREQ = 2000;


unsigned long lastStepTime = 0;
unsigned long stepInterval = 1000;  // µs between steps


hw_timer_t *stepTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool step_state = false;
int step_frequency = 200;

volatile uint32_t stepIntervalMicros = 1000; // Start with 1 kHz

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  digitalWrite(STEP_PIN, step_state);
  step_state = !step_state;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setupDriver() {

  SPI.begin();  // Init SPI bus

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);   // Enable driver
  digitalWrite(DIR_PIN, LOW);  // Set default direction

  driver.begin();
  driver.rms_current(600);        // mA — adjust to match your NEMA17
  driver.microsteps(16);          // You can try 16, 32, 64, or 256
  driver.en_pwm_mode(true);       // Enable extremely quiet StealthChop
  driver.pwm_autoscale(true);     // Let it scale automatically
  driver.TCOOLTHRS(0xFFFFF);      // Threshold for switching to StealthChop
  driver.THIGH(0);                // Keep it in StealthChop
  driver.diag0_stall(false);      // Don't use stallguard for now
  driver.pwm_freq(1);               // Higher PWM freq for quieter operation
  driver.pwm_ampl(180);             // PWM amplitude (lower = quieter, but less torque)
  driver.pwm_grad(10);              // PWM gradient (steepness)

  driver.TPWMTHRS(500);             // Keep stealthChop active at higher speeds


}


void setup() {

  Serial.begin(115200);
  setupDriver();
  pinMode(POT_PIN, INPUT);
//  setupTimer(step_frequency);  // Initial freq

  stepTimer = timerBegin(0, 80, true); // 80 prescaler → 1 tick = 1µs
  timerAttachInterrupt(stepTimer, &onTimer, true);
  timerAlarmWrite(stepTimer, stepIntervalMicros, true);
  timerAlarmWrite(stepTimer, stepIntervalMicros, true);
  timerAlarmEnable(stepTimer);

}

void loop() {

// Read and map pot value
  int potRaw = analogRead(POT_PIN);  // 0–4095
  int freq = map(potRaw, 0, 4095, 500, 100000);  // Range: 500 Hz to 100 kHz

  // Quantize to nearest 100 Hz
  int quantizedFreq = ((freq + 150) / 300) * 300;

  // Update timer only if quantized frequency changed
  static int lastFreq = 0;
  if (quantizedFreq != lastFreq) {
    uint32_t newInterval = 1000000UL / (quantizedFreq * 2);  // half-period

    portENTER_CRITICAL(&timerMux);
    timerAlarmWrite(stepTimer, newInterval, true);
    portEXIT_CRITICAL(&timerMux);

    lastFreq = quantizedFreq;

    Serial.print("Raw: ");
    Serial.print(potRaw);
    Serial.print(" -> Freq: ");
    Serial.print(quantizedFreq);
    Serial.print(" Hz -> Interval: ");
    Serial.println(newInterval);
  }

  delay(10); // Give ADC some breathing room
}

