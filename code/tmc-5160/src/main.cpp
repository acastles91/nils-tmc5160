#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <stdlib.h>


// =================== CONFIG ===================
// Uncomment one of the two roles
//#define ROLE_MASTER
#define ROLE_SLAVE


#define EN_PIN     12   // Enable (LOW = enabled)
#define DIR_PIN    13   // Direction
#define STEP_PIN   14   // STEP pin
#define CS_PIN     2    // SPI CS
#define MOSI_PIN   23
#define MISO_PIN   19
#define CLK_PIN    18
#define STOP_PIN   27   // Optional STOP input
#define POT_PIN    36    // Potentiometer analog input
#define UP_PIN     5   // Optional UP input
#define DOWN_PIN   25   // Optional DOWN input

#define R_SENSE 0.11f// Sense resistor used on your board, adjust if needed

#define QUANT_STEP_HZ 300
#define MIN_FREQ_HZ   0
#define MAX_FREQ_HZ   12000


#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8
#define STALL_VALUE 0



uint8_t slaveAddress[] = {0xA0, 0xB7, 0x65, 0x48, 0xBB, 0x18};


TMC5160Stepper driver = TMC5160Stepper(CS_PIN, R_SENSE);

// Optional: limit range of STEP PWM (Hz)

const int MIN_STEP_FREQ = 10;
const int MAX_STEP_FREQ = 2000;


unsigned long lastStepTime = 0;
unsigned long stepInterval = 1000;  // µs between steps


hw_timer_t *stepTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool stepState = false;
int step_frequency = 200;

volatile uint32_t stepIntervalMicros = 1000; // Start with 1 kHz


typedef struct struct_message {
  uint16_t targetFreqHz;
  bool enableMotor;
} struct_message;

struct_message data;

#ifdef ROLE_MASTER
void sendCommand(uint16_t freq, bool enable) {
  data.targetFreqHz = freq;
  data.enableMotor = enable;
  esp_now_send(slaveAddress, (uint8_t*)&data, sizeof(data));
}

uint16_t quantizeFreq(uint16_t freq) {
  return min(max((freq + QUANT_STEP_HZ / 2) / QUANT_STEP_HZ * QUANT_STEP_HZ, MIN_FREQ_HZ), MAX_FREQ_HZ);
}

void setup() {

  pinMode(POT_PIN, INPUT);
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Master ready.");
}

void loop() {

    int potRaw = analogRead(POT_PIN);  // 0–4095
    int freq = map(potRaw, 0, 4095, 500, 22000);  // 500 Hz to 100 kHz
    int quantizedFreq = ((freq + 300) / 600) * 600;
    
    Serial.printf("Pot: %d → Freq: %d Hz\n", potRaw, quantizedFreq);
    delay(100);

    static int lastFreq = 0;
    if (quantizedFreq != lastFreq) {
      uint32_t newInterval = 1000000UL / (quantizedFreq * 2);  // half-period
      sendCommand(quantizedFreq, true);  // enable motor/
      digitalWrite(EN_PIN, LOW); // enable motor

      Serial.print("Local: Pot=");
      Serial.print(potRaw);
      Serial.print(" → Freq=");
      Serial.println(quantizedFreq);

      lastFreq = quantizedFreq;
    }
}


#endif

#ifdef ROLE_SLAVE

bool localControlMode = false;  // true = pot control, false = wireless control
bool remoteControlMode = false;
bool screenMode = true;
bool screenMoving = false;
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  stepState = !stepState;
  digitalWrite(STEP_PIN, stepState);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setStepFrequency(uint16_t freq) {
  if (freq == 0) {
    ledcWrite(PWM_CHANNEL, 0); // stop PWM
  } else {
    ledcWriteTone(PWM_CHANNEL, freq); // set PWM frequency (50% duty)
    //ledcWrite(PWM_CHANNEL, 128);      // 128/255 ~ 50% duty
  }
}


void updateTimer(uint16_t freq) {
  if (freq == 0) {
    timerAlarmDisable(stepTimer);
    digitalWrite(STEP_PIN, LOW);
    return;
  }
  uint32_t intervalMicros = 1000000UL / (freq * 2); // Toggle twice per full wave
  timerAlarmWrite(stepTimer, intervalMicros, true);
  timerAlarmEnable(stepTimer);
}

void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));
  if (!localControlMode) {
    digitalWrite(EN_PIN, data.enableMotor ? LOW : HIGH);
    //updateTimer(data.targetFreqHz);
    setStepFrequency(data.targetFreqHz);
    Serial.printf("Remote: freq=%d Hz, enable=%s\n", data.targetFreqHz, data.enableMotor ? "true" : "false");
  }
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
  driver.rms_current(1200);        // mA — adjust to match your NEMA17
  driver.microsteps(16);          // You can try 16, 32, 64, or 256
  driver.en_pwm_mode(true);       //
  driver.pwm_autoscale(true);     // Let it scale automatically
  driver.TCOOLTHRS(0xFFFFF);      // Threshold for switching to StealthChop
  //driver.THIGH(0);                // Keep it in StealthChop
  driver.diag1_stall(false);  
  driver.sgt(STALL_VALUE);    
  driver.diag0_int_pushpull(true);
  driver.diag1_pushpull(true);
  driver.pwm_freq(1);               // Higher PWM freq for quieter operation
  driver.pwm_ampl(180);             // PWM amplitude (lower = quieter, but less torque)
  driver.pwm_grad(10);              // PWM gradient (steepness)
  driver.toff(4);  // Enables driver output (minimum value usually required)
//driver.stallguard()

  //driver.TPWMTHRS(500);             // Keep stealthChop active at higher speeds

  //driver.en_spreadCycle(false);  // Force StealthChop
  driver.pwm_autograd(true);     // Auto adjust gradient


}
void resetDriver() {
  Serial.println("Resetting driver...");
  driver.toff(0);           // disable driver output
  driver.GSTAT(0b111);      // clear errors
  delay(50);                // small pause
  driver.toff(4);           // re-enable driver output
  driver.push();            // re-apply your configuration
  Serial.println("Driver reset complete.");
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for Serial port to connect if needed (on some boards)
  };

  setupDriver();
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);


  digitalWrite(DIR_PIN, LOW); // Set direction constant
  digitalWrite(EN_PIN, HIGH); // Motor off initially

  //stepTimer = timerBegin(0, 80, true); // 80 MHz / 80 = 1 MHz
  //timerAttachInterrupt(stepTimer, &onTimer, true);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Slave ready.");

  ledcSetup(PWM_CHANNEL, 1000, PWM_RESOLUTION); // Initial: 1kHz, 8-bit resolution
  ledcAttachPin(STEP_PIN, PWM_CHANNEL);

}


  void loop() {
    static bool printed = false;
if (!printed) {
  Serial.print("screenMode = ");
  Serial.println(screenMode ? "true" : "false");
  printed = true;
}
  // Local pot-based control (only if enabled)
  if (localControlMode) {
    int potRaw = analogRead(POT_PIN);  // 0–4095
    int freq = map(potRaw, 0, 4095, 500, 22000);  // 500 Hz to 100 kHz
    int quantizedFreq = ((freq + 300) / 600) * 600;

    static int lastFreq = 0;
    if (quantizedFreq != lastFreq) {
      uint32_t newInterval = 1000000UL / (quantizedFreq * 2);  // half-period
      setStepFrequency(freq);
      
      digitalWrite(EN_PIN, LOW); // enable motor

      Serial.print("Local: Pot=");
      Serial.print(potRaw);
      Serial.print(" → Freq=");
      Serial.println(quantizedFreq);

      lastFreq = quantizedFreq;
    }
  }

  if (screenMode){
    // Screen control
    if (digitalRead(UP_PIN) == LOW) {
      driver.sgt(STALL_VALUE);
      Serial.println("Up");
      digitalWrite(DIR_PIN, HIGH);
      digitalWrite(EN_PIN, LOW); // enable
      setStepFrequency(10000);
      screenMoving = true;
    } else if (digitalRead(DOWN_PIN) == LOW) {
      driver.sgt(STALL_VALUE);
      Serial.println("Down");
      digitalWrite(DIR_PIN, LOW);
      digitalWrite(EN_PIN, LOW); // enable
      setStepFrequency(10000);
      screenMoving = true;
    }

    if (screenMoving && digitalRead(UP_PIN) == HIGH && digitalRead(DOWN_PIN) == HIGH) {
      Serial.println("Stop");
      digitalWrite(EN_PIN, HIGH);  
      setStepFrequency(0);
      screenMoving = false;
    }
  }
  localControlMode = false;
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 't':
        Serial.print("Stop pin = ");
        Serial.println(digitalRead(STOP_PIN));
        break;

      case 'd':
        driver.sgt(STALL_VALUE);
        Serial.println("Down");
        digitalWrite(DIR_PIN, LOW);
        digitalWrite(EN_PIN, LOW); // enable
        setStepFrequency(10000);
        screenMoving = true;
        break;

      case 'u':
        driver.sgt(STALL_VALUE);
        Serial.println("Up");
        digitalWrite(DIR_PIN, HIGH);
        digitalWrite(EN_PIN, LOW); // enable
        setStepFrequency(10000);
        screenMoving = true;
        break;

      case 's':
        Serial.println("Stop");
        digitalWrite(EN_PIN, HIGH);  
        setStepFrequency(0);
        screenMoving = false;
        break;

      case 'm':
        localControlMode = !localControlMode;
        Serial.print("Switched to ");
        Serial.println(localControlMode ? "LOCAL" : "REMOTE");
        if (localControlMode) {
          digitalWrite(EN_PIN, LOW); // enable
        } else {
          digitalWrite(EN_PIN, HIGH); // disable until master sends enable
        }
        break;

        case 'r':
        Serial.println("Reset");
        resetDriver();
        break;

        case 'l':
        {
        uint32_t drvStatus = driver.DRV_STATUS();
        int sg_result = (drvStatus >> 10) & 0x3FF; // Bits 19:10
        Serial.print("SG_RESULT=");
        Serial.println(sg_result);
        break;
        }

      default:
        // Optionally handle unknown commands
        Serial.print("Unknown command: ");
        Serial.println(c);
        break;
    }
  }
  if (screenMode && screenMoving) {
    if (digitalRead(STOP_PIN) == HIGH) {
     Serial.println("Virtual endstop triggered. Stopping motor.");
      digitalWrite(EN_PIN, HIGH);
      //setStepFrequency(0);
      //screenMoving = false;
      uint32_t drvStatus = driver.DRV_STATUS();
      Serial.print("DRV_STATUS: 0x");
      Serial.println(drvStatus, HEX);
      //driver.sgt(0); 
       }
    }
  }


#endif
//if (Serial.available()) {
  //  char c = Serial.read();
  //  Serial.print("Received: ");
  //  Serial.println(c);
  //  // Virtual endstop test
  //  if (c == 't') {
  //    Serial.print("Stop pin = ");
  //    Serial.println(digitalRead(STOP_PIN));
  //  }
  //  // Down
  //  else if (c == 'd') {
  //    Serial.println("Down");
  //    digitalWrite(DIR_PIN, HIGH);
  //    digitalWrite(EN_PIN, LOW); // enable
  //    setStepFrequency(10000);
  //    screenMoving = true;
  //  }
  //  // Up
  //  else if (c == 'u') {
  //    Serial.println("Up");
  //    digitalWrite(DIR_PIN, LOW);
  //    digitalWrite(EN_PIN, LOW); // enable
  //    setStepFrequency(10000);
  //    screenMoving = true;
  //  }
  //  // Stop
  //  else if (c == 's') {
  //    Serial.println("Stop");
  //    digitalWrite(EN_PIN, HIGH);  
  //    setStepFrequency(0);
  //    screenMoving = false;
  //  }
  //  // Toggle control mode
  //  else if (c == 'm') {  // Changed to 'm' to avoid conflict with 't'
  //    localControlMode = !localControlMode;
  //    Serial.print("Switched to ");
  //    Serial.println(localControlMode ? "LOCAL" : "REMOTE");
  //    if (localControlMode) {
  //      digitalWrite(EN_PIN, LOW); // enable
  //    } else {
  //      digitalWrite(EN_PIN, HIGH); // disable until master sends enable
  //    }
  //  }
  //   
  //  }
  //}
  //if (screenMode && screenMoving) {
  //  if (digitalRead(STOP_PIN) == HIGH) {
  //   Serial.println("Virtual endstop triggered. Stopping motor.");
  //    digitalWrite(EN_PIN, HIGH);
  //    setStepFrequency(0);
  //    screenMoving = false;
  //}