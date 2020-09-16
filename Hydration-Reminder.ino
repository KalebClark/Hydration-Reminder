/*
 * Hydration Reminder
 * By: Kaleb Clark
 * 
 * Playlists for this code
 * - Steely Dan, Slayer, B52's, Talulah Gosh, Social Distortion
 */
#include <CircularBuffer.h>
#include <Arduino_LSM6DS3.h>
#include <math.h>

#define DEBUG 1

/*
 * Variables
 */
// Pins
const int led00_pin = 10;
const int led01_pin = 11;
const int vibe_pin  = 9;
const int piezo_pin = 12;
const int batt_pin = 7;

// States
int led00_state = LOW;
int led01_state = LOW;
int tone_state = HIGH;
int vibe_state = LOW;
int no_drink = 0;

// Timer vars
unsigned long led_prev_millis = 0;
unsigned long vibe_prev_millis = 0;
unsigned long tone_prev_millis = 0;
unsigned long drink_prev_millis = 0;

// Interval Vars
const long led_blink_interval = 166.5;
const long vibe_interval = 333;
const long tone_interval = 666;

// Drink Interval. First number is in minutes. Change that.
const long drink_interval = (15 * 60) * 1000;

// Vars for calibration
const int calibration_duration = 1000;
float ax_offset, ay_offset, az_offset;
float gx_offset, gy_offset, gz_offset;

// Tuning variables
int queue_max = 250;
float tilt_threshold = 3.00;
float min_threshold = 2.00;

CircularBuffer<float, 30> tilt_queue;

int wait_time = 1000;
unsigned long end_time;
bool waiting = 0;
int tilt_count = 0;


/*
 * setup()
 * Arduino Setup Method
 * ============================================================================
 */
void setup() {
  // Serial Setup
  Serial.begin(115200);
  
  // Pin Modes
  pinMode(led00_pin,  OUTPUT);
  pinMode(led01_pin,  OUTPUT);
  pinMode(vibe_pin,   OUTPUT);
  pinMode(piezo_pin,  OUTPUT);
  pinMode(batt_pin, INPUT);

  // IMU Setup
  if(!IMU.begin()) { Serial.println("Failed to init IMU!"); while(1); }
  imu_calibrate();
}

/*
 * loop()
 * Arduino Main Loop
 * ============================================================================
 */
void loop() {
  // Bypass Everything when charging
  if(digitalRead(batt_pin) == HIGH) {
    return;
  }

  unsigned long current_millis = millis();
  if(current_millis - drink_prev_millis >= drink_interval) {
    drink_prev_millis = current_millis;
    no_drink = true;
    Serial.println("No Drink in a while!");
  }

  /*
   * Gather data for tilt check
   * TODO: Gather drink timer data here.
   */
  if(IMU.accelerationAvailable()) {
    float pitch = getPitch();             // Get current pitch

    if(pitch > min_threshold) {           // Check to see if it is above minimum
      tilt_queue.push(pitch);             // Add current pitch to circular buffer

      if(pitch > tilt_threshold && waiting == 0) {
        end_time = millis();
        waiting = 1;
      }

      if(waiting == 1 && (millis() - end_time) >= wait_time) {
        if(queueTotal() > queue_max) {
          waiting = 0;
          tilt_count++;
        } else {
          waiting = 0;
        }
      }
    } else {
      tilt_queue.push(0.00);
    }
  }

  /*
   * Tilt Check.
   * Turn off audio after one second so that its not beeping in my ear. After 3
   * seconds of annoyance, user has probably taken a drink. Give them a break.
   * Delay for 10 seconds to let them finish their drink without any sensor data.
   */
  if(tilt_count == 1) {
    digitalWrite(piezo_pin, LOW);
  } else if(tilt_count == 3) {
    Serial.println("Turn off Annoyers");
    tilt_count = 0;
    no_drink = false;
    annoyOff();
    delay(10000);
    Serial.println("Grace Period Over");
    drink_prev_millis = millis();             // Reset Drink Timer
  }

  if(no_drink) {
    nbLedAnnoy();
    nbVibeAnnoy();
    nbToneAnnoy();
  } else {
    annoyOff();
  }
}

void annoyOff() {
  digitalWrite(led00_pin, LOW);
  digitalWrite(led01_pin, LOW);
  digitalWrite(vibe_pin, LOW);
  digitalWrite(piezo_pin, LOW);
}

/*
 * nbVibeAnnoy()
 * Run Non Blocking Vibration Annoyer
 * ============================================================================
 */
void nbVibeAnnoy() {
  unsigned long current_millis = millis();
  
  if(current_millis - vibe_prev_millis >= vibe_interval) {
    vibe_prev_millis = current_millis;

    if(vibe_state == LOW) {
      vibe_state = HIGH;
    } else {
      vibe_state = LOW;
    }
    digitalWrite(vibe_pin, vibe_state);
  }
}

/*
 * nbLedAnnoy()
 * Run Non Blocking LED Annoyer
 * ============================================================================
 */
void nbLedAnnoy() {
  unsigned long current_millis = millis();

  if(current_millis - led_prev_millis >= led_blink_interval) {
    led_prev_millis = current_millis;

    if(led00_state == LOW) {
      led00_state = HIGH;
      led01_state = LOW;
    } else {
      led00_state = LOW;
      led01_state = HIGH;
    }
    digitalWrite(led00_pin, led00_state);
    digitalWrite(led01_pin, led01_state);
  }
}

/*
 * nbToneAnnoy()
 * Run Non Blocking Tone Annoyer
 * ============================================================================
 */
void nbToneAnnoy() {
  unsigned long current_millis = millis();

  if(current_millis - tone_prev_millis >= tone_interval) {
    tone_prev_millis = current_millis;

    if(tone_state == LOW) {
      tone_state = HIGH;
    } else {
      tone_state = LOW;
    }
    digitalWrite(piezo_pin, tone_state);
  }
}

/*
 * getPitch()
 * ===================================================================
 */
float getPitch() {
  float x, y, z; 
  double pitch = 0.00;

  if(IMU.accelerationAvailable()) {
    getAcceleration(x, y, z);
    double x_buff = float(x);
    double y_buff = float(y);
    double z_buff = float(z);

    pitch = atan2((- x_buff), sqrt(y_buff * y_buff + z_buff * z_buff)) * 57.296;
    if(pitch < 0) {
      return pitch * -1;
    } else {
      return pitch;
    }
  }
}

/*
 * queueTotal()
 * Get the sum of the queue
 * ===================================================================
 */
float queueTotal() {
  float tq_sum = 0.00;
  using index_t = decltype(tilt_queue)::index_t;
  for (index_t i = 0; i < tilt_queue.size(); i++) {
    tq_sum += (float)tilt_queue[i];
  }
  return tq_sum;
}

/*
 * getAcceleration()
 * ===================================================================
 */
int getAcceleration(float& x, float& y, float& z) {
  float tx, ty, tz;

  IMU.readAcceleration(tx, ty, tz);
  x = (ax_offset - tx);
  y = (ay_offset - ty);
  z = (az_offset - tz);

  return 1;
}

/*
 * getGyroscope()
 * ===================================================================
 */
int getGyroscope(float& x, float& y, float& z) {
  float tx, ty, tz;

  IMU.readGyroscope(tx, ty, tz);
  x = (gx_offset - tx);
  y = (gy_offset - ty);
  z = (gz_offset - tz);

  return 1;
}

/*
 * imu_calibrate()
 * Calibrate the IMU by finding current values over 1 second and averaging them,
 * then store them in offset variables for use.
 * ============================================================================
 */
void imu_calibrate() {
  Serial.println("===: Start Calibration =========================");
  
  float avg_ax, avg_ay, avg_az, avg_gx, avg_gy, avg_gz;
  int accel_cnt = 0; int gyro_cnt = 0;

  unsigned long time_now = millis();

  while(millis() < time_now + calibration_duration) {
    float ax, ay, az, gx, gy, gz;

    // Gather Accelerometer Data
    if(IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      avg_ax += ax; avg_ay += ay; avg_az += az;
      accel_cnt++;
    }

    // Gather Gyroscope Data
    if(IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      avg_gx += gx; avg_gy += gy; avg_gz += gz;
      gyro_cnt++;
    }
  }

  // Calculate offsets
  ax_offset = (avg_ax / accel_cnt); ay_offset = (avg_ay / accel_cnt); az_offset = (avg_ax / accel_cnt);
  gx_offset = (avg_gx / gyro_cnt); gy_offset = (avg_gy / gyro_cnt); gz_offset = (avg_gz / gyro_cnt);

  if(DEBUG) {
    Serial.print("Accelerometer Calibration. Samples: "); Serial.println(accel_cnt);
    Serial.println("Axis: Total / Offset");
    Serial.print("X: ");
    Serial.print(avg_ax); Serial.print(" / "); Serial.println(ax_offset);

    Serial.print("Y: ");
    Serial.print(avg_ay); Serial.print(" / "); Serial.println(ay_offset);

    Serial.print("Z: ");
    Serial.print(avg_az); Serial.print(" / "); Serial.println(az_offset);

    // Gyroscope Debug 
    Serial.print("\nGyroscope Calibration. Samples: "); Serial.println(gyro_cnt);
    Serial.println("Axis: Total / Offset");
    Serial.print("X: ");
    Serial.print(avg_gx); Serial.print(" / "); Serial.println(gx_offset);

    Serial.print("Y: ");
    Serial.print(avg_gy); Serial.print(" / "); Serial.println(gy_offset);

    Serial.print("Z: ");
    Serial.print(avg_gz); Serial.print(" / "); Serial.println(gz_offset);
  }
}
