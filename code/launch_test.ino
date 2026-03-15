//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// You can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <WiFi.h>

#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "Launch Test"
#define REMOTEXY_WIFI_PASSWORD "nimbus2024"
#define REMOTEXY_SERVER_PORT 6377


// RemoteXY configuration
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 51 bytes
  { 255, 1, 0, 0, 0, 44, 0, 16, 31, 1, 10, 48, 23, 40, 15, 15, 4, 26, 31, 79,
    78, 0, 31, 79, 70, 70, 0, 129, 0, 6, 28, 52, 6, 24, 83, 84, 79, 32, 80, 69,
    82, 32, 76, 65, 78, 67, 73, 65, 82, 69, 0 };

// This structure defines all the variables and events of your control interface
struct {

  // input variables
  uint8_t lanciato;  // =1 if state is ON, else =0

  // other variables
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

// Preferences for flash-stored variables
#include <Preferences.h>

Preferences preferences;

// FILE SYSTEM INCLUDES
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <FS.h>
#include "SPIFFS.h"
#include <SPIFFS.h>

// Sensors
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

// Linear algebra
#include <BasicLinearAlgebra.h>
#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / M_PI)
#define G_CONST (9.81)
using namespace BLA;

// LED colors
#define ledbianco() (digitalWrite(RGB_BUILTIN, HIGH))                                // Turn the RGB LED white
#define ledspento() (digitalWrite(RGB_BUILTIN, LOW))                                 // Turn the RGB LED off
#define ledrosso() (neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0))                 // Red
#define ledverde() (neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0))                 // Green
#define ledblue() (neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS))                  // Blue
#define ledazzurro() (neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, RGB_BRIGHTNESS))  // Cyan
#define ledviola() (neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS))    // Violet
#define ledgiallo() (neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0))   // Yellow
#define ledoffblack() (neopixelWrite(RGB_BUILTIN, 0, 0, 0))                           // Off / black

#define BUZZER_PIN 21
#define BNO08X_RESET 20
#define SEALEVELPRESSURE_HPA (1013.25)
#define ACCTRASHOLD 20


// FILE SYSTEM
File file_flash;
int bytesWritten_flash = 0;
char* linea = (char*)malloc(1000 * sizeof(char));

File file_log;
int bytesWritten_log = 0;

// SD CARD FILES
File flash_sd;
File log_sd;


// State vectors
// Accelerometer column vector
BLA::Matrix<3> acc = { 0, 0, 0 };
// Vertical acceleration
float acc_vert = 0;
// Vertical velocity and altitude
float base_altitude = 0;
float altitude_baro = 0;
float h_baro = 0;
float sealevelpressure = 0;
BLA::Matrix<2, 1> S_h = { 0, 0 };
BLA::Matrix<2, 2> A_h = { 1.000, 0.004,
                          0.000, 1.000 };
BLA::Matrix<2, 1> B_h = { 0.5 * 0.004 * 0.004, 0.004 };
BLA::Matrix<1, 2> C_h = { 1, 0 };
BLA::Matrix<2, 2> U_h = { 0, 0,
                          0, 0 };
float std_dev_acc = pow(0.50, 2);                  // 0.50^2 m/s^2 accelerometer standard deviation
BLA::Matrix<1, 1> std_dev_baro = { pow(0.2, 2) };  // 0.2 m barometer uncertainty
float std_dev_acc_attitude = pow(3,2);             // Accelerometer standard deviation for attitude
float std_dev_mag_attitude = pow(5,2);             // Magnetometer standard deviation for attitude
BLA::Matrix<2, 1> K_h = { 0, 0 };
BLA::Matrix<2, 2> I = {
  1,
  0,
  0,
  1,
};
BLA::Matrix<1, 1> M = { 0 };

// Gyroscope
BLA::Matrix<3> gyro = { 0, 0, 0 };
// Magnetometer
BLA::Matrix<3> mag = { 0, 0, 0 };
// Attitude
BLA::Matrix<3> attitude_kalman = { 0, 90, 90 };
BLA::Matrix<3> attitude_acc = { 0, 0, 0 };
BLA::Matrix<3> uncertainty_attitude = { 5 * 5, 2 * 2, 2 * 2 };
BLA::Matrix<3> kalman_gain = {
  0,
  0,
  0,
};
float h_kalman;
float v_kalman;


long int last_sample_gyro = 0;
float dt_gyro = 0;
long int last_sample_acc = 0;
float dt_acc = 0;
long int last_sample_bmp = 0;


// Flags indicating if sensor samples arrived in the current loop iteration
bool new_acc = false;
bool new_gyro = false;
bool new_mag = false;
bool new_baro = false;
unsigned long t_accensionemotore = 0;
unsigned long int t5;


// Accelerometer calibration matrices
BLA::Matrix<3, 3> A_cal_acc = { 1.0084 ,  0.0111,   -0.0143,
                                0.0119,   0.9996,    0.0145,
                                0.0028,   0.0139,    1.0010 };
BLA::Matrix<3> b_cal_acc = { 0.1498,   0.1510,   0.1626 };


// SENSORS
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValueIMU;

// Controller state
int stato = 1;
// Flight number count
unsigned int n_flight;


void setup() {
  // Serial initialization
  Serial.begin(115200);
  while (!Serial) delay(100);
  delay(3000);
  Serial.println("ROCKET-SYSTEM");

  // REMOTE XY
  RemoteXY_Init();

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // I2C SET-UP
  Wire.begin();
  Wire.setClock(3400000);  // Optimal clock speed identified after extensive testing
  // TODO you setup code

  // PREFERENCES
  // Setup preferences for flash-based variable storage
  if (!preferences.begin("my-app", false)) {
    Serial.println("Error initializing preferences");
    while (1)
      ;
  }
  n_flight = preferences.getUInt("n_flight", 0);
  Serial.println("PREFERENCES:\tOK");
  Serial.print("Flight number: ");
  Serial.println(n_flight);

  // FILESYSTEM
  if (SPIFFS.begin()) {
    Serial.println("SPIFFS:\tOK");
  } else {
    Serial.println("Failed to mount SPIFFS flash file system");
    while (1)
      ;
  }
  // List files currently on flash
  list_file_flash();
  // If flash contains data, move it to SD card 
  if (move_file_flash_sd("/old_data")) {
    // If move is successful, delete flash files
    remove_file_flash();
  }

  // Open flash file for writing
  file_flash = SPIFFS.open(String("/flash_data_") + n_flight + String(".csv"), "w");
  if (!file_flash) {
    Serial.println("Error opening flash data file for writing");
    while (1)
      ;
  }
  bytesWritten_flash += file_flash.print("time_millis, v_kalman, h_kalman, attitude_kalman_x, attitude_kalman_y, attitude_kalman_z, gyro_x, gyro_y, gyro_z, h_baro, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z");

  // Open log file for writing
  file_log = SPIFFS.open(String("/log_file_") + n_flight + String(".txt"), "w");
  if (!file_log) {
    Serial.println("Error opening log file for writing");
    while (1)
      ;
  }
  bytesWritten_log += file_log.println("Critical Events");

  // INITIALIZE SENSORS
  // BNO085 IMU
  if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("BNO085 not found!");
    while (1)
      ;
  }
  Serial.println("BNO085:\tOK");
  // Set up IMU reporting
  setReports();
  // BMP390 Barometer
  if (!bmp.begin_I2C(119, &Wire)) {
    Serial.println("BMP390 not found!");
    while (1)
      ;
  }
  Serial.println("BMP390:\tOK");
  // Set up barometer parameters
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  // Initialize ground altitude
  Serial.println("Initializing ground altitude");
  setGroundAltitude();

  Serial.println("SETUP COMPLETE");

  delay(1000);
}

// ========================= FUNCTIONS =========================

void setReports(void) {
  Serial.println("setReports");
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER, 4000)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 2500)) {
    Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable raw magnetometer");
  }
}

void list_file_flash() {
  // List all files in SPIFFS
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  Serial.println("Flash file list: ");
  while (file) {
    Serial.print("FILE: ");
    Serial.println(file.name());

    file = root.openNextFile();
  }
  Serial.println();
  file.close();
  root.close();
}

bool move_file_flash_sd(String dir) {
  ledgiallo();
  // Mount SD card
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return false;
  }
  // Create directory if it doesn't exist
  SD.mkdir(dir);
  // List files for migration
  File root_flash = SPIFFS.open("/");
  File file = root_flash.openNextFile();
  while (file) {
    Serial.print("Moving file: ");
    Serial.println(file.name());
    // Open a file on SD with the same name as the flash file
    String file_path = file.name();
    file_path = dir + "/" + file_path;
    // SD file name
    Serial.print("Creating file: ");
    Serial.println(file_path);
    // Create SD file
    File file_sd = SD.open(file_path, FILE_WRITE);
    // Verify file opened successfully
    if (!file_sd) {
      Serial.println("Failed to open SD file for writing");
      while (1)
        ;
    }
    Serial.println("Starting file migration to SD");
    String riga_da_spostare = "";
    while (file.available()) {
      riga_da_spostare = file.readStringUntil('\n');
      file_sd.println(riga_da_spostare);
    }
    // Close SD file
    file_sd.close();
    // Close flash file
    file.close();
    // Proceed to next file
    file = root_flash.openNextFile();
  }
  root_flash.close();
  ledspento();
  return true;
}

void remove_file_flash() {
  Serial.println("remove");
  ledgiallo();
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.print("Deleting file: ");
    Serial.println("/" + String(file.name()));
    // Delete file
    SPIFFS.remove("/" + String(file.name()));
    // Proceed to next file
    file.close();
    file = root.openNextFile();
  }
  root.close();
  ledspento();
}

void print_su_flash() {
  // Data log format: time, v_kalman, h_kalman, attitude_kalman_x, attitude_kalman_y, attitude_kalman_z, gyro_x, gyro_y, gyro_z, h_baro, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z
  long unsigned int t = millis();
  sprintf(linea, "%lu, %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", t, v_kalman, h_kalman, attitude_kalman(0), attitude_kalman(1), attitude_kalman(2), gyro(0), gyro(1), gyro(2), h_baro, acc(0), acc(1), acc(2), mag(0), mag(1), mag(2));
  //Serial.println(linea);
  bytesWritten_flash += file_flash.print(linea);
}

void log_flash(String log) {
  log = String(millis()) + String(": ") + log + String("\n");
  bytesWritten_log += file_log.print(log);
}

// ========================= KALMAN FILTER =========================

// Estimate rocket state (attitude, vertical velocity, and altitude above ground)
void stima_stato_razzo() {
  readIMU();
  readBaro();
  if (new_gyro) {
    // Attitude prediction
    attitude_kalman = attitude_kalman + gyro * dt_gyro;
    // Update prediction uncertainty
    uncertainty_attitude(0) = uncertainty_attitude(0) + pow(dt_gyro, 2) * 2 * 2;  // 2^2 is the gyroscope standard deviation
    uncertainty_attitude(1) = uncertainty_attitude(1) + pow(dt_gyro, 2) * 2 * 2;
    uncertainty_attitude(2) = uncertainty_attitude(2) + pow(dt_gyro, 2) * 2 * 2;
  }
  // Calculate Euler angles from accelerometer and magnetometer
  calc_attitude_acc();

  if (new_acc) {
    // Calculate Kalman gain for Y
    kalman_gain(1) = uncertainty_attitude(1) / (uncertainty_attitude(1) + std_dev_acc_attitude);  // Based on accelerometer angular standard deviation
    // Update estimate
    attitude_kalman(1) = attitude_kalman(1) + kalman_gain(1) * (attitude_acc(1) - attitude_kalman(1));
    // Calculate Kalman gain for Z
    kalman_gain(2) = uncertainty_attitude(2) / (uncertainty_attitude(2) + std_dev_acc_attitude);
    // Update estimate
    attitude_kalman(2) = attitude_kalman(2) + kalman_gain(2) * (attitude_acc(2) - attitude_kalman(2));

    // Update estimation uncertainty
    uncertainty_attitude(1) = (1 - kalman_gain(1)) * uncertainty_attitude(1);
    uncertainty_attitude(2) = (1 - kalman_gain(2)) * uncertainty_attitude(2);

    // Calculate vertical acceleration
    calc_acc_vert();
    // Update sampling time in the state transition matrix
    A_h(0,1) = dt_acc;
    // Update input matrix B
    B_h(0) = 0.5 * pow(dt_acc,2);
    B_h(1) = dt_acc;
    // Velocity and altitude prediction
    S_h = A_h * S_h + B_h * acc_vert;
    // Update prediction uncertainty
    U_h = A_h * U_h * ~A_h + (B_h * ~B_h) * std_dev_acc;
  }

  if (new_mag) {
     // Calculate Kalman gain for X
    kalman_gain(0) = uncertainty_attitude(0) / (uncertainty_attitude(0) + std_dev_mag_attitude);
    // Update estimate
    attitude_kalman(0) = attitude_kalman(0) + kalman_gain(0) * (attitude_acc(0) - attitude_kalman(0));
    // Update estimation uncertainty
    uncertainty_attitude(0) = (1 - kalman_gain(0)) * uncertainty_attitude(0);
  }
    
  if (new_baro) {
    // Update vertical velocity and altitude estimates
    // Update uncertainty
    // Calculate Kalman gain
    K_h = U_h * ~C_h * Inverse(C_h * U_h * ~C_h + std_dev_baro);
    // M represents the altitude measurement
    M = {h_baro};
    // Update estimate
    S_h = S_h + K_h * (M - C_h * S_h);
    // Update estimation uncertainty
    U_h = (I - K_h * C_h) * U_h;
  }
  h_kalman=S_h(0,0);
  v_kalman=S_h(1,0);
}

void calc_acc_vert() {
  // Calculate acceleration on the inertial X-axis
  acc_vert = +cos(degToRad(attitude_kalman(1))) * cos(degToRad(attitude_kalman(2))) * acc(0)
             - sin(degToRad(attitude_kalman(2))) * cos(degToRad(attitude_kalman(1))) * acc(1)
             + sin(degToRad(attitude_kalman(1))) * acc(2);

  // Compensate for gravity
  acc_vert = acc_vert - 9.81;
}



void readBaro() {
  new_baro = false;
  if (millis() - last_sample_bmp > 10) {
    if (sealevelpressure == 0) {
      altitude_baro = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
      altitude_baro = bmp.readAltitude(sealevelpressure);
    }
    // n_baro_samples++;
    new_baro = true;
    last_sample_bmp = millis();
  }
  h_baro = altitude_baro - base_altitude;
}


void setGroundAltitude() {
  ledgiallo();
  // Perform 50 initial readings to clear transient errors
  for (int i = 0; i < 50; i++) {
    if (sealevelpressure == 0) {
      altitude_baro = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
      altitude_baro = bmp.readAltitude(sealevelpressure);
      Serial.println(altitude_baro);
    }
  }
  // Set base_altitude as the average of 500 measurements
  int n = 500;
  for (int i = 0; i < n; i++) {
    if (sealevelpressure == 0) {
      altitude_baro = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
      altitude_baro = bmp.readAltitude(sealevelpressure);
    }
    base_altitude += altitude_baro;
  }
  base_altitude /= n;
  Serial.print("Ground altitude: "); Serial.println(base_altitude);
  ledspento();
}

void readIMU() {
  new_acc = false;
  new_gyro = false;
  new_mag = false;

  if (bno08x.wasReset()) {
    Serial.println("BNO085 was reset");
    setReports();
  }

  // Retrieve sensor event from IMU
  while (!bno08x.getSensorEvent(&sensorValueIMU)) ;

  long int mill;
  switch (sensorValueIMU.sensorId) {
    case SH2_RAW_ACCELEROMETER:
      acc(0) = -sensorValueIMU.un.rawAccelerometer.y / 65535. * 156.96;
      acc(1) = sensorValueIMU.un.rawAccelerometer.x / 65535. * 156.96;
      acc(2) = sensorValueIMU.un.rawAccelerometer.z / 65535. * 156.96;

      cal_acc();

      mill = millis();
      if (last_sample_acc != 0) {
        dt_acc = (mill - last_sample_acc) * 0.001;
      }
      last_sample_acc = mill;

      //n_acc_samples++;
      new_acc = true;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      mag(0) = sensorValueIMU.un.magneticField.x;
      mag(1) = sensorValueIMU.un.magneticField.y;
      mag(2) = sensorValueIMU.un.magneticField.z;
      //n_mag_samples++;
      new_mag = true;
      break;
    case SH2_GYROSCOPE_CALIBRATED:  // rad/s
      gyro(0) = degrees(sensorValueIMU.un.gyroscope.x);
      gyro(1) = degrees(sensorValueIMU.un.gyroscope.y);
      gyro(2) = degrees(sensorValueIMU.un.gyroscope.z);

      mill = millis();
      if (last_sample_gyro != 0) {
        dt_gyro = (mill - last_sample_gyro) * 0.001;
      }
      last_sample_gyro = mill;

      //n_gyro_samples++;
      new_gyro = true;
      break;
  }
}

// Accelerometer calibration: c = A * u + b (u=uncalibrated, c=calibrated)
void cal_acc() {
  acc = A_cal_acc * acc + b_cal_acc;
}

void calc_attitude_acc() {
  if (new_acc) {
    // Calculate Euler angles for Y and Z
    attitude_acc(1) = atan2(acc(2), sqrt(pow(acc(1), 2) + pow(acc(0), 2)));
    attitude_acc(2) = atan2(-acc(1), acc(0));
    // Convert angles to degrees
    attitude_acc(1) = degrees(attitude_acc(1));
    attitude_acc(2) = degrees(attitude_acc(2));
  }
  if (new_mag) {
    // Transform Y and Z angles
    float acc_rad_y = degToRad(attitude_acc(1));
    float acc_rad_z = degToRad(attitude_acc(2));
    attitude_acc(0) = - atan2(mag(0) * sin(acc_rad_y) - mag(1) * cos(acc_rad_y), mag(2) * cos(acc_rad_z) + sin(acc_rad_z) * (mag(1) * sin(acc_rad_y) + mag(0) * cos(acc_rad_y)));
    attitude_acc(0) = degrees(attitude_acc(0));
  }
}



// ========================= MAIN LOOP =========================

void loop() {
  RemoteXY_Handler();
  switch (stato) {
    case 1:
      {
        // Waiting for user interface connection
        if (RemoteXY_isConnected()) {
          stato = 2;
        }
        break;
      }
    case 2:
      {
        RemoteXY_Handler();
        // Rocket state estimation
        stima_stato_razzo();
        // Waiting for user launch feedback
        if (RemoteXY.lanciato) {
          stato = 3;
          tone(BUZZER_PIN, 3000, 1000);
          ledverde();
          stima_stato_razzo();
          // Data logging initiation
          print_su_flash();
          // Log launch attempt
          log_flash("launch attempt");
        }
        break;
      }
    case 3:
      {
        stima_stato_razzo();
        print_su_flash();
        // Motor ignition sequence
        t_accensionemotore = millis();  // Record ignition timestamp
        log_flash("attempting motor ignition");
        bool acceso = false;
        while (!acceso && millis() - t_accensionemotore < 2000) {
          stima_stato_razzo();
          print_su_flash();
          if (acc(0) >= ACCTRASHOLD) {
            acceso = true;
          }
        }
        if (acceso) {
          log_flash("launch successful");
          stato = 4;
          ledblue();
        }
        break;
      }
    case 4:
      {
        stima_stato_razzo();
        print_su_flash();
        // Apogee detection
        if (v_kalman < 0) {
          tone(BUZZER_PIN, 4000, 1000);
          ledgiallo();
          log_flash("apogee reached");
          stima_stato_razzo();
          print_su_flash();
          t5=millis();
          stato = 5;
          // Start timer to determine if vehicle has landed/stopped
        }
        break;
      }
    case 5:
      {
        stima_stato_razzo();
        print_su_flash();
        // Monitoring for touchdown (zero velocity check)
        if (v_kalman >= -0.3 && v_kalman <= 0.1) {
          if (millis() - t5 > 3000) {
            ledverde();
            tone(BUZZER_PIN, 2000, 1000);
            // Close data and log files
            file_log.close();
            file_flash.close();
            // Migrate files to SD card
            if (move_file_flash_sd(String("/flight_") + n_flight)) {
              // Clear flash storage after migration
              remove_file_flash();
            }
            // Increment flight counter
            n_flight++;
            // Save counter to non-volatile storage
            preferences.putUInt("n_flight", n_flight);
            preferences.end();
            stato = 6;
          } else {
            t5 = millis();
          }
        }
        break;
      }
    case 6:
      {
        // End of mission
      }
  }
}
