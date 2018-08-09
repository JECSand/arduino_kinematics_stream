// Connor Sanders
// Arduino Kinematics - Sensor Binary Data Stream
// 08-09-2018

#include "Wire.h"
#include "math.h" 
#include "HMC5883Llib.h"


// I2C Address of the MPU-6050.
const int MPU_ADDR=0x68;

// Variables to Store Sensor Values
int16_t accelerometer_x, accelerometer_y, accelerometer_z, g_accelerometer_x, g_accelerometer_y, g_accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z, d_gyro_x, d_gyro_y, d_gyro_z;
int16_t initial_mag_x, initial_mag_z, cur_mag_x, cur_mag_z;
int16_t roll, pitch, yaw;
int16_t temperature;
int16_t initial_mag_angel, current_mag_angel, base_yaw, yaw_drift;
double heading;
double mag_x, mag_y, mag_z;
int i = 0;

// Temporary variable used in convert function
char tmp_str[7];

// Compass Variables
Magnetometer mag;
bool fail;

// Convert int16 to string for output to debug monitor
char* convert_int16_to_str(int16_t i) {
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(9600);

  // Setup MPU-6050 Data Stream
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Setup HMC5883 Data Stream
  if (mag.begin() != 0)
    {
        Serial.println("Error connecting to Magnetometer");
        fail = true;
        return;
    }
    mag.setGain(HMC5833L_GAIN_1370);
}

void loop() {

  // Run 1 second looped MPU-6050 Transmission Stream
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7*2, true);
  
  // Read registers to get MPU-6050 sensor values
  accelerometer_x = Wire.read()<<8 | Wire.read();
  accelerometer_y = Wire.read()<<8 | Wire.read();
  accelerometer_z = Wire.read()<<8 | Wire.read(); 
  temperature = Wire.read()<<8 | Wire.read(); 
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();

  // Cleaned Records
  g_accelerometer_x = accelerometer_x / 16384;
  g_accelerometer_y = accelerometer_y / 16384;
  g_accelerometer_z = accelerometer_z / 16384; 
  d_gyro_x = gyro_x / 131.0;
  d_gyro_y = gyro_y / 131.0;
  d_gyro_z = gyro_z / 131.0;
  

  // Run 1 second looped HMC5883 Transmission Stream
  if (fail)
      return;

  // reads the heading in degrees using the X and Y axis
  int8_t ret = mag.readHeadingDeg(&heading);

  switch (ret)
  {
      case HMC5833L_ERROR_GAINOVERFLOW:
          Serial.println("Gain Overflow");
          return;
      case 0:
          // success
          break;
      default:
          Serial.println("Failed to read Magnetometer");
          return;
  }

  int8_t mag_ret = mag.readGauss(&mag_x, &mag_y, &mag_z);
  switch (mag_ret)
  {
        case HMC5833L_ERROR_GAINOVERFLOW:
            Serial.println("Gain Overflow");
            return;
        case 0:
            // success
            break;
        default:
            Serial.println("Failed to read Magnetometer");
            return;
  }
  
  // Calculated Records
  if (i == 0) {
    initial_mag_x = mag_x;
    initial_mag_z = mag_z;
    cur_mag_x = initial_mag_x;
    cur_mag_z = initial_mag_z;
  } else {
    cur_mag_x = mag_x;
    cur_mag_z = mag_z;
  }
  roll = atan2(g_accelerometer_y, g_accelerometer_z) * 180 / PI;
  pitch = atan2(-g_accelerometer_x, sqrt(g_accelerometer_y * g_accelerometer_y + g_accelerometer_z * g_accelerometer_z)) * 180 / PI;
  initial_mag_angel = atan2(initial_mag_z, initial_mag_x);
  current_mag_angel = atan2(cur_mag_z, cur_mag_x);
  base_yaw = 180 * atan(g_accelerometer_z / sqrt(g_accelerometer_x * g_accelerometer_x + g_accelerometer_z * g_accelerometer_z)) / PI;
  yaw_drift = (initial_mag_angel - current_mag_angel) * 180 / PI;
  yaw = base_yaw - yaw_drift;
  
    
  // Print sensor data to console
  Serial.print("\ni= "); Serial.print(i);
  Serial.print(" | aX= "); Serial.print(convert_int16_to_str(g_accelerometer_x));
  Serial.print(" | aY= "); Serial.print(convert_int16_to_str(g_accelerometer_y));
  Serial.print(" | aZ= "); Serial.print(convert_int16_to_str(g_accelerometer_z));
  Serial.print(" | tmp= "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX= "); Serial.print(convert_int16_to_str(d_gyro_x));
  Serial.print(" | gY= "); Serial.print(convert_int16_to_str(d_gyro_y));
  Serial.print(" | gZ= "); Serial.print(convert_int16_to_str(d_gyro_z));
  Serial.print(" | mX= "); Serial.print(mag_x);
  Serial.print(" | mY= "); Serial.print(mag_y);
  Serial.print(" | mZ= "); Serial.print(mag_z);
  Serial.print(" | pitch= "); Serial.print(convert_int16_to_str(pitch));
  Serial.print(" | roll= "); Serial.print(convert_int16_to_str(roll));
  Serial.print(" | yaw= "); Serial.print(convert_int16_to_str(yaw));
  Serial.print(" | Heading= "); Serial.print(heading); Serial.print(" degrees");

  // 1 Second Loop Delay
  i += 1;
  delay(1000);
}
