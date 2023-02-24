// Adafruit Feather nRF52840 Sense
// IMU Peripheral

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <bluefruit.h>
#include <PDM.h>

Adafruit_NeoPixel onePixel = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

const float accel_min = -10.0;
const float accel_max = 10.0;  //range will be to be further tested, the min and value would be all that needs to change if range is not suitable

const float gyro_min = 0.0;
const float gyro_max = 40.0;

unsigned short accels[3];
unsigned short gyros[3];

unsigned short accels_previous[3];
unsigned short gyros_previous[3];

const int mean = 25; 

int avg[mean];
long tick = 0; 

typedef struct axis_t {  //t for accel union
  float x;
  float y;
  float z;
};

const int union_size = sizeof(axis_t);

typedef union axis_packet {
  axis_t structure;
  byte byteArray[union_size];
};

axis_packet accel;
axis_packet mag;
axis_packet gyro;


// 555a0002-val-467a-9538-01f0652c74e8"
#define create_UUID(val) \
  (const uint8_t[]) { \
    0xe8, 0x74, 0x2c, 0x65, 0xf0, 0x01, 0x38, 0x95, \
      0x7a, 0x46, (uint8_t)(val & 0xff), (uint8_t)(val >> 8), 0x02, 0x00, 0x5a, 0x55 \
  }

BLEService service(create_UUID(0x0000));
BLECharacteristic accelerationCharacteristic(create_UUID(0x0011));
BLECharacteristic gyroscopeCharacteristic(create_UUID(0x0012));
BLECharacteristic buttonCharacteristic(create_UUID(0x0020));

#include <Adafruit_LSM6DS33.h>
Adafruit_LSM6DS33 lsm6ds33;  //library for the accel and gyro chip built in to the board
Adafruit_Sensor* accel_sensor;

int button_value;
int button_value_previous;
const int BUTTON = 7;

void setupSensors(void) {
  lsm6ds33.begin_I2C();
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_104_HZ);
  Wire.setClock(400000);
  accel_sensor = lsm6ds33.getAccelerometerSensor();
  pinMode(BUTTON, INPUT_PULLUP);
  digitalWrite(BUTTON, HIGH);
}



void setupBLEScience(void) {
  service.begin();

  accelerationCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  accelerationCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelerationCharacteristic.setFixedLen(sizeof(accels));
  accelerationCharacteristic.begin();

  gyroscopeCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  gyroscopeCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  gyroscopeCharacteristic.setFixedLen(sizeof(gyros));
  gyroscopeCharacteristic.begin();

  buttonCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  buttonCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  buttonCharacteristic.setFixedLen(sizeof(uint8_t));
  buttonCharacteristic.begin();
}


void setup() {
  delay(1000);
  Serial.begin(57600);

  setupSensors();

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configUuid128Count(3);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  setupBLEScience();
  startAdv();

  onePixel.begin();
  onePixel.clear();
  onePixel.setBrightness(20);  // Affects all subsequent settings
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addService(service);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}

void loop() {
  if (Bluefruit.connected()) {
    updateSubscribedCharacteristics();
    delay(5);
  }
}

void updateSubscribedCharacteristics(void) {
  sensors_event_t event;

  if (accelerationCharacteristic.notifyEnabled()) {
    accel_sensor->getEvent(&event);
    accels[0] = constrain(map(event.acceleration.x, accel_min, accel_max, 0.0, 65535.0), 0, 65535);
    accels[1] = constrain(map(event.acceleration.y, accel_min, accel_max, 0.0, 65535.0), 0, 65535);
    accels[2] = constrain(map(event.acceleration.z, accel_min, accel_max, 0.0, 65535.0), 0, 65535);
    if (accels[0] != accels_previous[0] || accels[1] != accels_previous[1] || accels[2] != accels_previous[2]) {
      accels_previous[0] = accels[0];
      accels_previous[1] = accels[1];
      accels_previous[2] = accels[2];

      accelerationCharacteristic.notify(accels, 6);

      int r = accels[0] >> 8, g = accels[1] >> 8, b = accels[2] >> 8;  //  Red, green and blue intensity to display

      onePixel.setPixelColor(0, r, g, b);                              //  Set pixel 0 to (r,g,b) color value
      onePixel.show();                                                 //  Update pixel state
    }
  }
  //accel is the array, acceleration the event name, x the member

  if (gyroscopeCharacteristic.notifyEnabled()) {
    lsm6ds33.getGyroSensor()->getEvent(&event);

    // Convert gyro from Rad/s to Degree/s
    event.gyro.x *= SENSORS_RADS_TO_DPS;
    event.gyro.y *= SENSORS_RADS_TO_DPS;
    event.gyro.z *= SENSORS_RADS_TO_DPS;  //stored value is 1rad=57.2958 degress

    gyros[0] = constrain(map(event.gyro.x, gyro_min, gyro_max, 0.0, 65535.0), 0, 65535);
    gyros[1] = constrain(map(event.gyro.y, gyro_min, gyro_max, 0.0, 65535.0), 0, 65535);
    gyros[2] = constrain(map(event.gyro.z, gyro_min, gyro_max, 0.0, 65535.0), 0, 65535);

    if (gyros[0] != gyros_previous[0] || gyros[1] != gyros_previous[1] || gyros[2] != gyros_previous[2]) {
      gyros_previous[0] = gyros[0];
      gyros_previous[1] = gyros[1];
      gyros_previous[2] = gyros[2];

      gyroscopeCharacteristic.notify(gyros, 6);

      long brightness = 0; 

      avg[tick] = (gyros[0] / 3) + (gyros[1] / 3) + (gyros[2] / 3); 
      for(int i = 0; i < mean; i ++) {
        brightness = brightness + avg[i];
      }
      tick = (tick + 1) % mean; 

      brightness = brightness / mean; 

      brightness = constrain(map(brightness, 0, 65535, 0, 255), 0, 255); 
      Serial.println(brightness);
      
      onePixel.setBrightness(brightness);
    }
  }


  if (buttonCharacteristic.notifyEnabled()) {
    button_value = digitalRead(BUTTON);
    if (button_value != button_value_previous) {
      button_value_previous = button_value;
      buttonCharacteristic.notify8(button_value);
    }
  }
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
}
