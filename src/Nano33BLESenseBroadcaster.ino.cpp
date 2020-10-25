# 1 "C:\\Users\\micro\\AppData\\Local\\Temp\\tmp0bfjvma1"
#include <Arduino.h>
# 1 "C:/Users/micro/Documents/PlatformIO/Projects/Nano33BLESense_Broadcaster/src/Nano33BLESenseBroadcaster.ino"
# 15 "C:/Users/micro/Documents/PlatformIO/Projects/Nano33BLESense_Broadcaster/src/Nano33BLESenseBroadcaster.ino"
#include <ArduinoBLE.h>

#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>


byte data[26] = {};
unsigned long previousSlowSensorReadMillis = 0;
unsigned long SlowSensorReadInterval = 1000;
void setup();
void loop();
#line 25 "C:/Users/micro/Documents/PlatformIO/Projects/Nano33BLESense_Broadcaster/src/Nano33BLESenseBroadcaster.ino"
void setup()
{

  Serial.begin(115200);
  delay(2000);


  if (!HTS.begin())
  {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  if (!BARO.begin())
  {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0x01;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x01;
  data[7] = 0x01;
  data[8] = 0x08;
  data[9] = 0x09;
  data[10] = 0x0A;
  data[11] = 0x0B;
  data[12] = 0x0C;
  data[13] = 0x0D;
  data[14] = 0x0E;
  data[15] = 0x0F;
  data[16] = 0x10;
  data[17] = 0x11;
  data[18] = 0x12;
  data[19] = 0x13;
  data[20] = 0x14;
  data[21] = 0x15;
  data[22] = 0x16;
  data[23] = 0x17;
  data[24] = 0x18;
  data[25] = 0x19;
  BLE.setLocalName("LocalName");
  BLE.setDeviceName("DeviceName");
  BLE.setAdvertisingInterval(160);







  BLE.setManufacturerData(data, 26);
  BLE.advertise();
  Serial.println("1st Advert is running...");
}

void loop()
{

  if (millis() - previousSlowSensorReadMillis > SlowSensorReadInterval)
  {
    Serial.println("SlowSensorReadInterval Processing started...");
    float temperature = HTS.readTemperature();
    float humidity = HTS.readHumidity();
    float pressure = BARO.readPressure();
    data[13] = int(temperature + 40);
    Serial.print("...HTS221_Temperature Update     :: data[13] Bitpack = degC+40 = ");
    Serial.print(data[13]);
    Serial.print("     :: RealTemp (degC) = ");
    Serial.println(temperature);
    data[14] = int(humidity);
    Serial.print("...HTS221_Humidity (%) Update    :: data[14] Bitpack = % = ");
    Serial.print(data[14]);
    Serial.print("           :: RealHumidity (%) = ");
    Serial.println(humidity);
    data[15] = int((pressure * 10) -850);
    Serial.print("...LPS22HB_Pressure (kPa) Update :: data[15] Bitpack = kPa*10-850 = ");
    Serial.print(data[15]);
    Serial.print(" :: RealPressure (kPa) = ");
    Serial.println(pressure);
    BLE.stopAdvertise();
    Serial.println("...Advertising Stopped...");
    BLE.setManufacturerData(data, 26);
    Serial.println("...Advertising Data updated...");
    BLE.advertise();
    Serial.println("...Advertising Started...");
    Serial.println("SlowSensorReadInterval Processing Complete");
    Serial.println("...");
    Serial.println("...");
    Serial.println("...");
    previousSlowSensorReadMillis = millis();
  }
}