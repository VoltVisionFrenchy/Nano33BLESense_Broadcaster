/*
    Name: vBridge Advert Peripheral
    Note: dfvevvvrvre
    Github Repo: https://github.com/VoltVisionFrenchy/Nano33BLESenseBroadcaster
    GoogleDoc: https://docs.google.com/spreadsheets/d/1OlF2bIO_ECabBYWCA-t-zRxWsCUxbNybzl1R09yZdUE/edit?usp=sharing

    ToDo:
      - Restructure code for minimal implementation
      - PDM SPL detector?
      - Add other modes with better resolution?!??!??

*/

#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>

byte data[26] = {}; //Define data structure for the BLE advertisment data
int proximity = 0;
int r = 0, g = 0, b = 0;
unsigned long lastUpdate = 0;
unsigned long lastGesture = 0;
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float temp = 0;

void setup()
{
  Serial.begin(9600);
  delay(2000);  //frenchy found this was necessary when not doing the !Serial trick
  // while (!Serial)
  //   ; // Wait for serial monitor to open <-- frenchy doesnt want BLE to wait for serial!!
  if (!APDS.begin())
  {
    Serial.println("Error initializing APDS9960 sensor.");
    while (true)
      ; // Stop forever
  }
  if (!HTS.begin())
  {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1)
      ;
  }
  if (!BARO.begin())
  {
    Serial.println("Failed to initialize pressure sensor!");
    while (1)
      ;
  }
  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    while (1)
      ;
  }

  //Initialize all bytes with their slot number to help with development/debug
  data[0] = 0xFF;  //Byte0 = Manu ID (MSB) (needs to be 0xFF for SteveC's app)
  data[1] = 0xFF;  //Byte1 = Manu ID (LSB) (needs to be 0xFF for SteveC's app)
  data[2] = 0xFF;  //Byte2 = Volt Vision DeviceType (MSB) (0xFF = Non-Volt Vision Dev Boards)
  data[3] = 0x01;  //Byte3 = Volt Vision DeviceType (LSB) (0x01 = Arduino Nano 33 BLE)
  data[4] = 0x00;  //Byte4 = Error Code (0x00 = No error)
  data[5] = 0x00;  //Byte5 = SW Version Major
  data[6] = 0x01;  ///////Byte6 = SW Version Minor
  data[7] = 0x01;  //Byte7 = Mode
  data[8] = 0xFF;  //Byte8 = APDS9960_colorRGB_R
  data[9] = 0xFF;  //Byte9 = APDS9960_colorRGB_G
  data[10] = 0xFF; //Byte10 = APDS9960_colorRGB_B
  data[11] = 0xFF; //Byte11 = APDS9960_proximity
  data[12] = 0xFF; //Byte12 = APDS9960_gesture
  data[13] = 0x13; //Byte13 = HTS221_Temperature (degC)
  data[14] = 0x14; //Byte14 = HTS221_Humidity (%)
  data[15] = 0x15; //Byte15 = LPS22HB_Pressure (kPa)
  data[16] = 0x16; //Byte16 = LSM9DS1_AccelX
  data[17] = 0x17; //Byte17 = LSM9DS1_AccelY
  data[18] = 0x18; //Byte18 = LSM9DS1_AccelZ
  data[19] = 0x19; //Byte19
  data[20] = 0x20; //Byte20
  data[21] = 0x21; //Byte21
  data[22] = 0x22; //Byte22
  data[23] = 0x23; //Byte23
  data[24] = 0x24; //Byte24
  data[25] = 0x25; //Byte25
  BLE.setLocalName("LocalName");
  BLE.setDeviceName("DeviceName");
  BLE.setAdvertisingInterval(320); //160*0.625ms = 200ms
                                   //  BLE.setAdvertisingInterval(160);  //160*0.625ms = 100ms
                                   //  BLE.setAdvertisingInterval(80);  //80*0.625ms = 50ms
                                   //  BLE.setAdvertisingInterval(40);  //40*0.625ms = 25ms
  BLE.setManufacturerData(data, 26);
  BLE.advertise();
  Serial.println("1st Advert is running...");
}

void loop()
{
  if (IMU.accelerationAvailable())
  {
    Serial.println("IMU.accelerationAvailable() Success!!!!");
    IMU.readAcceleration(ax, ay, az);
  }
  // orig range was -1.1 to +1.1
  // Scaling strategy is this: Scale up to +/- 127, then offset to 0-255
  // 3extra because was offset
  temp = int(ax * 127) + 130; 
  data[16] = constrain(temp, 0, 255);
  temp = int(ay * 127) + 130;
  data[17] = constrain(temp, 0, 255);
  temp = int(az * 127) + 130;
  data[18] = constrain(temp, 0, 255);

  if (IMU.gyroscopeAvailable())
  {
    Serial.println("IMU.gyroscopeAvailable() Success!!!!");
    IMU.readGyroscope(gx, gy, gz);
  }
  // orig range was -500 to +500 when I whipped it around
  // Scaling strategy is this: Scale down to +/- 127 (~0.25x), then offset to 0-255
  // No offset was needed
  temp = int(gx/4) + 128;
  data[19] = constrain(temp, 0, 255);
  temp = int(gy/4) + 128;
  data[20] = constrain(temp, 0, 255);
  temp = int(gz/4) + 128;
  data[21] = constrain(temp, 0, 255);

  if (IMU.magneticFieldAvailable())
  {
    Serial.println("IMU.magneticFieldAvailable() Success!!!!");
    IMU.readMagneticField(mx, my, mz);
  }
  // orig range was -300 to +300 when I placed next to SnS motor....lets use same as gyro for now.
  // Scaling strategy is this: Scale down to +/- 127 (~0.25x), then offset to 0-255
  // No offset was needed
  temp = int(mx/4) + 128;
  data[22] = constrain(temp, 0, 255);
  temp = int(my/4) + 128;
  data[23] = constrain(temp, 0, 255);
  temp = int(mz/4) + 128;
  data[24] = constrain(temp, 0, 255);


  //  Occasionally reset the last recognized gesture.
  if (millis() - lastGesture > 2000)
  {
    data[12] = 0xFF;
  }

  if (millis() - lastUpdate > 500)
  {

// This block of code used to run each loop....but does that cause the sensors to crash?
// lets try it inside this slower update loop?
    float temperature = HTS.readTemperature();
    float humidity = HTS.readHumidity();
    float pressure = BARO.readPressure(); //if Pressure fails @ 0 again, force re-init?
    data[13] = int(temperature + 40); // HTS221_Temperature (+40)
    data[14] = int(humidity); // HTS221_Humidity
    data[15] = int((pressure * 10) -850);  // LPS22HB_Pressure


  // // check if a proximity reading is available
  //   if (APDS.proximityAvailable())
  //   {
  //     Serial.println("APDS.proximityAvailable() Success!!!!");
  //     proximity = APDS.readProximity();
  //     // read the proximity
  //     // - 0   => close
  //     // - 255 => far
  //     // - -1  => error
  //     if (proximity > 255)
  //     {
  //       proximity = 255;
  //     }
  //     if (proximity == -1)
  //     {
  //       data[4] = 1;
  //     }
  //     data[11] = proximity;
  //     Serial.println(proximity);
  //   }

    // if (APDS.colorAvailable())
    // {
    //   Serial.println("APDS.colorAvailable() Success!!!!");
    //   APDS.readColor(r, g, b);
    //   if (r > 255)
    //   {
    //     r = 255;
    //   }
    //   if (g > 255)
    //   {
    //     g = 255;
    //   }
    //   if (b > 255)
    //   {
    //     b = 255;
    //   }
    //   data[8] = r;
    //   data[9] = g;
    //   data[10] = b;
    // }


  // if (APDS.gestureAvailable())
  // {
  //   Serial.println("APDS.gestureAvailable() Success!!!!");
  //   int gesture = APDS.readGesture();
  //   switch (gesture)
  //   {
  //   case GESTURE_UP:
  //     // Serial.println("Detected UP gesture (0)");
  //     data[12] = 0;
  //     lastGesture = millis();
  //     break;

  //   case GESTURE_DOWN:
  //     // Serial.println("Detected DOWN gesture (1)");
  //     data[12] = 1;
  //     lastGesture = millis();
  //     break;

  //   case GESTURE_LEFT:
  //     // Serial.println("Detected LEFT gesture (2)");
  //     data[12] = 2;
  //     lastGesture = millis();
  //     break;

  //   case GESTURE_RIGHT:
  //     // Serial.println("Detected RIGHT gesture (3)");
  //     data[12] = 3;
  //     lastGesture = millis();
  //     break;

  //   default:
  //     break;
  //   }
  // }


    lastUpdate = millis();
    Serial.print("...Millis = ");
    Serial.println(millis());
    Serial.println("...");
    Serial.println("...");
    Serial.println("...");
    Serial.println("...");
    Serial.println("...");
    Serial.println("...");
    Serial.println("...");
    Serial.println("MAC = C5:D6:D4:49:36:5F");
    Serial.print("data[0] = Manu ID (MSB) = ");
    Serial.println(data[0]);
    Serial.print("data[1] = Manu ID (LSB) = ");
    Serial.println(data[1]);
    Serial.print("data[2] = Volt Vision DeviceType (MSB) = ");
    Serial.println(data[2]);
    Serial.print("data[3] = Volt Vision DeviceType (LSB) = ");
    Serial.println(data[3]);
    Serial.print("data[4] = Error Code = ");
    Serial.println(data[4]);
    Serial.print("data[5] = SW Version Major = ");
    Serial.println(data[5]);
    Serial.print("data[6] = SW Version Minor = ");
    Serial.println(data[6]);
    Serial.println("data[7] = Mode = 0xFF = Lo_Res_Minimal Pack");
    // Serial.println(data[7]);
    Serial.print("data[8] = r = ");
    Serial.println(r);
    Serial.print("data[9] = g = ");
    Serial.println(g);
    Serial.print("data[10] = b = ");
    Serial.println(b);
    Serial.print("data[11] = Proximity = ");
    Serial.println(proximity);
    Serial.print("data[12] = Gesture = ");
    Serial.print(data[12]);
    //add special display of current gesture
    if (data[12] == 0xFF)
    {
      Serial.println("   <------- Idle");
    }
    if (data[12] == 0)
    {
      Serial.println("   <------- Up! ");
    }
    if (data[12] == 1)
    {
      Serial.println("   <------- Down! ");
    }
    if (data[12] == 2)
    {
      Serial.println("   <------- Left! ");
    }
    if (data[12] == 3)
    {
      Serial.println("   <------- Right! ");
    }
    Serial.print("data[13] = HTS221_Temperature .....Bitpack (degC+40) = ");
    Serial.print(data[13]);
    Serial.print("....... Real Temp is ");
    Serial.println(data[13]-40);
    Serial.print("data[14] = HTS221_Humidity (%) = ");
    Serial.println(data[14]);
    Serial.print("data[15] = LPS22HB_Pressure (kPa) = ");
    Serial.print(data[15]);
    Serial.print("....... Real Pressure is ");
    Serial.println(pressure);
    Serial.print("data[16] = LSM9DS1_AccelX ...... Bitpack (*127 + 130) = ");
    Serial.print(data[16]);
    Serial.print(".........raw ax (g's) = ");
    Serial.println(ax);
    Serial.print("data[17] = LSM9DS1_AccelY ...... Bitpack (*127 + 130) = ");
    Serial.print(data[17]);
    Serial.print(".........raw ay (g's) = ");
    Serial.println(ay);
    Serial.print("data[18] = LSM9DS1_AccelZ ...... Bitpack (*127 + 130) = ");
    Serial.print(data[18]);
    Serial.print(".........raw az (g's) = ");
    Serial.println(az);
    Serial.print("data[19] = LSM9DS1_GyroX ...... Bitpack (/4+128) = ");
    Serial.print(data[19]);
    Serial.print(".........raw gx (deg/s) = ");
    Serial.println(gx);
    Serial.print("data[20] = LSM9DS1_GyroY ...... Bitpack (/4+128) = ");
    Serial.print(data[20]);
    Serial.print(".........raw gy (deg/s) = ");
    Serial.println(gy);
    Serial.print("data[21] = LSM9DS1_GyroZ ...... Bitpack (/4+128) = ");
    Serial.print(data[21]);
    Serial.print(".........raw gz (deg/s) = ");
    Serial.println(gz);
    Serial.print("data[22] = LSM9DS1_MagX .....(/4+128) = ");
    Serial.print(data[22]);
    Serial.print(".........raw mx (uT) = ");
    Serial.println(mx);
    Serial.print("data[23] = LSM9DS1_MagY .....(/4+128) = ");
    Serial.print(data[23]);
    Serial.print(".........raw my (uT) = ");
    Serial.println(my);
    Serial.print("data[24] = LSM9DS1_MagZ .....(/4+128) = ");
    Serial.print(data[24]);
    Serial.print(".........raw mz (uT) = ");
    Serial.println(mz);

    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.print("Magnetic field sample rate = ");
    Serial.print(IMU.magneticFieldSampleRate());
    Serial.println(" Hz"); //Frenchy says it was uT, but it should be Hz?

    BLE.stopAdvertise();
    Serial.print("Advertising Stopped...");
    BLE.setManufacturerData(data, 26);
    Serial.print("......Data updated...");
    BLE.advertise();
    Serial.println("..........Advertising Started...");
  }
}
