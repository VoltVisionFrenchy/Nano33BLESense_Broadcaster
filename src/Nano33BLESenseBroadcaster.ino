/*
    Name: vBridge Advert Peripheral
    Note: dfvevvvrvre
    Github Repo: https://github.com/VoltVisionFrenchy/Nano33BLESense_Broadcaster
    GoogleDoc: https://docs.google.com/spreadsheets/d/1OlF2bIO_ECabBYWCA-t-zRxWsCUxbNybzl1R09yZdUE/edit?usp=sharing

    ToDo:
      - Better resolution temp sensor
      - Restructure code for minimal implementation
      - PDM SPL detector?
      - Add other modes with better resolution?!??!??
*/

#include <ArduinoBLE.h>
// #include <Arduino_APDS9960.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
// #include <Arduino_LSM9DS1.h>

byte data[26] = {}; //Define data structure for the BLE advertisment data
unsigned long previousSlowSensorReadMillis = 0;
unsigned long SlowSensorReadInterval = 1000;

void setup()
{
  // Serial.begin(9600);
  Serial.begin(115200);
  delay(2000);  //frenchy found this was necessary when not doing the !Serial trick
  // while (!Serial)
  //   ; // Wait for serial monitor to open <-- frenchy doesnt want BLE to wait for serial!!
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
  //Initialize all bytes with their slot number to help with development/debug
  data[0] = 0xFF;  //Byte0 = Manu ID (MSB)
  data[1] = 0xFF;  //Byte1 = Manu ID (LSB)
  data[2] = 0xFF;  //Byte2 = Volt Vision DeviceType (MSB) (0xFF = Non-Volt Vision Dev Boards)
  data[3] = 0x01;  //Byte3 = Volt Vision DeviceType (LSB) (0x01 = Arduino Nano 33 BLE)
  data[4] = 0x00;  //Byte4 = Error Code (0x00 = No error)
  data[5] = 0x00;  //Byte5 = SW Version Major
  data[6] = 0x01;  //Byte6 = SW Version Minor
  data[7] = 0x01;  //Byte7 = Mode
  data[8] = 0x08;  //Byte8
  data[9] = 0x09;  //Byte9
  data[10] = 0x0A; //Byte10
  data[11] = 0x0B; //Byte11
  data[12] = 0x0C; //Byte12
  data[13] = 0x0D; //Byte13
  data[14] = 0x0E; //Byte14
  data[15] = 0x0F; //Byte15
  data[16] = 0x10; //Byte16
  data[17] = 0x11; //Byte17
  data[18] = 0x12; //Byte18
  data[19] = 0x13; //Byte19
  data[20] = 0x14; //Byte20
  data[21] = 0x15; //Byte21
  data[22] = 0x16; //Byte22
  data[23] = 0x17; //Byte23
  data[24] = 0x18; //Byte24
  data[25] = 0x19; //Byte25
  BLE.setLocalName("LocalName");
  BLE.setDeviceName("DeviceName");
  BLE.setAdvertisingInterval(160);
    // 1600*0.625ms = 1000ms
    // 800*0.625ms = 500ms
    // 640*0.625ms = 400ms
    // 320*0.625ms = 200ms
    // 160*0.625ms = 100ms <-- Default
    // 80*0.625ms = 50ms
    // 40*0.625ms = 25ms
  BLE.setManufacturerData(data, 26);
  BLE.advertise();
  Serial.println("1st Advert is running...");
}

void loop()
{
  // Interval:  SlowSensorReadInterval
  if (millis() - previousSlowSensorReadMillis > SlowSensorReadInterval)
  {
    Serial.println("SlowSensorReadInterval Processing started...");
    float temperature = HTS.readTemperature();
    float humidity = HTS.readHumidity();
    float pressure = BARO.readPressure(); //if Pressure fails @ 0 again, force re-init?
    data[13] = int(temperature + 40); // HTS221_Temperature (+40)
    Serial.print("...HTS221_Temperature Update     :: data[13] Bitpack = degC+40 = ");
    Serial.print(data[13]);
    Serial.print("     :: RealTemp (degC) = ");
    Serial.println(temperature);
    data[14] = int(humidity);
    Serial.print("...HTS221_Humidity (%) Update    :: data[14] Bitpack = % = ");
    Serial.print(data[14]);
    Serial.print("           :: RealHumidity (%) = ");
    Serial.println(humidity);
    data[15] = int((pressure * 10) -850);  // LPS22HB_Pressure
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
  } //end of SlowSensorReadInterval
} //End of main loop






////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// Below this is old
////////////////////////////////////////////////////////////////////////////////
// int proximity = 0;
// int r = 0, g = 0, b = 0;
// unsigned long lastUpdate = 0;
// unsigned long lastGesture = 0;
// float ax, ay, az;
// float gx, gy, gz;
// float mx, my, mz;
// float temp = 0;



// Below is the clipping from the setup routine
  // if (!APDS.begin())
  // {
  //   Serial.println("Error initializing APDS9960 sensor.");
  //   while (1); // Stop forever
  // }

  // if (!IMU.begin())
  // {
  //   Serial.println("Failed to initialize IMU!");
  //   while (1)
  //     ;
  // }
  // if (!BLE.begin())
  // {
  //   Serial.println("starting BLE failed!");
  //   while (1)
  //     ;
  // }




//Stripped out main loop
  // if (IMU.accelerationAvailable())
  // {
  //   Serial.println("IMU.accelerationAvailable() Success!!!!");
  //   IMU.readAcceleration(ax, ay, az);
  // }
  // orig range was -1.1 to +1.1
  // Scaling strategy is this: Scale up to +/- 127, then offset to 0-255
  // 3extra because was offset
  // temp = int(ax * 127) + 130; 
  // data[16] = constrain(temp, 0, 255);
  // temp = int(ay * 127) + 130;
  // data[17] = constrain(temp, 0, 255);
  // temp = int(az * 127) + 130;
  // data[18] = constrain(temp, 0, 255);

  // if (IMU.gyroscopeAvailable())
  // {
  //   Serial.println("IMU.gyroscopeAvailable() Success!!!!");
  //   IMU.readGyroscope(gx, gy, gz);
  // }
  // orig range was -500 to +500 when I whipped it around
  // Scaling strategy is this: Scale down to +/- 127 (~0.25x), then offset to 0-255
  // No offset was needed
  // temp = int(gx/4) + 128;
  // data[19] = constrain(temp, 0, 255);
  // temp = int(gy/4) + 128;
  // data[20] = constrain(temp, 0, 255);
  // temp = int(gz/4) + 128;
  // data[21] = constrain(temp, 0, 255);

  // if (IMU.magneticFieldAvailable())
  // {
  //   Serial.println("IMU.magneticFieldAvailable() Success!!!!");
  //   IMU.readMagneticField(mx, my, mz);
  // }
  // orig range was -300 to +300 when I placed next to SnS motor....lets use same as gyro for now.
  // Scaling strategy is this: Scale down to +/- 127 (~0.25x), then offset to 0-255
  // No offset was needed
  // temp = int(mx/4) + 128;
  // data[22] = constrain(temp, 0, 255);
  // temp = int(my/4) + 128;
  // data[23] = constrain(temp, 0, 255);
  // temp = int(mz/4) + 128;
  // data[24] = constrain(temp, 0, 255);


  //  Occasionally reset the last recognized gesture.
  // if (millis() - lastGesture > 2000)
  // {
  //   data[12] = 0xFF;
  // }


//update block 
// This block of code used to run each loop....but does that cause the sensors to crash?
// lets try it inside this slower update loop?


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


//     Serial.print("...Millis = ");
//     Serial.println(millis());
//     Serial.println("...");
//     Serial.println("...");
//     Serial.println("...");
//     Serial.println("...");
//     Serial.println("...");
//     Serial.println("...");
//     Serial.println("...");
//     Serial.println("MAC = C5:D6:D4:49:36:5F");
//     Serial.print("data[0] = Manu ID (MSB) = ");
//     Serial.println(data[0]);
//     Serial.print("data[1] = Manu ID (LSB) = ");
//     Serial.println(data[1]);
//     Serial.print("data[2] = Volt Vision DeviceType (MSB) = ");
//     Serial.println(data[2]);
//     Serial.print("data[3] = Volt Vision DeviceType (LSB) = ");
//     Serial.println(data[3]);
//     Serial.print("data[4] = Error Code = ");
//     Serial.println(data[4]);
//     Serial.print("data[5] = SW Version Major = ");
//     Serial.println(data[5]);
//     Serial.print("data[6] = SW Version Minor = ");
//     Serial.println(data[6]);
//     Serial.println("data[7] = Mode = 0xFF = Lo_Res_Minimal Pack");
//     // Serial.println(data[7]);
//     Serial.print("data[8] = r = ");
//     Serial.println(r);
//     Serial.print("data[9] = g = ");
//     Serial.println(g);
//     Serial.print("data[10] = b = ");
//     Serial.println(b);
//     Serial.print("data[11] = Proximity = ");
//     Serial.println(proximity);
//     Serial.print("data[12] = Gesture = ");
//     Serial.print(data[12]);
//     //add special display of current gesture
//     if (data[12] == 0xFF)
//     {
//       Serial.println("   <------- Idle");
//     }
//     if (data[12] == 0)
//     {
//       Serial.println("   <------- Up! ");
//     }
//     if (data[12] == 1)
//     {
//       Serial.println("   <------- Down! ");
//     }
//     if (data[12] == 2)
//     {
//       Serial.println("   <------- Left! ");
//     }
//     if (data[12] == 3)
//     {
//       Serial.println("   <------- Right! ");
//     }
//     Serial.print("data[16] = LSM9DS1_AccelX ...... Bitpack (*127 + 130) = ");
//     Serial.print(data[16]);
//     Serial.print(".........raw ax (g's) = ");
//     Serial.println(ax);
//     Serial.print("data[17] = LSM9DS1_AccelY ...... Bitpack (*127 + 130) = ");
//     Serial.print(data[17]);
//     Serial.print(".........raw ay (g's) = ");
//     Serial.println(ay);
//     Serial.print("data[18] = LSM9DS1_AccelZ ...... Bitpack (*127 + 130) = ");
//     Serial.print(data[18]);
//     Serial.print(".........raw az (g's) = ");
//     Serial.println(az);
//     Serial.print("data[19] = LSM9DS1_GyroX ...... Bitpack (/4+128) = ");
//     Serial.print(data[19]);
//     Serial.print(".........raw gx (deg/s) = ");
//     Serial.println(gx);
//     Serial.print("data[20] = LSM9DS1_GyroY ...... Bitpack (/4+128) = ");
//     Serial.print(data[20]);
//     Serial.print(".........raw gy (deg/s) = ");
//     Serial.println(gy);
//     Serial.print("data[21] = LSM9DS1_GyroZ ...... Bitpack (/4+128) = ");
//     Serial.print(data[21]);
//     Serial.print(".........raw gz (deg/s) = ");
//     Serial.println(gz);
//     Serial.print("data[22] = LSM9DS1_MagX .....(/4+128) = ");
//     Serial.print(data[22]);
//     Serial.print(".........raw mx (uT) = ");
//     Serial.println(mx);
//     Serial.print("data[23] = LSM9DS1_MagY .....(/4+128) = ");
//     Serial.print(data[23]);
//     Serial.print(".........raw my (uT) = ");
//     Serial.println(my);
//     Serial.print("data[24] = LSM9DS1_MagZ .....(/4+128) = ");
//     Serial.print(data[24]);
//     Serial.print(".........raw mz (uT) = ");
//     Serial.println(mz);

//     Serial.print("Accelerometer sample rate = ");
//     Serial.print(IMU.accelerationSampleRate());
//     Serial.println(" Hz");
//     Serial.print("Gyroscope sample rate = ");
//     Serial.print(IMU.gyroscopeSampleRate());
//     Serial.println(" Hz");
//     Serial.print("Magnetic field sample rate = ");
//     Serial.print(IMU.magneticFieldSampleRate());
//     Serial.println(" Hz"); //Frenchy says it was uT, but it should be Hz?

//   }
// } //End of main loop
