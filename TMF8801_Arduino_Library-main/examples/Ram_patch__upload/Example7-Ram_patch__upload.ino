
#include <Wire.h>
#include "SparkFun_TMF8801_Arduino_Library.h"

TMF8801 tmf8801;

// This array will hold the updated calibration data
byte newCalibrationData[14] = { 0 };

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  tmf8801.reset_SW();
  tmf8801.download_INT_RAM();
  tmf8801.write_ram ();
  tmf8801.remap_rst_RAM();
  if (tmf8801.begin() == true)
  {
    tmf8801.enableInterrupt();
    Serial.print("TMF8801 serial number ");
    Serial.print(tmf8801.getSerialNumber());
    Serial.println(" connected");
    Serial.print("Chip revision: ");
    Serial.println(tmf8801.getHardwareVersion());
    Serial.print("App version: ");
    Serial.print(tmf8801.getApplicationVersionMajor());
    Serial.print(".");
    Serial.println(tmf8801.getApplicationVersionMinor());
  }
  else
  {
    Serial.println("TMF8801 connection failed.");
    Serial.print("Status register = 0x");
    Serial.println(tmf8801.getStatus(), HEX);
    printErrorMessage();
    Serial.println("System halted.");
    while (true);
  }

  Serial.println("Calibrating... please wait !");

  // Pass  newCalibrationData array to getCalibrationData function and check if calibration was successful or not
  if (tmf8801.getCalibrationData(newCalibrationData) == true)
  { 
    // You must call tmf8801.setCalibrationData after calling tmf8801.begin() so it can update the default calibration data with
    // the new values. You can, alternatively, store this array into the microcontroller's EEPROM and load it on powerup.
    tmf8801.setCalibrationData(newCalibrationData);   
    
    Serial.println("Updated calibration data was set.");
    Serial.println("New calibration data array is as follows:");

    // Prints out the updated calibration array
    for (int i = 0; i < 13; i++)
    {
      if (newCalibrationData[i] < 10)
        Serial.print("0x0");
      else
        Serial.print("0x");
      Serial.print(newCalibrationData[i], HEX);
      Serial.print(", ");
    }

    if (newCalibrationData[13] < 10)
      Serial.print("0x0");
    else
      Serial.print("0x");
    Serial.println(newCalibrationData[13], HEX);
  }
  else
  {
    printErrorMessage();
    Serial.println("Default values will be used as calibration data.");
  }
}


void loop()
{
  // Restart the application when ENABLE pin returns to HIGH
  if (!tmf8801.isConnected())
  {
    Serial.println("Not connected or ENABLE pin is low.");
    tmf8801.wakeUpDevice();
    tmf8801.begin();
  }

  if (tmf8801.dataAvailable())
  {
    Serial.print("Distance: ");
    Serial.print(tmf8801.getDistance());
    Serial.println(" mm");
  }

  // Wait half a second and start over
  delay(500);
}

// This function will print a user-friendly error message.
void printErrorMessage()
{
  switch (tmf8801.getLastError())
  {
  case ERROR_I2C_COMM_ERROR:
    Serial.println("Error: I2C communication error");
    break;

  case ERROR_CPU_RESET_TIMEOUT:
    Serial.println("Error: Timeout on CPU reset");
    break;

  case ERROR_WRONG_CHIP_ID:
    Serial.println("Error: Chip ID mismatch");
    break;

  case ERROR_CPU_LOAD_APPLICATION_ERROR:
    Serial.println("Error: Load application error");
    break;

  case ERROR_FACTORY_CALIBRATION_ERROR:
    Serial.println("Error: Calibration was not successful. Please try again.");
    break;

  default:
    break;
  }
}
