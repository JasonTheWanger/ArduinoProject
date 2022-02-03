#include "DFRobot_CCS811.h"
#include <SPI.h>
#include <SD.h>
#include "Zanshin_BME680.h"
#include <Seeed_HM330X.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
File myFile;
/*
   IIC address default 0x5A, the address becomes 0x5B if the ADDR_SEL is soldered.
*/
//DFRobot_CCS811 CCS811(&Wire, /*IIC_ADDRESS=*/0x5A);
DFRobot_CCS811 CCS811;
BME680_Class BME680;
float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  static float Altitude;
  Altitude =
    44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}
#ifdef  ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL_OUTPUT SerialUSB
#else
#define SERIAL_OUTPUT Serial
#endif

HM330X sensor;
uint8_t buf[30];


const char* str[] = {"sensor num: ", "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                    };
int timer, threshold;
int tempCO2, tempTVOC;
int32_t tempTemp, tempHumidity, tempPressure, tempGas;
unsigned long int timeStamp;
double sumCO2, sumTVOC;
int32_t sumTemp, sumHumidity, sumPressure, sumGas;
int button = 7;
int switchVal;
boolean ifSwitched;
int count;
void setup(void)
{
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
#ifdef __AVR_ATmega32U4__      // If this is a 32U4 processor, then wait 3 seconds to init USB port
  delay(3000);
#endif
  Serial.print("Starting I2CDemo example program for BME680\n");
  Serial.print("- Initializing BME680 sensor\n");
  while (!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
    Serial.print("-  Unable to find BME680. Trying again in 5 seconds.\n");
    delay(5000);
  }  // of loop until device is located
  Serial.print("- Setting 16x oversampling for all sensors\n");
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  Serial.print("- Setting IIR filter to a value of 4 samples\n");
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  Serial.print("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n");  // "�C" symbols
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds
  /*Wait for the chip to be initialized completely, and then exit*/
  while (CCS811.begin() != 0) {
    Serial.println("failed to init chip, please check if the chip connection is fine");
    delay(1000);
  }
  SERIAL_OUTPUT.println("Serial start");
  if (sensor.init()) {
    SERIAL_OUTPUT.println("HM330X init failed!!");
    while (1);
  }
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  timer = 0;
  threshold = 15;
  sumCO2 = 0.000;
  sumTVOC = 0.000;
  sumTemp = sumHumidity = sumPressure = sumGas = 0.000;
  timeStamp = millis();
  Serial.print(timeStamp);
  Serial.println(" millisecond takes to start this sketch");
  myFile = SD.open("BMECCSHM.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(timeStamp);
    myFile.println(" millisecond takes to start this sketch");
    myFile.println("Ignore the first reading, might be incorrect");
    myFile.println("Time,CO2,TVOC,Temperature,Humidity,Pressure,Gas_Res");
  }
  myFile.close();
  pinMode(button, INPUT);
  ifSwitched = false;
  count = 0;
}
void loop() {
  timer++;
  timeStamp = millis();
  Serial.print(timeStamp);
  if (sensor.read_sensor_value(buf, 29)) {
    SERIAL_OUTPUT.println("HM330X read result failed!!");
  }
  BME680.getSensorData(tempTemp, tempHumidity, tempPressure, tempGas);
  Serial.print(" CO2: ");
  tempCO2 = CCS811.getCO2PPM();
  Serial.print(tempCO2);
  Serial.print("ppm, TVOC: ");
  tempTVOC = CCS811.getTVOCPPB();
  Serial.print(tempTVOC);
  Serial.print("ppb, ");
  Serial.print("Temperature: ");
  Serial.print(tempTemp / 100.000);
  Serial.print("decidegree, Humidity: ");
  Serial.print(tempHumidity / 1000.000);
  Serial.print("milli-pct, Pressure: ");
  Serial.print(tempPressure / 100.000);
  Serial.print("Pa, Gas_Resistnace: ");
  Serial.print(tempGas / 100.000);
  Serial.println("milliohms");
  //parse_result_value(buf);
  parse_result(buf);
  SERIAL_OUTPUT.println("");
  sumCO2 += tempCO2;
  sumTVOC += tempTVOC;
  sumTemp += tempTemp;
  sumHumidity += tempHumidity;
  sumPressure += tempPressure;
  sumGas += tempGas;
  myFile = SD.open("BMECCSHM.txt", FILE_WRITE);
  if (myFile) {
    if (timer >= threshold) {
      Serial.print("Writing to txt on the SD card...");
      myFile.print(timeStamp);
      myFile.print(",");
      myFile.print(sumCO2 / threshold);
      myFile.print(",");
      myFile.print(sumTVOC / threshold);
      myFile.print(",");
      myFile.print(sumTemp / 100.000 / threshold);
      myFile.print(",");
      myFile.print(sumHumidity / 1000.000 / threshold);
      myFile.print(",");
      myFile.print(sumPressure / 100.000 / threshold);
      myFile.print(",");
      myFile.println(sumGas / 100.000 / threshold);
      Serial.println("done.");
      timer = 0;
      sumCO2 = 0.000;
      sumTVOC = 0.000;
      sumTemp = sumHumidity = sumPressure = sumGas = 0.000;
    }
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening txt");
  }
  switchVal = digitalRead(button);
  if (switchVal == HIGH && !ifSwitched) {
    count++;
    ifSwitched = true;
  } else if (switchVal == LOW) {
    ifSwitched = false;
  }
  if (count > 5) {
    count = 0;
  }
  /*!
     @brief Set baseline
     @param get from getBaseline.ino
  */
  if (count == 0) {
    lcd.clear();
    lcd.print("CO2: ");
    lcd.setCursor(0, 1);
    lcd.print(tempCO2);
    lcd.print("ppm");
    lcd.setCursor(0, 0);
  } else if (count == 1) {
    lcd.clear();
    lcd.print("TVOC: ");
    lcd.setCursor(0, 1);
    lcd.print(tempTVOC);
    lcd.print("ppb");
    lcd.setCursor(0, 0);
  } else if (count == 2) {
    lcd.clear();
    lcd.print("Temperature: ");
    lcd.setCursor(0, 1);
    lcd.print(tempTemp / 100);
    lcd.print("Degree Celsius");
    lcd.setCursor(0, 0);
  } else if (count == 3) {
    lcd.clear();
    lcd.print("Humidity: ");
    lcd.setCursor(0, 1);
    lcd.print(tempHumidity / 1000);
    lcd.print("milli-pct");
    lcd.setCursor(0, 0);
  } else if (count == 4) {
    lcd.clear();
    lcd.print("Pressure: ");
    lcd.setCursor(0, 1);
    lcd.print(tempPressure / 100);
    lcd.print("pa");
    lcd.setCursor(0, 0);
  } else if (count == 5) {
    lcd.clear();
    lcd.print("Gas Resistance: ");
    lcd.setCursor(0, 1);
    lcd.print(tempGas / 100);
    lcd.print("milliohms");
    lcd.setCursor(0, 0);
  }
  Serial.println(count);
  CCS811.writeBaseLine(0x447B);
  //delay cannot be less than measurement cycle
  delay(1000);
}

HM330XErrorCode print_result(const char* str, uint16_t value) {
  if (NULL == str) {
    return ERROR_PARAM;
  }
  SERIAL_OUTPUT.print(str);
  SERIAL_OUTPUT.println(value);
//  myFile = SD.open("BMECCSHM.txt", FILE_WRITE);
//  if (myFile) {
//  myFile.print(value);
//  myFile.print(",");
//  myFile.close();
//  }
  return NO_ERROR;
}

/*parse buf with 29 uint8_t-data*/
HM330XErrorCode parse_result(uint8_t* data) {
  uint16_t value = 0;
  if (NULL == data) {
    return ERROR_PARAM;
  }
  for (int i = 2; i < 8; i++) {
    value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
    print_result(str[i - 1], value);
  }

  return NO_ERROR;
}

HM330XErrorCode parse_result_value(uint8_t* data) {
  if (NULL == data) {
    return ERROR_PARAM;
  }
  for (int i = 0; i < 28; i++) {
    SERIAL_OUTPUT.print(data[i], HEX);
    SERIAL_OUTPUT.print("  ");
    if ((0 == (i) % 5) || (0 == i)) {
      SERIAL_OUTPUT.println("");
    }
  }
  uint8_t sum = 0;
  for (int i = 0; i < 28; i++) {
    sum += data[i];
  }
  if (sum != data[28]) {
    SERIAL_OUTPUT.println("wrong checkSum!!");
  }
  SERIAL_OUTPUT.println("");
  return NO_ERROR;
}
