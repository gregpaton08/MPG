//#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "ble.h"
#include "OBDLib.h"

#define SD_SS 4
#define BLE_REQN 9
#define MEGA_SS 53

enum operatingMode {
  OBD,
  BLE
};

enum SPIMode {
  sd,
  ble,
  unknown
};

File OBDLog;
unsigned long startTime;
const uint8_t FILE_LEN = 13;
operatingMode opMode = OBD;
SPIMode spiMode = unknown;
OBDLib obd;


void setup() {
  // set BAUD rate
  Serial.begin(38400);
  SPI.begin();
  
  // Setup pins
  pinMode(BLE_REQN, OUTPUT);
  pinMode(SD_SS, OUTPUT);
  
  // Setup SD card
  SDInit(); 
  
  // Setup OBD
  obd.init(); 
  
  // Setup BLE
  setBLEActive();
  ble_begin();
  
  // Keep track of start time
  startTime = millis();
}


void loop() {
  logPids();
}


void setSDActive() {
  if (spiMode == sd)
    return;
  else
    spiMode = sd;
  digitalWrite(BLE_REQN, HIGH);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  digitalWrite(SD_SS, LOW);
}


void setBLEActive() {
  if (spiMode == ble)
    return;
  else
    spiMode = ble;
  digitalWrite(SD_SS, HIGH);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(LSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  digitalWrite(BLE_REQN, LOW);
}

void logPids() {
  setSDActive();
  // RPM
  logMode01PID(0x0C);
  // Speed
  logMode01PID(0x0D);
  // MAF
  logMode01PID(0x10);
  
  // flush data to SD card
  OBDLog.flush();
}


void checkForBLE() {
  char c = -1;
  if (ble_available() > 0) {
    c = ble_read();
    if (c == -1) {
      return;
    }
    else if (c == 'l') {
      listFiles();
      setBLEActive();
    }
    else if (c == 'o') {
      BLEopenFile(OBDLog);
    }
    else if (c == 'r') {
      BLEreadLine(OBDLog);
    }
    else if (c == 'd') {
      BLEdeleteFile();
    }
    else {
      return;
    }
  }
  ble_do_events();
}


void SDInit() {
  setSDActive();
  
  // Set slave select pin for SD Card
  pinMode(SD_SS, OUTPUT);
  if (false == SD.begin(SD_SS)) return;
 
  // Determine file name
  // File name must be 8 characters or less
  char filename[13];
  for (uint16_t index = 1; index < 65535; index++) {
    sprintf(filename, "OBD%05d.csv", index);
    if (false == SD.exists(filename)) {
      break;
    }
  }
  
  // Open file
  OBDLog = SD.open(filename, FILE_WRITE);
}


void BLEInit() {  
  // Set up Serial Peripheral Interface for BLE
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(LSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  
  // Start BLE
  ble_begin();
}


void logMode01PID(byte pid) {
  uint8_t len = 0;
  uint8_t pidResSize = 10;
  char pidRes[pidResSize];
  
  // Query PID
  obd.sendCMD(0x01, pid);
  
  uint8_t strPidSize = 4;
  char strPid[strPidSize];
  String strPidtemp = String(pid, HEX);
  strPidtemp.toUpperCase();
  if (strPidtemp.length() == 1) {
      strPidtemp = "0" + strPidtemp;
  }
  strPidtemp.toCharArray(strPid, strPidSize);
  while (false == Serial.find(strPid));
    
  // Print time 
  OBDLog.print(millis() - startTime);
  OBDLog.print(",");
  if (pid < 0x10)
    OBDLog.print('0');
  OBDLog.print(pid, HEX);
  OBDLog.print(",");
  OBDLog.flush();
   
  // loop until new line character found
  while (len < pidResSize) {
    unsigned char c = Serial.read();
    if (c == (unsigned char)-1) continue;
    if (c == '\n' || c == '\r') break;
    pidRes[len] = c;
    ++len;
  }
  // print result to 2 decimal places
  OBDLog.println(obd.pidToDec(pid, pidRes), 2);
}


void writeToBLE(char *buf) {
  setBLEActive();
  uint8_t cnt = 0;
  while (buf[cnt] != '\0') {
    ble_write(buf[cnt]); 
    ++cnt; 
  }
  ble_do_events();
}


void listFiles() {
  setSDActive();
  File dir = SD.open("/");
  dir.rewindDirectory();
  while(true) {
    File entry =  dir.openNextFile();
    if (false == entry) {
      break;
    }
    if (false == entry.isDirectory() 
        && entry.name()[0] != '~') {
      writeToBLE(entry.name());
      setSDActive();
    }
  }
}


boolean BLEopenFile(File &file) {
  boolean ret = false;
  char name[FILE_LEN];
  uint8_t cnt = 0;
  
  while (ble_available() > 0 && cnt < FILE_LEN) {
    name[cnt] = ble_read();
    if (name[cnt] < 46) {
      name[cnt] = '\0';
      break;
    } 
    ++cnt;
  }
  setSDActive();
  if (strcmp("open", name)) {
    if (SD.exists(name)) {
      file = SD.open(name, FILE_READ);
      if (file) 
        ret = true;
    }
  }
  
  if (ret) 
    writeToBLE("OPEN\n");
  else     
    writeToBLE("FAIL\n");
  
  return ret;
}


boolean BLEreadLine(File &file) {
  boolean ret = true;
  const uint8_t len = 32;
  char line[len];
  uint8_t cnt = 0;
  char c = -1;
  
  setSDActive();
  
  if (false == file) 
    ret = false;
  
  while (ret) {
    line[cnt] = file.read();
    if (line[cnt] == (char)-1) {
      if (cnt > 0) {
        line[cnt] = '\0';
        writeToBLE(line);
      }
      writeToBLE("\nEOF\n");
      return false;
    }
    else if (line[cnt] == '\n') {
      line[cnt] = '\0';
      break;
    }
    ++cnt;
  }
  
  if (cnt > 0)
    writeToBLE(line);
  else if (false == ret) 
    writeToBLE("FAIL\n");
  
  return true;
}


boolean BLEdeleteFile() {
  boolean ret = true;
  char name[FILE_LEN];
  uint8_t cnt = 0;
  
  while (ble_available() > 0 && cnt < FILE_LEN) {
    name[cnt] = ble_read();
    if (name[cnt] < 46) {
      name[cnt] = '\0';
      break;
    } 
    ++cnt;
  }
  
  setSDActive();
  if (SD.exists(name)) {
    ret = SD.remove(name);
  }
  
  if (ret) 
    writeToBLE("DEL\n");
  else     
    writeToBLE("FAIL\n");
    
  return ret;
}
