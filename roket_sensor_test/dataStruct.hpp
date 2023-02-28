#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <sstream>

#pragma pack(push, 1)
struct TransmitData
{
  struct GNSS{
    bool fix;
    float lat;
    float lon;
    uint8_t siv;
  } gps;

  struct BMP{
    float pressure;
    float temp;
  } bmp;

  struct Acc{
    float x;
    float y;
    float z;
  } acc;

  uint8_t checksum;
  void fillCheckSum(){
    checksum = 0;
    uint8_t *ptr = reinterpret_cast<uint8_t*>(this);
    for(size_t i = 0; i < (sizeof(TransmitData) - sizeof(checksum)); i++){
      checksum ^= ptr[i];
    }
  }
  bool checkCheckSum(){
    decltype(checksum) res = 0;
    uint8_t *ptr = reinterpret_cast<uint8_t*>(this);
    for(size_t i = 0; i < sizeof(TransmitData); i++){
      res ^= ptr[i];
    }
    return (res == 0);
  }

  operator std::string(){
    std::stringstream ss;
    ss << "GPS: \r\n";
    ss << "\t Fix: " << gps.fix << "\r\n";
    ss << "\t Lat: " << gps.lat << "\r\n";
    ss << "\t Lon: " << gps.lon << "\r\n";
    ss << "BMP: \r\n";
    ss << "\t Pressure: " << bmp.pressure << "\r\n";
    ss << "\t Temperature: " << bmp.temp << "\r\n";
    ss << "Accelerometer: \r\n";
    ss << "\t X: " << acc.x << "\r\n";
    ss << "\t Y: " << acc.y << "\r\n";
    ss << "\t Z: " << acc.z << "\r\n";
    return ss.str();
  }

};
#pragma pack(pop)