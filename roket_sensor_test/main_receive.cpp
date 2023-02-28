#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include <iostream>

#include <pico/stdio.h>
#include <hardware/uart.h>
#include "tusb.h"

#include "SerialUART.h"
#include "dataStruct.hpp"

#include <vector>

static void core1_entry()
{
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  while (true)
  {
    gpio_put(LED_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_PIN, 0);
    sleep_ms(1000);
  }
}

void print(TransmitData);

struct Parser
{
  static const size_t maxLen = sizeof(TransmitData);

  enum class ParserState
  {
    ESCAPE = '$',
    PARSER = 0
  };

  ParserState state = ParserState::PARSER;

  std::vector<uint8_t> current;

  void parse(uint8_t ch)
  {
    switch (state)
    {
    case ParserState::PARSER:
    {
      if (ch == '$')
      {
        state = ParserState::ESCAPE;
        return;
      }
      else
      {
        current.push_back(ch);
        if (current.size() > maxLen)
        {
          current = decltype(current)(); // Clear buffer
        }
      }
    }
    break;
    case ParserState::ESCAPE:
    {
      switch (ch)
      {
      case '$':
      {
        current.push_back(ch);
        if (current.size() > maxLen)
        {
          goto clearAndReturn;
        }
      }
      break;
      case '{':
      {
        goto clearAndReturn;
      }
      break;
      case '}':
      {
        if(current.size() == maxLen){ // Correct size, look checksum
          TransmitData *td = reinterpret_cast<decltype(td)>(current.data());
          if(td->checkCheckSum()){
            print(*td);
          } else { 
            goto clearAndReturn;
          }
        } else {
          goto clearAndReturn;
        }
      }
      break;
      default:
        break;
      }


      clearAndReturn:
      current = decltype(current)(); // Clear buffer
      normalReturn:
      state = ParserState::PARSER;
      return;
    }
    break;
    default:
      break;
    }
  }
};

int main()
{
  stdio_init_all();
  multicore_launch_core1(core1_entry);

  Serial1.begin(115200);

  uint32_t t0, t1;

  t0 = time_us_32();
  while (!tud_cdc_connected())
  {
  }
  t1 = time_us_32();
  printf("\nusb host detected! (%dus)\n", t1 - t0);

  Parser parser;

  while (1)
  {
    while(Serial1.available()){
      parser.parse(Serial1.read());
    }
  }
  return 0;
}


void print(TransmitData td){
  Serial.print(static_cast<std::string>(td).c_str());
  Serial.println("********************************\r\n");
}
