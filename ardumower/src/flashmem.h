#ifndef FLASHMEM_H
#define FLASHMEM_H

#include <Arduino.h>

class FlashClass
{
public:
  bool verboseOutput;
  FlashClass();
  byte read(uint32_t address);
  byte *readAddress(uint32_t address);
  bool write(uint32_t address, byte value);
  bool write(uint32_t address, byte *data, uint32_t dataLength);
  void dump();
  void test();
};

extern FlashClass Flash;

template <class T>
int eewrite(int &ee, const T &value)
{
  const byte *p = (const byte *)(const void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    Flash.write(ee++, *p++);
  watchdogReset();
  return i;
}

template <class T>
int eeread(int &ee, T &value)
{
  byte *p = (byte *)(void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = Flash.read(ee++);
  watchdogReset();
  return i;
}

template <class T>
int eereadwrite(bool readflag, int &ee, T &value)
{
  byte *p = (byte *)(void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
  {
    if (readflag)
      *p++ = Flash.read(ee++);
    else
      Flash.write(ee++, *p++);
    watchdogReset();
  }
  return i;
}

int eereadwriteString(bool readflag, int &ee, String &value);

#endif
