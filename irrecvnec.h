#include "Arduino.h"

class IrRecvNec
{

public:

  IrRecvNec(uint8_t pin )
  {
    Reset();  
    _pin = pin;
    pinMode(_pin, INPUT_PULLUP);
    _lastSensorValue = digitalRead(_pin);
  }

  uint8_t GetCode()
  {
    if (_bits == 32 )
    {
      _bits = 0;
      return _code3;
    }
    return 0x00;
  }

  // current assumption that this method is called every 300 us
  void Sample()
  {
     uint8_t sensorValue = digitalRead(_pin);

      if  (sensorValue == _lastSensorValue )
      {
        ++_sameCnt;
      }
      else
      {
        if ( _sameCnt > 10 )
        {
          Reset();
        }
        else 
        {
          if ( sensorValue == 0 )
          {
            ++_bits;
            uint8_t orValue = 0x00;

            if ( _sameCnt >= 3 )
            {
              orValue = 0x80;
            }

            if ( _bits > 24 )
            {
              _code3 >>= 1;
              _code3 |= orValue;
            } else if ( _bits > 16 )
            {
              _code2 >>= 1;
              _code2 |= orValue;
            } else if ( _bits > 8 )
            {
              _code1 >>= 1;
              _code1 |= orValue;          
            } else 
            {
              _code0 >>= 1;
              _code0 |= orValue;
            }
          }
        }
        _sameCnt = 0; 
        _lastSensorValue = sensorValue;
      }
    }
private:
  
  void Reset()
  {
    _bits = 0;
    _sameCnt = 0;
    _code0 = 0;
    _code1 = 0;
    _code2 = 0;
    _code3 = 0;    
  }

  uint8_t _pin;
  uint8_t _lastSensorValue;

  uint8_t _sameCnt;
  
  volatile uint8_t _bits;
  volatile uint8_t _code0;
  volatile uint8_t _code1;
  volatile uint8_t _code2;
  volatile uint8_t _code3;
};

