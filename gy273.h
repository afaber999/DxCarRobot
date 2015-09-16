// NOTE - ANY TILT on the compass will cause LARGE variations in the readings.
// GY-273, 3 axis compass module (no tilt compensation)
// (c) 2015 A.L. Faber
//
// GY-273 Compass Module (HMC5883)  -> Arduino 
// VCC -> VCC (See Note Below)
// GND -> GND
// SCL -> A5/SCL, (Use Pin 21 on the Arduino Mega)
// SDA -> A4/SDA, (Use Pin 20 on the Arduino Mega)
// DRDY -> Not Connected (in this example

#include <Wire.h>

#define Gy273Address (0x1E)

class Gy273
{
public:
	struct Vector3i
    {
      int X;
      int Y;
      int Z;
    };

	Gy273( )
	{  
	}
 
  void Init()
  {
    Wire.begin();
		Wire.beginTransmission(Gy273Address); 
		Wire.write(byte(0x02));
		Wire.write(byte(0x00));
		Wire.endTransmission();
		Update();
	}
	

	float GetHeading(int offsetX = 0, int offsetY = 0)
	{
		Vector3i vec = Update();
		float heading = atan2(vec.X + offsetX, vec.Y + offsetY);
		return heading;		
	}
	
	Vector3i Update()
	{
		Vector3i result;
		result.X = 0;
		result.Y = 0;
		result.Z = 0;
		
		Wire.beginTransmission(Gy273Address);
		Wire.write(byte(0x03)); 
		Wire.endTransmission();
		Wire.requestFrom(Gy273Address, 6);
	  
		if (6 <= Wire.available())
		{
			result.X = Wire.read() << 8;
			result.X |= Wire.read();
			result.Z = Wire.read() << 8;
			result.Z |= Wire.read();
			result.Y = Wire.read() << 8;
			result.Y |= Wire.read();
		}
		return result;
	}
	
};


