//	Motor.h - Library for powering robot motors.
//	Created by Maen Artimy, September 30, 2012.
//	Copyright (c) 2012  Maen Artimy

#ifndef Motor298_h
#define Motor298_h

#include "Arduino.h"
class Motor298{

public:
	static const uint8_t Motor_LR = 0x11;
	static const uint8_t Motor_R  = 0x01;
	static const uint8_t Motor_L  = 0x10;
	
	Motor298()
	{
		pinMode (fwdPinA, OUTPUT); 
		pinMode (bkdPinA, OUTPUT); 
		pinMode (enbPinA, OUTPUT); 
		pinMode (fwdPinB, OUTPUT); 
		pinMode (bkdPinB, OUTPUT); 
		pinMode (enbPinB, OUTPUT); 
	}
	
	void onFwd(uint8_t motor, uint8_t speed)
	{
		if ( motor & Motor_R ) 
		{
			analogWrite( enbPinA, speed );
			digitalWrite( fwdPinA, HIGH ); 
			digitalWrite( bkdPinA, LOW );
		}
		
		if ( motor & Motor_L ) 
		{
			analogWrite( enbPinB, speed );
			digitalWrite( fwdPinB, HIGH ); 
			digitalWrite( bkdPinB, LOW );
		}
	}
	
	void onRev(uint8_t motor, uint8_t speed)
	{
		if ( motor & Motor_R ) 
		{
			analogWrite( enbPinA, speed );
			digitalWrite( fwdPinA, LOW ); 
			digitalWrite( bkdPinA, HIGH );
		}
		if ( motor & Motor_L ) 
		{
			analogWrite (enbPinB, speed);
			digitalWrite (fwdPinB, LOW); 
			digitalWrite (bkdPinB, HIGH);	
		}
	}
	
	void stop( uint8_t motor, uint8_t speed=255 )
	{
		if ( motor & Motor_R ) 
		{
			analogWrite( enbPinA, speed );
			digitalWrite( fwdPinA, LOW ); 
			digitalWrite( bkdPinA, LOW );
		}
		if ( motor & Motor_L ) 
		{
			analogWrite( enbPinB, speed );
			digitalWrite( fwdPinB, LOW ); 
			digitalWrite( bkdPinB, LOW );
		}
	}

	void off(uint8_t  motor)
	{
		stop(motor, 0);
	}
	
	uint8_t sspeed( uint8_t speed )
	{
		int res = (int)speed * 2/3;
		return (uint8_t)res;		
	}
	
	void fwdRight(uint8_t speed)
	{
		onFwd(Motor_L, speed);
		onFwd(Motor_R, sspeed(speed));
	}
	
	void fwdLeft(uint8_t speed)
	{
		onFwd(Motor_L, sspeed(speed));
		onFwd(Motor_R, speed);
	}

	void revRight(int)
	{
		onRev(Motor_L, speed);
		onRev(Motor_R, sspeed(speed));
	}
	
	void revLeft(int)
	{
		onRev(Motor_R, speed);
		onRev(Motor_L, sspeed(speed));
	}

	void turnRight(int)
	{
		onFwd(Motor_L, speed);
		onRev(Motor_R, speed);
	}
	
	void turnLeft(int)
	{
		onFwd(Motor_R, speed);
		onRev(Motor_L, speed);
	}
	
private:
	//Right Motor (A)
	static const int fwdPinA = 4;
	static const int bkdPinA = 7; 
	static const int enbPinA = 6;

	//Left Motor (B)
	static const int fwdPinB = 2;
	static const int bkdPinB = 3; 
	static const int enbPinB = 5;

};

#endif