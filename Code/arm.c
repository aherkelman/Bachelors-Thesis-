//Code for getting pulse width/duty rate from an angle. For joint 1.
//Goes from -90 to 90 degrees.
float getPulseWidthJoint1(float angle)
{
	float pulseWidth;
	if (angle<=0)
	{
		pulseWidth = 1500 - angle*9.44444444444444;
		//duty = 0.077-angle*0.00043889;
	}
	else
	{
		pulseWidth = 1500 - angle*8.888888888889;
		//duty = 0.077-angle*.00048333;
	}
	return pulseWidth;
}

//Code for getting pulse width/duty rate from an angle. For joint 2.
//Goes from -50 to 120 degrees. Negative is in the possitive z direction.
float getPulseWidthJoint2(float angle)
{
	float pulseWidth;
	if (angle<=0)
	{
		pulseWidth = 2000 - angle*11.1666667;
		//duty = 0.0995-angle*0.000446;
	}
	else
	{
		pulseWidth = 2000 - angle*8.8;
		//duty = 0.0995-angle*.0004667;
	}
	return pulseWidth;
}

// for getting pulse width/duty rate from an angle. For joint 3.
//Goes from -55 to 120 degrees. Negative is in the possitive z direction.
float getPulseWidthJoint3(float angle)
{
	float pulseWidth;
	if (angle<=0)
	{
		pulseWidth = 1900 - angle*9.5;
		//duty = 0.0945-angle*0.000455;
	}
	else
	{
		pulseWidth = 1900 - angle*9.27272727273;
		//duty = 0.0945-angle*.000433;
	}
	return pulseWidth;
}

//Code for getting pulse width/duty rate from an angle. For joint 4.
//Goes from -110 to 90 degrees. Negative is in the possitive z direction.
float getPulseWidthJoint4(float angle)
{
	float pulseWidth;
	if (angle<=0)
	{
		pulseWidth = 1650 + angle*9.88888888889;
		//duty = 0.0815+angle*0.00051818;
	}
	else
		{
		pulseWidth = 1650 + angle*10.81818181818182;
		//duty = 0.0815+angle*.0005;
		}
	return pulseWidth;
}

//Close appropriate amount for specific object.
void closeGripper()
{
	// Value found to work best closing around objects is 1700ms pulse width
	float pulseWidthMicroseconds = 1700;
	changePulseWidth(7, pulseWidthMicroseconds);
}
//open Gripper
void openGripper()
{
	// Value found to work best for open is 200ms pulse width
	float pulseWidthMicroseconds = 2600;
	changePulseWidth(7, pulseWidthMicroseconds);
}

//Given a set of angles sends the appropriate duty cycles to the arm to set the given
//angles.
void sendDuty(float angle1, float angle2, float angle3, float angle4)
{
	float pulseWidth1,pulseWidth2,pulseWidth3,pulseWidth4;

	pulseWidth1 = getPulseWidthJoint1(angle1);
	pulseWidth2 = getPulseWidthJoint2(angle2);
	pulseWidth3 = getPulseWidthJoint3(angle3);
	pulseWidth4 = getPulseWidthJoint4(angle4);

	changePulseWidth(2, pulseWidth1);
	changePulseWidth(3, pulseWidth2);
	changePulseWidth(4, pulseWidth3);
	changePulseWidth(5, pulseWidth4);

}

//Put arm into relaxed position and then shut it off.
void shutOffArm()
{
	sendDuty(0,90,0,0);
	prevTick = Ticks;
	while ((Ticks-prevTick)<25);
	///////////////////gioSetBit(gioPORTA, 0, 0);
	clearGPIO(PD1);
}

//Turns on arm and puts in straigt out position.
void turnOnArm()
{
	////////////////////////gioSetBit(gioPORTA,0,1);
	setGPIO(PD1);
	sendDuty(0,0,0,0);
}

