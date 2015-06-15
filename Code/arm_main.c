#include "arm_main.h"
#include "arm.h"
// Library for string handling
#include <string.h>
// Library for sprintf
#include <stdio.h>
// Library for atof
#include <stdlib.h>

// Library for using delay function
#include "BAPlib/bap_main.h"
// Library for GPIO
#include "BAPlib/bap1_1_gpio.h"
// Library for using the BAP UART
#include "BAPlib/bap1_1_uart.h"
// Library for BAP PWM
#include "BAPlib/bap1_1_pwm.h"
// Library to read adc and battery

extern struct s_mympu mympu;

// Library for using the Tivaware Command Line interface
#include "utils/cmdline.h"

#include "BAPlib/bap1_1_tester.h"

#define SCI_DELIMr '\r'
#define SCI_DELIMn '\n'
#define TB_BUFF_LEN 100

char uartTxBuffer1[TB_BUFF_LEN];
float theta1;
float theta2;
float theta3;
float theta4;
int Data_Ready=0;
int pathLength=0;

union Data
{
	unsigned char bytes[16];
	float result[4];
};

void arm_main()
{
	attachUartCallback(0, camdata_callBack);
	// Welcome message
	sprintf(uartTxBuffer1,"ARM test start sending camera data.\n");
	BAPSendMessage(uartTxBuffer1);

	//initalize arm
	sendDuty(0,0,0,0);
	setGPIO(PB7);

	openGripper();

	while(1)
	{
		if (Data_Ready)
		{
			if(theta1==-1000)
			{
				sprintf(uartTxBuffer1,"Gripper closed.\n");
				BAPSendMessage(uartTxBuffer1);
				closeGripper();
			}
			else if (theta1==-2000)
			{
				sprintf(uartTxBuffer1,"Gripper opened.\n");
				BAPSendMessage(uartTxBuffer1);
				openGripper();
			}
			else
			{
				sprintf(uartTxBuffer1,"1: %f 2: %f 3: %f 4: %f \n",theta1,theta2,theta3,theta4);
				BAPSendMessage(uartTxBuffer1);
				sendDuty(theta1, theta2, theta3, theta4);
			}
			Data_Ready=0;
		}

	}
}


void camdata_callBack(unsigned char c)
{
	static int index = 0;
	static union Data data;
	//sprintf(uartTxBuffer1,"a");
	//BAPSendMessage(uartTxBuffer1);
	if (index < 16)
	{
		data.bytes[index]=c;
		++index;
	}
	else
	{
		unsigned char chkSum = 0;
		chkSum=c;

		if(isValid(data.bytes,chkSum))
		{
			theta1 = data.result[0];
			theta2 = data.result[1];
			theta3 = data.result[2];
			theta4 = data.result[3];
			Data_Ready = 1;
		}
		index=0;
	}
}

int isValid(unsigned char *dataArray, unsigned char checkSum)
{
	int i;
	int Sum=0;
	for(i=0;i<16;i++)
	{
		Sum+=dataArray[i];
	}
	if ((Sum&0xFF)==checkSum)
	{
		return 1;
	}
	return 0;
}
