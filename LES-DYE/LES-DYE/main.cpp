//#include <stdlib.h>
#include "ftd2xx.h"
#include "myFTDI2.h"
#include <fstream>
#include <iostream>
#include <string>
#include "conio.h"
//#define _USE_MATH_DEFINES
#include <math.h>


#define M_PI       3.14159265358979323846


using namespace std;




//--------------------------------
// DECLARE VARIABLES
//--------------------------------

//--------------------------------
// START FTDI
//--------------------------------
//	myFTDI2 ftdi;
//	ftdi.FT_Initialize();
//
//	for(int i =0; i<1000; i++)
//	{
//		yHI = ftdi.ReadGYRO();
//		yLO = ftdi.ReadGYRO();
//		ARRY[i] = yHI;
//		i++;
//		ARRY[i] = yLO;
//	}
//
//	ftdi.Close();
//}

int main(){

	unsigned char RxBuffer[35];
	myFTDI2 ftdi;
	ftdi.FT_Initialize();
	short x_bias = 0;
	short y_bias = 0;
	short z_bias = 0;

	int IDnum;
	short xL;
	short xH;
	short x;
	short yL;
	short yH;
	short y;
	short zL;
	short zH;
	short z;
	short PW1;
	short PW2;
	char keyVal;
	short PW3;
	short PW4;
	double xdoub;
	double ydoub;
	short PW_1H;
	short PW_1L;
	short PW_2H;
	short PW_2L;
	short PW_4H;
	short PW_4L;
	short PW_3L;
	short PW_3H;
	unsigned char PWBuffer[8];
	short roll_Scaled;///M_PI;
	short pitch_Scaled;
	double roll;
	double pitch;
	ofstream myfile;
	while(1)
	{

		//keyVal = getch();
		cout << "Enter input" << endl;
		cin >> keyVal;

		switch(keyVal){
		case 'a':
			//Initialize Sensor communication
			ftdi.Write(keyVal);
			keyVal =0;
			break;

		case 'b': 
			ftdi.Write(keyVal);
			ftdi.Read(RxBuffer);
			xL=(short)RxBuffer[1];
			xH=(short)RxBuffer[2]<<8;
			x_bias = xH + xL;
			yL=(short)RxBuffer[3];
			yH=(short)RxBuffer[4]<<8;
			y_bias = yH + yL;
			zL=(short)RxBuffer[5];
			zH=(short)RxBuffer[6]<<8;
			z_bias = zH + zL;
			keyVal =0;
			break;
		case 's':	
			//Stop the aquisition
			ftdi.Write(keyVal);
			keyVal =0;
			break;

		case 'k':
			//Start the aquisition
			myfile.open("C:\\Users\\youngem1\\Documents\\Visual Studio 2010\\Projects\\LES-DYE\\HW2Data.txt");
			do{	
				ftdi.Write(keyVal);
				ftdi.Read(RxBuffer);
				IDnum = RxBuffer[0];
				xL=(short)RxBuffer[1];
				xH=(short)RxBuffer[2]<<8;
				x = xH + xL - x_bias;
				yL=(short)RxBuffer[3];
				yH=(short)RxBuffer[4]<<8;
				y = yH + yL - y_bias;
				zL=(short)RxBuffer[5];
				zH=(short)RxBuffer[6]<<8;
				z = zH + zL - z_bias;
				cout<<"Packet Number: "<<IDnum<<endl;
				cout<<"x: "<<x<<endl;
				cout<<"y: "<<y<<endl;
				cout<<"z: "<<z<<endl;

				myfile<<" Packet Number: "<<IDnum;
				myfile<<" x: "<<x;
				myfile<<" y: "<<y;
				myfile<<" z: "<<z << endl;


				if(x>=16384)
				{
					x= 16384;
				}
				if(x<=-16384)
				{
					x= -16384;
				}
				xdoub = (double) x;
				roll = asin(xdoub/16384);

				if(y>=16384)
				{
					y= 16384;
				}
				if(y<=-16384)
				{
					y= -16384;
				}
				ydoub = (double) y;
				pitch = asin(ydoub/16384);

				roll_Scaled = (short)((roll*1999)/M_PI);
		
				pitch_Scaled = (short) ((pitch*1999)/M_PI);
				//-------------------------------------------------
				//     Set&Send PWM X
				//-------------------------------------------------

				if(roll_Scaled>=0)
				{
					PW1 = 999 - roll_Scaled;
					PW2 = 999;
				}
				else
				{
					PW1 = 999;
					PW2 = 999 - abs(roll_Scaled);
				}

				PW_1H =(PW1/256);
				PW_1L = (PW1%256);
				PW_2H = (PW2/256);
				PW_2L = (PW2%256);



				ftdi.WritePWM((char)PW_1H);
				ftdi.WritePWM((char)PW_1L);
				//Sleep(50);
				ftdi.WritePWM((char)PW_2H);
				ftdi.WritePWM((char)PW_2L);

				//-------------------------------------------------
				//     Set&Send PWM Y
				//-------------------------------------------------
				if(pitch_Scaled>=0)
				{
					PW3 = 999 - pitch_Scaled;
					PW4 = 999;
				}
				else
				{
					PW3 = 999;
					PW4 = 999 - abs(pitch_Scaled);
				}

				PW_3H = PW3/256;
				PW_3L = PW3%256;
				PW_4H = PW4/256;
				PW_4L = PW4%256;


				ftdi.WritePWM((char)PW_3H);
				ftdi.WritePWM((char)PW_3L);
				ftdi.WritePWM((char)PW_4H);
				ftdi.WritePWM((char)PW_4L);

			}while(!kbhit());
			myfile.close();
			keyVal =0;
			break;
		default:
			cout << "Please enter a valid option" << endl;
			//for debugging... 
			//ftdi.Write(keyVal);
			break;
		}
	}
}







///----------
// MAIN LOOP
//----------

//cout << "Enter Input" << endl;
//cin >> keyVal;
//ftdi.Read();


//THIS IS HOW WE CAN PACKAGE THE INFO BEFORE HAND
//ie take advantage of faster processer to reduce load on microcontroller -- maybe will speed up 

//	switch(keyVal){
//		case 'a':
//			//Initialize Sensor communication
//			ftdi.Write(keyVal);
//			break;
//
//			//	case 'b': 
//		//as the sensor
//			//		break;
//			//	
//			//	case 's':	
//			//ftdi.Write(keyVal);
//			//		break;
//
//		case 'k':
//			ftdi.Write(keyVal);
//			ftdi.Read();
//			break;
//
//			//	case 'w':
//			//		ftdi.Write(keyVal);
//			//		ftdi.Read();
//			//		break;
//			//	case 'x':
//			//		ftdi.Write(keyVal);
//			//		ftdi.Read();
//			//		break;
//
//		default:
//			cout << "Please enter a valid option" << endl;
//			//for debugging... 
//			ftdi.Write(keyVal);
//			break;
//		}
//	}
//
//}
////
////
////	}//end of main loop
////
////	ftdi.Close();
////
////}
