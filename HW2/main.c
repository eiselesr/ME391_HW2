#include <ioCC2530.h>
//--------------------------------------------------------------------------
// DEFINE MASKS
//--------------------------------------------------------------------------
#define CS_A P1_3 // CHIP SELECT FOR ACCELEROMETER Pin P10_17 on board
#define CS_G P1_4 //CHIP SELECT FOR GYROSCOPE
#define WHO_AM_I_G 0x0F
#define READ 0x80
#define WRITE 0x00
#define JUNK 0xAA
#define CTRL_REG5_A 0x20
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D

//--------------------------------------------------------------------------
// DEFINE GLOBAL VARIABLES
//--------------------------------------------------------------------------
unsigned char whoami;// NEED TO DECLARE HERE TO AVOID BEING OPTIMIZED OUT
unsigned short uartRxIndex;
unsigned short uartTxIndex;
unsigned char spiTxBuffer[10];
unsigned char uartRxBuffer[10];
//unsigned char uartTxBuffer[10]; //NOT USING I DON'T THINK
unsigned char spiRxBuffer[10];  
unsigned int start;//flag for starting IMU communication
unsigned int acquire;// flag for data aquisition
unsigned int bias;
int x_bias;
int y_bias;
int z_bias;
unsigned short xL;
unsigned short yL;
unsigned short zL;
unsigned short xH;
unsigned short yH;
unsigned short zH;
short x;
short y;
short z;
//-------------------------------------------
//      CONFIGURE USART0
//-------------------------------------------
void configureUSART0forUART_ALT1(){   
  PERCFG &= ~0X01; //SET USART0 TO ALT LOCATION 1 - FAMILY PG. 85
  P0SEL  |=  0x0C; //SET RX(bit2) and TX(bit3) to PERIPHERAL FUNCTION [---- 11--] - FAMILY PG. 85
  P0DIR  |=  0x08; //SET TX(bit3) TO OUTPUT(1) [---- 1---] - FAMILY PG.86
  P0DIR  &= ~0X04; //SET RX(bit2) TO INPUT(0) [---- -0--] - FAMILY PG.86
  
  U0CSR  |=  0x80; //SET USART0 TO UART MODE -FAMILY PG.160
  
  // set stop/start bit levels parity, number of stop bits etc...
  U0UCR |=  0x06;//Flow control disabled, 8bit transfer, Parity diabled, 2 stop bits, high stop bit, low start bit[-0-0 0110]
  U0UCR &= ~0x59;
  
  // Chose 28800 baud rate... because it seemed like a good number
  U0BAUD =  0xD8; //SET BAUD_M = 216(0xD8) - Family pdf pg159 
  U0GCR |=  0x09; //SET U0GCR.BAUD_E = 9 => Set 0 and 3 bit to 1 [---0 1001]
  U0GCR &= ~0x16; //                        and bits 1,2, and 4 to 0.    
}

//-------------------------------------------
//      BIAS IMU SENSOR
//-------------------------------------------
void biasSensor()
{
 for(int i=0; i<7; i++)//READ EXTRA BIT. LAST ONE IS JUNK TO FLUSH OUT NEEDED BITS
      {   
        CS_A = 0;
        U1TX_BYTE = 0;
        U1DBUF = (OUT_X_L_A + i) | READ;
        while(!U1TX_BYTE);    
        U1TX_BYTE = 0; //NOT SURE IF THIS IS NEEDED OR NOT
        U1DBUF = JUNK;
        while (!U1TX_BYTE);    
        CS_A =1;
        spiRxBuffer[i+1] = U1DBUF;// START FROM 0 with a junk bit or 1 if bit is good.     
      }	
      
      xL=(unsigned short)spiRxBuffer[1];
      yL=(unsigned short)spiRxBuffer[3];
      zL=(unsigned short)spiRxBuffer[5];
      xH=(unsigned short)spiRxBuffer[2]<<8;
      yH=(unsigned short)spiRxBuffer[4]<<8;
      zH=(unsigned short)spiRxBuffer[6]<<8;
      
      x = xH + xL;
      y = yH + yL;
      z = zH + zL;
        
      //--------------------------------------------------------------------------
      // Transmit Package
      //--------------------------------------------------------------------------   
      
      IEN0 &= ~0x80;
      IEN0 &= ~0x04;//USART0 RX INTERRUPT
      
      for (uartTxIndex = 0; uartTxIndex<7; uartTxIndex++)
      {
        U0CSR &= ~0x02; //SET U0TX_BYTE to 0
        U0DBUF = spiRxBuffer[uartTxIndex];      
        while (!(U0CSR&0x02));
      }
      
      IEN0 |= 0x80;
      IEN0 |= 0x04;
}
//-------------------------------------------
//      Turn on RX
//-------------------------------------------
void uartStartRxForIsr()
{
  uartRxIndex = 0;
  URX0IF = 0;
  U0CSR |= 0x40;//Enables UART receiver
  IEN0 |= 0x04; //Enable USART0 RX interrupt => Set bit 2 to 1 (family pg. 45)
  IEN0 |= 0x80; //ENABLE INTERRUPTS GENERALLY
}

//-------------------------------------------
// RX interrupt service routine
//-------------------------------------------
_Pragma("vector=0x13") __near_func __interrupt void UART0_RX_ISR(void);
_Pragma("vector=0x13") __near_func __interrupt void UART0_RX_ISR(void)
{	
  IEN0 &= ~0x80;//DISABLE INTERRUPTS
  URX0IF = 0; //Interrupt not pending
  unsigned int keyVal = U0DBUF;
  switch(keyVal)
  {
  case 97:// 'a' key
    start =1;//Start communication with the Sensor
    break;
  case 98:// 'b' key
    if(start==1)
    {
      bias=1;	
      acquire=0;
    }
    break;
  case 107:// 'k' key
    if (start==1)
    {
      acquire = 1;//Start aquisition
    }
    break;
  case 115:// 's' key
    acquire = 0;//stop aquisition
    break;
  }
  IEN0 |= 0x80; //ENABLE INTERRUPTS GENERALLY
}


void main(void)
{
  CLKCONCMD &= 0x00;//Set system clock to 32MHZ => set bit 6 to 1 => [-1-- ----]
  while(CLKCONSTA&0x40);//waiting for clock to become stable
  //--------------------------------------------------------------------------
  // DECLARE VARIABLES
  //--------------------------------------------------------------------------
  int packetNum = 0;
  start = 0; //Make sure communcation will not start immediately
  acquire = 0; //Make sure data from IMU will not be aquired.
  
  //--------------------------------------------------------------------------
  // CONFIGURE UART
  //--------------------------------------------------------------------------
  configureUSART0forUART_ALT1();
  //--------------------------------------------------------------------------
  // CONFIGURE SPI
  //--------------------------------------------------------------------------
  PERCFG |= 0x02; //SET USART 1 I/O location TO ALTERNATIVE 2 => set bit 1 to 1: [---- --1-] - Family pg. 85
  U1CSR &= ~0xA0; //Set USART 1 TO SPI MODE and Set UART 1 SPI TO MASTER MODE[0-0- ----]
  
  //-----------------------------------------------------
  // CONFIGURE SPI PERIPHERALS
  //-----------------------------------------------------  
  P1SEL |=0xE0; //set P1_5, P1_6, P1_7 are peripherals [111- ----]
  P1SEL &= ~0x1F;  //P1_3 and P1_4 are GP I/O (SSN) [---0 0---]
  P1DIR |= 0x7F; // SET MO, C, CS_G, CS_A to output [-111 1---]
  P1DIR &=~0x80; //SET MI to input[0--- ----]
  //-----------------------------------------------------
  // CONFIGURE SPI BAUD RATE
  //-----------------------------------------------------   
  U1BAUD = 0x3B;//BAUD_M= 59  //0x00;// BAUD_M = 0
  U1GCR |= 0x06;//BAUD_E = 6  //0x11;// BAUD_E = 17
  //-----------------------------------------------------
  // CONFIGURE SPI POLARITY, DATA TRANSFER, AND BIT ORDER
  //-----------------------------------------------------   
  //CPHA = 0 means:
  //Data is output on MOSI when SCK goes from CPOL inverted to CPOL, and data input
  //is sampled on MISO when SCK goes from CPOL to CPOL inverted.
  //CPOL = 0 => Clock polarity is negative
  U1GCR &= ~0xC0; //U1GCR.CPOL = U1GCR.CPHA = 0 [00-- ----] - familiy pg. 163
  U1GCR |=0x20;// U1GCR.ORDER = 1=> MSB first  [--1- ----]
  //-----------------------------------------------------
  // CONFIGURE TIMER 1
  //----------------------------------------------------- 
  PERCFG |= 0x40;  // [-1-- ----]
  P0SEL |= 0xC0;  // [11-- ----]
  P1SEL |= 0x07;  //[---- -111]
  P0DIR |= 0xC0;  //[11-- ----]
  P1DIR |= 0x07;  //[---- -111]
  T1CTL |= 0x0A;  //[---- 1-1-]
  T1CTL &=~ 0x05;  // [---- -0-0]
  T1CC0H = 0x03;  //[0000 0011]
  T1CC0L = 0xE8;  //[1110 1000]
  T1CCTL1 |= 0x2C; //[--1- 11--]
  T1CCTL1 &= ~0x10; //[---0 ----]
  T1CCTL2 |= 0x2C; //[--1- 11--]
  T1CCTL2 &= ~0x10; //[---0 ----]
  T1CCTL3 |= 0x2C; //[--1- 11--]
  T1CCTL3 &= ~0x10; //[---0 ----]
  T1CCTL4 |= 0x2C; //[--1- 11--]
  T1CCTL4 &= ~0x10; //[---0 ----]
  
  //TEST MOTORS!!!
  // T1CC1H=0x0
  
  //-----------------------------------------------------
  // ENSURE BOTH CHIP SELECT FOR ACCEL AND GYRO ARE HIGH
  //-----------------------------------------------------  
  CS_G = 1;
  CS_A = 1;
  
  //-----------------------------------------------------
  // CHECK COMMUNICATION (DEBUG WITH WHO AM I G)
  //----------------------------------------------------- 
  CS_G = 0;
  
  U1TX_BYTE = 0;
  U1DBUF = WHO_AM_I_G | READ;
  while (!U1TX_BYTE);
  
  U1TX_BYTE = 0;
  U1DBUF = JUNK;
  while (!U1TX_BYTE);
  
  whoami = whoami;
  whoami = U1DBUF;
  
  CS_G = 1;
  //--------------------------------------------------------------------------
  // START ISR's
  //-------------------------------------------------------------------------- 
  uartStartRxForIsr();
  //--------------------------------------------------------------------------
  // CONFIGURE SPI
  //--------------------------------------------------------------------------    
  while(start==0);
  
  spiTxBuffer[0] = CTRL_REG5_A; //WRITE DATA AT 0x20
  spiTxBuffer[1] = 0x77; //[1001 -111] data rate to 1600hz and enable xyz axis
  
  CS_A = 0;
  for (int i = 0; i < 2; i++) 
  { 
    U1TX_BYTE = 0;
    U1DBUF = spiTxBuffer[i];  
    while (!U1TX_BYTE); 
  }      
  CS_A = 1;
  
  
  //	//======================================================================================
  while(1)
  {
    if(bias==1)
    {
      biasSensor();
      bias=0;
    }
    while(acquire==1)
    {
      //--------------------------------------------------------------------------
      // READ ACCELEROMETER/ Assemble package
      //-------------------------------------------------------------------------- 
      
      for(int i=0; i<7; i++)//READ EXTRA BIT. LAST ONE IS JUNK TO FLUSH OUT NEEDED BITS
      {   
        CS_A = 0;
        U1TX_BYTE = 0;
        U1DBUF = (OUT_X_L_A + i) | READ;
        while(!U1TX_BYTE);    
        U1TX_BYTE = 0; //NOT SURE IF THIS IS NEEDED OR NOT
        U1DBUF = JUNK;
        while (!U1TX_BYTE);    
        CS_A =1;
        spiRxBuffer[i+1] = U1DBUF;// START FROM 0 with a junk bit or 1 if bit is good.     
      }	
      
      xL=(unsigned short)spiRxBuffer[1];
      yL=(unsigned short)spiRxBuffer[3];
      zL=(unsigned short)spiRxBuffer[5];
      xH=(unsigned short)spiRxBuffer[2]<<8;
      yH=(unsigned short)spiRxBuffer[4]<<8;
      zH=(unsigned short)spiRxBuffer[6]<<8;
      
      x = xH + xL;
      y = yH + yL;
      z = zH + zL;
      
      spiRxBuffer[0] = packetNum;
      packetNum++;  
      // unsigned char testBuffer[3] = {
      //--------------------------------------------------------------------------
      // Transmit Package
      //--------------------------------------------------------------------------   
      
      //IEN0 &= ~0x80;
      IEN0 &= ~0x04;//USART0 RX INTERRUPT
      
      for (uartTxIndex = 0; uartTxIndex<7; uartTxIndex++)
      {
        U0CSR &= ~0x02; //SET U0TX_BYTE to 0
        U0DBUF = spiRxBuffer[uartTxIndex];      
        while (!(U0CSR&0x02));
      }
      
      //--------------------------------------------------------------------------
      // GET PWM
      //--------------------------------------------------------------------------
      for (int i=0;i<8;i++) {//Read 8 things
        //U0CSR &= ~0x04; //SET U0RX_BYTE to 0
        while (!(U0CSR&0x04));
        uartRxBuffer[i]=U0DBUF;
        U0CSR &= ~0x04; //SET U0RX_BYTE to 0
      }
      acquire=0;// WAIT TO SEE IF PC SIDE IS STILL ACQUIRING DATA OR IF ANOTHER KEY HAS BEEN PRESSED.
      
      //--------------------------------------------------------------------------
      // SEND TO MOTOR
      //--------------------------------------------------------------------------  
      T1CC1H = uartRxBuffer[0];
      T1CC1L = uartRxBuffer[1];
      T1CC2H = uartRxBuffer[2];
      T1CC2L = uartRxBuffer[3];
      T1CC3H = uartRxBuffer[4];
      T1CC3L = uartRxBuffer[5];
      T1CC4H = uartRxBuffer[6];
      T1CC4L = uartRxBuffer[7];
      
      //IEN0 |= 0x80;
      IEN0 |= 0x04;
      
    }
  }
  
}