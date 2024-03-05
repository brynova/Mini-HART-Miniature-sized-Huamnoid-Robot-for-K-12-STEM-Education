#include <OpenCM904.h>

#include <Dynamixel2Arduino.h>
#include <actuator.h>

/*
 * Example showing how to send position commands to AX-12A
 */

#include <Arduino.h>
#include <AX12A.h>

#include <DynamixelSDK.h>

// Control table addresses
#define ADDR_AX_TORQUE_ENABLE          24
#define ADDR_AX_GOAL_POSITION          30
#define ADDR_AX_PRESENT_POSITION       36

// Protocol version
#define PROTOCOL_VERSION               1.0


#define DXL_BUS_SIERIAL1 1 //Dynamixel on Serial 1(USART1) <- OpenCM9.04
#define DXL_BUS_SIERIAL2 2 //Dynamixel on Serial 1(USART2) <- LN191,BT210
#define DXL_BUS_SIERIAL3 3 //Dynamixel on Serial 1(USART1) <- OpenCM 485 EXP

//Defined Motor ID's
#define ID_NUM1 1    // ID of the first motor 
#define ID_NUM2 2    // ID of the first motor
#define ID_NUM3 3    // ID of the first motor
#define ID_NUM4 4    // ID of the first motor
#define ID_NUM5 5    // ID of the first motor
#define ID_NUM6 6    // ID of the first motor
#define ID_NUM7 7    // ID of the first motor
#define ID_NUM8 8    // ID of the first motor
#define ID_NUM9 9    // ID of the first motor
#define ID_NUM10 10    // ID of the first motor
#define ID_NUM11 11    // ID of the first motor
#define ID_NUM12 12    // ID of the first motor


#define BAUDRATE                       1000000
#define DEVICENAME                     "3"  // Use Serial3 for OpenCM 485 EXP
#define TORQUE_ENABLE                  1
#define TORQUE_DISABLE                 0

// Dynamixel Dxl(DXL_BUS_SIERIAL1);

int initial_pos = 512;
int max = initial_pos + 100;
int min = initial_pos - 100;

int pos = initial_pos;
int delta = 5;

  void setup()
  {
   
	  ax12a.begin(BaudRate, DirectionPin, &Serial);
    Dxl.jointMode(ID_1); //jointMode() is to use position mode
    Dxl.jointMode(ID_2); //jointMode() is to use position mode
    Dxl.jointMode(ID_3); //jointMode() is to use position mode
    Dxl.jointMode(ID_4); //jointMode() is to use position mode
    Dxl.jointMode(ID_5); //jointMode() is to use position mode
    Dxl.jointMode(ID_6); //jointMode() is to use position mode
    Dxl.jointMode(ID_7); //jointMode() is to use position mode
    Dxl.jointMode(ID_8); //jointMode() is to use position mode
    Dxl.jointMode(ID_9); //jointMode() is to use position mode
    Dxl.jointMode(ID_10); //jointMode() is to use position mode
    Dxl.jointMode(ID_11); //jointMode() is to use position mode
    Dxl.jointMode(ID_12); //jointMode() is to use position mode
  }


  void loop()
  {
	  pos = pos + delta;
	  if (pos > max)
	  { pos = max;
		  delta *= -1;
	  }

	  if (pos < min)
	  { pos = min;
		  delta *= -1;
	  } 


    //Use this when including that of the commands from Arduino board
    char in =   SerialUSB.read();

    SerialUSB.println(in);

    //Turn dynamixel ID 1,2 and 3 to position 1, 512 and 512 which is middle 

   //Moving the left 
    if(in=='1'){
    Dxl.writeWord(ID_1, GOAL_POSITION, 212); //Compatible with all dynamixel serise
    Dxl.writeWord(ID_2, GOAL_POSITION, 512);
    Dxl.writeWord(ID_3, GOAL_POSITION, 512);
    Dxl.writeWord(ID_4, GOAL_POSITION, 512);
    Dxl.writeWord(ID_5, GOAL_POSITION, 512);
    Dxl.writeWord(ID_6, GOAL_POSITION, 512);
    delay(1000);
    } 
  
    if(in=='2'){ 
    Dxl.writeWord(ID_1, GOAL_POSITION, 512); //Compatible with all dynamixel serise
    Dxl.writeWord(ID_2, GOAL_POSITION, 512);
    Dxl.writeWord(ID_3, GOAL_POSITION, 512);
    Dxl.writeWord(ID_4, GOAL_POSITION, 512);
    Dxl.writeWord(ID_5, GOAL_POSITION, 512);
    Dxl.writeWord(ID_6, GOAL_POSITION, 512);
    delay(1000);
    } 
  
   if(in=='3'){ 
    Dxl.writeWord(ID_1, GOAL_POSITION, 212); //Compatible with all dynamixel serise
    Dxl.writeWord(ID_2, GOAL_POSITION, 512);
    Dxl.writeWord(ID_3, GOAL_POSITION, 212);
    Dxl.writeWord(ID_4, GOAL_POSITION, 512);
    Dxl.writeWord(ID_5, GOAL_POSITION, 512);
    Dxl.writeWord(ID_6, GOAL_POSITION, 512);
    delay(1000);
    }

    if(in=='4'){ 
    Dxl.writeWord(ID_1, GOAL_POSITION, 512); //Compatible with all dynamixel serise
    Dxl.writeWord(ID_2, GOAL_POSITION, 212);
    Dxl.writeWord(ID_3, GOAL_POSITION, 112);
    Dxl.writeWord(ID_4, GOAL_POSITION, 512);
    Dxl.writeWord(ID_5, GOAL_POSITION, 512);
    Dxl.writeWord(ID_6, GOAL_POSITION, 512);
    delay(1000);
    }
    if(in=='5'){ 
    Dxl.writeWord(ID_1, GOAL_POSITION, 512); //Compatible with all dynamixel serise
    Dxl.writeWord(ID_2, GOAL_POSITION, 212);
    Dxl.writeWord(ID_3, GOAL_POSITION, 712);
    Dxl.writeWord(ID_4, GOAL_POSITION, 512);
    Dxl.writeWord(ID_5, GOAL_POSITION, 512);
    Dxl.writeWord(ID_6, GOAL_POSITION, 512);
    delay(1000);

    //Movement in the Right leg
	  if(in=='6'){ 
    Dxl.writeWord(ID_7, GOAL_POSITION, 512); //Compatible with all dynamixel serise
    Dxl.writeWord(ID_8, GOAL_POSITION, 212);
    Dxl.writeWord(ID_9, GOAL_POSITION, 712);
    Dxl.writeWord(ID_10, GOAL_POSITION, 512);
    Dxl.writeWord(ID_11, GOAL_POSITION, 512);
    Dxl.writeWord(ID_12, GOAL_POSITION, 512);
    delay(1000);

    if(in=='7'){ 
    Dxl.writeWord(ID_7, GOAL_POSITION, 512); //Compatible with all dynamixel serise
    Dxl.writeWord(ID_8, GOAL_POSITION, 212);
    Dxl.writeWord(ID_9, GOAL_POSITION, 712);
    Dxl.writeWord(ID_10, GOAL_POSITION, 512);
    Dxl.writeWord(ID_11, GOAL_POSITION, 512);
    Dxl.writeWord(ID_12, GOAL_POSITION, 512);
    delay(1000);

    if(in=='5'){ 
    Dxl.writeWord(ID_7, GOAL_POSITION, 512); //Compatible with all dynamixel serise
    Dxl.writeWord(ID_8, GOAL_POSITION, 212);
    Dxl.writeWord(ID_9, GOAL_POSITION, 712);
    Dxl.writeWord(ID_10, GOAL_POSITION, 512);
    Dxl.writeWord(ID_11, GOAL_POSITION, 512);
    Dxl.writeWord(ID_12, GOAL_POSITION, 512);
    delay(1000);
   
    
  class forward(){

    
  }


    ax12a.move(ID, pos);
	  delay(20);
  } twi
