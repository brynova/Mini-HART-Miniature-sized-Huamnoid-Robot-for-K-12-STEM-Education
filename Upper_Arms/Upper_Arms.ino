/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

/* Dynamixel ID defines */
#define ID_NUM1 13
#define ID_NUM2 14
#define ID_NUM3 15
#define ID_NUM3 16
#define ID_NUM3 17
#define ID_NUM3 18

/* Control table defines */
#define GOAL_POSITION 30
Dynamixel Dxl(DXL_BUS_SERIAL1);

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
uint8_t dxl_error = 0; // Declare dxl_error
int dxl_comm_result = COMM_TX_FAIL; // Declare dxl_comm_result

void enableTorque(int id);
void disableTorque(int id);
void setMovingSpeed(int id, int speed, uint8_t &dxl_error, int &dxl_comm_result);
void moveMotor(int id, float angle);
int angleToPosition(float angle, int id);

void setup() {
  // put your setup code here, to run once:


void enableTorque(int id) {
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_AX_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        // handle error
    }
}

void disableTorque(int id) {
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_AX_TORQUE_ENABLE, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        // handle error
    }
}

void setMovingSpeed(int id, int speed, uint8_t &dxl_error, int &dxl_comm_result) {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_AX_MOVING_SPEED, speed, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        Serial.println("Failed to set moving speed!");
    }
}

void moveMotor(int id, float angle) {
    // Adjust angle to be negative for motor IDs 7 to 12
    if (id >= 7 && id <= 12) {
        angle *= -1;
    }
    
    int position = angleToPosition(angle, id);
    
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_AX_GOAL_POSITION, position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        // handle error
    }
}

int angleToPosition(float angle, int id) {
    int position;
    position = (int)(((angle + 150) / 300) * 1023); 
    return position;
}


}

void loop() {
  // put your main code here, to run repeatedly:

}
