#include <DynamixelSDK.h>

#define ADDR_AX_TORQUE_ENABLE          24
#define ADDR_AX_GOAL_POSITION          30
#define ADDR_AX_MOVING_SPEED           32
#define PROTOCOL_VERSION               1.0
#define BAUDRATE                       1000000
#define DEVICENAME                     "3"

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
uint8_t dxl_error = 0; // Declare dxl_error
int dxl_comm_result = COMM_TX_FAIL; // Declare dxl_comm_result

int angleToPosition(float angle) {
    return (int)(((angle + 150) / 300) * 1023); 
}

void setup() {
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
        return;
    }

    for (int id = 1; id <= 12; id++) {
        enableTorque(id);
        setMovingSpeed(id, 150, dxl_error, dxl_comm_result);
        moveMotor(id, angleToPosition(0)); //motors to full stnad position
    }

    delay(2000);

    moveMotor(3, angleToPosition(30));
    setMovingSpeed(4, 300, dxl_error, dxl_comm_result);
    moveMotor(4, angleToPosition( -60));
    moveMotor(5, angleToPosition( -30)); //moving all motors needed to squat position
    moveMotor(9, angleToPosition( -30));
    setMovingSpeed(10, 300, dxl_error, dxl_comm_result);
    moveMotor(10, angleToPosition(60));
    moveMotor(11, angleToPosition(30)); //moving all motors needed to squat position
    delay(2000);

    for (int id = 1; id <= 12; id++) {
        moveMotor(id, angleToPosition(0)); //returning motors to stand position
    }

    delay(2000);

    for (int id = 1; id <= 12; id++) {
        disableTorque(id);
    }

    portHandler->closePort();
}

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

void moveMotor(int id, int anglePosition) {
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_AX_GOAL_POSITION, anglePosition, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        // handle error
    }
}


void loop() {
    // Empty loop
}
