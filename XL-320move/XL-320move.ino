#include <DynamixelSDK.h>

#define ADDR_XL_TORQUE_ENABLE          24
#define ADDR_XL_GOAL_POSITION          30
#define ADDR_XL_MOVING_SPEED           32  // XL-320 supports simple speed control
#define ADDR_XL_PRESENT_POSITION       37
#define PROTOCOL_VERSION               2.0
#define BAUDRATE                       1000000
#define DEVICENAME                     "1"

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;

void enableTorque(int id);
void disableTorque(int id);
void setMovingSpeed(int id, int speed);
void moveMotor(int id, float angle, int time);
int angleToPosition(float angle, int id);
int getCurrentPosition(int id);

void setup() {
    Serial.begin(57600);
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
        Serial.println("Failed to open port or set baud rate");
        return;
    }

    for (int id = 1; id <= 18; id++) {
        enableTorque(id);
        moveMotor(id, 0, 200);
    }
    delay(4000);
    
    moveMotor(  13,  30,  400);
    moveMotor(  16,  30,  400);
    delay(4000);
    
    // Example: Move motors to various positions as needed
    for (int id = 1; id <= 18; id++) {
        moveMotor(id, 0, 200);
    }
    delay(1000);

    // Disable torque for all motors at the end
    for (int id = 1; id <= 18; id++) {
        disableTorque(id);
    }

    portHandler->closePort();
}

void enableTorque(int id) {
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_XL_TORQUE_ENABLE, 1, &dxl_error);
}

void disableTorque(int id) {
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_XL_TORQUE_ENABLE, 0, &dxl_error);
}

void setMovingSpeed(int id, int speed) {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_XL_MOVING_SPEED, speed, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        Serial.println("Failed to set moving speed!");
    }
}

void moveMotor(int id, float angle, int time) {
    int currentPosition = getCurrentPosition(id);
    int goalPosition = angleToPosition(angle, id);

    if (time > 0) {
        int movementMagnitude = abs(goalPosition - currentPosition);
        int speed = (movementMagnitude * 1000 / time) / 2.286;
        
        // Manually constrain the speed to be non-negative and no greater than 1023
        if(speed < 0){
            speed = -speed;  // Make speed positive if negative
        }
        if(speed > 1023){
            speed = 1023;  // Constrain speed to a maximum of 1023
        }

        setMovingSpeed(id, speed);
    }

    packetHandler->write2ByteTxRx(portHandler, id, ADDR_XL_GOAL_POSITION, goalPosition, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        Serial.println("Failed to move motor!");
    }
}

int angleToPosition(float angle, int id) {
    return (int)(((angle + 150) / 300) * 1023);  // Adjust as per XL-320 specifics, if different
}

int getCurrentPosition(int id) {
    uint16_t dxl_present_position;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_XL_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        return dxl_present_position;
    } else {
        Serial.println("Failed to read present position!");
        return -1;
    }
}

void loop() {
    // Empty - Actions are taken in setup
}
