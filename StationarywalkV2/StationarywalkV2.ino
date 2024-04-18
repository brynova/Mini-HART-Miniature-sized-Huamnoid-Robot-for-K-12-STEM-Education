#include <DynamixelSDK.h>

#define ADDR_AX_TORQUE_ENABLE          24
#define ADDR_AX_GOAL_POSITION          30
#define ADDR_AX_MOVING_SPEED           32
#define ADDR_AX_PRESENT_POSITION       36  // Address to read the current position
#define PROTOCOL_VERSION               1.0
#define BAUDRATE                       1000000
#define DEVICENAME                     "3"

#define GenSpeed                       500

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
uint8_t dxl_error = 0; // Variable to hold Dynamixel error codes
int dxl_comm_result = COMM_TX_FAIL; // Variable to hold the result of Dynamixel communication operations

void enableTorque(int id);
void disableTorque(int id);
void setMovingSpeed(int id, int speed);
void moveMotor(int id, float angle, int time);
int angleToPosition(float angle, int id);
int getCurrentPosition(int id); // Function to read the current position of a motor

void setup() {
    Serial.begin(57600);
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
        Serial.println("Failed to open port or set baud rate");
        return;
    }

    for (int id = 1; id <= 12; id++) {
        enableTorque(id);
        moveMotor(id, 0, GenSpeed); // Initialize motors to stand position
    }
    delay(1000);

     // Begin stationary walk movement
    // Step 1: Squat Down - Aligning HP and AP
    moveMotor(  2,     0, GenSpeed); // HR Right
    moveMotor(  3,    20, GenSpeed); // HP Right
    moveMotor(  4,   -40, GenSpeed); // KP Right
    moveMotor(  5,   -20, GenSpeed); // AP Right
    moveMotor(  6,     0, GenSpeed); // AR Right

    moveMotor(  8,     0, GenSpeed); // HR Left 
    moveMotor(  9,    20, GenSpeed); // HP Left
    moveMotor( 10,   -40, GenSpeed); // KP Left
    moveMotor( 11,   -20, GenSpeed); // AP Left
    moveMotor( 12,     0, GenSpeed); // AR Left
    delay(2000);

    // Step 2: Rotate Right Roll motors to shift COB to right leg, Raise left left and twiat right hip as well
    moveMotor(  2,    37, GenSpeed); // HR Right
    moveMotor(  3,    20, GenSpeed); // HP Right
    moveMotor(  4,   -40, GenSpeed); // KP Right
    moveMotor(  5,   -20, GenSpeed); // AP Right
    moveMotor(  6,    37, GenSpeed); // AR Right

    moveMotor(  8,   -40, GenSpeed); // HR Left 
    moveMotor(  9,    50, GenSpeed); // HP Left
    moveMotor( 10,  -100, GenSpeed); // KP Left
    moveMotor( 11,   -50, GenSpeed); // AP Left
    moveMotor( 12,   -40, GenSpeed); // AR Left
    delay(2000);

    // Step 3: Return to Squat
    moveMotor(  2,     0, GenSpeed); // HR Right
    moveMotor(  3,    20, GenSpeed); // HP Right
    moveMotor(  4,   -40, GenSpeed); // KP Right
    moveMotor(  5,   -20, GenSpeed); // AP Right
    moveMotor(  6,     0, GenSpeed); // AR Right

    moveMotor(  8,     0, GenSpeed); // HR Left 
    moveMotor(  9,    20, GenSpeed); // HP Left
    moveMotor( 10,   -40, GenSpeed); // KP Left
    moveMotor( 11,   -20, GenSpeed); // AP Left
    moveMotor( 12,     0, GenSpeed); // AR Left
    delay(2000);

    // Step 4: Rotate Left Roll motors to shift COB to left leg, Raise Right leg and twist right hip as well
    moveMotor(  2,   -40, GenSpeed); // HR Right
    moveMotor(  3,    50, GenSpeed); // HP Right
    moveMotor(  4,  -100, GenSpeed); // KP Right
    moveMotor(  5,   -50, GenSpeed); // AP Right
    moveMotor(  6,   -40, GenSpeed); // AR Right

    moveMotor(  8,    40, GenSpeed); // HR Left 
    moveMotor(  9,    20, GenSpeed); // HP Left
    moveMotor( 10,   -40, GenSpeed); // KP Left
    moveMotor( 11,   -20, GenSpeed); // AP Left
    moveMotor( 12,    40, GenSpeed); // AR Left
    delay(2000);


    moveMotor(  2,     0, GenSpeed); // HR Right
    moveMotor(  3,    20, GenSpeed); // HP Right
    moveMotor(  4,   -40, GenSpeed); // KP Right
    moveMotor(  5,   -20, GenSpeed); // AP Right
    moveMotor(  6,     0, GenSpeed); // AR Right

    moveMotor(  8,     0, GenSpeed); // HR Left 
    moveMotor(  9,    20, GenSpeed); // HP Left
    moveMotor( 10,   -40, GenSpeed); // KP Left
    moveMotor( 11,   -20, GenSpeed); // AP Left
    moveMotor( 12,     0, GenSpeed); // AR Left
    delay(2000);


    for (int id = 1; id <= 12; id++) {
        moveMotor(id, 0, GenSpeed); // Initialize motors to stand position
    }
    delay(5000);
    
    for (int id = 1; id <= 12; id++) {
        disableTorque(id);
    }

    portHandler->closePort();
}

void enableTorque(int id) {
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_AX_TORQUE_ENABLE, 1, &dxl_error);
}

void disableTorque(int id) {
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_AX_TORQUE_ENABLE, 0, &dxl_error);
}

void setMovingSpeed(int id, int speed) {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_AX_MOVING_SPEED, speed, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        Serial.println("Failed to set moving speed!");
    }
}

void moveMotor(int id, float angle, int time) {
    // Adjust angle for motors on one side, if necessary
    if (id >= 7 && id <= 12) {
        angle *= -1;
    }
    
    // Get current and goal positions
    int currentPosition = getCurrentPosition(id);
    int goalPosition = angleToPosition(angle, id);

    // Calculate and set speed if time is valid
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

        setMovingSpeed(id, speed); // Set motor speed
    }

    // Move motor to goal position
    packetHandler->write2ByteTxRx(portHandler, id, ADDR_AX_GOAL_POSITION, goalPosition, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        Serial.println("Failed to move motor!");
    }
}


int angleToPosition(float angle, int id) {
    return (int)(((angle + 150) / 300) * 1023);
}

// Function to read the current position of a motor
int getCurrentPosition(int id) {
    uint16_t dxl_present_position;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_AX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        return dxl_present_position;
    } else {
        Serial.println("Failed to read present position!");
        return -1; // Handle error appropriately
    }
}

void loop() {
    // Empty - Actions are taken in setup
}