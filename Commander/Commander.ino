#include <DynamixelSDK.h>

//Define Settings for AX motors
#define ADDR_AX_TORQUE_ENABLE          24
#define ADDR_AX_GOAL_POSITION          30
#define ADDR_AX_MOVING_SPEED           32
#define ADDR_AX_PRESENT_POSITION       36  
#define PROTOCOL_VERSION               1.0
#define BAUDRATE                       1000000
#define DEVICENAME                     "3"

//Define Settings for XL motors
#define ADDR_XL_TORQUE_ENABLE          24
#define ADDR_XL_GOAL_POSITION          30
#define ADDR_XL_MOVING_SPEED           32  
#define ADDR_XL_PRESENT_POSITION       37
#define PROTOCOL_VERSION_XL            2.0
#define BAUDRATE_XL                    1000000
#define DEVICENAME_XL                  "1"

#define GenSpeed                       700

String in;

//Porting AX motors
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
uint8_t dxl_error = 0; // Variable to hold Dynamixel error codes
int dxl_comm_result = COMM_TX_FAIL; // Variable to hold the result of Dynamixel communication operations

//Porting XL motors
dynamixel::PortHandler *portHandler_XL;
dynamixel::PacketHandler *packetHandler_XL;
uint8_t dxl_error_XL = 0;
int dxl_comm_result_XL = COMM_TX_FAIL;


void enableTorque(int id);
void disableTorque(int id);
void setMovingSpeed(int id, int speed);
void moveMotor(int id, float angle, int time);
int angleToPosition(float angle, int id);
int getCurrentPosition(int id); // Function to read the current position of a motor

void enableTorque_XL(int id);
void disableTorque_XL(int id);
void setMovingSpeed_XL(int id, int speed);
void moveMotor_XL(int id, float angle, int time);
int angleToPosition_XL(float angle, int id);
int getCurrentPosition_XL(int id);

void setup() {
    Serial.begin(57600);
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
        Serial.println("Failed to open port or set baud rate");
        return;
    }

    portHandler_XL = dynamixel::PortHandler::getPortHandler(DEVICENAME_XL);
    packetHandler_XL = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION_XL);

    if (!portHandler_XL->openPort() || !portHandler_XL->setBaudRate(BAUDRATE_XL)) {
        Serial.println("Failed to open port or set baud rate");
        return;
    }

    for (int id = 1; id <= 12; id++) {
        enableTorque(id);
        moveMotor(id, 0, 200); // Initialize motors to stand position
    }
    delay(200);

    for (int id = 13; id <= 18; id++) {
        enableTorque_XL(id);
        moveMotor_XL(id, 0, 200);
    }
    delay(1000);


  
}


void loop() {
    if (Serial.available() > 0) { // Check if data is available to read
        in = Serial.readStringUntil('\n'); // Read the string until a newline character

        // Remove any whitespace or non-printable characters that might affect string comparison
        in.trim();

        // Process the command
        if (in == "Wave") {
            // Execute wave sequence
            wave();
        } else if (in == "Bow") {
            // Execute bow sequence
            bow();
        } else if (in == "Squat") {
            // Execute bow sequence
            squat();
        } else if (in == "Split") {
            // Execute bow sequence
            split();
        } else if (in == "Balance") {
            // Execute bow sequence
            balance();
        } else if (in == "Walk") {
            walk();
        }

        in = "";
    }

    // Delay slightly to prevent overloading the serial buffer with continuous commands
    delay(500);
    
    // Prompt user for input, only if there is no pending data in the buffer
    if (!Serial.available()) {
        Serial.println("Enter movement command:");
    }
}


//////////////////////////////////Start of Void Commands for AX//////////////////////////////////


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
    // Adjust angle for motors for robotics coding needs
    if (id >= 11 && id <= 12) {
        angle *= -1;
    }
    if (id >= 1 && id <= 1) {
        angle *= -1;
    }
    if (id >= 6 && id <= 7) {
        angle *= -1;
    }
    if (id >= 3 && id <= 4) {
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

//////////////////////////////////Start of Void Commands for XL//////////////////////////////////


void enableTorque_XL(int id) {
    packetHandler_XL->write1ByteTxRx(portHandler_XL, id, ADDR_XL_TORQUE_ENABLE, 1, &dxl_error_XL);
}

void disableTorque_XL(int id) {
    packetHandler_XL->write1ByteTxRx(portHandler_XL, id, ADDR_XL_TORQUE_ENABLE, 0, &dxl_error_XL);
}

void setMovingSpeed_XL(int id, int speed) {
    dxl_comm_result_XL = packetHandler_XL->write2ByteTxRx(portHandler_XL, id, ADDR_XL_MOVING_SPEED, speed, &dxl_error_XL);
    if (dxl_comm_result_XL != COMM_SUCCESS) {
        Serial.println("Failed to set moving speed!");
    }
}

void moveMotor_XL(int id, float angle, int time) {
    if (id >= 13 && id <= 15) {
        angle *= -1;
    }
    if (id >= 17 && id <= 18) {
        angle *= -1;
    }
    

    int currentPosition_XL = getCurrentPosition_XL(id);
    int goalPosition_XL = angleToPosition_XL(angle, id);

    if (time > 0) {
        int movementMagnitude_XL = abs(goalPosition_XL - currentPosition_XL);
        int speed_XL = (movementMagnitude_XL * 1000 / time) / 2.286;
        
        // Manually constrain the speed to be non-negative and no greater than 1023
        if(speed_XL < 0){
            speed_XL = -speed_XL;  // Make speed positive if negative
        }
        if(speed_XL > 1023){
            speed_XL = 1023;  // Constrain speed to a maximum of 1023
        }

        setMovingSpeed_XL(id, speed_XL);
    }

    packetHandler_XL->write2ByteTxRx(portHandler_XL, id, ADDR_XL_GOAL_POSITION, goalPosition_XL, &dxl_error_XL);
    if (dxl_comm_result_XL != COMM_SUCCESS) {
        Serial.println("Failed to move motor!");
    }
}

int angleToPosition_XL(float angle, int id) {
    return (int)(((angle + 150) / 300) * 1023);
}

int getCurrentPosition_XL(int id) {
    uint16_t dxl_present_position_XL;
    dxl_comm_result_XL = packetHandler_XL->read2ByteTxRx(portHandler_XL, id, ADDR_XL_PRESENT_POSITION, &dxl_present_position_XL, &dxl_error_XL);
    if (dxl_comm_result_XL == COMM_SUCCESS) {
        return dxl_present_position_XL;
    } else {
        Serial.println("Failed to read present position!");
        return -1;
    }
}

//////////////////////////////////Start of Preset Movement Commands//////////////////////////////////

void end(){
  for (int id = 1; id <= 12; id++) {
        enableTorque(id);
        moveMotor(id, 0, 200);
    }
    delay(200);

    for (int id = 13; id <= 18; id++) {
        enableTorque_XL(id);
        moveMotor_XL(id, 0, 200);
    }
    delay(1000);
}

void wave(){
    moveMotor_XL(  13, -150, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,    0, 400); // FP Right

    moveMotor_XL(  16,    0, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,    0, 400); // FP Left

    delay(500);

    moveMotor_XL(  13, -150, 300); // SR Right
    moveMotor_XL(  14,   10, 300); // SP Right
    moveMotor_XL(  15,   40, 300); // FP Right

    moveMotor_XL(  16,    0, 300); // SR Left
    moveMotor_XL(  17,    0, 300); // SP Left
    moveMotor_XL(  18,    0, 300); // FP Left

    delay(400);

    moveMotor_XL(  13, -150, 300); // SR Right
    moveMotor_XL(  14,   10, 300); // SP Right
    moveMotor_XL(  15,  -40, 300); // FP Right

    moveMotor_XL(  16,    0, 300); // SR Left
    moveMotor_XL(  17,    0, 300); // SP Left
    moveMotor_XL(  18,    0, 300); // FP Left

    delay(400);

    moveMotor_XL(  13, -150, 300); // SR Right
    moveMotor_XL(  14,   10, 300); // SP Right
    moveMotor_XL(  15,   40, 300); // FP Right

    moveMotor_XL(  16,    0, 300); // SR Left
    moveMotor_XL(  17,    0, 300); // SP Left
    moveMotor_XL(  18,    0, 300); // FP Left

    delay(400);

    moveMotor_XL(  13, -150, 300); // SR Right
    moveMotor_XL(  14,   10, 300); // SP Right
    moveMotor_XL(  15,  -40, 300); // FP Right

    moveMotor_XL(  16,    0, 300); // SR Left
    moveMotor_XL(  17,    0, 300); // SP Left
    moveMotor_XL(  18,    0, 300); // FP Left

    delay(400);

//other arm

    moveMotor_XL(  13,    0, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,    0, 400); // FP Right

    moveMotor_XL(  16, -150, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,    0, 400); // FP Left

    delay(500);

    moveMotor_XL(  13,    0, 300); // SR Right
    moveMotor_XL(  14,    0, 300); // SP Right
    moveMotor_XL(  15,    0, 300); // FP Right

    moveMotor_XL(  16, -150, 300); // SR Left
    moveMotor_XL(  17,   10, 300); // SP Left
    moveMotor_XL(  18,  -40, 300); // FP Left

    delay(400);

    moveMotor_XL(  13,    0, 300); // SR Right
    moveMotor_XL(  14,    0, 300); // SP Right
    moveMotor_XL(  15,    0, 300); // FP Right

    moveMotor_XL(  16, -150, 300); // SR Left
    moveMotor_XL(  17,   10, 300); // SP Left
    moveMotor_XL(  18,   40, 300); // FP Left

    delay(400);

    moveMotor_XL(  13,    0, 300); // SR Right
    moveMotor_XL(  14,    0, 300); // SP Right
    moveMotor_XL(  15,    0, 300); // FP Right

    moveMotor_XL(  16, -150, 300); // SR Left
    moveMotor_XL(  17,   10, 300); // SP Left
    moveMotor_XL(  18,  -40, 300); // FP Left

    delay(400);

    moveMotor_XL(  13,    0, 300); // SR Right
    moveMotor_XL(  14,    0, 300); // SP Right
    moveMotor_XL(  15,    0, 300); // FP Right

    moveMotor_XL(  16, -150, 300); // SR Left
    moveMotor_XL(  17,   10, 300); // SP Left
    moveMotor_XL(  18,   40, 300); // FP Left

    delay(400);

//Both Arms
    moveMotor_XL(  13, -150, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,    0, 400); // FP Right

    moveMotor_XL(  16, -150, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,    0, 400); // FP Left

    delay(500);

    moveMotor_XL(  13, -150, 300); // SR Right
    moveMotor_XL(  14,   10, 300); // SP Right
    moveMotor_XL(  15,   40, 300); // FP Right

    moveMotor_XL(  16, -150, 300); // SR Left
    moveMotor_XL(  17,   10, 300); // SP Left
    moveMotor_XL(  18,  -40, 300); // FP Left

    delay(400);

    moveMotor_XL(  13, -150, 300); // SR Right
    moveMotor_XL(  14,   10, 300); // SP Right
    moveMotor_XL(  15,  -40, 300); // FP Right

    moveMotor_XL(  16, -150, 300); // SR Left
    moveMotor_XL(  17,   10, 300); // SP Left
    moveMotor_XL(  18,   40, 300); // FP Left

    delay(400);

    moveMotor_XL(  13, -150, 300); // SR Right
    moveMotor_XL(  14,   10, 300); // SP Right
    moveMotor_XL(  15,   40, 300); // FP Right

    moveMotor_XL(  16, -150, 300); // SR Left
    moveMotor_XL(  17,   10, 300); // SP Left
    moveMotor_XL(  18,  -40, 300); // FP Left

    delay(400);

    moveMotor_XL(  13, -150, 300); // SR Right
    moveMotor_XL(  14,   10, 300); // SP Right
    moveMotor_XL(  15,  -40, 300); // FP Right

    moveMotor_XL(  16, -150, 300); // SR Left
    moveMotor_XL(  17,   10, 300); // SP Left
    moveMotor_XL(  18,   40, 300); // FP Left

    delay(400);

    end();
    delay(1000);

}

void bow(){
    moveMotor(  1,     0, 400); // HY Right
    moveMotor(  2,     0, 400); // HR Right
    moveMotor(  3,   -80, 400); // HP Right
    moveMotor(  4,    60, 400); // KP Right
    moveMotor(  5,     0, 400); // AP Right
    moveMotor(  6,     0, 400); // AR Right

    moveMotor(  7,     0, 400); // HY Left
    moveMotor(  8,     0, 400); // HR Left 
    moveMotor(  9,   -80, 400); // HP Left
    moveMotor( 10,    60, 400); // KP Left
    moveMotor( 11,     0, 400); // AP Left
    moveMotor( 12,     0, 400); // AR Left

    moveMotor_XL(  13,  -45, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,    0, 400); // FP Right

    moveMotor_XL(  16,   45, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,    0, 400); // FP Left

    delay(500);

    moveMotor(  1,     0, 400); // HY Right
    moveMotor(  2,     0, 400); // HR Right
    moveMotor(  3,   -80, 400); // HP Right
    moveMotor(  4,    60, 400); // KP Right
    moveMotor(  5,     0, 400); // AP Right
    moveMotor(  6,     0, 400); // AR Right

    moveMotor(  7,     0, 400); // HY Left
    moveMotor(  8,     0, 400); // HR Left 
    moveMotor(  9,   -80, 400); // HP Left
    moveMotor( 10,    60, 400); // KP Left
    moveMotor( 11,     0, 400); // AP Left
    moveMotor( 12,     0, 400); // AR Left

    moveMotor_XL(  13,  -45, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,   90, 400); // FP Right

    moveMotor_XL(  16,   45, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,   -90, 400); // FP Left

    delay(1000);

    moveMotor(  1,     0, 400); // HY Right
    moveMotor(  2,     0, 400); // HR Right
    moveMotor(  3,   -80, 400); // HP Right
    moveMotor(  4,    60, 400); // KP Right
    moveMotor(  5,     0, 400); // AP Right
    moveMotor(  6,     0, 400); // AR Right

    moveMotor(  7,     0, 400); // HY Left
    moveMotor(  8,     0, 400); // HR Left 
    moveMotor(  9,   -80, 400); // HP Left
    moveMotor( 10,    60, 400); // KP Left
    moveMotor( 11,     0, 400); // AP Left
    moveMotor( 12,     0, 400); // AR Left

    moveMotor_XL(  13,  -45, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,    0, 400); // FP Right

    moveMotor_XL(  16,   45, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,    0, 400); // FP Left

    delay(500);

    moveMotor(  1,     0, 400); // HY Right
    moveMotor(  2,     0, 400); // HR Right
    moveMotor(  3,   -80, 400); // HP Right
    moveMotor(  4,    60, 400); // KP Right
    moveMotor(  5,     0, 400); // AP Right
    moveMotor(  6,     0, 400); // AR Right

    moveMotor(  7,     0, 400); // HY Left
    moveMotor(  8,     0, 400); // HR Left 
    moveMotor(  9,   -80, 400); // HP Left
    moveMotor( 10,    60, 400); // KP Left
    moveMotor( 11,     0, 400); // AP Left
    moveMotor( 12,     0, 400); // AR Left

    moveMotor_XL(  13,   45, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,    0, 400); // FP Right

    moveMotor_XL(  16,  -45, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,    0, 400); // FP Left

    delay(100);


    moveMotor(  1,     0, 400); // HY Right
    moveMotor(  2,     0, 400); // HR Right
    moveMotor(  3,   -80, 400); // HP Right
    moveMotor(  4,    60, 400); // KP Right
    moveMotor(  5,     0, 400); // AP Right
    moveMotor(  6,     0, 400); // AR Right

    moveMotor(  7,     0, 400); // HY Left
    moveMotor(  8,     0, 400); // HR Left 
    moveMotor(  9,   -80, 400); // HP Left
    moveMotor( 10,    60, 400); // KP Left
    moveMotor( 11,     0, 400); // AP Left
    moveMotor( 12,     0, 400); // AR Left

    moveMotor_XL(  13,   45, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,   90, 400); // FP Right

    moveMotor_XL(  16,  -45, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,  -90, 400); // FP Left

    delay(1000);


    moveMotor(  1,     0, 400); // HY Right
    moveMotor(  2,     0, 400); // HR Right
    moveMotor(  3,   -80, 400); // HP Right
    moveMotor(  4,    60, 400); // KP Right
    moveMotor(  5,     0, 400); // AP Right
    moveMotor(  6,     0, 400); // AR Right

    moveMotor(  7,     0, 400); // HY Left
    moveMotor(  8,     0, 400); // HR Left 
    moveMotor(  9,   -80, 400); // HP Left
    moveMotor( 10,    60, 400); // KP Left
    moveMotor( 11,     0, 400); // AP Left
    moveMotor( 12,     0, 400); // AR Left

    moveMotor_XL(  13,   45, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,    0, 400); // FP Right

    moveMotor_XL(  16,  -45, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,    0, 400); // FP Left

    delay(500);

    end();
    delay(2000);
}

void split(){
    moveMotor(  1,     0, 400); // HY Right
    moveMotor(  2,     0, 400); // HR Right
    moveMotor(  3,   -40, 400); // HP Right
    moveMotor(  4,    80, 400); // KP Right
    moveMotor(  5,   -40, 400); // AP Right
    moveMotor(  6,     0, 400); // AR Right

    moveMotor(  7,     0, 400); // HY Left
    moveMotor(  8,     0, 400); // HR Left 
    moveMotor(  9,   -40, 400); // HP Left
    moveMotor( 10,    80, 400); // KP Left
    moveMotor( 11,   -40, 400); // AP Left
    moveMotor( 12,     0, 400); // AR Left

    moveMotor_XL(  13,  -90, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,    0, 400); // FP Right

    moveMotor_XL(  16,  -90, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,    0, 400); // FP Left

    delay(500);

    moveMotor(  1,   -15, 400); // HY Right
    moveMotor(  2,     0, 400); // HR Right
    moveMotor(  3,   -40, 400); // HP Right
    moveMotor(  4,    80, 400); // KP Right
    moveMotor(  5,   -40, 400); // AP Right
    moveMotor(  6,     0, 400); // AR Right

    moveMotor(  7,    15, 400); // HY Left
    moveMotor(  8,     0, 400); // HR Left 
    moveMotor(  9,   -40, 400); // HP Left
    moveMotor( 10,    80, 400); // KP Left
    moveMotor( 11,   -40, 400); // AP Left
    moveMotor( 12,     0, 400); // AR Left

    moveMotor_XL(  13,     0, 400); // SR Right
    moveMotor_XL(  14,   -90, 400); // SP Right
    moveMotor_XL(  15,     0, 400); // FP Right

    moveMotor_XL(  16,     0, 400); // SR Left
    moveMotor_XL(  17,    90, 400); // SP Left
    moveMotor_XL(  18,     0, 400); // FP Left

    delay(1000);

    moveMotor(  1,   -20, 800); // HY Right
    moveMotor(  2,   -90, 800); // HR Right
    moveMotor(  3,   -40, 800); // HP Right
    moveMotor(  4,    80, 800); // KP Right
    moveMotor(  5,   -40, 800); // AP Right
    moveMotor(  6,    90, 800); // AR Right

    moveMotor(  7,    20, 800); // HY Left
    moveMotor(  8,    90, 800); // HR Left 
    moveMotor(  9,   -40, 800); // HP Left
    moveMotor( 10,    80, 800); // KP Left
    moveMotor( 11,   -40, 800); // AP Left
    moveMotor( 12,   -90, 800); // AR Left

    moveMotor_XL(  13,  -150, 400); // SR Right
    moveMotor_XL(  14,   -90, 400); // SP Right
    moveMotor_XL(  15,    90, 400); // FP Right

    moveMotor_XL(  16,  -150, 400); // SR Left
    moveMotor_XL(  17,    90, 400); // SP Left
    moveMotor_XL(  18,   -90, 400); // FP Left

    delay(2000);

    moveMotor(  1,     0, 800); // HY Right
    moveMotor(  2,   -90, 800); // HR Right
    moveMotor(  3,     0, 800); // HP Right
    moveMotor(  4,     0, 800); // KP Right
    moveMotor(  5,     0, 800); // AP Right
    moveMotor(  6,    90, 800); // AR Right

    moveMotor(  7,     0, 800); // HY Left
    moveMotor(  8,    90, 800); // HR Left 
    moveMotor(  9,     0, 800); // HP Left
    moveMotor( 10,     0, 800); // KP Left
    moveMotor( 11,     0, 800); // AP Left
    moveMotor( 12,   -90, 800); // AR Left

    moveMotor_XL(  13,  -150, 400); // SR Right
    moveMotor_XL(  14,   -90, 400); // SP Right
    moveMotor_XL(  15,     0, 400); // FP Right

    moveMotor_XL(  16,  -150, 400); // SR Left
    moveMotor_XL(  17,    90, 400); // SP Left
    moveMotor_XL(  18,     0, 400); // FP Left

    delay(2000);


    moveMotor(  1,     0, 800); // HY Right
    moveMotor(  2,   -90, 800); // HR Right
    moveMotor(  3,   -40, 800); // HP Right
    moveMotor(  4,    80, 800); // KP Right
    moveMotor(  5,   -40, 800); // AP Right
    moveMotor(  6,    90, 800); // AR Right

    moveMotor(  7,     0, 800); // HY Left
    moveMotor(  8,    90, 800); // HR Left 
    moveMotor(  9,   -40, 800); // HP Left
    moveMotor( 10,    80, 800); // KP Left
    moveMotor( 11,   -40, 800); // AP Left
    moveMotor( 12,   -90, 800); // AR Left

    moveMotor_XL(  13,     0, 400); // SR Right
    moveMotor_XL(  14,   -90, 400); // SP Right
    moveMotor_XL(  15,     0, 400); // FP Right

    moveMotor_XL(  16,     0, 400); // SR Left
    moveMotor_XL(  17,    90, 400); // SP Left
    moveMotor_XL(  18,     0, 400); // FP Left

    delay(2000);


    moveMotor(  1,   -20, 1000); // HY Right
    moveMotor(  2,     0, 1000); // HR Right
    moveMotor(  3,   -40, 1000); // HP Right
    moveMotor(  4,    80, 1000); // KP Right
    moveMotor(  5,   -40, 1000); // AP Right
    moveMotor(  6,     0, 1000); // AR Right

    moveMotor(  7,    20, 1000); // HY Left
    moveMotor(  8,     0, 1000); // HR Left 
    moveMotor(  9,   -40, 1000); // HP Left
    moveMotor( 10,    80, 1000); // KP Left
    moveMotor( 11,   -40, 1000); // AP Left
    moveMotor( 12,     0, 1000); // AR Left

    moveMotor_XL(  13,     0, 400); // SR Right
    moveMotor_XL(  14,   -20, 400); // SP Right
    moveMotor_XL(  15,    20, 400); // FP Right

    moveMotor_XL(  16,     0, 400); // SR Left
    moveMotor_XL(  17,    20, 400); // SP Left
    moveMotor_XL(  18,    -20, 400); // FP Left

    delay(2000);


    moveMotor(  1,     0, 500); // HY Right
    moveMotor(  2,     0, 500); // HR Right
    moveMotor(  3,   -40, 500); // HP Right
    moveMotor(  4,    80, 500); // KP Right
    moveMotor(  5,   -40, 500); // AP Right
    moveMotor(  6,     0, 500); // AR Right

    moveMotor(  7,     0, 500); // HY Left
    moveMotor(  8,     0, 500); // HR Left 
    moveMotor(  9,   -40, 500); // HP Left
    moveMotor( 10,    80, 500); // KP Left
    moveMotor( 11,   -40, 500); // AP Left
    moveMotor( 12,     0, 500); // AR Left

    moveMotor_XL(  13,     0, 400); // SR Right
    moveMotor_XL(  14,   -20, 400); // SP Right
    moveMotor_XL(  15,     0, 400); // FP Right

    moveMotor_XL(  16,     0, 400); // SR Left
    moveMotor_XL(  17,    20, 400); // SP Left
    moveMotor_XL(  18,     0, 400); // FP Left
    delay(1000);

    moveMotor(  1,     0, 700); // HY Right
    moveMotor(  2,     0, 700); // HR Right
    moveMotor(  3,     5, 700); // HP Right
    moveMotor(  4,     0, 700); // KP Right
    moveMotor(  5,     0, 700); // AP Right
    moveMotor(  6,     0, 700); // AR Right

    moveMotor(  7,     0, 700); // HY Left
    moveMotor(  8,     0, 700); // HR Left 
    moveMotor(  9,     5, 700); // HP Left
    moveMotor( 10,     0, 700); // KP Left
    moveMotor( 11,     0, 700); // AP Left
    moveMotor( 12,     0, 700); // AR Left

    moveMotor_XL(  13,     0, 400); // SR Right
    moveMotor_XL(  14,    0, 400); // SP Right
    moveMotor_XL(  15,    0, 400); // FP Right

    moveMotor_XL(  16,     0, 400); // SR Left
    moveMotor_XL(  17,    0, 400); // SP Left
    moveMotor_XL(  18,    0, 400); // FP Left

    delay(100);

  end();
  delay(2000);
}

void squat(){
       moveMotor(  1,     0, 500); // HY Right
       moveMotor(  2,     0, 500); // HR Right
       moveMotor(  3,   -50, 500); // HP Right
       moveMotor(  4,   100, 500); // KP Right
       moveMotor(  5,   -50, 500); // AP Right
       moveMotor(  6,     0, 500); // AR Right

       moveMotor(  7,     0, 500); // HY Left
       moveMotor(  8,     0, 500); // HR Left 
       moveMotor(  9,   -50, 500); // HP Left
       moveMotor( 10,   100, 500); // KP Left
       moveMotor( 11,   -50, 500); // AP Left
       moveMotor( 12,     0, 500); // AR Left

    moveMotor_XL(  13,  -90, 500); // SR Right
    moveMotor_XL(  14,    0, 500); // SP Right
    moveMotor_XL(  15,    0, 500); // FP Right

    moveMotor_XL(  16,  -90, 500); // SR Left
    moveMotor_XL(  17,    0, 500); // SP Left
    moveMotor_XL(  18,    0, 500); // FP Left

    delay(1000);

       moveMotor(  1,     0, 600); // HY Right
       moveMotor(  2,     0, 600); // HR Right
       moveMotor(  3,     0, 600); // HP Right
       moveMotor(  4,     0, 600); // KP Right
       moveMotor(  5,     0, 600); // AP Right
       moveMotor(  6,     0, 600); // AR Right

       moveMotor(  7,     0, 600); // HY Left
       moveMotor(  8,     0, 600); // HR Left 
       moveMotor(  9,     0, 600); // HP Left
       moveMotor( 10,     0, 600); // KP Left
       moveMotor( 11,     0, 600); // AP Left
       moveMotor( 12,     0, 600); // AR Left

    moveMotor_XL(  13,    0, 600); // SR Right
    moveMotor_XL(  14,    0, 600); // SP Right
    moveMotor_XL(  15,    0, 600); // FP Right

    moveMotor_XL(  16,    0, 600); // SR Left
    moveMotor_XL(  17,    0, 600); // SP Left
    moveMotor_XL(  18,    0, 600); // FP Left

    delay(1000);

    end();
    delay(2000);
}

void balance(){
    moveMotor(  2,     0, 500); // HR Right
    moveMotor(  3,   -20, 500); // HP Right
    moveMotor(  4,    40, 500); // KP Right
    moveMotor(  5,   -20, 500); // AP Right
    moveMotor(  6,     0, 500); // AR Right

    moveMotor(  8,     0, 500); // HR Left 
    moveMotor(  9,   -20, 500); // HP Left
    moveMotor( 10,    40, 500); // KP Left
    moveMotor( 11,   -20, 500); // AP Left
    moveMotor( 12,     0, 500); // AR Left
    delay(2000);

    moveMotor(  2,    37, 500); // HR Right
    moveMotor(  3,   -20, 500); // HP Right
    moveMotor(  4,    40, 500); // KP Right
    moveMotor(  5,   -20, 500); // AP Right
    moveMotor(  6,   -37, 500); // AR Right

    moveMotor(  8,    40, 500); // HR Left 
    moveMotor(  9,   -50, 500); // HP Left
    moveMotor( 10,   100, 500); // KP Left
    moveMotor( 11,   -50, 500); // AP Left
    moveMotor( 12,   -40, 500); // AR Left
    delay(2000);

    moveMotor(  2,     0, 500); // HR Right
    moveMotor(  3,   -20, 500); // HP Right
    moveMotor(  4,    40, 500); // KP Right
    moveMotor(  5,   -20, 500); // AP Right
    moveMotor(  6,     0, 500); // AR Right

    moveMotor(  8,     0, 500); // HR Left 
    moveMotor(  9,   -20, 500); // HP Left
    moveMotor( 10,    40, 500); // KP Left
    moveMotor( 11,   -20, 500); // AP Left
    moveMotor( 12,     0, 500); // AR Left
    delay(2000);

    moveMotor(  2,   -40, 500); // HR Right
    moveMotor(  3,   -50, 500); // HP Right
    moveMotor(  4,   100, 500); // KP Right
    moveMotor(  5,   -50, 500); // AP Right
    moveMotor(  6,    40, 500); // AR Right

    moveMotor(  8,   -40, 500); // HR Left 
    moveMotor(  9,   -20, 500); // HP Left
    moveMotor( 10,    40, 500); // KP Left
    moveMotor( 11,   -20, 500); // AP Left
    moveMotor( 12,    40, 500); // AR Left
    delay(2000);


    moveMotor(  2,     0, 500); // HR Right
    moveMotor(  3,   -20, 500); // HP Right
    moveMotor(  4,    40, 500); // KP Right
    moveMotor(  5,   -20, 500); // AP Right
    moveMotor(  6,     0, 500); // AR Right

    moveMotor(  8,     0, 500); // HR Left 
    moveMotor(  9,   -20, 500); // HP Left
    moveMotor( 10,    40, 500); // KP Left
    moveMotor( 11,   -20, 500); // AP Left
    moveMotor( 12,     0, 500); // AR Left
    delay(2000);

    for (int id = 1; id <= 12; id++) {
        moveMotor(id, 0, 500); // Motors to stand position
    }
    delay(5000);
}

void walk(){
    moveMotor(  2,     0, 1000); // HR Right
    moveMotor(  3,   -20, 1000); // HP Right
    moveMotor(  4,    40, 1000); // KP Right
    moveMotor(  5,   -20, 1000); // AP Right
    moveMotor(  6,     0, 1000); // AR Right

    moveMotor(  8,     0, 1000); // HR Left 
    moveMotor(  9,   -20, 1000); // HP Left
    moveMotor( 10,    40, 1000); // KP Left
    moveMotor( 11,   -20, 1000); // AP Left
    moveMotor( 12,     0, 1000); // AR Left
    delay(2000);

    moveMotor(  1,     0, 700); // HY Right
    moveMotor(  2,    35, 700); // HR Right
    moveMotor(  3,   -35, 700); // HP Right
    moveMotor(  4,    40, 700); // KP Right
    moveMotor(  5,   -20, 700); // AP Right
    moveMotor(  6,   -37, 700); // AR Right

    moveMotor(  7,     0, 700); // HY Left
    moveMotor(  8,    55, 700); // HR Left 
    moveMotor(  9,   -65, 700); // HP Left
    moveMotor( 10,   100, 700); // KP Left
    moveMotor( 11,   -50, 700); // AP Left
    moveMotor( 12,   -55, 700); // AR Left
    delay(700);

    // Move leg forwards
    moveMotor(  1,     0, 700); // HY Right
    moveMotor(  2,    35, 700); // HR Right
    moveMotor(  3,   -35, 700); // HP Right
    moveMotor(  4,    40, 700); // KP Right
    moveMotor(  5,   -20, 700); // AP Right
    moveMotor(  6,   -37, 700); // AR Right

    moveMotor(  7,     0, 700); // HY Left
    moveMotor(  8,    40, 700); // HR Left 
    moveMotor(  9,   -75, 700); // HP Left
    moveMotor( 10,    75, 700); // KP Left
    moveMotor( 11,   -20, 700); // AP Left
    moveMotor( 12,   -40, 700); // AR Left

    delay(700);
    //Place Plant foot and push off back leg
    moveMotor(  1,     0, 700); // HY Right
    moveMotor(  2,   -35, 700); // HR Right
    moveMotor(  3,    65, 700); // HP Right
    moveMotor(  4,    60, 700); // KP Right
    moveMotor(  5,   -70, 700); // AP Right
    moveMotor(  6,    20, 700); // AR Right
    
    moveMotor(  7,     0, 700); // HY Left
    moveMotor(  8,   -35, 700); // HR Left
    moveMotor(  9,   -65, 700); // AP Left
    moveMotor( 10,    75, 700); // AP Left
    moveMotor( 11,   -20, 700); // AP Left
    moveMotor( 12,    35, 700); // AR Left
    
    delay(700);

    moveMotor(  1,     0, 700); // HY Right
    moveMotor(  2,   -55, 700); // HR Left
    moveMotor(  3,   -65, 700); // HP Left
    moveMotor(  4,   100, 700); // KP Left
    moveMotor(  5,   -50, 700); // AP Left
    moveMotor(  6,    55, 700); // AR Left
    
    moveMotor(  7,     0, 700); // HY Left
    moveMotor(  8,   -35, 700); // HR Left
    moveMotor(  9,   -35, 700); // HP Left
    moveMotor(  10,   40, 700); // KP Left
    moveMotor(  11,  -20, 700); // AP Left
    moveMotor(  12,   35, 700); // AR Left
    delay(700);

    moveMotor(  1,     0, 700); // HY Right
    moveMotor(  2,   -40, 700); // HR Left 
    moveMotor(  3,   -75, 700); // HP Left
    moveMotor(  4,    75, 700); // KP Left
    moveMotor(  5,   -20, 700); // AP Left
    moveMotor(  6,    40, 700); // AR Left

    moveMotor(  7,     0, 700); // HY Left
    moveMotor(  8,   -35, 700); // HR Left
    moveMotor(  9,   -35, 700); // HP Left
    moveMotor(  10,   40, 700); // KP Left
    moveMotor(  11,  -20, 700); // AP Left
    moveMotor(  12,   35, 700); // AR Left
    delay(700);

    moveMotor(  1,     0, 700); // HY Right
    moveMotor(  2,    35, 700); // HR Left
    moveMotor(  3,   -65, 700); // AP Left
    moveMotor(  4,    75, 700); // KP Left
    moveMotor(  5,   -20, 700); // AP Left
    moveMotor(  6,   -35, 700); // AR Left

    moveMotor(  7,     0, 700); // HY Left
    moveMotor(  8,    35, 700); // HR Right
    moveMotor(  9,    65, 700); // HP Right
    moveMotor(  10,   60, 700); // KP Right
    moveMotor(  11,  -70, 700); // AP Right
    moveMotor(  12,  -35, 700); // AR Right

    delay(700);

    moveMotor(  2,     0, 1000); // HR Right
    moveMotor(  3,   -20, 1000); // HP Right
    moveMotor(  4,    40, 1000); // KP Right
    moveMotor(  5,   -20, 1000); // AP Right
    moveMotor(  6,     0, 1000); // AR Right

    moveMotor(  8,     0, 1000); // HR Left 
    moveMotor(  9,   -20, 1000); // HP Left
    moveMotor( 10,    40, 1000); // KP Left
    moveMotor( 11,   -20, 1000); // AP Left
    moveMotor( 12,     0, 1000); // AR Left
    delay(2000);

    end();
    delay(2000);

}