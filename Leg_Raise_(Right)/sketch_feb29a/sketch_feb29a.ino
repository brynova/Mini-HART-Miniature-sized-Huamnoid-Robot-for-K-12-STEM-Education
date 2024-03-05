#include <DynamixelSDK.h>

// Control table addresses
#define ADDR_AX_TORQUE_ENABLE          24
#define ADDR_AX_GOAL_POSITION          30
#define ADDR_AX_PRESENT_POSITION       36

// Protocol version
#define PROTOCOL_VERSION               1.0

// Default settings
#define DXL_ID_7                       9    // ID of the first motor
#define DXL_ID_8                       10    // ID of the second motor
#define DXL_ID_9                       11   // ID of the third motor
#define DXL_ID_10                      12    // ID of the fourth motor
#define DXL_ID_11                      13   // ID of the fith motor
#define DXL_ID_12                      14   // ID of the sixth motor

#define BAUDRATE                       1000000
#define DEVICENAME                     "3"  // Use Serial3 for OpenCM 485 EXP
#define TORQUE_ENABLE                  1
#define TORQUE_DISABLE                 0
// Updated to use angle for initial and goal positions
//#define DXL_INITIAL_ANGLE              150 // Initial angle
//#define DXL_GOAL_ANGLE                 240  // New target angle for both motors

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// Function to convert angle to position
int angleToPosition(float angle) {
  return (int)(angle / 300 * 1023);
}

void setup() {
  Serial.begin(115200);
  while(!Serial); // Wait for Serial port to connect

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (!portHandler->openPort()) {
    Serial.println("Failed to open the port!");
    return;
  }

  // Set port baudrate
  if (!portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("Failed to set the baudrate!");
    return;
  }

  // Enable Dynamixel Torque for both motors
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Enable torque for both motors
  enableTorque(DXL_ID_7, dxl_error, dxl_comm_result);
  enableTorque(DXL_ID_8, dxl_error, dxl_comm_result);
  enableTorque(DXL_ID_9, dxl_error, dxl_comm_result);
  enableTorque(DXL_ID_10, dxl_error, dxl_comm_result);
  enableTorque(DXL_ID_11, dxl_error, dxl_comm_result);
  enableTorque(DXL_ID_12, dxl_error, dxl_comm_result);

  // Convert angles to positions and move motors
  moveMotor(DXL_ID_7, angleToPosition(150), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_8, angleToPosition(150), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_9, angleToPosition(150), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_10, angleToPosition(150), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_11, angleToPosition(150), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_12, angleToPosition(150), dxl_error, dxl_comm_result);
  delay(2000); // Wait for motors to reach the initial position

  // Move both motors to new goal position
  moveMotor(DXL_ID_7, angleToPosition(240), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_8, angleToPosition(60), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_9, angleToPosition(60), dxl_error, dxl_comm_result);//need to modify the angle
  moveMotor(DXL_ID_10, angleToPosition(60), dxl_error, dxl_comm_result);//need to modify the angle
  moveMotor(DXL_ID_11, angleToPosition(60), dxl_error, dxl_comm_result);//need to modify the angle
  moveMotor(DXL_ID_12, angleToPosition(60), dxl_error, dxl_comm_result);//need to modify the angle


  delay(2000); // Wait for motors to reach the new goal position

  // Move both motors back to the initial angle
  moveMotor(DXL_ID_7, angleToPosition(150), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_8, angleToPosition(150), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_9, angleToPosition(150), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_10, angleToPosition(150), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_11, angleToPosition(150), dxl_error, dxl_comm_result);
  moveMotor(DXL_ID_12, angleToPosition(150), dxl_error, dxl_comm_result);
  delay(2000); // Wait for motors to move back to the initial position

  // Disable Dynamixel Torque for both motors
  disableTorque(DXL_ID_7, dxl_error, dxl_comm_result);
  disableTorque(DXL_ID_8, dxl_error, dxl_comm_result);
  disableTorque(DXL_ID_9, dxl_error, dxl_comm_result);
  disableTorque(DXL_ID_10, dxl_error, dxl_comm_result);
  disableTorque(DXL_ID_11, dxl_error, dxl_comm_result);
  disableTorque(DXL_ID_12, dxl_error, dxl_comm_result);
  
  // Close por
  portHandler->closePort();
}

void enableTorque(int id, uint8_t &dxl_error, int &dxl_comm_result) {
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.println("Failed to enable torque!");
  }
}

void disableTorque(int id, uint8_t &dxl_error, int &dxl_comm_result) {
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.println("Failed to disable torque!");
  }
}

void moveMotor(int id, int anglePosition, uint8_t &dxl_error, int &dxl_comm_result) {
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_AX_GOAL_POSITION, anglePosition, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.println("Failed to move motor!");
  }
}

void loop() {
  // Empty loop
}
