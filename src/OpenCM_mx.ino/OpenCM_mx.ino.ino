
#include <DynamixelSDK.h>

// Control table address (MX-series with Protocol 2.0)
#define ADDR_MX_TORQUE_ENABLE          64             // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION          116
#define ADDR_MX_PRESENT_POSITION       132

// Protocol version
#define PROTOCOL_VERSION                2.0           // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1             // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "3"           // DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
// DEVICENAME "2" -> Serial2
// DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1             // Value for enabling the torque
#define TORQUE_DISABLE                  0             // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000          // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20            // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

void waiting_master() {
  while (Serial.available() == 0);
  String msg;
  msg = Serial.readString();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL; // Communication result
  uint8_t dxl_error = 0;              // Dynamixel error

  // Open port
  if ( portHandler->openPort() ) {
    Serial.println("succeeded_open_port");
  }
  else {
    Serial.println("failed_open_port");
    return;
  }

  waiting_master();

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("succeeded_change_baudrate");
  }
  else {
    Serial.println("failed_change_baudrate");
    return;
  }

  waiting_master();

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packetHandler->getTxRxResult(dxl_comm_result);
    Serial.println("dynamixel_failed_connected");
  }
  else if (dxl_error != 0) {
    packetHandler->getRxPacketError(dxl_error);
    Serial.println("dynamixel_failed_connected");
  }
  else {
    Serial.println("dynamixel_successfully_connected");
  }

  String input_tmp;
  int goal_posse = 0;
  int32_t present_pose = 0;

  while (1) {

    if ( Serial.available() ) {

      input_tmp = Serial.readString();
      input_tmp.replace("\n", "");

      if (input_tmp == "get_present_pose") {

        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&present_pose, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
          packetHandler->getTxRxResult(dxl_comm_result);
          Serial.println("failed_get_present_pose");
          break;
        }
        else if (dxl_error != 0) {
          packetHandler->getRxPacketError(dxl_error);
          Serial.print("failed_get_present_pose:");
          break;
        }
        Serial.println(present_pose);
      }

      if (input_tmp == "set_preset_pose") {
        Serial.println("");
        
        while (Serial.available() == 0);
        input_tmp = Serial.readString();
        goal_posse = input_tmp.toInt();

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, goal_posse, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
          packetHandler->getTxRxResult(dxl_comm_result);
          Serial.println("failed_set_preset_pose");
          break;
        }
        else if (dxl_error != 0) {
          packetHandler->getRxPacketError(dxl_error);
          Serial.println("failed_set_preset_pose");
          break;
        }
        Serial.println("succeeded_set_preset_pose");
      }

      if (input_tmp == "exit") {
        Serial.println("succeeded_exit");
        break;
      }

    }
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // Close port
  portHandler->closePort();

}

void loop() {
  // put your main code here, to run repeatedly:

}
