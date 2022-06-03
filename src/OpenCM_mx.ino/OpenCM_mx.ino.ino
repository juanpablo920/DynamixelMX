
#include <DynamixelSDK.h>

// Control table address (MX-series with Protocol 2.0)
#define ADDR_MX_TORQUE_ENABLE           64             // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           116
#define ADDR_MX_PRESENT_POSITION        132

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
  int ch;
  ch = Serial.read();
}

void open_port(dynamixel::PortHandler *portHandler) {

  if ( portHandler->openPort() ) {
    Serial.println("succeeded_open_port");
  }
  else {
    Serial.println("failed_open_port");
    exit(0);
  }

  waiting_master();
}

void set_port_baudrate(dynamixel::PortHandler *portHandler) {

  if (portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("succeeded_change_baudrate");
  }
  else {
    portHandler->closePort();
    Serial.println("failed_change_baudrate");
    exit(0);
  }

  waiting_master();
}

void enable_Torque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {

  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;

  comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &error);

  if (comm_result != COMM_SUCCESS) {
    packetHandler->getTxRxResult(comm_result);
    portHandler->closePort();
    Serial.println("dynamixel_failed_connected");
    exit(0);
  }
  else if (error != 0) {
    packetHandler->getRxPacketError(error);
    portHandler->closePort();
    Serial.println("dynamixel_failed_connected");
    exit(0);
  }
  else {
    Serial.println("dynamixel_successfully_connected");
  }
}

void disable_Torque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {

  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;

  comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &error);

  if (comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(comm_result);
  }
  else if (error != 0)
  {
    packetHandler->getRxPacketError(error);
  }
}

int32_t get_pose(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {

  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  int32_t present_pose = 0;

  comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&present_pose, &error);

  if (comm_result != COMM_SUCCESS) {
    packetHandler->getTxRxResult(comm_result);
    return -1;
  }
  else if (error != 0) {
    packetHandler->getRxPacketError(error);
    return -1;
  }
  return present_pose;
}

int set_pose(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int goal_posse) {

  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  int goal_posse_x = 1000;

  if (goal_posse <= 1000) {
    goal_posse_x = 1000;
  }
  else if (goal_posse >= 2500) {
    goal_posse_x = 2500;
  }
  else {
    goal_posse_x = goal_posse;
  }

  comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, goal_posse_x, &error);

  if (comm_result != COMM_SUCCESS) {
    packetHandler->getTxRxResult(comm_result);
    return -1;
  }
  else if (error != 0) {
    packetHandler->getRxPacketError(error);
    return -1;
  }
  return 1;
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  open_port(portHandler);

  set_port_baudrate(portHandler);

  enable_Torque(portHandler, packetHandler);

  int goal_posse = 0;
  int32_t present_pose = 0;

  String input_tmp;
  String sub_input_tmp;

  while (1) {

    if ( Serial.available() ) {

      input_tmp = Serial.readString();
      input_tmp.replace("\n", "");
      sub_input_tmp = getValue(input_tmp, ':', 0);

      if (sub_input_tmp == "get_pose") {
        present_pose = get_pose(portHandler, packetHandler);
        if (present_pose == -1) {
          disable_Torque(portHandler, packetHandler);
          portHandler->closePort();
          Serial.println("failed_get_pose");
          exit(0);
        }
        else {
          Serial.println(present_pose);
        }
      }

      if (sub_input_tmp == "set_pose") {
        sub_input_tmp = getValue(input_tmp, ':', 1);
        goal_posse = sub_input_tmp.toInt();
        int state_set_pose = set_pose(portHandler, packetHandler, goal_posse);

        if (state_set_pose == -1) {
          disable_Torque(portHandler, packetHandler);
          portHandler->closePort();
          Serial.println("failed_set_pose");
          exit(0);
        }
        else {
          Serial.println("succeeded_set_pose");
        }
      }

      if (sub_input_tmp == "go_origin") {

        int origin;
        int step_org;
        int delta_error;
        int delta_poses;
        int new_pos;

        sub_input_tmp = getValue(input_tmp, ':', 1);
        origin = sub_input_tmp.toInt();

        sub_input_tmp = getValue(input_tmp, ':', 2);
        step_org = sub_input_tmp.toInt();

        sub_input_tmp = getValue(input_tmp, ':', 3);
        delta_error = sub_input_tmp.toInt();

        present_pose = get_pose(portHandler, packetHandler);

        if (present_pose == -1) {
          disable_Torque(portHandler, packetHandler);
          portHandler->closePort();
          Serial.println("failed_go_origin");
          exit(0);
        }

        delta_poses = abs(present_pose - origin);

        while (delta_poses > delta_error) {

          if ( delta_poses < step_org) {
            step_org = delta_poses;
          }

          if (present_pose > origin) {
            new_pos = present_pose - step_org;
          }
          else {
            new_pos = present_pose + step_org;
          }

          int state_set_pose = set_pose(portHandler, packetHandler, new_pos);

          if (state_set_pose == -1) {
            disable_Torque(portHandler, packetHandler);
            portHandler->closePort();
            Serial.println("failed_go_origin");
            exit(0);
          }

          present_pose = get_pose(portHandler, packetHandler);

          if (present_pose == -1) {
            disable_Torque(portHandler, packetHandler);
            portHandler->closePort();
            Serial.println("failed_go_origin");
            exit(0);
          }

          delay(50);
          delta_poses = abs(present_pose - origin);
        }
        Serial.println("arrived_origin");
      }

      if (sub_input_tmp == "go_scan") {

        int fin;
        int step_scan;
        int delta_error;
        int delta_poses;
        int new_pos;

        sub_input_tmp = getValue(input_tmp, ':', 1);
        fin = sub_input_tmp.toInt();

        sub_input_tmp = getValue(input_tmp, ':', 2);
        step_scan = sub_input_tmp.toInt();

        sub_input_tmp = getValue(input_tmp, ':', 3);
        delta_error = sub_input_tmp.toInt();

        present_pose = get_pose(portHandler, packetHandler);

        if (present_pose == -1) {
          disable_Torque(portHandler, packetHandler);
          portHandler->closePort();
          Serial.println("failed_go_scan");
          exit(0);
        }

        Serial.println(present_pose);
        waiting_master();

        delta_poses = abs(present_pose - fin);

        while (delta_poses > delta_error) {

          if ( delta_poses < step_scan) {
            step_scan = delta_poses;
          }

          if (present_pose > fin) {
            new_pos = present_pose - step_scan;
          }
          else {
            new_pos = present_pose + step_scan;
          }

          int state_set_pose = set_pose(portHandler, packetHandler, new_pos);

          if (state_set_pose == -1) {
            disable_Torque(portHandler, packetHandler);
            portHandler->closePort();
            Serial.println("failed_go_scan");
            exit(0);
          }

          delay(150);

          present_pose = get_pose(portHandler, packetHandler);

          if (present_pose == -1) {
            disable_Torque(portHandler, packetHandler);
            portHandler->closePort();
            Serial.println("failed_go_scan");
            exit(0);
          }

          Serial.println(present_pose);
          waiting_master();

          delta_poses = abs(present_pose - fin);
        }
        Serial.println("arrived_scan");
      }

      if (sub_input_tmp == "exit") {
        disable_Torque(portHandler, packetHandler);
        portHandler->closePort();
        Serial.println("succeeded_exit");
        exit(0);
      }
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
