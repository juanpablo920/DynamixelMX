#define ESC_ASCII_VALUE                 0x1b


void setup() {
  Serial.begin(115200);
  while(!Serial);

  String input_strin_tmp;
  
  Serial.println("Start");

  Serial.println("input_tmp:");
  while( Serial.available() == 0){ 
    }
  input_strin_tmp = Serial.readString();  
  Serial.println("Succeeded to open the port");
  return;

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position

  // Open port
  if ( portHandler->openPort() ) {
    Serial.println("Succeeded to open the port");
  }
  else {
    Serial.println("Failed to open the port");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)){
    Serial.println("Succeeded to change the baudrate!\\n");
  }
  else{
    Serial.println("Failed to change the baudrate!\\n");
    return;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  else
  {
    Serial.println("Dynamixel has been successfully connected \\n");
  }
  
  Serial.println("Press any key to continue! (or press q to quit!)\\n");

  int input_tmp;
  int goal_position_tmp = 20;
  while(1)
  {
    if ( Serial.available() ){
      
      input_tmp = Serial.read();
      Serial.println(" ");
      Serial.print("input_tmp:"); Serial.print(input_tmp);
      Serial.println(" ");
      if (input_tmp == 'q'){
        break;
      }
      
      input_tmp = input_tmp - '0';
      
      Serial.println(" ");
      Serial.print("input_tmp:"); Serial.print(input_tmp);
      Serial.println(" ");

      if (input_tmp == 0){
        goal_position_tmp = 0;
      }
      if (input_tmp == 1){
        goal_position_tmp = 810;// 1024
      }
      if (input_tmp == 2){
        goal_position_tmp = 1834;// 2048
      }
      if (input_tmp == 3){
        goal_position_tmp = 2858;// 3072
      }
      if (input_tmp == 4){
        goal_position_tmp = 3882;// 4095
      }

      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }
      Serial.println("---");
      Serial.print("ID:");      Serial.print(DXL_ID);
      Serial.print("GoalPos:"); Serial.print(dxl_goal_position[index]);
      Serial.print("PresPos:"); Serial.println(dxl_present_position);
      Serial.println("---");

       // Write goal position
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION,goal_position_tmp, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS){
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0){
        packetHandler->getRxPacketError(dxl_error);
      }

      delay(500);

      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }
      Serial.println("---");
      Serial.print("ID:");      Serial.println(DXL_ID);
      Serial.print("GoalPos:"); Serial.println(dxl_goal_position[index]);
      Serial.print("PresPos:"); Serial.println(dxl_present_position);
      Serial.println("---");
     
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