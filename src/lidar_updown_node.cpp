#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <math.h>

#define PROTOCOL_VERSION                2.0
#define DXL_ID                          1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"

#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_TORQUE_ENABLE          64
#define ADDR_PRO_PRESENT_POSITION       132

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0


#define PI 3.1415
#define ABS(X) ((X) < 0 ? -(X) : (X))
#define EPSILON 0.001

int goal_updown = 0; //default : 1
bool is_moving = false;
int present_posi=0;
int goal_posi = 0;
int start=0;

double acc_time_area=1.5;
double dt = 0.01; //sec
double t = 0.0;

void updownCallback(const std_msgs::Int8::ConstPtr& msg)
{
  if(msg->data != goal_updown){
    start = present_posi;
    goal_updown = msg->data;
    //is_moving = true;
    t = 0.0;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_updown_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/lidar_updown", 1000, updownCallback);  //( 0 : down) / ( 1: up )
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Int32>("present", 1000);

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position

  // Open port
  if (portHandler->openPort()){
    ROS_INFO("Succeeded to open the port!\n");
  }
  else{
    ROS_ERROR("Failed to open the port!");
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)){
    ROS_INFO("Succeeded to change the baudrate!");
  }
  else{
    ROS_ERROR("Failed to change the baudrate!\n");
    return 0;
  }


  // Enable Dynamixel Torque
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS) {
     packetHandler->getTxRxResult(dxl_comm_result);
   }
   else if (dxl_error != 0){
     packetHandler->getRxPacketError(dxl_error);
   }
   else{
     ROS_INFO("Dynamixel has been successfully connected");
   }

   dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
     packetHandler->getTxRxResult(dxl_comm_result);
   }
   else if (dxl_error != 0)
   {
     packetHandler->getRxPacketError(dxl_error);
   }
   present_posi = dxl_present_position;
   start = present_posi;
   ROS_INFO("present posi : %d ",present_posi);
   ros::Rate loop_rate(100); //100hz = 0.01 s
   while (ros::ok())
   {
     std_msgs::Int32 msg;

     if(t < acc_time_area){
       if(goal_updown == 1){
         goal_posi = 3235;
       }
       else if(goal_updown == 0){
         goal_posi = 2090;

       }
       if(ABS(t-acc_time_area)<EPSILON){
         present_posi = goal_posi;
       }
       else present_posi = 0.5*(1-cos((PI)*(t/acc_time_area)))*(goal_posi-start) + start;
 //       msg.data = present_posi;
       t+=dt;

      }

      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, (int)present_posi, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }


     msg.data = present_posi;
     chatter_pub.publish(msg);

     ros::spinOnce();

     loop_rate.sleep();
   }

   // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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




  return 0;
}
