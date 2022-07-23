#include "ros/ros.h"
#include "Test_Program/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<Test_Program::AddTwoInts>("add_two_ints");
  Test_Program::AddTwoInts srv;

  ros::Rate loop_rate(5);

  for(int i=0; i<10; i++){
    for(int j=0; j<10; j++){
      srv.request.a = i;
      srv.request.b = j;
      if (client.call(srv))
      {
        ROS_INFO("Sum: %ld", (long int)srv.response.sum);
      }
      else
      {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
      }
      if(!ros::ok()){
        ROS_INFO("Service Closed");
        return -1;
      }
      loop_rate.sleep();
    }
    if(i == 9){
      i = 0;
    }
  }

  while(ros::ok()){
    

    loop_rate.sleep();
  }

  

  return 0;
}