#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ilk_paket/servis_ornegi.h"
#include <iostream>
#include <sstream>

char op;
float num1, num2;
//Defining namespace using in this code
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_servis_istemci");
  ros::NodeHandle dugum_nesnesi;
  ros::Rate loop_rate(10);

  ros::ServiceClient istemci = dugum_nesnesi.serviceClient<ilk_paket::servis_ornegi>("ros_servis");

	while (ros::ok())
	{


	  ilk_paket::servis_ornegi srv;



          cin >> num1;
          cin >> op;
		  cin >> num2;
          srv.request.islem=op;
          srv.request.a=num1;
          srv.request.b=num2;
          if (istemci.call(srv))
	  {

	    ROS_INFO("%.2f  %c  %.2f = %.2f ",num1,op,num2,srv.response.sonuc);

	  }
	  else
	  {
	    ROS_ERROR("Failed to call service");
	    return 1;
	  }




	ros::spinOnce();
	//Setting the loop rate
	loop_rate.sleep();

	}

  return 0;
}
