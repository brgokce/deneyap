#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ilk_paket/servis_ornegi.h"
#include <iostream>
#include <sstream>


using namespace std;


bool ros_servis_bidirimi(ilk_paket::servis_ornegi::Request  &req,
         				 ilk_paket::servis_ornegi::Response &res)
{

      switch (req.islem) {
          
        // If user enter + 
        case '+':
            res.sonuc= req.a + req.b;
            //printf("%.2f  ,%.2f   ,%.2f",req.a,req.b,res.sonuc);
            ROS_INFO("%.2f %c %.2f = %.2f",req.a,req.islem,req.b,res.sonuc);
            break;
          
        // If user enter - 
        case '-':
            res.sonuc= req.a - req.b;
            //printf("%.2f  ,%.2f   ,%.2f",req.a,req.b,res.sonuc);
            ROS_INFO("%.2f %c %.2f = %.2f",req.a,req.islem,req.b,res.sonuc);            
            break;
          
        // If user enter *
        case '*':
            res.sonuc= req.a * req.b;
            //printf("%.2f  ,%.2f   ,%.2f",req.a,req.b,res.sonuc);
            ROS_INFO("%.2f %c %.2f = %.2f",req.a,req.islem,req.b,res.sonuc);           
            break;
          
        // If user enter /
        case '/':
            res.sonuc= req.a / req.b;
            //printf("%.2f  ,%.2f   ,%.2f",req.a,req.b,res.sonuc);
            ROS_INFO("%.2f %c %.2f = %.2f",req.a,req.islem,req.b,res.sonuc);
            break;
        // If the operator is other than +, -, * or /, 
        // error message will display

        default:
            
            break;
    }

//43 +
//45 -
//42 *
//47 /
  return true;
}


int main(int argc, char **argv)

{
	//bir_konu_yayinla bir adıyla ROS düğümü başlatılır.
	ros::init(argc, argv,"ros_servis_sunucu");

	//Bir düğüm tanıtıcı nesnesi oluşturulur
	ros::NodeHandle dugum_nesnesi;

	//Bir servis nesnesi oluşturulur
  	ros::ServiceServer servis = dugum_nesnesi.advertiseService("ros_servis", ros_servis_bidirimi);
  	ROS_INFO("Ready to receive from client.");
  	ros::spin();
	return 0;
}