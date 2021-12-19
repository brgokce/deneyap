#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

//Konu başlığı için çağrılan fonksiyon
void abone_bildirim_fonksiyonu(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("Alinan Deger:  [%d]",msg->data);
}

int main(int argc, char **argv)
{
	//Konuya abone ol adıyla bir ROS düğümü başlatılır
	ros::init(argc, argv,"konuya_abone_ol_subscriber");
	//Bir düğüm tanıtıcı nesnesi oluşturulur
	ros::NodeHandle dugum_nesnesi;
	//Bir yayınlama nesnesi oluşturulur
	ros::Subscriber abone_ol;
	abone_ol = dugum_nesnesi.subscribe("/SaymaDegeri",10,abone_bildirim_fonksiyonu);
	//Düğümün tekrarlı çevrimi sağlanır
	ros::spin();
	return 0;
}


