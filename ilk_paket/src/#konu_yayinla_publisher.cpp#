#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

int main(int argc, char **argv)

{
	//bir_konu_yayinla bir adıyla ROS düğümü başlatılır.
	ros::init(argc, argv,"bir_konu_yayinla");

	//Bir düğüm tanıtıcı nesnesi oluşturulur
	ros::NodeHandle dugum_nesnesi;

	//Bir yayıncı nesnesi oluşturulur
	ros::Publisher sayi_nesnesi; 
	sayi_nesnesi = dugum_nesnesi.advertise<std_msgs::Int32>("/SaymaDegeri",10);

	//Bir hız nesnesi oluşturulur
	ros::Rate loop_rate(10);

	//sayi adinda bir  değişken tanımlanır
	int sayi = 0;

	//Sayıyı artırmak ve konuyu yayınlamak için while döngüsü oluşturulur
	while (ros::ok())
	{
		std_msgs::Int32 msg;

		msg.data = sayi;

		//Mesaj verisi ekrana yazdırılır
		ROS_INFO("Gonderilen Deger: %d",msg.data);

		//konu başlığı yayınlanır.
		sayi_nesnesi.publish(msg);

		//Tüm işlemi bir kez yapmak için en az bir kez işlem döndürülür
		ros::spinOnce();

		//Sleeping for sometime
		loop_rate.sleep();

		//sayi degeri arttırılır
		++sayi;
	}
	
	return 0;
}


