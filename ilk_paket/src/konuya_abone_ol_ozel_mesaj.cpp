#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ilk_paket/ozellestirilmis_mesaj.h"
#include <iostream>

//Konu başlığı için çağrılan fonksiyon
void abone_bildirim_fonksiyonu(const ilk_paket::ozellestirilmis_mesaj::ConstPtr& mesaj)
{
	ROS_INFO("Sensor Adi:  		[%s]",mesaj->sensorGirisAdi.c_str());
	ROS_INFO("Sensor Durumu:  	[%d]",mesaj->durumu);
	ROS_INFO("Alinan Deger:  	[%d]",mesaj->sensorDegeri);
}

int main(int argc, char **argv)
{
	//Konuya abone ol adıyla bir ROS düğümü başlatılır
	ros::init(argc, argv,"konuya_abone_ol_ozel_mesaj");
	//Bir düğüm tanıtıcı nesnesi oluşturulur
	ros::NodeHandle dugum_nesnesi;
	//Bir yayınlama nesnesi oluşturulur
	ros::Subscriber abone_ol;
	abone_ol = dugum_nesnesi.subscribe("/Ozel_Mesaj",10,abone_bildirim_fonksiyonu);
	//Düğümün tekrarlı çevrimi sağlanır
	ros::spin();
	return 0;
}


