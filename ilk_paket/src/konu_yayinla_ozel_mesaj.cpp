#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ilk_paket/ozellestirilmis_mesaj.h"
#include <iostream>


using namespace std;

int main(int argc, char **argv)

{
	//bir_konu_yayinla bir adıyla ROS düğümü başlatılır.
	ros::init(argc, argv,"ozel_mesaj_yayinla");

	//Bir düğüm tanıtıcı nesnesi oluşturulur
	ros::NodeHandle dugum_nesnesi;

	//Bir yayıncı nesnesi oluşturulur
	ros::Publisher sayi_nesnesi; 
	sayi_nesnesi = dugum_nesnesi.advertise<ilk_paket::ozellestirilmis_mesaj>("/Ozel_Mesaj",10);

	//Bir hız nesnesi oluşturulur
	ros::Rate loop_rate(10);

	//sayi adinda bir  değişken tanımlanır
	int sayi = 0;
	ilk_paket::ozellestirilmis_mesaj mesaj;
	mesaj.sensorGirisAdi = "ENKODER";
	mesaj.durumu   =  true;
	mesaj.sensorDegeri   =  sayi;
	ROS_INFO("Sensor Adi : %s",   mesaj.sensorGirisAdi.c_str());
	ROS_INFO("Sensor Durumu : %d",mesaj.durumu);

	while (ros::ok())
	{
		//Mesaj verisi ekrana yazdırılır
		mesaj.sensorDegeri   =  sayi;
		ROS_INFO("Sensor Degeri: %d", mesaj.sensorDegeri );

		//konu başlığı yayınlanır.
		sayi_nesnesi.publish(mesaj);

		//Tüm işlemi bir kez yapmak için en az bir kez işlem döndürülür
		ros::spinOnce();

		//Sleeping for sometime
		loop_rate.sleep();

		//sayi degeri arttırılır
		++sayi;
	}
	
	return 0;
}


