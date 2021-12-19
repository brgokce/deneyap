#include "ros/ros.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ilk_paket/action_ornegiAction.h"

char op;
float num1, num2;
using namespace std;

/* void feedbackCb(const ilk_paket::action_ornegiFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback of length %d", feedback->hedef_feedback);
} */
 

int main (int argc, char **argv)
{
  ros::init(argc, argv, "ros_action_istemci");

   if(argc != 2){
	ROS_INFO("%d",argc);
	ROS_WARN("Usage: demo_action_client <goal> <time_to_preempt_in_sec>");
	return 1;
	}

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ilk_paket::action_ornegiAction> ac("ros_action", true);

  ROS_INFO("Action sunucusunun baslatilmasi bekleniyor");

  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  
  //cout<< argv[1];
  ROS_INFO("hesap icin 1. sayiyi giriniz");
  cin >> num1;
  ROS_INFO("hesap icin islem turu giriniz");
  cin >> op;
  ROS_INFO("hesap icin 2. sayiyi giriniz");
  cin >> num2;
  ROS_INFO("Action sunucusu baslatilmis, Hesap icin gonderiliyor.");
  // send a goal to the action
  ilk_paket::action_ornegiGoal goal;
  goal.islem = op;
  goal.a=num1;
  goal.b=num2;
  goal.sinir=atoi(argv[1]);
  ROS_INFO("islem( %.2f %c %.2f ) sinir(%d)",goal.a, goal.islem, goal.b, atoi(argv[1]));
  ac.sendGoal(goal);

  bool sinir_asimi = ac.waitForResult(ros::Duration(atoi(argv[1])));
  //int deneme=atoi(argv[1]);
  //ROS_INFO("%d ",sinir_asimi);
  //int sinir_control = atoi(argv[1]);
  //Preempting task
  //ac.getResult(const action_ornegiResultConstPtr& deneme);
  ilk_paket::action_ornegiResultConstPtr result=ac.getResult();

  //ilk_paket::action_ornegiFeedbackConstPtr feedback=ac.feedbackCb();

  ac.cancelGoal();
  if (result->sonuc <= atoi(argv[1]))
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    
    ROS_INFO("Action finished: %s ",state.toString().c_str());
    
    //Preempting the process
    ac.cancelGoal();

  }
  else
    ROS_INFO("Hesaplanan deger sinirdan buyuk!!!");

  //exit
  return 0;
}
