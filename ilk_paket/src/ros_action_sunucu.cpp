#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <actionlib/server/simple_action_server.h>
#include "ilk_paket/action_ornegiAction.h"
#include <iostream>
#include <sstream>

class action_ornegiAction
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<ilk_paket::action_ornegiAction> as; 
  // create messages that are used to published feedback/result
  ilk_paket::action_ornegiFeedback feedback;
  ilk_paket::action_ornegiResult result;

  std::string action_name;
  int hesap;
  float hesap_sonuc;

public:
  action_ornegiAction(std::string name) :
    as(nh_, name, boost::bind(&action_ornegiAction::executeCB, this, _1), false),
    action_name(name)
  {
	as.registerPreemptCallback(boost::bind(&action_ornegiAction::preemptCB, this));
	as.start();
  }

  ~action_ornegiAction(void)
  {
  }

  void preemptCB()
  {
	ROS_WARN("%s got preempted!", action_name.c_str());
	 
	as.setPreempted(result,"I got Preempted"); 
  }


  void executeCB(const ilk_paket::action_ornegiGoalConstPtr &hesap)
  {
    if(!as.isActive() || as.isPreemptRequested()) return;
    ros::Rate rate(5);
	  ROS_INFO("%s calisiyor sinir= %d", action_name.c_str(), hesap->sinir);


    

    switch (hesap->islem) {
          

        // If user enter + 
        case '+':
            hesap_sonuc = hesap->a + hesap->b;
            // printf("%.2f  ,%.2f   ,%.2d",hesap->a,hesap->b,hesap_sonuc);
            ROS_INFO("%.2f %c %.2f = %.2f",hesap->a,hesap->islem,hesap->b,hesap_sonuc);
            break;
          
        // If user enter - 
        case '-':
            hesap_sonuc = hesap->a - hesap->b;
            //printf("%.2f  ,%.2f   ,%.2f",hesap->a,hesap->b,res.sonuc);
            ROS_INFO("%.2f %c %.2f = %.2f",hesap->a,hesap->islem,hesap->b,hesap_sonuc);            
            break;
          
        // If user enter *
        case '*':
            hesap_sonuc= hesap->a * hesap->b;
            //printf("%.2f  ,%.2f   ,%.2f",hesap->a,hesap->b,res.sonuc);
            ROS_INFO("%.2f %c %.2f = %.2f",hesap->a,hesap->islem,hesap->b,hesap_sonuc);           
            break;
          
        // If user enter /
        case '/':
            hesap_sonuc= hesap->a / hesap->b;

            ROS_INFO("%.2f %c %.2f = %.2f",hesap->a,hesap->islem,hesap->b,hesap_sonuc);
            break;
          
        // If the operator is other than +, -, * or /, 
        // error message will display
        default:
            
            break; 
            
          }
          
     

    
    if(!as.isActive() || as.isPreemptRequested()){
			return;
		}	

    if(hesap->sinir >= hesap_sonuc){
			ROS_INFO("%s basarili hesaplanan sayi= %.2f", action_name.c_str(), hesap_sonuc);
			result.sonuc = hesap_sonuc;
			as.setSucceeded(result);

		}else{
			ROS_INFO("Hesaplanan sayi sinirdan daha buyuk (%.2f > %d)",hesap_sonuc,hesap->sinir);
			feedback.hedef_feedback = result.sonuc;
			as.publishFeedback(feedback);
      as.setAborted(result);
		}
		rate.sleep();

  }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ros_action");
  ROS_INFO("Action Sunucu Baslatildi");
  action_ornegiAction action_ornegi_obj(ros::this_node::getName());
  ros::spin();
  return 0;
}