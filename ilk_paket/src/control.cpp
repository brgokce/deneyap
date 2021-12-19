#include "ros/ros.h"
#include <iostream>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <ctime> 


using namespace std;

struct Quaternion
{
    double w, x, y, z;
};

struct EulerAngles 
{
    double roll, pitch, yaw;
};

struct PID_back
{
    double err_roll, err_pitch, err_yaw;
    std_msgs::Float64MultiArray f;
};

struct PID_in
{
    double roll, pitch, yaw;
    // std_msgs::Float64MultiArray f;
};

Quaternion ToQuaternion(EulerAngles aci)
{
    double cy = cos(aci.yaw * 0.5);
    double sy = sin(aci.yaw * 0.5);
    double cp = cos(aci.pitch * 0.5);
    double sp = sin(aci.pitch * 0.5);
    double cr = cos(aci.roll * 0.5);
    double sr = sin(aci.roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (abs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); 
    else
        angles.pitch = asin(sinp);

    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}

PID_back PID(PID_in pid_in)
{
    PID_back pid_back;
    double kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw;
    double prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, flag, setpoint, sampleTime, prevTime, dTime;
    double dErr_pitch, dErr_roll, dErr_yaw;
    kp_roll = 70;
    ki_roll = 0.0002;
    kd_roll = 89;
    kp_pitch = kp_roll;
    ki_pitch = ki_roll;
    kd_pitch = kd_roll;
    kp_yaw = 0.1;
    ki_yaw = 0;
    kd_yaw = 0;
    flag = 0;
    sampleTime = 0;
    setpoint = 0;
    pid_back.err_pitch = pid_in.pitch*(180 / 3.141592653) - setpoint;
    pid_back.err_roll = pid_in.roll*(180 / 3.141592653) - setpoint;
    pid_back.err_yaw = pid_in.yaw*(180/3.14159263) - setpoint;
    // auto start = std::chrono::system_clock::now();
    clock_t currTime = clock();




    if (flag == 0)
    {
        prevTime = 0;
        prevErr_roll = 0;
        prevErr_pitch = 0;
        prevErr_yaw = 0;
        pMem_roll = 0;
        pMem_pitch = 0;
        pMem_yaw = 0;
        iMem_roll = 0;
        iMem_pitch = 0;
        iMem_yaw = 0;
        dMem_roll = 0;
        dMem_pitch = 0;
        dMem_yaw = 0;
        flag += 1;
    }
    dTime = currTime - prevTime;
    dErr_pitch = pid_back.err_pitch - prevErr_pitch;
    dErr_roll = pid_back.err_roll - prevErr_roll;
    dErr_yaw = pid_back.err_yaw - prevErr_yaw;

    if(dTime >= sampleTime)
    {
        //#Kp*e(t)
        pMem_roll = kp_roll * pid_back.err_roll;
        pMem_pitch = kp_pitch * pid_back.err_pitch;
        pMem_yaw = kp_yaw * pid_back.err_yaw;

        //#integral(e(t))
        iMem_roll += pid_back.err_roll * dTime;
        iMem_pitch += pid_back.err_pitch * dTime;
        iMem_yaw += pid_back.err_yaw * dTime;
            
        if(iMem_roll > 400)
            iMem_roll = 400;
        if(iMem_roll < -400)
            iMem_roll = -400;
        if(iMem_pitch > 400)
            iMem_pitch = 400;
        if(iMem_pitch < -400)
            iMem_pitch = -400;
        if(iMem_yaw > 400)
            iMem_yaw = 400;
        if(iMem_yaw < -400)
            iMem_yaw = 400;
            
        //#derivative(e(t))
        dMem_roll = dErr_roll / dTime;
        dMem_pitch = dErr_pitch / dTime;
        dMem_yaw = dErr_yaw / dTime;
    }
    //	#Store the current variables into previous variables for the next iteration.
    prevTime = currTime;
    prevErr_roll = pid_back.err_roll;
    prevErr_pitch = pid_back.err_pitch;
    prevErr_yaw = pid_back.err_yaw;

    //#output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
    double output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll;
    double output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch;
    double output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw ;

    //#br in my code is fr in gazebos world
    double esc_br = 1500 + output_roll + output_pitch - output_yaw;
    //#bl in my code is br in gazebo's world
    double esc_bl = 1500 + output_roll - output_pitch + output_yaw;
    //#fl in my code is bl in gazebo's world
    double esc_fl = 1500 - output_roll - output_pitch - output_yaw;
    //#fr in my code is fl in gazebo's world
    double esc_fr = 1500 - output_roll + output_pitch + output_yaw;


    //#Limit the ESC pulses to upper limit and lower limit, in case the PID algorithm goes crazy and high af.
    if(esc_br > 2000)
        esc_br = 2000;
    if(esc_bl > 2000)
        esc_bl = 2000;
    if(esc_fr > 2000)
        esc_fr = 2000;
    if(esc_fl > 2000)
        esc_fl = 2000;

    if(esc_br < 1100)
        esc_br = 1100;
    if(esc_bl < 1100)
        esc_bl = 1100;
    if(esc_fr < 1100)
        esc_fr = 1100;
    if(esc_fl < 1100)
        esc_fl = 1100; 


    double br_motor_vel = ((esc_br - 1500)/25) + 80;
    double bl_motor_vel = ((esc_bl - 1500)/25) + 80;
    double fr_motor_vel = ((esc_fr - 1500)/25) + 80;
    double fl_motor_vel = ((esc_fl - 1500)/25) + 80;


    pid_back.f.data.push_back(fr_motor_vel);
    pid_back.f.data.push_back(-fl_motor_vel);
    pid_back.f.data.push_back(bl_motor_vel);
    pid_back.f.data.push_back(-br_motor_vel);

    return pid_back;
}

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void control_drone(const gazebo_msgs::ModelStates::ConstPtr& msg, ros::Publisher a1, ros::Publisher a2, ros::Publisher a3, ros::Publisher a4)
{
    double roll, pitch, yaw, err_roll, err_pitch, err_yaw;
    PID_back cikis;
    PID_in giris;
    std_msgs::Float32 m;
    // std_msgs::Float64MultiArray f;
    Quaternion s;
    EulerAngles k;
    int ind=getIndex(msg->name, "drone");
    s.x=msg->pose[ind].orientation.x;
    s.y=msg->pose[ind].orientation.y;
    s.z=msg->pose[ind].orientation.z;
    s.w=msg->pose[ind].orientation.w;
    k=ToEulerAngles(s);
    giris.roll=k.roll;
    giris.pitch=k.pitch;
    giris.yaw=k.yaw;
    cikis = PID(giris);

    a1.publish(cikis.f);
    m.data=cikis.err_roll;
    a2.publish(m);
    m.data=cikis.err_pitch;
    a3.publish(m);
    m.data=cikis.err_yaw;
    a4.publish(m);

}

void control_drone_v1(const gazebo_msgs::ModelStates msg)
{
    double roll, pitch, yaw, err_roll, err_pitch, err_yaw;
    PID_back cikis;
    PID_in giris;
    Quaternion s;
    EulerAngles k;
    int ind=getIndex(msg.name, "drone");
    s.x=msg.pose[ind].orientation.x;
    s.y=msg.pose[ind].orientation.y;
    s.z=msg.pose[ind].orientation.z;
    s.w=msg.pose[ind].orientation.w;
    k=ToEulerAngles(s);
    giris.roll=k.roll;
    giris.pitch=k.pitch;
    giris.yaw=k.yaw;
    cikis = PID(giris);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle control;
    // ros::Rate loop_rate(10);
    int sayi = 10;
    ros::Subscriber gazebo_model_state;
    ros::Publisher drone_joint_control; 
    ros::Publisher err_rollPub; 
    ros::Publisher err_pitchPub; 
    ros::Publisher err_yawPub; 
    drone_joint_control = control.advertise<std_msgs::Float64MultiArray>("/drone/joint_motor_controller/command",4);
    err_rollPub = control.advertise<std_msgs::Float32>("err_roll",1);
    err_pitchPub = control.advertise<std_msgs::Float32>("err_pitch",1);
    err_yawPub = control.advertise<std_msgs::Float32>("err_yaw",1);
    gazebo_model_state = control.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(control_drone, _1, drone_joint_control, err_rollPub, err_pitchPub, err_yawPub));
    // gazebo_model_state = control.subscribe("/gazebo/model_states", 1000, control_drone_v1);
    ros::spin();
    return 0;
}