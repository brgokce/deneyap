#include "ros/ros.h"
#include <iostream>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <ctime> 
#include <sensor_msgs/Imu.h>
#include <math.h> 
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
using namespace std;
float g = 9.81;
float Ixx=7e-3;
float Iyy=7e-3;
float Izz=12e-3;
float kf=2.13e-4;
float b=8.5e-7;
float l=0.17;
float m = 0.68;
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
    double x, y, z;
    std_msgs::Float64MultiArray f;
};

struct PID_pose_back
{
    double err_roll, err_pitch, err_yaw;
    double x, y, z;
    std_msgs::Float64MultiArray f;
};



struct PID_in
{
    double roll, pitch, yaw;
    double x, y, z;
    // std_msgs::Float64MultiArray f;
};

struct PID_pose_in
{
    double x_ref, y_ref, z_ref;
    double x, y, z;
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

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}


sensor_msgs::Imu imu_deger;
// gazebo_msgs::ModelStates drone_pose;
geometry_msgs::Pose drone_pose;
geometry_msgs::Twist drone_twist;

void imu_print()
{
    cout<< imu_deger.angular_velocity<< "\n" <<imu_deger.linear_acceleration << "\n";

}




PID_back PID(PID_in pid_in)
{
    PID_back pid_back;

    double kpx, kix, kdx, kpy, kiy, kdy, kpz, kiz, kdz;
    kpx=3;
    kix=0.13;
    kdx=1.1;
    kpy=kpx;
    kiy=kix;
    kdy=kdx;
    kpz=3;
    kiz=0.13;
    kdz=1.1;


    double kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw;
    double prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, flag, setpoint, sampleTime, prevTime, dTime;
    double prevErr_x, prevErr_y, prevErr_z, pMem_x, pMem_y, pMem_z, iMem_x, iMem_y,iMem_z, dMem_x, dMem_y, dMem_z; 
    double dErr_pitch, dErr_roll, dErr_yaw, dErr_z, dErr_y, dErr_x;

    ki_roll = 0.02;
    ki_pitch = ki_roll;
    ki_yaw = 0;

    double Kphi_p, Kphi_d, Ktheta_p, Ktheta_d, Kpsi_p, Kpsi_d ,Kz_p ,Kz_d;
    double xdesired ,ydesired, zdesired;
    // double phi_desired, theta_desired, psi_desired;
    double phidesired, thetadesired, psidesired;
    double ex ,dex, ey, dey, ez, dez ,dt, ephi ,dephi ,etheta ,detheta ,epsi ,depsi;
    
    xdesired=0;
    ydesired=0;
    zdesired=0;
    phidesired=-0*M_PI/180;
    thetadesired=0*M_PI/180;
    psidesired=0*M_PI/180;
    Kz_p=3;
    Kz_d=1;
    Kphi_p=100;
    Kphi_d=2;
    Ktheta_p=100;
    Ktheta_d=2;
    Kpsi_p=0.4;
    Kpsi_d=0.1;
    flag = 0;
    sampleTime = 0;
    setpoint = 0;



    pid_back.x=xdesired-drone_pose.position.x;
    pid_back.y=ydesired-drone_pose.position.y; 
    pid_back.z=zdesired-drone_pose.position.z;
    pid_back.err_roll = phidesired-pid_in.roll;
    pid_back.err_pitch = thetadesired-pid_in.pitch;
    pid_back.err_yaw = psidesired-pid_in.yaw;
    clock_t currTime = clock();

    if (flag == 0)
    {
        prevTime = 0;
        prevErr_x=0;
        prevErr_y=0;
        prevErr_z=0;
        prevErr_roll = 0;
        prevErr_pitch = 0;
        prevErr_yaw = 0;
        pMem_x = 0;
        pMem_y = 0;
        pMem_z = 0;
        pMem_roll = 0;
        pMem_pitch = 0;
        pMem_yaw = 0;
        iMem_x = 0;
        iMem_y = 0;
        iMem_z = 0;
        iMem_roll = 0;
        iMem_pitch = 0;
        iMem_yaw = 0;
        dMem_x = 0;
        dMem_y = 0;
        dMem_z = 0;
        dMem_roll = 0;
        dMem_pitch = 0;
        dMem_yaw = 0;
        flag += 1;
    }
    dTime = currTime - prevTime;
    dt = dTime ;
    
    dErr_x=pid_back.x-prevErr_x;
    dErr_y=pid_back.y-prevErr_y;
    dErr_z=pid_back.z-prevErr_z;
    dErr_pitch = pid_back.err_pitch - prevErr_pitch;
    dErr_roll = pid_back.err_roll - prevErr_roll;
    dErr_yaw = pid_back.err_yaw - prevErr_yaw;

    if(dTime >= sampleTime)
    {
        //#Kp*e(t)
        pMem_roll = Kphi_p * pid_back.err_roll;
        pMem_pitch = Ktheta_p * pid_back.err_pitch;
        pMem_yaw = Kpsi_p * pid_back.err_yaw;
        pMem_x = kpx * pid_back.x;
        pMem_y = kpy * pid_back.y;
        pMem_z = Kz_p * pid_back.z;

        //#integral(e(t))
        iMem_roll += pid_back.err_roll * dTime;
        iMem_pitch += pid_back.err_pitch * dTime;
        iMem_yaw += pid_back.err_yaw * dTime;
        iMem_x += pid_back.x * dTime;
        iMem_y += pid_back.y * dTime;
        iMem_z += pid_back.z * dTime;

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
        dMem_x = dErr_x / dTime;
        dMem_y = dErr_y / dTime;
        dMem_z = dErr_z / dTime;
        cout<<"turev"<< dMem_roll <<" ; "<< dMem_pitch<<" ; "<< dMem_yaw <<" ; "<< dMem_z <<"\n";
    }
    prevTime = currTime;
    prevErr_roll = pid_back.err_roll;
    prevErr_pitch = pid_back.err_pitch;
    prevErr_yaw = pid_back.err_yaw;
    prevErr_z=pid_back.z;
    double u_1, u_2, u_3, u_4;

    u_1=(g+pMem_z+Kz_d*dMem_z)*m/(cos(pid_in.roll)*cos(pid_in.pitch));
    u_2=(pMem_roll + ki_roll * iMem_roll + Kphi_d*dMem_roll)*Ixx;
    u_3=(pMem_pitch + ki_pitch * iMem_pitch + Ktheta_d*dMem_pitch)*Iyy;
    u_4=(pMem_yaw + ki_yaw * iMem_yaw + Kpsi_d*dMem_yaw)*Izz;

    /* u_1=(g+pMem_z+Kz_d*dMem_z)*m/(cos(pid_in.roll)*cos(pid_in.pitch));
    u_2=(pMem_roll + Kphi_d*dMem_roll)*Ixx;
    u_3=(pMem_pitch + Ktheta_d*dMem_pitch)*Iyy;
    u_4=(pMem_yaw + Kpsi_d*dMem_yaw)*Izz; */
    cout<<"u"<< u_1 <<" ; "<< u_2<<" ; "<< u_3 <<" ; "<< u_4 <<"\n";
    double w12, w22, w32, w42;
    w12=(u_1/(4*kf))-(u_3/(2*kf*l))-(u_4/(4*b));  
    w22=(u_1/(4*kf))-(u_2/(2*kf*l))  +(u_4/(4*b));
    w32=(u_1/(4*kf))+(u_3/(2*kf*l))-(u_4/(4*b));
    w42=(u_1/(4*kf))+(u_2/(2*kf*l))  +(u_4/(4*b));
    double br_motor_vel = sqrt(w22)+0;
    double bl_motor_vel = sqrt(w32)+0;
    double fr_motor_vel = sqrt(w12)+0;
    double fl_motor_vel = sqrt(w42)+0;

    if(br_motor_vel > 150)
        br_motor_vel = 150;
    if(bl_motor_vel > 150)
        bl_motor_vel = 150;
    if(fr_motor_vel > 150)
        fr_motor_vel = 150;
    if(fl_motor_vel > 150)
        fl_motor_vel = 150;

    if(br_motor_vel < 0)
        br_motor_vel = 0;
    if(bl_motor_vel < 0)
        bl_motor_vel = 0;
    if(fr_motor_vel < 0)
        fr_motor_vel = 0;
    if(fl_motor_vel < 0)
        fl_motor_vel = 0;
    pid_back.f.data.push_back(fr_motor_vel);
    pid_back.f.data.push_back(-fl_motor_vel);
    pid_back.f.data.push_back(bl_motor_vel);
    pid_back.f.data.push_back(-br_motor_vel);
    /*     pid_back.f.data.push_back(10);
    pid_back.f.data.push_back(-10);
    pid_back.f.data.push_back(10);
    pid_back.f.data.push_back(-10); */
    cout<< fr_motor_vel <<" ; "<< -fl_motor_vel <<" ; "<<bl_motor_vel <<" ; "<< -br_motor_vel <<"\n";

    return pid_back;
}
int sinir=0;
void control_drone(const gazebo_msgs::ModelStates::ConstPtr& msg, ros::Publisher a1, ros::Publisher a2, ros::Publisher a3, ros::Publisher a4)
{
    gazebo_msgs::ModelStates deneme;
    double roll, pitch, yaw, err_roll, err_pitch, err_yaw;
    PID_back cikis;
    PID_in giris;
    std_msgs::Float32 m;
    EulerAngles k;
    // std_msgs::Float64MultiArray f;
    Quaternion s;

    int ind=getIndex(msg->name, "drone");
    drone_pose.position.x=msg->pose[ind].position.x;
    drone_pose.position.y=msg->pose[ind].position.y;
    drone_pose.position.z=msg->pose[ind].position.z;
    drone_pose.orientation.x=msg->pose[ind].orientation.x;
    drone_pose.orientation.y=msg->pose[ind].orientation.y;
    drone_pose.orientation.z=msg->pose[ind].orientation.z;
    drone_pose.orientation.w=msg->pose[ind].orientation.w;
    drone_twist.linear.x=msg->twist[ind].linear.x;
    drone_twist.linear.y=msg->twist[ind].linear.y;
    drone_twist.linear.z=msg->twist[ind].linear.z;
    drone_twist.angular.x=msg->twist[ind].angular.x;
    drone_twist.angular.y=msg->twist[ind].angular.y;
    drone_twist.angular.z=msg->twist[ind].angular.z;

    // deneme.name[0]="drone";
    // int ink=getIndex(deneme.name, "drone");
    // deneme.pose[0]=msg->pose[ind];
    // drone_pose.pose[0].position=msg->pose[ind].position;
    // drone_pose.twist[0].linear=msg->twist[ind].linear;
    // drone_pose.twist[0].angular=msg->twist[ind].angular;
    s.x=msg->pose[ind].orientation.x;
    s.y=msg->pose[ind].orientation.y;
    s.z=msg->pose[ind].orientation.z;
    s.w=msg->pose[ind].orientation.w;
    k=ToEulerAngles(s);
    // cout<< k.roll <<" ; "<< k.pitch<<" ; "<<k.yaw<<"\n";

    // cout<< imu_deger.angular_velocity<< "\n" <<imu_deger.linear_acceleration << "\n";
    // imu_print();


    giris.roll=k.roll;
    giris.pitch=k.pitch;
    giris.yaw=k.yaw;
    /* if (sinir<=4)
    { */
    cikis = PID(giris);

    a1.publish(cikis.f);
    m.data=cikis.err_roll;
    a2.publish(m);
    m.data=cikis.err_pitch;
    a3.publish(m);
    m.data=cikis.err_yaw;
    a4.publish(m);
    /* sinir=sinir+1;
    } */

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_value)
{
    imu_deger.angular_velocity = imu_value->angular_velocity;
    imu_deger.linear_acceleration= imu_value-> linear_acceleration;
    // imu_print();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle control;
    // ros::Rate loop_rate(10);
    int sayi = 10;
    ros::Subscriber gazebo_model_state;
    ros::Subscriber imu_sub;
    ros::Publisher drone_joint_control; 
    ros::Publisher err_rollPub; 
    ros::Publisher err_pitchPub; 
    ros::Publisher err_yawPub; 
    drone_joint_control = control.advertise<std_msgs::Float64MultiArray>("/drone/joint_motor_controller/command",4);
    err_rollPub = control.advertise<std_msgs::Float32>("err_roll",1);
    err_pitchPub = control.advertise<std_msgs::Float32>("err_pitch",1);
    err_yawPub = control.advertise<std_msgs::Float32>("err_yaw",1);
    imu_sub = control.subscribe<sensor_msgs::Imu>("/drone/imu", 100, imu_callback);
    gazebo_model_state = control.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(control_drone, _1, drone_joint_control, err_rollPub, err_pitchPub, err_yawPub));
    // gazebo_model_state = control.subscribe("/gazebo/model_states", 1000, control_drone_v1);
    ros::spin();
    return 0;
}