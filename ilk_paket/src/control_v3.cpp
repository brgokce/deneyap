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
float g = 9.81; //yer cekimi ivmesi
float Ixx=7e-3; //x'teki toplam atalet
float Iyy=7e-3; //y'teki toplam atalet
float Izz=12e-3; //z'teki toplam atalet
float kf=4.13e-4; //pervane itki kuvveti(trust)
float b=8.5e-7; //
float l=0.17; //iki kol arasındaki mesafe
float m = 0.48; //dronun kutlesi
float sayi=0;
float flag_pose=0; 
float flag_rotate=0;
struct Quaternion
{
    double w, x, y, z;
};
struct EulerAngles 
{
    double roll, pitch, yaw;
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
EulerAngles ToEulerAngles(Quaternion q) 
{
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
struct PID_rotation_in
{
    double ref_roll, ref_pitch, ref_yaw, u_1;
    double preverr_roll, preverr_pitch, preverr_yaw;
};
struct PID_rotation_back
{
    double roll, pitch, yaw;
    double preverr_roll, preverr_pitch, preverr_yaw;
    std_msgs::Float64MultiArray f;
};

struct PID_position_in
{
    double ref_x, ref_y, ref_z;
    double preverr_x, preverr_y, preverr_z;
};

struct PID_position_back
{
    double roll, pitch, u_1;
    double preverr_x, preverr_y, preverr_z;
};

sensor_msgs::Imu imu_deger;
geometry_msgs::Pose drone_pose;
geometry_msgs::Twist drone_twist;
EulerAngles k;
PID_rotation_back pid_rotation_back;
PID_rotation_in pid_rotation_in;
PID_position_back pid_position_back;
PID_position_in pid_position_in;


/* pid_position_in.preverr_x=0;
pid_position_in.preverr_y=0;
pid_position_in.preverr_z=0; */
void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_value)
{
    imu_deger.angular_velocity = imu_value->angular_velocity;
    imu_deger.linear_acceleration= imu_value-> linear_acceleration;
    // imu_print();
}

double iMem_x, iMem_y,iMem_z;
double prevErr_x, prevErr_y, prevErr_z, prevTime_pose;
PID_position_back PID_position(PID_position_in pid_in)
{
    PID_position_back pid_back;
    double kpx, kix, kdx, kpy, kiy, kdy, kpz, kiz, kdz;
    kpx=0.08;
    kix=0.001;
    kdx=0.001;
    kpy=kpx;
    kiy=kix;
    kdy=kdx;
    kpz=3;
    kiz=0;
    kdz=0;
    double pMem_x, pMem_y, pMem_z;
    double dMem_x, dMem_y, dMem_z; 
    double dErr_x, dErr_y, dErr_z;
    double iErr_x, iErr_y, iErr_z;
    double sampleTime, dTime, dt;
    double Err_x, Err_y, Err_z;
    sampleTime = 0;
    Err_x=pid_in.ref_x-drone_pose.position.x;
    Err_y=pid_in.ref_y-drone_pose.position.y;
    Err_z=pid_in.ref_z-drone_pose.position.z;
    clock_t currTime = clock();
    if (flag_pose == 0)
    {
        prevTime_pose = 0;
        prevErr_x=0;
        prevErr_y=0;
        prevErr_z=0;
        pMem_x = 0;
        pMem_y = 0;
        pMem_z = 0;
        iMem_x = 0;
        iMem_y = 0;
        iMem_z = 0;
        dMem_x = 0;
        dMem_y = 0;
        dMem_z = 0;
        flag_pose += 1;
    }
    dTime = currTime - prevTime_pose;
    dt = dTime ;
    dTime=dTime/1000;
    dErr_x = Err_x - prevErr_x;
    dErr_y = Err_y - prevErr_y;
    dErr_z = Err_z - prevErr_z;
    iErr_x = Err_x + prevErr_x;
    iErr_y = Err_y + prevErr_y;
    iErr_z = Err_z + prevErr_z;
    
    if(dTime >= sampleTime)
    {
        //#Kp*e(t)
        pMem_x =  Err_x;
        pMem_y =  Err_y;
        pMem_z =  Err_z;

        //#integral(e(t))
        iMem_x += iErr_x / 2 * dTime;
        iMem_y += iErr_y / 2 * dTime;
        iMem_z += iErr_z / 2 * dTime;

        if(iMem_x > 10)
            iMem_x = 10;
        if(iMem_x < -10)
            iMem_x = -10;
        if(iMem_y > 10)
            iMem_y = 10;
        if(iMem_y < -10)
            iMem_y = -10;
        if(iMem_z > 10)
            iMem_z = 10;
        if(iMem_z < -10)
            iMem_z = -10;


        dMem_x = dErr_x / dTime;
        dMem_y = dErr_y / dTime;
        dMem_z = dErr_z / dTime;
        if(dMem_x > 10)
            dMem_x = 10;
        if(dMem_x < -10)
            dMem_x = -10;
        if(dMem_y > 10)
            dMem_y = 10;
        if(dMem_y < -10)
            dMem_y = -10;
        if(dMem_z > 10)
            dMem_z = 10;
        if(dMem_z < -10)
            dMem_z = -10;
        // cout<<"integralpos"<< iMem_x <<" ; "<< iMem_y<<" ; "<< iMem_z <<"\n";
        // cout<<"turevpos"<< dMem_x <<" ; "<< dMem_y<<" ; "<< dMem_z <<"\n";
    }
    prevTime_pose = currTime;
    prevErr_x = Err_x;
    prevErr_y = Err_y;
    prevErr_z = Err_z;

    double u_x, u_y, u_z;
    u_x = kpx * pMem_x + kix * iMem_x + kdx * dMem_x;
    u_y = kpy * pMem_y + kiy * iMem_y + kdy * dMem_y;
    u_z = kpz * pMem_z + kiz * iMem_z + kdz * dMem_z;

    // cout<<"controlpos"<< u_x <<" ; "<< u_y<<" ; "<< u_z <<"\n";
    
    pid_back.u_1 = m * sqrt(u_x * u_x + u_y * u_y + (u_z + g) * (u_z + g));
    pid_back.roll=asin(-m* u_y / pid_back.u_1);
    pid_back.pitch=atan(u_x /(u_z + g));
    // cout<<"aciout"<< pid_back.u_1 <<" ; "<< pid_back.roll<<" ; "<< pid_back.pitch <<"\n";
    if (pid_back.roll>10)
        pid_back.roll=10;
    if (pid_back.roll<-10)
        pid_back.roll=-10;
    if (pid_back.pitch>10)
        pid_back.pitch=10;
    if (pid_back.pitch<-10)
        pid_back.pitch=-10;

    pid_back.preverr_x = prevErr_x;
    pid_back.preverr_y = prevErr_y;
    pid_back.preverr_z = prevErr_z;
    return pid_back;
}



double iMem_roll, iMem_pitch,iMem_yaw;
double prevErr_roll, prevErr_pitch, prevErr_yaw, prevTime_rotate;
PID_rotation_back PID_rotation(PID_rotation_in pid_in)
{
    PID_rotation_back pid_back;
    double kproll, kiroll, kdroll, kppitch, kipitch, kdpitch, kpyaw, kiyaw, kdyaw;
    kproll = 80;
    kiroll = 0.02;
    kdroll = 0,01;
    kppitch = kproll;
    kipitch = kiroll;
    kdpitch = kdroll;
    kpyaw = 0.4;
    kiyaw = 0;
    kdyaw = 0;
    double ref_yaw=0;
    double pMem_roll, pMem_pitch, pMem_yaw;
    double dMem_roll, dMem_pitch, dMem_yaw; 
    double dErr_roll, dErr_pitch, dErr_yaw;
    double iErr_roll, iErr_pitch, iErr_yaw;
    double sampleTime, dTime, dt;
    double Err_roll, Err_pitch, Err_yaw;
    sampleTime = 0;
    Err_roll = pid_in.ref_roll-k.roll;
    Err_pitch = pid_in.ref_pitch-k.pitch;
    Err_yaw = pid_in.ref_yaw-k.yaw;
    clock_t currTime = clock();

    if (flag_rotate == 0)
    {
        prevTime_rotate = 0;
        prevErr_roll=0;
        prevErr_pitch=0;
        prevErr_yaw=0;
        pMem_roll = 0;
        pMem_pitch = 0;
        pMem_yaw = 0;
        iMem_roll = 0;
        iMem_pitch = 0;
        iMem_yaw = 0;
        dMem_roll = 0;
        dMem_pitch = 0;
        dMem_yaw = 0;
        flag_rotate += 1;
    }
    dTime = currTime - prevTime_rotate;
    dt = dTime ;
    dTime=dTime/1000;
    dErr_roll=Err_roll - prevErr_roll;
    dErr_pitch=Err_pitch - prevErr_pitch;
    dErr_yaw=Err_yaw - prevErr_yaw;
    iErr_roll = Err_roll + prevErr_roll;
    iErr_pitch = Err_pitch + prevErr_pitch;
    iErr_yaw = Err_yaw + prevErr_yaw;

    if(dTime >= sampleTime)
    {
        //#Kp*e(t)
        pMem_roll =  Err_roll;
        pMem_pitch =  Err_pitch;
        pMem_yaw =  Err_yaw;

        //#integral(e(t))
        iMem_roll += iErr_roll / 2 * dTime;
        iMem_pitch += iErr_pitch / 2 * dTime;
        iMem_yaw += iErr_yaw / 2 * dTime;

        if(iMem_roll > 10)
            iMem_roll = 10;
        if(iMem_roll < -10)
            iMem_roll = -10;
        if(iMem_pitch > 10)
            iMem_pitch = 10;
        if(iMem_pitch < -10)
            iMem_pitch = -10;
        if(iMem_yaw > 10)
            iMem_yaw = 10;
        if(iMem_yaw < -10)
            iMem_yaw = -10;


        dMem_roll = dErr_roll / dTime;
        dMem_pitch = dErr_pitch / dTime;
        dMem_yaw = dErr_yaw / dTime;
        if(dMem_roll > 10)
            dMem_roll = 10;
        if(dMem_roll < -10)
            dMem_roll = -10;
        if(dMem_pitch > 10)
            dMem_pitch = 10;
        if(dMem_pitch < -10)
            dMem_pitch = -10;
        if(dMem_yaw > 10)
            dMem_yaw = 10;
        if(dMem_yaw < -10)
            dMem_yaw = -10;

    }
    prevTime_pose = currTime;
    prevErr_roll = Err_roll;
    prevErr_pitch = Err_pitch;
    prevErr_yaw = Err_yaw;

    double u_roll, u_pitch, u_yaw;
    u_roll = kproll * pMem_roll + kiroll * iMem_roll + kdroll * dMem_roll;
    u_pitch = kppitch * pMem_pitch + kipitch * iMem_pitch + kdpitch * dMem_pitch;
    u_yaw = kpyaw * pMem_yaw + kiyaw * iMem_yaw + kdyaw * dMem_yaw;

    double ixx,iyy,izz;
    ixx=0.1247618;
    iyy=0.1247618;
    izz=0.60168977;

    // ixx=0.007;
    // iyy=0.007;
    // izz=0.012;

    u_roll = u_roll * ixx;
    u_pitch = u_pitch * iyy;
    u_yaw = u_yaw * izz;

    double w12, w22, w32, w42;
    /* w12=(pid_in.u_1/(4*kf))-(u_3/(2*kf*l))-(u_4/(4*b));  
    w22=(pid_in.u_1/(4*kf))-(u_2/(2*kf*l))  +(u_4/(4*b));
    w32=(pid_in.u_1/(4*kf))+(u_3/(2*kf*l))-(u_4/(4*b));
    w42=(pid_in.u_1/(4*kf))+(u_2/(2*kf*l))  +(u_4/(4*b)); */
    w12=(pid_in.u_1/(4*kf))-(u_pitch/(2*kf*l))-(u_yaw/(4*b));  
    w22=(pid_in.u_1/(4*kf))-(u_roll/(2*kf*l))  +(u_yaw/(4*b));
    w32=(pid_in.u_1/(4*kf))+(u_pitch/(2*kf*l))-(u_yaw/(4*b));
    w42=(pid_in.u_1/(4*kf))+(u_roll/(2*kf*l))  +(u_yaw/(4*b));
    // if(w12 > 80)
    //     w12 = 80;
    // if(w22 > 80)
    //     w22 = 80;
    // if(w32 > 80)
    //     w32 = 80;
    // if(w42 > 80)
    //     w42 = 80;

    if(w12 < 0)
        w12 = 0;
    if(w22 < 0)
        w22 = 0;
    if(w32 < 0)
        w32 = 0;
    if(w42 < 0)
        w42 = 0;
   
    double br_motor_vel = sqrt(w22)+0;
    double bl_motor_vel = sqrt(w32)+0;
    double fr_motor_vel = sqrt(w12)+0;
    double fl_motor_vel = sqrt(w42)+0;

    if(br_motor_vel > 80)
        br_motor_vel = 80;
    if(bl_motor_vel > 80)
        bl_motor_vel = 80;
    if(fr_motor_vel > 80)
        fr_motor_vel = 80;
    if(fl_motor_vel > 80)
        fl_motor_vel = 80;

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
    pid_back.preverr_roll = prevErr_roll;
    pid_back.preverr_pitch = prevErr_pitch;
    pid_back.preverr_yaw = prevErr_yaw;
    // cout<< fr_motor_vel <<" ; "<< -fl_motor_vel <<" ; "<<bl_motor_vel <<" ; "<< -br_motor_vel <<"\n";
    return pid_back;
}

//////////////////////////////////////////////////////////////////////////////////

double pid_rotate_c_PrevvErr_r, pid_rotate_c_PrevvErr_p, pid_rotate_c_PrevvErr_y, pid_rotate_c_PrevvErr_z;
double pid_rotate_iMem_r , pid_rotate_iMem_p , pid_rotate_iMem_y, pid_rotate_iMem_z;
double pid_rotate_flag =0;
double prevTime=0;
int sinir=0;
PID_rotation_back PID_control(PID_rotation_in pid_in)
{
    PID_rotation_back pid_back;
    double kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw;
    double Kphi_p, Kphi_d, Ktheta_p, Ktheta_d, Kpsi_p, Kpsi_d ,kp_z ,ki_z ,kd_z;
    double phidesired, thetadesired, psidesired, zdesired;



    kp_roll=80;
    ki_roll =0.08;
    kd_roll=0.2;
    // ki_roll = 0;
    // kd_roll=0;
    kp_pitch=kp_roll;
    ki_pitch = ki_roll;
    kd_pitch=kd_roll;
    kp_yaw=0.4;
    ki_yaw = 0;
    kd_yaw=0;
    kp_z=3;
    ki_z=0;
    
    phidesired=-0*M_PI/180;
    thetadesired=0*M_PI/180;
    psidesired=0*M_PI/180;
    zdesired = 0;
    double dTime, sampleTime, dt;
    double pMem_roll, pMem_yaw, pMem_pitch, pMem_z;
    double dMem_roll, dMem_pitch, dMem_yaw, dMem_z;
    double dErr_pitch, dErr_roll, dErr_yaw, dErr_z;
    double Err_roll, Err_pitch, Err_yaw, Err_z;
    double iErr_roll, iErr_pitch, iErr_yaw, iErr_z;
    sampleTime = 0;
    Err_roll = phidesired-k.roll;
    Err_pitch = thetadesired-k.pitch;
    Err_yaw = psidesired-k.yaw;
    Err_z=zdesired-drone_pose.position.z;
    clock_t currTime = clock();

    if (pid_rotate_flag == 0)
    {
        prevTime = 0;
        pid_rotate_c_PrevvErr_r = 0;
        pid_rotate_c_PrevvErr_p = 0;
        pid_rotate_c_PrevvErr_y = 0;
        pid_rotate_c_PrevvErr_y = 0;
        pMem_roll = 0;
        pMem_pitch = 0;
        pMem_yaw = 0;
        pMem_z = 0;
        pid_rotate_iMem_r = 0;
        pid_rotate_iMem_p = 0;
        pid_rotate_iMem_y = 0;
        pid_rotate_iMem_z = 0;
        dMem_roll = 0;
        dMem_pitch = 0;
        dMem_yaw = 0;
        dMem_z = 0;
        pid_rotate_flag += 1;
    }
    dTime = currTime - prevTime;
    dt = dTime ;
    dErr_roll = Err_roll - pid_rotate_c_PrevvErr_r;
    dErr_pitch = Err_pitch - pid_rotate_c_PrevvErr_p;
    dErr_yaw = Err_yaw - pid_rotate_c_PrevvErr_y;
    dErr_z = Err_z - pid_rotate_c_PrevvErr_z;
    iErr_roll = Err_roll + pid_rotate_c_PrevvErr_r;
    iErr_pitch = Err_pitch + pid_rotate_c_PrevvErr_p;
    iErr_yaw = Err_yaw + pid_rotate_c_PrevvErr_y;
    iErr_z = Err_z + pid_rotate_c_PrevvErr_z;

    if(dTime >= sampleTime)
    {
        //#Kp*e(t)
        pMem_roll =  Err_roll;
        pMem_pitch =  Err_pitch;
        pMem_yaw =  Err_yaw;
        pMem_z = Err_z;

        //#integral(e(t))
        pid_rotate_iMem_r += iErr_roll / 2 * dTime;
        pid_rotate_iMem_p += iErr_pitch / 2 * dTime;
        pid_rotate_iMem_y += iErr_yaw / 2 * dTime;
        pid_rotate_iMem_z += iErr_z / 2 * dTime;

        if(pid_rotate_iMem_r > 100)
            pid_rotate_iMem_r = 100;
        if(pid_rotate_iMem_r < -100)
            pid_rotate_iMem_r = -100;
        if(pid_rotate_iMem_p > 100)
            pid_rotate_iMem_p = 100;
        if(pid_rotate_iMem_p < -100)
            pid_rotate_iMem_p = -100;
        if(pid_rotate_iMem_y > 100)
            pid_rotate_iMem_y = 100;
        if(pid_rotate_iMem_y < -100)
            pid_rotate_iMem_y = -100;
        if(pid_rotate_iMem_z > 100)
            pid_rotate_iMem_z = 100;
        if(pid_rotate_iMem_z < -100)
            pid_rotate_iMem_z = -100;


        //#derivative(e(t))
        dMem_roll = dErr_roll / dTime;
        dMem_pitch = dErr_pitch / dTime;
        dMem_yaw = dErr_yaw / dTime;
        dMem_z = dErr_z / dTime;
        // cout<<"turev"<< dMem_roll <<" ; "<< dMem_pitch<<" ; "<< dMem_yaw <<" ; "<< dMem_z <<"\n";
    }
    prevTime = currTime;
    pid_rotate_c_PrevvErr_r = Err_roll;
    pid_rotate_c_PrevvErr_p = Err_pitch;
    pid_rotate_c_PrevvErr_y = Err_yaw;
    pid_rotate_c_PrevvErr_z = Err_z;
    double u_1, u_2, u_3, u_4;

    u_1=(g+kp_z * pMem_z +  ki_z * pid_rotate_iMem_z + kd_z * dMem_z )*m/(cos(k.roll)*cos(k.pitch));
    u_2=(kp_roll * pMem_roll + ki_roll * pid_rotate_iMem_r + kd_roll * dMem_roll)*Ixx;
    u_3=(kp_pitch * pMem_pitch + ki_pitch * pid_rotate_iMem_p + kd_pitch * dMem_pitch)*Iyy;
    u_4=(kp_yaw * pMem_yaw + ki_yaw * pid_rotate_iMem_y + kd_yaw * dMem_yaw)*Izz;

    // cout<<"u"<< u_1 <<" ; "<< u_2<<" ; "<< u_3 <<" ; "<< u_4 <<"\n";
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
    
    // cout<< fr_motor_vel <<" ; "<< -fl_motor_vel <<" ; "<<bl_motor_vel <<" ; "<< -br_motor_vel <<"\n";





    return pid_back;
}
double pre_dt=0;
double control_dt=0;
void control_drone(const gazebo_msgs::ModelStates::ConstPtr& msg, ros::Publisher a1, ros::Publisher a2, ros::Publisher a3, ros::Publisher a4)
{
    gazebo_msgs::ModelStates deneme;
    double roll, pitch, yaw, err_roll, err_pitch, err_yaw;
    /* PID_back cikis;
    PID_in giris; */
    std_msgs::Float32 mk;
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
    s.x=msg->pose[ind].orientation.x;
    s.y=msg->pose[ind].orientation.y;
    s.z=msg->pose[ind].orientation.z;
    s.w=msg->pose[ind].orientation.w;
    k=ToEulerAngles(s);
    
    ///////////////////////////////////////position_control//////////////////////////////////

    /* if (sinir<=6050)
    { */
    double tra[28][3]={{0,0,2},{2,2,2},{2,-2,2},{-2,-2,2},{-2,2,2},{2,2,2}};
    // tra[2][2]=6;
    //cout<< tra[0][2]<<"\n";
    double tolerans=0.25;
    

    // clock_t currTime = clock();

    /* if (drone_pose.position.x < (tra[sinir][0] + tolerans))
        {
             //cout<< sinir << "a" << "\n";
            if (drone_pose.position.x > (tra[sinir][0] - tolerans))
            {
                // cout<< sinir << "b"  << "\n";
                if (drone_pose.position.y < (tra[sinir][1] + tolerans))
                {
                     //cout<< sinir << "c"  << "\n";
                    if (drone_pose.position.y > (tra[sinir][1] - tolerans))
                    {
                         //cout<< sinir << "d"  << "\n";
                        if (drone_pose.position.z < (tra[sinir][2] + tolerans + 1))
                        {
                             //cout<< sinir << "f"  << "\n";
                            if (drone_pose.position.z > (tra[sinir][2] - tolerans - 1))
                            {
                                
                                sinir=sinir+1;
                                cout<< sinir << "e"  << "\n";
                                if (sinir>4)
                                {
                                    sinir=0;

                                }}}}}}} */

    // cout<< sinir << "\n";
    pid_position_in.ref_x=tra[sinir][0];
    pid_position_in.ref_y=tra[sinir][1];
    pid_position_in.ref_z=tra[sinir][2];
    
    // pid_position_in.ref_x=-2;
    // pid_position_in.ref_y=-1;
    // pid_position_in.ref_z=2;


    pid_position_back = PID_position(pid_position_in);
    // cout<<"position_err " <<pid_position_back.preverr_x <<" ; "<< pid_position_back.preverr_y <<" ; "<<pid_position_back.preverr_z <<"\n";

    pid_rotation_in.u_1 = pid_position_back.u_1;
    pid_rotation_in.ref_roll = pid_position_back.roll;
    pid_rotation_in.ref_pitch = pid_position_back.pitch;
    pid_rotation_in.ref_yaw = 0*M_PI/180;
    pid_rotation_back = PID_rotation(pid_rotation_in);
    // cout<<"rotation_err " <<pid_rotation_back.preverr_roll <<" ; "<< pid_rotation_back.preverr_pitch <<" ; "<<pid_rotation_back.preverr_yaw <<"\n";
    a1.publish(pid_rotation_back.f);
    mk.data=k.roll;
    a2.publish(mk);
    mk.data=k.pitch;
    a3.publish(mk);
    mk.data=k.yaw;
    a4.publish(mk);

    // control_dt = currTime - pre_dt;
    // pre_dt=currTime;
    // cout<< currTime << " current time ";
    // cout<<control_dt << "\n";
    /* } */


    /////////////////////////////////////////////////////////////////////////////////////////
    // cout<< k.roll << " ; " << k.pitch << " ; " << k.yaw << " ; \n" ;

    ////////////////////////////////////////rotation_control////////////////////////////////

    /* pid_rotation_in.ref_roll = 0;
    pid_rotation_in.ref_pitch = 0;
    pid_rotation_in.ref_yaw = 0;
    pid_rotation_back = PID_control(pid_rotation_in);
    cout<<"rotation_err " <<pid_rotation_back.preverr_roll <<" ; "<< pid_rotation_back.preverr_pitch <<" ; "<<pid_rotation_back.preverr_yaw <<"\n";
    a1.publish(pid_rotation_back.f);
    mk.data=k.roll;
    a2.publish(mk);
    mk.data=k.pitch;
    a3.publish(mk);
    mk.data=k.yaw;
    a4.publish(mk); */



    /////////////////////////////////////////////////////////////////////////////////////////
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle control;

    ros::Subscriber gazebo_model_state;
    ros::Subscriber imu_sub;
    ros::Publisher drone_joint_control; 
    ros::Publisher err_rollPub; 
    ros::Publisher err_pitchPub; 
    ros::Publisher err_yawPub;
    pid_position_in.preverr_x=0;
    pid_position_in.preverr_y=0;
    pid_position_in.preverr_z=0; 
    pid_rotation_in.preverr_roll=0;
    pid_rotation_in.preverr_pitch=0;
    pid_rotation_in.preverr_yaw=0;
    drone_joint_control = control.advertise<std_msgs::Float64MultiArray>("/drone/joint_motor_controller/command",4);
    err_rollPub = control.advertise<std_msgs::Float32>("err_roll",1);
    err_pitchPub = control.advertise<std_msgs::Float32>("err_pitch",1);
    err_yawPub = control.advertise<std_msgs::Float32>("err_yaw",1);
    imu_sub = control.subscribe<sensor_msgs::Imu>("/drone/imu", 100, imu_callback);
    gazebo_model_state = control.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(control_drone, _1, drone_joint_control, err_rollPub, err_pitchPub, err_yawPub));
    ros::spin();
    return 0;
}