// OS Specific sleep
#include <unistd.h>

#include "serial/serial.h"
#include "clothoid_msgs/ublox_msgs.h"
#include "clothoid_msgs/nav_msgs.h"
#include "clothoid_msgs/clothoid_CAN.h"

#include "ros/ros.h"
#include <boost/thread.hpp>

#include <string>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <fstream>

#include <eigen3/Eigen/Dense>

#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*

#define PI 3.141592

//Earth parameter
#define R0 6378000   //Earth radius from equator
#define earth_e 0.016710219

using namespace std;
using namespace Eigen;

class NavNode{
    ros::NodeHandle nh;
    ros::Publisher ublox_pub, nav_pub;
    serial::Serial my_serial;

    boost::thread serial_grab;

    struct ublox_data
    {
        double time_tag;
        double lon, lat, hgt, heading;
        double gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature;

        int sat_num;

        int PVT_flag, RAW_flag, ORB_flag;

    };

    struct PCAN_data
    {
        double time_tag;
        double wheel_v_fl, wheel_v_fr, wheel_v_rl, wheel_v_rr;
        double steering_angle;

        int Vehicle_flag;

    };

    struct Nav_result
    {   //position
        double dot_lat, dot_lon, dot_hgt;		//delta position
        double dot_P_N, dot_P_E, dot_P_D;

        //velocity
        double dot_V_N, dot_V_E, dot_V_D;		//delta veocity
        double V_CG;

        //attitude
        double dot_q0, dot_q1, dot_q2, dot_q3;	//delta attitude(quternion)
        Quaterniond dot_quater;

        double dot_roll, dot_pitch, dot_yaw;//delta yawrate(for DR)

        Quaterniond quater;
        double roll, pitch, yaw;

        Vector3d d_theta;

        MatrixXd P_now;

    };


    struct Kalman_est
    {   //navigation error
        double error_P_N, error_P_E, error_P_D;

        double error_V_N, error_V_E, error_V_D;

        double error_roll, error_pitch, error_yaw;

        //sensor error
        double bias_gyro_x, bias_gyro_y, bias_gyro_z, bias_acc_x, bias_acc_y, bias_acc_z;

        double scale_gyro_x, scale_gyro_y, scale_gyro_z;

        MatrixXd P_now;


    };

    struct Nav_state
    {
        double time_tag;
        double lat, lon ,hgt;
        double P_N, P_E, P_D;
        double V_N, V_E, V_D;
        Quaterniond quater;
        double roll, pitch, yaw;
        double yaw_rate;

        MatrixXd P_now;

    };

    //struct for callback function to get_PCAN_message using global variance
    PCAN_data temp_PCAN_data;

    int start_flag;
    double bias_gyro_x, bias_gyro_y, bias_gyro_z;
    double bias_acc_x, bias_acc_y, bias_acc_z;

    double d_x, d_y, d_z;

    double var_gyro_x, var_gyro_y, var_gyro_z, var_z0 , var_z1, var_z2, var_z3;
    double predict_roll, predict_pitch, measure_roll, measure_pitch, est_roll, est_pitch;
    string line;

public:
    NavNode(ros::NodeHandle _nh, char *_port, int _baudrate):nh(_nh),my_serial(_port, _baudrate, serial::Timeout::simpleTimeout(1000))
    {
        ROS_INFO("ublox_node is trying to connect to serial port");

        if(my_serial.isOpen())
            ROS_INFO("ublox receiver is connected");

        ublox_pub = _nh.advertise<clothoid_msgs::ublox_msgs>("ublox_msgs",1);
        nav_pub = _nh.advertise<clothoid_msgs::nav_msgs>("nav_msgs",1);



        serial_grab = boost::thread(boost::bind(&NavNode::nav_filter, this));

    }
    ~NavNode()
    {
        serial_grab.interrupt();
        if(my_serial.isOpen())
            my_serial.close();

    }

    //main function
    void nav_filter()
    {
        d_x = 0.0;
        d_y = 0.0;
        d_z = 0.0;

        //set variance

        //Navigation system are calculate? check flag
        int navsys_flag_INS = 0;
        int navsys_flag_DR = 0;
        int navsys_flag_GPS = 0;

        //Navigation Result
        Nav_state nav_solution;

        double pretime_INS = ros::Time::now().sec + ros::Time::now().nsec/pow(10,-9);
        double pretime_DR = ros::Time::now().sec + ros::Time::now().nsec/pow(10,-9);

        double dt_INS = 0.0;
        double dt_DR = 0.0;

        start_flag = 1;
        bias_gyro_x = 0.0;
        bias_gyro_y = 0.0;
        bias_gyro_z = 0.0;
        bias_acc_x = 0.0;
        bias_acc_y = 0.0;
        bias_acc_z = 0.0;

        //Result structure of function
        ublox_data New_data_Ublox = {}, State_sensor_data;  //ublox message
        PCAN_data New_data_PCAN = {};    //PCAN message

        Nav_result update_INS = {}, update_DR = {}, pre_INS = {};  //update result of each navigation system

        Nav_state state_Nav = {};    //navigation solution from kalman filter(Result of navigation system)
        Nav_state state_INS = {}, state_DR = {}, state_GPS = {};

        ofstream resultdata_GPS("/home/sans/Desktop/Nav_result/result_GPS.txt");
        ofstream resultdata_INS("/home/sans/Desktop/Nav_result/result_INS.txt");
        ofstream resultdata_DR("/home/sans//Desktop/Nav_result/result_DR.txt");

        ofstream check_var("/home/sans//Desktop/Nav_result/check_var.txt");


        Kalman_est est_nav_error = {};

        //set initial estimate
        Kalman_est est_pre_error = {};

        //init position
        //        state_Nav.lat = 36.624989821002000;
        //        state_Nav.lon = 127.457842337097000;
        //        state_Nav.hgt = 91.612832657399900;

        //navigation loop
        while (1)
        {
            //get message
            New_data_Ublox = get_ublox_msgs();
            New_data_PCAN = get_PCAN_msgs();

            //ublox_message_sort
            //GPS
            if (New_data_Ublox.PVT_flag == 1)
            {
                state_GPS.lat = New_data_Ublox.lat;
                state_GPS.lon = New_data_Ublox.lon;
                state_GPS.hgt = New_data_Ublox.hgt;
                state_GPS.yaw = New_data_Ublox.heading;

                navsys_flag_GPS = 1;
                //cout << std::setprecision(15) << state_GPS.lat << " " << state_GPS.lon << " " << state_GPS.hgt << " " << state_GPS.yaw << endl;
                state_Nav.lat = state_GPS.lat;
                state_Nav.lon = state_GPS.lon;
                state_Nav.hgt = state_GPS.hgt;

                //WGS84 to ECEF
                //                state_Nav.P_N = ;
                //                state_Nav.P_E = ;
                //                state_Nav.P_D = ;

                State_sensor_data.lat = New_data_Ublox.lat;
                State_sensor_data.lon = New_data_Ublox.lon;
                State_sensor_data.hgt = New_data_Ublox.hgt;
                State_sensor_data.heading = New_data_Ublox.heading;

            }

            //INS
            if (New_data_Ublox.RAW_flag == 1)
            {
                State_sensor_data.acc_x = New_data_Ublox.acc_x;
                State_sensor_data.acc_y = New_data_Ublox.acc_y;
                State_sensor_data.acc_z = New_data_Ublox.acc_z;
                State_sensor_data.gyro_x = New_data_Ublox.gyro_x;
                State_sensor_data.gyro_y = New_data_Ublox.gyro_y;
                State_sensor_data.gyro_z = New_data_Ublox.gyro_z;
                State_sensor_data.temperature = New_data_Ublox.temperature;

                dt_INS = New_data_Ublox.time_tag - pretime_INS;

                if(start_flag)
                    dt_INS = 0.1;

                pretime_INS = New_data_Ublox.time_tag;	//pretime_INS update

                //INS function
                update_INS = INS_cal(New_data_Ublox, state_Nav, dt_INS);


                check_var << predict_roll << " " << predict_pitch << " " << measure_roll << " " << measure_pitch << " " << est_roll << " " << est_pitch << endl;

                //Navigation result update
                state_INS.time_tag = New_data_Ublox.time_tag;
                state_INS.quater = update_INS.quater;   //attitude

                Vector3d euler = quater2euler(update_INS.quater);
                state_INS.roll = update_INS.roll;   //euler(0);
                state_INS.pitch = update_INS.pitch; //euler(1);
                state_INS.P_now = update_INS.P_now;

                //save file
                resultdata_INS << std::setprecision(10) << state_INS.time_tag << " ";
                resultdata_INS << std::setprecision(10) << state_INS.quater.w() << " " << state_INS.quater.x() << " " << state_INS.quater.y() << " " << state_INS.quater.z() << endl;

                pre_INS = update_INS;   //save INS result

                navsys_flag_INS = 1;    //INS is calculated

                //for dr not kalman DR에서 필요한 피치, 요레이트 임시저장용(아직 칼만필터가 없으니까..)
                state_Nav.roll = state_INS.roll;
                state_Nav.pitch = state_INS.pitch;
                state_Nav.yaw = 0;
                state_Nav.quater = state_INS.quater;
                state_Nav.P_now = state_INS.P_now;

                //                cout << "roll, pitch test : " << state_Nav.roll << " " << state_Nav.pitch << endl;

            }

            if (New_data_Ublox.RAW_flag == 1)
            {
                State_sensor_data.sat_num = New_data_Ublox.sat_num;

            }
            //DR
            if (New_data_PCAN.Vehicle_flag)
            {
                dt_DR = New_data_PCAN.time_tag - pretime_DR;
                pretime_DR = New_data_PCAN.time_tag;

                //DR function
                update_DR = DR_cal(New_data_PCAN, state_Nav.pitch, state_Nav.yaw_rate, est_nav_error);

                //Navigation result update
                state_DR.P_N += update_DR.dot_P_N;
                state_DR.P_E += update_DR.dot_P_E;
                state_DR.P_D += update_DR.dot_P_D;

                state_DR.V_N = update_DR.dot_P_N;
                state_DR.V_E = update_DR.dot_P_E;
                state_DR.V_D = update_DR.dot_P_D;

                state_DR.yaw += update_DR.dot_yaw;
                state_DR.time_tag = New_data_PCAN.time_tag;

                state_Nav.P_N = state_DR.P_N;
                state_Nav.P_E = state_DR.P_E;
                state_Nav.P_D = state_DR.P_D;


                //save file
                resultdata_DR << state_DR.time_tag << " ";
                resultdata_DR << state_DR.P_N << " " << state_DR.P_E << " " << state_DR.P_D << " ";
                resultdata_DR << state_DR.V_N << " " << state_DR.V_E << " " << state_DR.V_D << " ";
                resultdata_DR << state_DR.yaw << endl;

                navsys_flag_DR = 1;
            }


            //            if (navsys_flag_GPS)
            //            {
            //                //Kalman filter function
            //                est_nav_error = Kalman_filter(state_Nav, state_INS, update_INS, state_DR, state_GPS, est_pre_error);

            //                est_pre_error = est_nav_error;

            //                //navsys flag initialization
            //                navsys_flag_GPS = 0;
            //                navsys_flag_INS = 0;
            //                navsys_flag_DR = 0;

            //            }

            if(New_data_Ublox.PVT_flag || New_data_Ublox.RAW_flag || New_data_Ublox.ORB_flag)
            {//ROS message publish
                clothoid_msgs::ublox_msgs ublox_msgs;
                ublox_msgs.header.frame_id="ublox";
                ublox_msgs.header.stamp = ros::Time::now();
                ublox_msgs.lon = State_sensor_data.lon;
                ublox_msgs.lat = State_sensor_data.lat;
                ublox_msgs.height = State_sensor_data.hgt;
                ublox_msgs.heading = State_sensor_data.heading;

                ublox_msgs.gyro_x = State_sensor_data.gyro_x;
                ublox_msgs.gyro_y = State_sensor_data.gyro_y;
                ublox_msgs.gyro_z = State_sensor_data.gyro_z;
                ublox_msgs.acc_x = State_sensor_data.acc_x;
                ublox_msgs.acc_y = State_sensor_data.acc_y;
                ublox_msgs.acc_z = State_sensor_data.acc_z;
                ublox_msgs.temperature = State_sensor_data.temperature;
                ublox_msgs.sat_num = State_sensor_data.sat_num;
                ublox_pub.publish(ublox_msgs);

            }


            if(navsys_flag_INS == 1)
            {
                //ROS message publish
                clothoid_msgs::nav_msgs nav_msgs;
                nav_msgs.header.frame_id="Navi result";
                nav_msgs.header.stamp = ros::Time::now();
                nav_msgs.P_N = state_Nav.P_N;
                nav_msgs.P_E = state_Nav.P_E;
                nav_msgs.P_D = state_Nav.P_D;
                nav_msgs.roll = state_Nav.roll;
                nav_msgs.pitch = state_Nav.pitch;
                nav_pub.publish(nav_msgs);

                navsys_flag_INS = 0;
            }

        }//end while

        //add file close
        resultdata_GPS.close();
        resultdata_INS.close();
        resultdata_DR.close();

        check_var.close();


    }//end nav_filter


    //Kalman filter function
    Kalman_est Kalman_filter(Nav_state state_Nav, Nav_state state_INS, Nav_result update_INS, Nav_state state_DR, Nav_state state_GPS, Kalman_est est_pre)
    {
        // Kalman filter function
        // input : Navigation now state, INS result, DR result, GPS result, pre_estimate Kalman filtered result, INS
        // output : Kalman_est struct

        Kalman_est est_new={};

        //        //0.Set system model
        //A matrix
        MatrixXd A = MatrixXd::Zero(18,18);

        //H matrix
        MatrixXd H = MatrixXd::Zero(5,8);


        //Q matrix
        MatrixXd Q = MatrixXd::Zero(8,8);

        //R matrix
        MatrixXd R = MatrixXd::Zero(5,5);

        //1.pridict estimated value & error covariance
        VectorXd est_pre_vec(8);

        //estimated value pridict
        VectorXd noise(8);

        for(int i=9;i<21;i++)
            noise(i) = rand()%1;  //적당한값으로 수정 필요해해
        VectorXd W_noise = Q*noise;

        VectorXd est_predict_vec = A*est_pre_vec + W_noise;

        //covariance matrix pridict
        MatrixXd P_predict = A*est_pre.P_now*A.transpose() + Q;

        //2.Calculate Kalman gain
        MatrixXd temp_for_kalman = H*P_predict*H.transpose() + R;
        MatrixXd Kalman_gain = P_predict*H.transpose()*temp_for_kalman.inverse();

        //3.Calculate estimated value
        VectorXd z(11);  //get measurement

        VectorXd est_new_vec = est_predict_vec + Kalman_gain*(z - H*est_predict_vec);

        //4.Calculate covariance matrix
        MatrixXd P = P_predict - (Kalman_gain*H*P_predict);

        //Final.return estimated values

        return est_new;

    }

    //DR navigation function
    Nav_result DR_cal(PCAN_data can_data, double pitch, double yawrate, Kalman_est est_err)
    {
        Nav_result update_DR;

        double len_cg2f = 1.0;  //length between CG and front wheel
        double len_cg2r = 1.5;  //length between CG and read wheel
        double scale_factor = 1/6;  //scale factor between steering angle and wheel angle

        double v_fl, v_fr, v_rl, v_rr, f_steering_angle;

        v_fl = can_data.wheel_v_fl;
        v_fr = can_data.wheel_v_fr;
        v_rl = can_data.wheel_v_rl;
        v_rr = can_data.wheel_v_rr;
        f_steering_angle = can_data.steering_angle;

        double pre_yawrate = yawrate;
        double V_CG = sqrt(((v_rl + v_rr)/2)*((v_rl + v_rr)/2) + (pre_yawrate*len_cg2r)*(pre_yawrate*len_cg2r));

        double f_wheel_angle = f_steering_angle * scale_factor;
        double slip_angle = atan2(len_cg2r * tan(f_steering_angle*PI/180),(len_cg2f + len_cg2r));

        double new_yawrate = (V_CG*cos(slip_angle*PI/180))*tan(f_wheel_angle*PI/180)/(len_cg2f + len_cg2r);

        update_DR.dot_yaw = new_yawrate;
        update_DR.dot_P_N = V_CG*cos(pitch*PI/180)*cos((new_yawrate + slip_angle)*PI/180);
        update_DR.dot_P_E = V_CG*cos(pitch*PI/180)*sin((new_yawrate + slip_angle)*PI/180);
        update_DR.dot_P_D = V_CG*sin(pitch*PI/180);
        update_DR.V_CG = V_CG;

        return(update_DR);

    }//end DR_cal

    //INS navigation function
    Nav_result INS_cal(ublox_data sensor_data, Nav_state state_now, double dt)
    {
        Nav_result update_INS;

        double gyro_x = sensor_data.gyro_x - bias_gyro_x;
        double gyro_y = sensor_data.gyro_y - bias_gyro_y;
        double gyro_z = sensor_data.gyro_z - bias_gyro_z;

        double acc_x = sensor_data.acc_x - bias_acc_x;
        double acc_y = sensor_data.acc_y - bias_acc_y;
        double acc_z = sensor_data.acc_z - bias_acc_z;

        Quaterniond z_now = state_now.quater;
        Quaterniond q_new;

        MatrixXd P_now = state_now.P_now;

        //Initialzation
        if(start_flag)
        {
            Vector3d euler(0,0,0);
            z_now = euler2quater(euler);
            cout << z_now.w() << " " << z_now.x() << " " << z_now.y() << " " << z_now.z() << endl;
            P_now = MatrixXd::Identity(4,4);

            bias_gyro_x = sensor_data.gyro_x;
            bias_gyro_y = sensor_data.gyro_y;
            bias_gyro_z = sensor_data.gyro_z;

            gyro_x = 0.0;
            gyro_y = 0.0;
            gyro_z = 0.0;

            bias_acc_x = sensor_data.acc_x;
            bias_acc_y = sensor_data.acc_y;
            bias_acc_z = sensor_data.acc_z - 9.780327;

            acc_x = 0.0;
            acc_y = 0.0;
            acc_z = 9.780327;

            start_flag = 0;
        }

        Vector3d gyro_measure(gyro_x, gyro_y, gyro_z);
        Vector3d rotate_earth(0.0041678*cos(state_now.lat*PI/180), 0, -0.0041678*sin(state_now.lat*PI/180));
        Matrix3d body2nav_transmatrix = z_now.toRotationMatrix();
        Vector3d gyro_vec = gyro_measure - body2nav_transmatrix*(rotate_earth);

        //        cout << "d_angular : " << gyro_vec(0)*dt << " " << gyro_vec(1)*dt << " " << gyro_vec(2)*dt << endl;
        d_x += gyro_vec(0)*dt;
        d_y += gyro_vec(1)*dt;
        d_z += gyro_vec(2)*dt;
        //        cout << "d_angle : " << d_x << " " << d_y << " " << d_z << endl;

        MatrixXd A;
        VectorXd dot_q_vec(4);
        //system model
        int model = 1;
        if(model==1)
        {
            double d_theta_2 = pow(gyro_vec(0)*dt,2) + pow(gyro_vec(1)*dt,2) * pow(gyro_vec(2)*dt,2);

            A = MatrixXd::Zero(4,4);
            A(0,0) = 1-d_theta_2/8;
            A(0,1) = -0.5*gyro_vec(0)*dt;
            A(0,2) = -0.5*gyro_vec(1)*dt;
            A(0,3) = -0.5*gyro_vec(2)*dt;

            A(1,0) = 0.5*gyro_vec(0)*dt;
            A(1,1) = 1-d_theta_2/8;
            A(1,2) = 0.5*gyro_vec(2)*dt;
            A(1,3) = -0.5*gyro_vec(1)*dt;

            A(2,0) = 0.5*gyro_vec(1)*dt;
            A(2,1) = -0.5*gyro_vec(2)*dt;
            A(2,2) = 1-d_theta_2/8;
            A(2,3) = 0.5*gyro_vec(0)*dt;

            A(3,0) = 0.5*gyro_vec(2)*dt;
            A(3,1) = 0.5*gyro_vec(1)*dt;
            A(3,2) = -0.5*gyro_vec(0)*dt;
            A(3,3) = 1-d_theta_2/8;
        }
        else if(model == 2)
        {
            Quaterniond q_gyro;
            q_gyro.w() = 0;
            q_gyro.x() = gyro_vec(0);
            q_gyro.y() = gyro_vec(1);
            q_gyro.z() = gyro_vec(2);

            Quaterniond dot_q = quatMult(z_now,q_gyro);

            dot_q.w() = dot_q.w()/2;
            dot_q.x() = dot_q.x()/2;
            dot_q.y() = dot_q.y()/2;
            dot_q.z() = dot_q.z()/2;

            //            dot_q.normalize();

            dot_q_vec(0) = dot_q.w();
            dot_q_vec(1) = dot_q.x();
            dot_q_vec(2) = dot_q.y();
            dot_q_vec(3) = dot_q.z();

            A = MatrixXd::Identity(4,4);
            A(0,1) = -0.5*gyro_vec(0)*dt;
            A(0,2) = -0.5*gyro_vec(1)*dt;
            A(0,3) = -0.5*gyro_vec(2)*dt;

            A(1,0) = 0.5*gyro_vec(0)*dt;
            A(1,2) = 0.5*gyro_vec(2)*dt;
            A(1,3) = -0.5*gyro_vec(1)*dt;

            A(2,0) = 0.5*gyro_vec(1)*dt;
            A(2,1) = -0.5*gyro_vec(2)*dt;
            A(2,3) = 0.5*gyro_vec(0)*dt;

            A(3,0) = 0.5*gyro_vec(2)*dt;
            A(3,1) = 0.5*gyro_vec(1)*dt;
            A(3,2) = -0.5*gyro_vec(0)*dt;

            //            cout << dot_q.w() << " " << dot_q.x() << " " << dot_q.y() << " " << dot_q.z() << endl;

        }
        MatrixXd temp_q_w(4,3);
        temp_q_w(0,0) = z_now.w();
        temp_q_w(0,1) = z_now.z();
        temp_q_w(0,2) = -z_now.y();

        temp_q_w(1,0) = -z_now.z();
        temp_q_w(1,1) = z_now.w();
        temp_q_w(1,2) = z_now.x();

        temp_q_w(2,0) = z_now.y();
        temp_q_w(2,1) = -z_now.x();
        temp_q_w(2,2) = z_now.w();

        temp_q_w(3,0) = -z_now.x();
        temp_q_w(3,1) = -z_now.y();
        temp_q_w(3,2) = -z_now.z();

        double x_rand = 0.0029*((double)(rand()%200)/100-1);
        double y_rand = 0.0036*((double)(rand()%200)/100-1);
        double z_rand = 0.0025*((double)(rand()%200)/100-1);
        Vector3d g_w(x_rand, y_rand, z_rand);

        VectorXd q_w = -(dt/2)*temp_q_w*g_w;

        Matrix3d sigma_g = MatrixXd::Identity(3,3);
        sigma_g(0,0) = 0.0029;   //SANS lab의 Ublox-EVK-M8L기준
        sigma_g(1,1) = 0.0036;
        sigma_g(2,2) = 0.0025;

        MatrixXd Q = (dt/2)*(dt/2)*temp_q_w*sigma_g*temp_q_w.transpose();

        MatrixXd H = MatrixXd::Identity(4,4);

        //측정치 쿼터니언의 스칼라값이 이상하게 많이 튄다?
        MatrixXd R = MatrixXd::Identity(4,4);
        R(0,0) = 0.1955;            //SANS lab의 Ublox-EVK-M8L기준
        R(1,1) = 0.000018864;
        R(2,2) = 0.0000100788;
        R(3,3) = 0.2332;


        //set predict value
        VectorXd q_now_vec(4);
        q_now_vec(0) = z_now.w();
        q_now_vec(1) = z_now.x();
        q_now_vec(2) = z_now.y();
        q_now_vec(3) = z_now.z();

        //set obervation measurement z[k]
        MatrixXd jaco_f(3,4);
        jaco_f(0,0) = -z_now.y();
        jaco_f(0,1) = z_now.z();
        jaco_f(0,2) = -z_now.w();
        jaco_f(0,3) = z_now.x();

        jaco_f(1,0) = z_now.x();
        jaco_f(1,1) = z_now.w();
        jaco_f(1,2) = z_now.z();
        jaco_f(1,3) = z_now.y();

        jaco_f(2,0) = z_now.w();
        jaco_f(2,1) = -z_now.x();
        jaco_f(2,2) = -z_now.y();
        jaco_f(2,3) = z_now.z();

        Vector3d g_vec(0,0,1);
        Vector3d a_vec(acc_x, acc_y, acc_z);
        double norm_a_vec = sqrt(pow(acc_x,2) + pow(acc_z,2) + pow(acc_y,2));
        a_vec = a_vec/norm_a_vec;

        double mu0 = 0.001;
        double beta = 0.01;
        double gyro_norm = sqrt(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
        double mu = mu0 + beta*gyro_norm*dt;

        Quaterniond z_new_q;
        Vector3d euler_z;
        VectorXd z_new = q_now_vec;
        for(int j=0; j<1000; j++)
        {   z_now.w() = z_new(0);
            z_now.x() = z_new(1);
            z_now.y() = z_new(2);
            z_now.z() = z_new(3);

            Vector3d f = z_now.toRotationMatrix().transpose()*g_vec - a_vec;
            //                        Vector3d f = MatrixXd::Identity(3,3)*g_vec - a_vec;
            VectorXd del_f = 2*jaco_f.transpose()*f;

            double norm_del_f = sqrt(pow(del_f(0),2) + pow(del_f(1),2) + pow(del_f(2),2) + pow(del_f(3),2));

            if(f(0)=0 || f(1)==0 || f(2)==0 )
                continue;

            z_new = z_new - mu*del_f/norm_del_f;

            if(j%100 == 0)
            {
                //                                cout << j << "    " << "f : " << f(0) << " " << f(1) << " " << f(2) << endl;
                //                                cout << "       z_new " << z_new(0) << " " << z_new(1) << " " << z_new(2) << " " << z_new(3) << endl;

                z_new_q;
                z_new_q.w() = z_new(0);
                z_new_q.x() = z_new(1);
                z_new_q.y() = z_new(2);
                z_new_q.z() = z_new(3);
                euler_z = quater2euler(z_new_q);
                //                                cout << "euler_z " << euler_z(0) << " " << euler_z(1) << endl;

            }
        }

        //        cout << z_new(0) - q_now_vec(0) << " " << z_new(1) - q_now_vec(1) << " "<< z_new(2) - q_now_vec(2) << " " << z_new(3) - q_now_vec(3) << endl;

        var_gyro_x = gyro_vec(0);
        var_gyro_y = gyro_vec(1);
        var_gyro_z = gyro_vec(2);
        var_z0 = z_new(0);
        var_z1 = z_new(1);
        var_z2 = z_new(2);
        var_z3 = z_new(3);

        //        cout << "before normalization :  " << z_new(0) << " " << z_new(1) << " " << z_new(2) << " " << z_new(3) << endl;

        double z_new_norm = sqrt(z_new(0)*z_new(0) + z_new(1)*z_new(1) + z_new(2)*z_new(2) + z_new(3)*z_new(3));
        z_new = z_new/z_new_norm;

        //        z_new_q.normalize();
        //        z_new(0) = z_new_q.w();
        //        z_new(1) = z_new_q.x();
        //        z_new(2) = z_new_q.y();
        //        z_new(3) = z_new_q.z();

        //        cout << "after normalization :  " << z_new(0) << " " << z_new(1) << " " << z_new(2) << " " << z_new(3) << endl;

        update_INS.roll = euler_z(0);
        update_INS.pitch = euler_z(1);

        //Kalman filter
        //predict
        VectorXd q_new_predict;
        if(model == 1)
            q_new_predict = A*q_now_vec + q_w;
        else if(model == 2)
            q_new_predict = q_now_vec + dot_q_vec*dt;

        Quaterniond q_predict;
        q_predict.w() = q_new_predict(0);
        q_predict.x() = q_new_predict(1);
        q_predict.y() = q_new_predict(2);
        q_predict.z() = q_new_predict(3);
        Vector3d euler_predict = quater2euler(q_predict);
        //        cout << "euler predict : " << euler_predict(0) << " " << euler_predict(1) << endl;
        //        cout << q_new_predict(0) << " " << q_new_predict(1) << " " << q_new_predict(2) << " " << q_new_predict(3) << endl;
        //        cout << endl << A << endl << endl;
        //        cout << q_w(0) << " " << q_w(1) << " " << q_w(2) << " " << q_w(3) << endl;

        MatrixXd P_predict = A*P_now*A.transpose();

        //        cout << P_now << endl << endl;
        //        cout << P_predict << endl << endl;

        //cal Kalman gain
        MatrixXd temp_4K = (H*P_predict*H + R);
        MatrixXd K_new = P_predict*H*temp_4K.inverse();

        //        cout << P_predict << endl << endl;
        //        cout << temp_4K << endl;
        //        cout << K_new << endl << endl;

        //estmate
        VectorXd q_new_vec = q_new_predict + K_new*(z_new - H*q_new_predict);
        MatrixXd P_new = (MatrixXd::Identity(4,4) - K_new*H)*P_predict;

        //        cout << K_new*(z_new - H*q_new_predict) << endl << endl;
        //        cout << P_new << endl << endl;

        q_new.w() = q_new_vec(0);
        q_new.x() = q_new_vec(1);
        q_new.y() = q_new_vec(2);
        q_new.z() = q_new_vec(3);

        q_new.normalize();


        //        cout << q_new_vec(0) << " " << q_new_vec(1) << " " << q_new_vec(2) << " " << q_new_vec(3) << endl;
        Vector3d euler_new = quater2euler(q_new);
        //        cout << ros::Time::now().sec << " " << euler_new(0) << " " << euler_new(1) << endl;

        //result update
        update_INS.quater = q_new;
        update_INS.P_now = P_new;

        update_INS.dot_yaw = gyro_vec(2)*dt;

        predict_roll = euler_predict(0);
        predict_pitch = euler_predict(1);
        measure_roll = euler_z(0);
        measure_pitch = euler_z(1);
        est_roll = euler_new(0);
        est_pitch = euler_new(1);


        cout << predict_roll << " " << predict_pitch << " " << measure_roll << " " << measure_pitch << " " << est_roll << " " << est_pitch << endl;



        return(update_INS);



    }//end INS_cal

    //Euler to Quaternion
    Quaterniond euler2quater(Vector3d Euler)
    {
        Quaterniond result;
        //XYZ wiki method
        result.w() = cos(Euler(0)/2*PI/180)*cos(Euler(1)/2*PI/180)*cos(Euler(2)/2*PI/180) + sin(Euler(0)/2*PI/180)*sin(Euler(1)/2*PI/180)*sin(Euler(2)/2*PI/180);
        result.x() = sin(Euler(0)/2*PI/180)*cos(Euler(1)/2*PI/180)*cos(Euler(2)/2*PI/180) - cos(Euler(0)/2*PI/180)*sin(Euler(1)/2*PI/180)*sin(Euler(2)/2*PI/180);
        result.y() = cos(Euler(0)/2*PI/180)*sin(Euler(1)/2*PI/180)*cos(Euler(2)/2*PI/180) + sin(Euler(0)/2*PI/180)*cos(Euler(1)/2*PI/180)*sin(Euler(2)/2*PI/180);
        result.z() = cos(Euler(0)/2*PI/180)*cos(Euler(1)/2*PI/180)*sin(Euler(2)/2*PI/180) - sin(Euler(0)/2*PI/180)*sin(Euler(1)/2*PI/180)*cos(Euler(2)/2*PI/180);

        return result;
    }

    //Quaternion to Euler
    Vector3d quater2euler(Quaterniond q)
    {
        Vector3d result;

        q.normalize();
        result(0) = atan2(2*(q.w()*q.x() + q.y()*q.z()),1-2*(pow(q.x(),2)+pow(q.y(),2))) * 180/PI;
        result(1) = asin(2*(q.w()*q.y() - q.z()*q.x())) * 180/PI;
        result(2) = atan2(2*(q.w()*q.z() + q.x()*q.y()),1-2*(pow(q.y(),2)+pow(q.z(),2))) * 180/PI;

        if(isnan(result(0)) || isnan(result(1)) || isnan(result(2)))
            //            cout << "quater to euler has nan " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;

            return result;
    }

    //PCAN message callback function -- ???;;;;;
    void msgCallback(const clothoid_msgs::clothoid_CAN::ConstPtr& msg)
    {
        PCAN_data new_data;

        clothoid_msgs::clothoid_CAN temp_msg;

        new_data.time_tag = ros::Time::now().sec + ros::Time::now().nsec*pow(10,-9);

        new_data.steering_angle = temp_msg.Gway_Steering_Angle;
        new_data.wheel_v_fr = temp_msg.Gway_Wheel_Velocity_FL;
        new_data.wheel_v_rl = temp_msg.Gway_Wheel_Velocity_FR;
        new_data.wheel_v_rr = temp_msg.Gway_Wheel_Velocity_RL;
        new_data.wheel_v_fl = temp_msg.Gway_Wheel_Velocity_RR;

        new_data.Vehicle_flag = 1;

        new_data = temp_PCAN_data;  //to global struct

    }


    PCAN_data get_PCAN_msgs()
    {
        PCAN_data new_data = {};
        //메세지좀 읽어오자
        //ros::Subscriber PCAN_sub = nh.subscribe("clothoid_CAN", 1000, msgCallback);

        new_data = temp_PCAN_data;

        return(new_data);
    }//end get_PCAN_msgs


    ublox_data get_ublox_msgs()
    {
        //ublox message variable
        ublox_data new_data={};
        uint8_t temp;

        int lon, lat, hgt,heading;
        ulong gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature;

        double sec, nano;

        new_data.PVT_flag = 0;
        new_data.RAW_flag = 0;

        //Find message
        my_serial.read(&temp,1);
        if(temp == 0xB5){
            my_serial.read(&temp,1);
            if(temp == 0x62){
                //cout << "header clear" << endl;
                string classid = my_serial.read(4);

                sec = ros::Time::now().sec;
                nano = ros::Time::now().nsec*pow(10,-9);

                new_data.time_tag = sec+nano;

                //cheksum check
                uint8_t CK_A=0, CK_B=0;
                for(int i=0;i<56;i++)
                {
                    CK_A = CK_A + (uint8_t)line.c_str()[i];
                    CK_B = CK_B + CK_A;
                }

                //1.UBX-NV-PVT message
                if((uint8_t)classid.c_str()[0]==0x01 && (uint8_t)classid.c_str()[1]==0x07)//check class and id
                {
                    line = my_serial.read(54);

                    //cout << "UBX-NAV-PVT : " << ros::Time::now().sec << "." << new_data.time_tag << endl;

                    lon = ((uint8_t)line.c_str()[24] | (uint8_t)line.c_str()[25] << 8 | (uint8_t)line.c_str()[26] << 16 | (uint8_t)line.c_str()[27] << 24);
                    lat = ((uint8_t)line.c_str()[28] | (uint8_t)line.c_str()[29] << 8 | (uint8_t)line.c_str()[30] << 16 | (uint8_t)line.c_str()[31] << 24);
                    hgt = ((uint8_t)line.c_str()[32] | (uint8_t)line.c_str()[33] << 8 | (uint8_t)line.c_str()[34] << 16 | (uint8_t)line.c_str()[35] << 24);
                    heading = ((uint8_t)line.c_str()[84] | (uint8_t)line.c_str()[85] << 8 | (uint8_t)line.c_str()[86] << 16 | (uint8_t)line.c_str()[87] << 24);

                    new_data.lon = (double)lon/10000000;
                    new_data.lat = (double)lat/10000000;
                    new_data.hgt = (double)hgt/1000;
                    new_data.heading = (double)heading/100000;

                    new_data.PVT_flag = 1;
                }//end class & id if

                //2.UBX-ESF-RAW message
                else if((uint8_t)classid.c_str()[0]==0x10 && (uint8_t)classid.c_str()[1]==0x03)//check class and id
                {
                    //cout << "UBX-ESF-Raw : " << ros::Time::now().sec << "." << new_data.time_tag << endl;
                    int data_length = (uint8_t)classid.c_str()[2] | (uint8_t)classid.c_str()[3]<<8;

                    string line = my_serial.read(data_length);

                    for(int iteration = 0; iteration < (data_length-4)/8 ; iteration++)
                    {
                        string raw_data = line;

                        int data = (uint8_t)raw_data.c_str()[8*iteration +4] | (uint8_t)raw_data.c_str()[8*iteration +5]<<8 | (uint8_t)raw_data.c_str()[8*iteration +6]<<16;
                        ushort data_type = (uint8_t)raw_data.c_str()[8*iteration +7];
                        ulong time_tag = (uint8_t)raw_data.c_str()[8*iteration + 8] | (uint8_t)raw_data.c_str()[8*iteration + 9]<<8 | (uint8_t)raw_data.c_str()[8*iteration + 10]<<16 | (uint8_t)raw_data.c_str()[8*iteration + 11]<<24;

                        //2's complement
                        int check_bit = data>>23;
                        if(check_bit == 1)
                        {    data = 0x00FFFFFF & ~(data - 1);
                            data = -1*data;
                        }

                        switch(data_type)
                        {
                        case 5: new_data.gyro_z = data / 4096.0; break;
                        case 13: new_data.gyro_y = data / 4096.0; break;
                        case 14: new_data.gyro_x = data / 4096.0; break;
                        case 16: new_data.acc_x = data / 1024.0; break;
                        case 17: new_data.acc_y = data / 1024.0; break;
                        case 18: new_data.acc_z = data / 1024.0; break;
                        case 12: new_data.temperature = data / 100.0; break;

                        }//end switch
                    }//end for iteration
                    new_data.RAW_flag = 1;

                }//end class & id if

                //3.UBX-NAV-ORB message
                else if((uint8_t)classid.c_str()[0]==0x01 && (uint8_t)classid.c_str()[1]==0x34)//check class and id
                {
                    int data_length = (uint8_t)classid.c_str()[2] | (uint8_t)classid.c_str()[3]<<8;

                    string line = my_serial.read(data_length);

                    //cout << "UBX-NAV-ORB : " << ros::Time::now().sec << "." << new_data.time_tag << endl;
                    new_data.sat_num = (uint8_t)line[5 + 4];
                    new_data.ORB_flag = 1;

                }//end class & id if



            }//end second header
        }// end first header

        //cout << new_data.time_tag << " " << new_data.lat << " " << new_data.acc_x << " " << new_data.sat_num << endl;

        return(new_data);

    }//end ublox_msgs



    //Quaternion Multiplication
    Quaterniond quatMult(Quaterniond q1, Quaterniond q2)
    {
        Eigen::Quaterniond resultQ;
        resultQ.setIdentity();

        resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
        resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

        return resultQ;
    }

};//end class NavNode


int main(int argc, char* argv[])
{
    cout << "ublox_node run" << endl;

    ros::init(argc, argv, "ublox_node");
    ros::NodeHandle nh;
    if(argc != 3)
    {
        ROS_ERROR("Usage : rosrun ublox ublox_node [port_name] [baudrate]");
        return -1;
    }

    cout << "start" << endl;

    NavNode kn(nh, argv[1], atoi(argv[2]));

    ros::spin();

    return 0;
}
