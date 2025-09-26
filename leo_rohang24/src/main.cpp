/**
 * 2022 로봇항공기경진대회 초급부문
 * 건국대학교 메카트론
 * VTOL 경로 비행 코드
 * 
 * - 2024 rohang lidar test 
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandVtolTransition.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <vector>

#include "algebra.h"
#include "config.h"
#include "control.h"
#include "frame.h"
#include "global_def.h"
#include "guidance.h"
#include "mission.h"

int current_flight_mode = MC;
unsigned long long timestamp;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pose;
void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pose = *msg;
}

sensor_msgs::NavSatFix global_pose;
void global_pose_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    global_pose = *msg;
    timestamp = global_pose.header.stamp.sec*1000000 + global_pose.header.stamp.nsec/1000;
}

mavros_msgs::ExtendedState current_extended_state;
void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    current_extended_state = *msg;
}

sensor_msgs::TimeReference time_reference;
unsigned long utc_timestamp;
void time_ref_cb(const sensor_msgs::TimeReference::ConstPtr &msg)
{
    time_reference = *msg;
    utc_timestamp = time_reference.time_ref.toNSec()/1000000;
}

// lidar variable
std::vector<double> lidar_range;
std::vector<double> lidar_angle;

sensor_msgs::LaserScan lidar_data;
void LidarMsgCallback(const sensor_msgs::LaserScan::ConstPtr &data)
{
    lidar_data = *data;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "rohang22");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, local_pose_cb);
    ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_pose_cb);
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 1, extended_state_cb);
    ros::Subscriber time_reference_sub = nh.subscribe<sensor_msgs::TimeReference>("mavros/time_reference", 1, time_ref_cb);

    ros::Subscriber msg_subs = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, &LidarMsgCallback);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    ros::ServiceClient vtol_transition_client = nh.serviceClient<mavros_msgs::CommandVtolTransition>("mavros/cmd/vtol_transition");

    ros::Publisher set_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_FREQUENCY_HZ);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    // PX4 native flight stack: 
    // MANUAL,ACRO,ALTCTL,POSCTL,OFFBOARD,STABILIZED,RATTITUDE,AUTO.MISSION,AUTO.LOITER,AUTO.RTL,AUTO.LAND,AUTO.RTGS,AUTO.READY,AUTO.TAKEOFF


    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL tol_cmd;

    mavros_msgs::CommandVtolTransition vtol_state;
    vtol_state.request.state = MC;
    
    geometry_msgs::PoseStamped pose;
    pose.pose = local_pose.pose;
    set_heading(pose, get_current_heading(local_pose));

    //send a few setpoints before starting
    for(int i = 50; ros::ok() && i > 0; --i){
        set_pose_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    std::vector<double> home_local = {local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z};
    std::vector<double> home_global = {global_pose.latitude, global_pose.longitude, global_pose.altitude};
    if(FRAME_MODE == LLH){
        ROS_INFO("global: %f, %f, %f", home_global[0], home_global[1], home_global[2]);
        ROS_INFO("local: %f, %f, %f", home_local[0], home_local[1], home_local[2]);
        for(int i=0; i<2; i++)
	    {
	        WPT[i][2] = WPT[i][2] + home_global[2];
	    }
        mission_llh2enu(WPT, home_global);
        mission_calib_local(WPT, home_local);
    }

    
    int process = 0;
    int i = 0;
    
    bool hold_flag = false;
    bool hold_flag2 = false;

    float step = MC_SPEED;
    double h_err = MC_HORIZONTAL_ERROR;
    double v_err = MC_VERTICAL_ERROR;
    double a_err = HEADING_ERROR;

    std::vector<double> start;
    std::vector<double> end;
    std::vector<double> center;
    std::vector<double> local;
    std::vector<double> local3;
    std::vector<double> obstacle = {0., 0.};
    std::vector<double> corr_alt;
    double avoid_radius;
    // std::vector<double> obstacle_fix;
    std::vector<double> set = {0, 0, 0};
    double heading;

    std::vector<double> vec_i;
    std::vector<double> vec_f;
    std::vector<double> vec_c;

    ROS_INFO("Mission Start");

    while (ros::ok())
    {
        // default setpoint signal
        update_posestamp_header(pose);
        set_position(pose, {local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z});

        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(1.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(1.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }

        //// processing lidar data
        unsigned int lens = static_cast<unsigned int>((lidar_data.angle_max - lidar_data.angle_min) / lidar_data.angle_increment);
        int lidar_len = 0;
        for (unsigned int i = 0; i < lens; i++){
            if (!isnan(lidar_data.ranges[i]))
            {
                lidar_len++;
            }
        }

        lidar_range.assign(lidar_len, std::numeric_limits<double>::quiet_NaN());
        lidar_angle.assign(lidar_len, std::numeric_limits<double>::quiet_NaN());
        int j = 0;
        for (unsigned int i = 0; i < lens; i++)
        {
            if(isnan(lidar_data.ranges[i]))
            {
                continue;
            }
            if(j > lidar_len)
            {
                break;
            }

            lidar_range[j] = lidar_data.ranges[i];
            lidar_angle[j] = lidar_data.angle_min + i * lidar_data.angle_increment;
            j++;
        }
        ////

        if(current_state.armed && current_state.mode == "OFFBOARD"){ 
            switch(process)
            {
            case 0:
                // i=0; Takeoff
                set = {local_pose.pose.position.x, local_pose.pose.position.y, WPT[i][2]};
                set_position(pose, set);
                set_heading(pose, get_current_heading(local_pose));

                if(is_arrived_verti(local_pose, WPT[i], v_err)){
                    if(hold_flag == false && hold(1))
                        hold_flag = true;
                    if(hold_flag){
                        start = {WPT[i][0], WPT[i][1]};
                        end = {WPT[i+1][0], WPT[i+1][1]};
                        heading = get_angle(start, end);
                        set_heading(pose, heading);

                        if(is_arrived_direc(local_pose, heading, a_err)){
                            if(hold_flag2 == false && hold(1))
                                hold_flag2 = true;
                            if(hold_flag2){
                            ROS_INFO("Take off Complete");
                            ROS_INFO("==================== move to BASE ");
                            process++; // 다음 단계
                            // i++;

                            hold_flag = false;
                            hold_flag2 = false;

                            // save_timestamp(FILE_PATH, log_index++, timestamp); // 2: move to WPT1
                            }
                        }
                    }
                }    

                break;


            case 1: // to BASE
                // i=0; WPT#0
                start = {local_pose.pose.position.x, local_pose.pose.position.y};
                end = {WPT[i][0], WPT[i][1]};
                local = {local_pose.pose.position.x, local_pose.pose.position.y};
                set = line_guidance(start, end, local, step);
                set = {local_pose.pose.position.x + set[0], local_pose.pose.position.y + set[1], WPT[i][2]};
                set_position(pose, set);

                if(is_arrived_hori(local_pose, WPT[i], h_err) && is_arrived_verti(local_pose, WPT[i], v_err)){
                    if(hold_flag == false)
                        hold_flag = true;
                    if(hold_flag){
                        start = {WPT[i][0], WPT[i][1]};
                        end = {WPT[i+1][0], WPT[i+1][1]};
                        heading = get_angle(start, end);
                        set_heading(pose, heading);

                        if(is_arrived_direc(local_pose, heading, a_err)){
                            if(hold_flag2 == false)
                                hold_flag2 = true;
                            if(hold_flag2){
                                // ROS_INFO("Take off Complete");
                                ROS_INFO("==================== move to WPT1 ");
                                process++; // 다음 단계
                                i++; // 미션 웨이포인트 인덱스 1 높임

                                hold_flag = false;
                                hold_flag2 = false;

                                // save_timestamp(FILE_PATH, log_index++, timestamp); // 2: move to WPT1
                            }
                        }
                    }
                }
                break;


            case 2:
                // i=1; start -> end | obstacle avoidance |
                start = {local_pose.pose.position.x, local_pose.pose.position.y}; //{WPT[i-1][0], WPT[i-1][1]};
                end = {WPT[i][0], WPT[i][1]};
                local = {local_pose.pose.position.x, local_pose.pose.position.y};
                local3 = {local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z}; 
                set = {0, 0}; 

                // find minimum lidar_range and it's index
                avoid_radius = 4;
                int direction;
                int idx_min;
                double range_min;

                static bool once_flag = false;
                static bool obstacle_flag = false;
                static int nan_count = 0;
                
                if(lidar_range.size() != 0){
                    ROS_INFO("============= LIDAR DATA IN ============");
                    ROS_INFO("Lidar Data Size: %d", lidar_range.size());
                    
                    nan_count = 0;  // initialize

                    idx_min = 0;
                    range_min = lidar_range[0];
                    for(int i=1; i<lidar_range.size(); i++){
                        if(range_min > lidar_range[i]){
                            range_min = lidar_range[i];
                            idx_min = i;
                        }
                    }

                    ROS_INFO("Lidar Min Range: %.2f", range_min);
                    ROS_INFO("Lidar Min Angle: %.2f", lidar_angle[idx_min]*180/M_PI);

                    if(lidar_range.size() > 9){
                        obstacle_flag = true;
                    }
                }
                else
                {
                    nan_count++;
                }

                if(obstacle_flag && once_flag){
                    // r,theta to ENU
                    // theta [rad], y축 기준 시계 방향  
                    ROS_INFO("=====================================");
                    ROS_INFO("=====================================");
                    ROS_INFO("=========Obstacle DETECTED!==========");
                    ROS_INFO("=====================================");
                    ROS_INFO("=====================================");
                    double theta = get_current_heading(local_pose) + lidar_angle[idx_min];
                    ROS_INFO("Obstacle Theta: %.2f",theta*180/M_PI);

                    obstacle[0] = range_min*sin(theta);
                    obstacle[1] = range_min*cos(theta);
                    obstacle = {local[0] + obstacle[0], local[1] + obstacle[1]};
                    ROS_INFO("Obstacle Point: %.2f, %.2f", obstacle[0], obstacle[1]);

                    std::vector<double> target_dir = eminus(end, local);
                    std::vector<double> obstacle_dir = eminus(obstacle, local);
                    double dir = target_dir[0]*obstacle_dir[1] - target_dir[1]*obstacle_dir[0]; // 외적 계산임
                    if(dir > 0) // 장애물이 경로상 우측에 있으면 반시계 방향으로 회피
                        direction = CCW;
                    else // 장애물이 경로상 좌측에 있으면 시계 방향으로 회피
                        direction = CW;

                    // obstacle_flag = false;
                    once_flag = false;
                }

                if(nan_count >= 20){   // no data for 2 seconds
                    ROS_INFO("-------------- Avoidance COMPLETE, Line --------------");
                    set = line_guidance(start, end, local, step);
                    obstacle_flag = false;
                }
                else if(obstacle_flag && nan_count < 20)
                {
                    ROS_INFO("----------- ObstacleAvoid - Circle -----------");
                    set = circle_guidance(obstacle, avoid_radius, direction, local, step);
                    ROS_INFO("Obstacle Point: %.2f, %.2f", obstacle[0], obstacle[1]);
                    // 장애물 좌표 중심으로 원형 회피
                }
                else if(obstacle_flag == false && nan_count < 20){
                    ROS_INFO("-------------- NO DATA, Line --------------");
                    set = line_guidance(start, end, local, step);
                }

                // set = obstacle_avoidance(start, end, local, step, obstacle, avoid_radius);

                corr_alt = corridor_alt(WPT[i-1], WPT[i], local3);

                set = {local_pose.pose.position.x + set[0], local_pose.pose.position.y + set[1], WPT[i-1][2] + corr_alt[2]}; // 하강률 다소 낮게 잡는게 좋을듯
                if(set[2] < WPT[i][2]){
                    set[2] = WPT[i][2];
                }

                set_position(pose, set);
                set_heading(pose, get_angle({WPT[i-1][0], WPT[i-1][1]}, end));
                
                if(is_arrived_hori(local_pose, WPT[i], h_err) && is_arrived_verti(local_pose, WPT[i], v_err) || remain_dist(local_pose, WPT[i]) <= 0.5 )
                { // 목표 위치 수평 지점 도착 후 고도 맞추기
                    set_position(pose, WPT[i]);
                    set_heading(pose, get_angle({WPT[i-1][0], WPT[i-1][1]}, end));
                    // hold(2);

                    if(hold_flag == false && hold(3))
                        hold_flag = true;
                    if(hold_flag){
                        ROS_INFO("Arrived at end..");
                        process++;

                        hold_flag = false;
                        hold_flag2 = false;
                    }
                }
                // std::cout << "13" << std::endl;
                break;

            case 3: // Land
                
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                if(set_mode_client.call(offb_set_mode) &&
                   offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Auto Land enabled");
                    process++;
                }
                break;
            }
        }
        // Update my pose
        set_pose_pub.publish(pose);
        // std::cout << "14" << std::endl;
        double remain = remain_dist(local_pose, WPT[i]);
        ROS_INFO("Timestamp: %lld", timestamp);
        ROS_INFO("Current: %.2f, %.2f, %.2f", local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z);
        ROS_INFO("Setpoint: %.2f, %.2f, %.2f", set[0], set[1], set[2]);
        ROS_INFO("Remain Dist: %f", remain);
        ROS_INFO("i: %d", i);
        ROS_INFO("\n");

        // std::cout << "15" << std::endl;
        ros::spinOnce();
        rate.sleep();
        // std::cout << "16" << std::endl;
    }
}
