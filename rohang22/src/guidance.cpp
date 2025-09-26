#include "guidance.h"

double get_angle(std::vector<double> start, std::vector<double> end)
{
    // +y축으로부터 직선에 대한 CW방향의 radian 각도 return(0~2*pi)
    // Input : 
    // start, end : 2차원 좌표

    std::vector<double> vec = eminus(start, end);
    double angle = atan2(vec[0], vec[1]) + M_PI;
    if(angle > 2*M_PI){
        angle -= 2*M_PI;
    }
    return angle;

}

std::vector<double> line_guidance(std::vector<double> start, std::vector<double> end, std::vector<double> local, double step)
{
    // 직선 경로 유도
    // Input 
    // start: 경로 시작점 좌표, end: 경로 끝점 좌표, local: 현재 위치 좌표, step: 이번 주기 setpoint 찍을 거리
    // Output
    // 이번 주기에서 목표로 할 setpoint 2차원 좌표

    double psi_path = get_angle(start, end);
    double psi_vehicle = get_angle(local, end);
    double dist_err = norm(eminus(local, end))*sin(psi_path - psi_vehicle);

    // double psi_err = - KP*atan(dist_err)*(M_PI/180);
    double psi_err = - KP*dist_err*(M_PI/180);
    // while(psi_err < -M_PI/2)
    //     psi_err += M_PI;
    // while(psi_err > M_PI/2)
    //     psi_err -= M_PI;

    double psi_control = psi_path + psi_err;

    // 0~360 deg to -180~180 deg
    if(psi_control > 2*M_PI) psi_control = psi_control - 2*M_PI;
    else if (psi_control < 0) psi_control = psi_control + 2*M_PI;
    if(psi_control >= M_PI) psi_control = psi_control - 2*M_PI;

    return {step*sin(psi_control), step*cos(psi_control)};
}

std::vector<double> circle_guidance(std::vector<double> center, double radius, double direc, std::vector<double> local, double step)
{
    // 원형 경로 유도
    // Input 
    // center: 선회 원 중심, radius: 선회 원 반경, direc: 선회방향(CW, CCW), local: 현재 위치 좌표, step: 이번 주기 setpoint 찍을 거리
    // Output
    // 이번 주기에서 목표로 할 setpoint 2차원 좌표

    double psi_vehicle = get_angle(local, center);
    double dist_center = norm(eminus(local, center));
    double psi_s;
    if(dist_center > radius){
        psi_s = atan2(radius, sqrt(abs(pow(dist_center, 2)-pow(radius, 2))));
    } else {
        psi_s = atan2(radius, -sqrt(abs(pow(dist_center, 2)-pow(radius, 2))));
    }
    if(direc == CW){
        psi_s = - psi_s;
    }
    double psi_control = psi_vehicle + psi_s;
    // ROS_INFO("guid angle: %f %f %f", psi_control, psi_vehicle, psi_s);

    // 0~360 deg to -180~180 deg
    if(psi_control > 2*M_PI) psi_control = psi_control - 2*M_PI;
    else if (psi_control < 0) psi_control = psi_control + 2*M_PI;
    if(psi_control >= M_PI) psi_control = psi_control - 2*M_PI;

    return {step*sin(psi_control), step*cos(psi_control)};
}
