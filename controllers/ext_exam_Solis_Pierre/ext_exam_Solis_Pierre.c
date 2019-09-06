#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP 64
#define PI 3.1415
#define OBSTACLE_DISTANCE .17

double encoder_L1,encoder_L1_dif;
double encoder_L2, encoder_L2_dif;
double encoder_R1, encoder_R1_dif;
double encoder_R2, encoder_R2_dif;
double dis_value_obs;
double dis_radar;
double dis_sen_obs;
double encoder_left;
double encoder_right;
double value_cm_obs;
double value_m_radar;
double pos_radar, pos_gun;
double turn_l;
double turn_r;
double radar_giro;
double dis_sen;
double radio=0.075;
double initial_angle_wheel1;

float encoder_L1_angv, encoder_L2_angv;
float encoder_R1_angv, encoder_R2_angv;
float encoder_L1_linv, encoder_L2_linv;
float encoder_R1_linv, encoder_R2_linv;
float encoder_L1_rpm, encoder_L2_rpm;
float encoder_R1_rpm, encoder_R2_rpm;

void goRobot(WbDeviceTag *wheels) {
  wb_motor_set_position(wheels[0], INFINITY);
  wb_motor_set_velocity(wheels[0], 4);
  wb_motor_set_position(wheels[1], INFINITY);
  wb_motor_set_velocity(wheels[1], 4);
  wb_motor_set_position(wheels[2], INFINITY);
  wb_motor_set_velocity(wheels[2], 4);
  wb_motor_set_position(wheels[3], INFINITY);
  wb_motor_set_velocity(wheels[3], 4);
}
void stopRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 0);
  wb_motor_set_velocity(wheels[3], 0);
}
void turnRight(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0],  -2);
  wb_motor_set_velocity(wheels[1],   2);
  wb_motor_set_velocity(wheels[2],  -2);
  wb_motor_set_velocity(wheels[3],   2);
}
void radar_on(WbDeviceTag radar_motor) {
  WbDeviceTag wheel_left_1  = wb_robot_get_device("motorL1");
  WbDeviceTag wheel_left_2  = wb_robot_get_device("motorL2");
  WbDeviceTag wheel_right_1 = wb_robot_get_device("motorR_1");
  WbDeviceTag wheel_right_2 = wb_robot_get_device("motorR_2");

  WbDeviceTag wheels[4];
  wheels[0] = wheel_left_1;
  wheels[1] = wheel_right_1;
  wheels[2] = wheel_left_2;
  wheels[3] = wheel_right_2;
  
  WbDeviceTag distance_radar = wb_robot_get_device("enemy_detector");
  wb_motor_set_position(radar_motor, INFINITY);
  wb_motor_set_velocity(radar_motor, .5);
  dis_radar = wb_distance_sensor_get_value(distance_radar);
  value_m_radar = ((dis_radar *2)/1023);
  if (value_m_radar<=1.5 ) {
    stopRobot(wheels);
    wb_motor_set_position(radar_motor, INFINITY);
    wb_motor_set_velocity(radar_motor, 0);
    radar_giro++;
    WbDeviceTag Radar_encoder = wb_robot_get_device("position_sensorT1");
    WbDeviceTag gun_motor     = wb_robot_get_device("gun_motor");
    pos_radar=wb_position_sensor_get_value(Radar_encoder);
    wb_motor_set_velocity(radar_motor, .01);
    wb_motor_set_position(gun_motor, (pos_radar-3.1416));
    wb_motor_set_velocity(radar_motor, 0);

  }
  if (radar_giro>=1) {
    stopRobot(wheels);
    wb_motor_set_position(radar_motor, INFINITY);
    wb_motor_set_velocity(radar_motor, 0);
    radar_giro++;
    if (value_m_radar<=.5) {
      printf("THA THAA THAAA!!!\n");
    } else if (value_m_radar<=1) {
        printf("THA THAA!!\n");
    } else if (value_m_radar<=1.25) {
        printf("THA!\n");
    }
    if (radar_giro >= 60 && value_m_radar >= 2 )
    {
      radar_giro = 0;
      wb_motor_set_position(radar_motor, INFINITY);
      wb_motor_set_velocity(radar_motor, .5);
    }
  }
}
void checkForObstacles(WbDeviceTag distance_sensor_obs) {
  WbDeviceTag wheel_left_1  = wb_robot_get_device("motorL1");
  WbDeviceTag wheel_left_2  = wb_robot_get_device("motorL2");
  WbDeviceTag wheel_right_1 = wb_robot_get_device("motorR_1");
  WbDeviceTag wheel_right_2 = wb_robot_get_device("motorR_2");

  WbDeviceTag wheels[4];
  wheels[0] = wheel_left_1;
  wheels[1] = wheel_right_1;
  wheels[2] = wheel_left_2;
  wheels[3] = wheel_right_2;
  
  dis_value_obs = wb_distance_sensor_get_value(distance_sensor_obs);
  value_cm_obs = ((dis_value_obs *.4)/255);
  if (value_cm_obs<=0.2 ) {
     turn_l++;
  }
  if (turn_l>=1 && turn_l<=50){
    turnRight(wheels);
    turn_l++;
  } else {
      turn_l=0;
  }
}

int main(int argc, char **argv)
{

wb_robot_init();
WbDeviceTag wheel_left_1  = wb_robot_get_device("motorL1");
WbDeviceTag wheel_left_2  = wb_robot_get_device("motorL2");
WbDeviceTag wheel_right_1 = wb_robot_get_device("motorR_1");
WbDeviceTag wheel_right_2 = wb_robot_get_device("motorR_2");
WbDeviceTag radar_motor   = wb_robot_get_device("motorT1");
WbDeviceTag gun_motor     = wb_robot_get_device("gun_motor");

WbDeviceTag wheels[4];
wheels[0] = wheel_left_1;
wheels[1] = wheel_right_1;
wheels[2] = wheel_left_2;
wheels[3] = wheel_right_2;

WbDeviceTag encoder_left_1  = wb_robot_get_device("position_sensorL1");
WbDeviceTag encoder_left_2  = wb_robot_get_device("position_sensorL2");
WbDeviceTag encoder_right_1 = wb_robot_get_device("position_sensorR_1");
WbDeviceTag encoder_right_2 = wb_robot_get_device("position_sensorR_2");
WbDeviceTag Radar_encoder   = wb_robot_get_device("position_sensorT1");
WbDeviceTag Gun_encoder     = wb_robot_get_device("gun_position");

WbDeviceTag distance_sensor_obs  = wb_robot_get_device("obs_detector");
WbDeviceTag distance_radar = wb_robot_get_device("enemy_detector");

wb_position_sensor_enable(encoder_left_1, TIME_STEP);
wb_position_sensor_enable(encoder_right_1, TIME_STEP);
wb_position_sensor_enable(encoder_left_2, TIME_STEP);
wb_position_sensor_enable(encoder_right_2, TIME_STEP);
wb_position_sensor_enable(Gun_encoder, TIME_STEP);
wb_position_sensor_enable(Radar_encoder, TIME_STEP);
wb_distance_sensor_enable(distance_sensor_obs, TIME_STEP);
wb_distance_sensor_enable(distance_radar, TIME_STEP);


while (wb_robot_step(TIME_STEP) != -1) {
  goRobot(wheels);
  radar_on(radar_motor);
  checkForObstacles(distance_sensor_obs);
}

wb_robot_cleanup();

return 0;
}