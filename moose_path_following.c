/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024/8)

#define LOWER_BOUND_DISTANCE 80.0
#define UPPER_BOUND_DISTANCE 700.0

#define SENSOR_MAX 4096.0
#define SPEED (7.536*0.5)
#define BORNE_1 100.0
#define BORNE_2 300.0
#define BORNE_3 500.0
#define BORNE_4 700.0



static void compute_odometry(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
  double da = (dr - dl) / AXLE_LENGTH;  // delta orientation
  printf("estimated distance covered by left wheel: %g m.\n", dl);
  printf("estimated distance covered by right wheel: %g m.\n", dr);
  printf("estimated change of orientation: %g rad.\n", da);
}
/*
static void compute_sensors(WbDeviceTag ps5,WbDeviceTag ps6,WbDeviceTag ps1,WbDeviceTag ps2){
  printf("e");
}
*/

//add a bias to one side to make navigation consistent
static double estProcheGauche(double ps5,double ps6,double ps1,double ps2){
  double averageValue=((ps5+ps6*4)/5);
  averageValue=fmax(averageValue-LOWER_BOUND_DISTANCE,0.0);
  averageValue=fmin(averageValue,UPPER_BOUND_DISTANCE);
  //double estProche=sqrt(averageValue/(averageValue+30));e
  double estProche=averageValue/UPPER_BOUND_DISTANCE;
  estProche=sqrt(estProche);
  printf("%g hhh \n", estProche);
  return estProche;
}

static double estProcheDroite(double ps5,double ps6,double ps1,double ps2){
  double averageValue=((ps1*4+ps2)/5); //bqseline
  averageValue=fmax(averageValue-LOWER_BOUND_DISTANCE,0.0);
  averageValue=fmin(averageValue,UPPER_BOUND_DISTANCE);
  //double estProche=sqrt(averageValue/(averageValue+30));
  double estProche=averageValue/UPPER_BOUND_DISTANCE;
  estProche=sqrt(estProche);
  printf("%g \n", estProche);
  return estProche;
}

//pour capteur trapeze est borne max 4000
static double trapeze(double value, double a, double b, double c, double d) {
  if (value>=b && value<=c){
    return 1.0;
  }
  if (value<=a || value>=d){
    return 0.0;
  }
  if (value < b){
    return(value-a)/(b-a);
  }
  return(d-value)/(d-c);
}

static double moyenneGauche(double ps5,double ps6){
  return(ps5+ps6*4)/5;
}
static double moyenneDroite(double ps1,double ps2){
  return (ps1*4+ps2)/5;
}

double tournerDroite(double procheD){
  double speed=7.536*(1.0-procheD)*0.5;
  return speed;
}

double tournerGauche(double procheG){
  double speed=7.536*(1.0-procheG*0.99)*0.5;
  return speed;
}

static double estProche(double value) {
  return trapeze(value, BORNE_3, BORNE_4, SENSOR_MAX, SENSOR_MAX);
}
static double estLoin(double value) {
  return trapeze(value,0.0,0.0, BORNE_1, BORNE_2);
}
static double estBon(double value) {
  return trapeze(value, BORNE_1, BORNE_2, BORNE_3, BORNE_4);
}

static double longerMurGauche(double ps5){
  return estProche(ps5)-estLoin(ps5);
}


static double estProcheCentre(double procheDroite,double procheGauche){
  return fmin(trapeze(procheDroite, BORNE_3, BORNE_4, SENSOR_MAX, SENSOR_MAX)
  ,trapeze(procheGauche, BORNE_3, BORNE_4, SENSOR_MAX, SENSOR_MAX));
}

static void inferer(double procheG, double procheD,double procheC, double *gauche, double *droite) {
  double Tout =1.0-fmax(procheG,procheD);
  double Droite=procheG;
  double Gauche=procheD;
  double Pivot=procheC;
  printf("%g \n", Pivot);
  double somme=Tout+Droite+Gauche+Pivot;
  *gauche =SPEED*(Tout+Droite+Pivot)/somme;
  *droite =SPEED*(Tout+Gauche-Pivot)/somme;
}


int main(int argc, char *argv[]) {
  /* define variables */
  WbDeviceTag distance_sensor[8], left_motor, right_motor, left_position_sensor, right_position_sensor;
  int i, j;
  double speed[2];
  double sensors_value[8];
  double braitenberg_coefficients[8][2] = {{0.942, -0.22}, {0.63, -0.1}, {0.5, -0.06},  {-0.06, -0.06},
                                           {-0.06, -0.06}, {-0.06, 0.5}, {-0.19, 0.63}, {-0.13, 0.942}};
  double procheG;
  double procheD;
  double procheC;
  double proximity;
  int time_step;
  int camera_time_step;
  /* initialize Webots */
  wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 64;
    camera_time_step = 64;
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 256;
    camera_time_step = 1024;
  }

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, camera_time_step);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  for (i = 0; i < 8; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
  }

  /* main loop */
  while (wb_robot_step(time_step) != -1) {
    /* get sensors values */
    for (i = 0; i < 8; i++)
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    const double *a = wb_accelerometer_get_values(accelerometer);
    printf("accelerometer values = %0.2f %0.2f %0.2f\n", a[0], a[1], a[2]);

    /* compute odometry and speed values*/
    compute_odometry(left_position_sensor, right_position_sensor);
    procheG=estProche(moyenneGauche(sensors_value[5],sensors_value[6]));
    procheD=estProche(moyenneDroite(sensors_value[1],sensors_value[2]));
    procheC=estProcheCentre(sensors_value[0],sensors_value[7]);
    /*for (i = 0; i < 2; i++) {
      speed[i] = 0.0;
      if(i==0)proximity=procheD;
      else proximity=procheG;
      for(int j=0;j<8;j++)
        speed[i] += braitenberg_coefficients[j][i] * (1.0 - proximity);
        
    }*/
    inferer(procheG, procheD, procheC, &speed[1], &speed[0]);
    
    wb_motor_set_velocity(left_motor, speed[1]);
    wb_motor_set_velocity(right_motor, speed[0]);

  }

  wb_robot_cleanup();

  return 0;
}