/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 /*
  * Description:  Default controller of the iRobot Create robot
  */

  /* include headers */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

/* device stuff */
#define BUMPERS_NUMBER 2
#define BUMPER_LEFT 0
#define BUMPER_RIGHT 1
static WbDeviceTag bumpers[BUMPERS_NUMBER];
static const char *bumpers_name[BUMPERS_NUMBER] = { "bumper_left", "bumper_right" };

#define CLIFF_SENSORS_NUMBER 4
#define CLIFF_SENSOR_LEFT 0
#define CLIFF_SENSOR_FRONT_LEFT 1
#define CLIFF_SENSOR_FRONT_RIGHT 2
#define CLIFF_SENSOR_RIGHT 3
static WbDeviceTag cliff_sensors[CLIFF_SENSORS_NUMBER];
static const char *cliff_sensors_name[CLIFF_SENSORS_NUMBER] = { "cliff_left", "cliff_front_left", "cliff_front_right",
															   "cliff_right" };

#define LEDS_NUMBER 3
#define LED_ON 0
#define LED_PLAY 1
#define LED_STEP 2
static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_name[LEDS_NUMBER] = { "led_on", "led_play", "led_step" };

static WbDeviceTag receiver;
static const char *receiver_name = "receiver";

WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;

/* Misc Stuff */
#define MAX_SPEED 16
#define NULL_SPEED 0
#define HALF_SPEED 8
#define MIN_SPEED -16

#define WHEEL_RADIUS 0.031
#define AXLE_LENGTH 0.271756
#define ENCODER_RESOLUTION 507.9188

static const char RIGHT = 'R';
static const char LEFT = 'L';
static const char TOP = 'T';
static const char BOTTOM = 'B';

static const int ROOM_WIDTH = 4000;
static const int ROOM_HEIGHT = 4000;

double coordX, coordY = 0;
double oldCoordX, oldCoordY = 0;
double lastAngle, oldAngle = 0;
bool walling = true;
bool isLastDir = true;
char lastSide;

/* helper functions */
static int get_time_step() {
	static int time_step = -1;
	if (time_step == -1)
		time_step = (int)wb_robot_get_basic_time_step();
	return time_step;
}

static void step() {
	if (wb_robot_step(get_time_step()) == -1) {
		wb_robot_cleanup();
		exit(EXIT_SUCCESS);
	}
}

static void init_devices() {
	int i;

	receiver = wb_robot_get_device(receiver_name);
	wb_receiver_enable(receiver, get_time_step());

	for (i = 0; i < LEDS_NUMBER; i++)
		leds[i] = wb_robot_get_device(leds_name[i]);

	for (i = 0; i < BUMPERS_NUMBER; i++) {
		bumpers[i] = wb_robot_get_device(bumpers_name[i]);
		wb_touch_sensor_enable(bumpers[i], get_time_step());
	}

	for (i = 0; i < CLIFF_SENSORS_NUMBER; i++) {
		cliff_sensors[i] = wb_robot_get_device(cliff_sensors_name[i]);
		wb_distance_sensor_enable(cliff_sensors[i], get_time_step());
	}

	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	wb_motor_set_velocity(left_motor, 0.0);
	wb_motor_set_velocity(right_motor, 0.0);

	left_position_sensor = wb_robot_get_device("left wheel sensor");
	right_position_sensor = wb_robot_get_device("right wheel sensor");
	wb_position_sensor_enable(left_position_sensor, get_time_step());
	wb_position_sensor_enable(right_position_sensor, get_time_step());
}

static bool is_there_a_collision_at_left() {
	return (wb_touch_sensor_get_value(bumpers[BUMPER_LEFT]) != 0.0);
}

static bool is_there_a_collision_at_right() {
	return (wb_touch_sensor_get_value(bumpers[BUMPER_RIGHT]) != 0.0);
}

static void fflush_ir_receiver() {
	while (wb_receiver_get_queue_length(receiver) > 0)
		wb_receiver_next_packet(receiver);
}

static bool is_there_a_virtual_wall() {
	return (wb_receiver_get_queue_length(receiver) > 0);
}

static bool is_there_a_cliff_at_left() {
	return (wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_LEFT]) < 100.0 ||
		wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_LEFT]) < 100.0);
}

static bool is_there_a_cliff_at_right() {
	return (wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_RIGHT]) < 100.0 ||
		wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_RIGHT]) < 100.0);
}

static bool is_there_a_cliff_at_front() {
	return (wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_LEFT]) < 100.0 ||
		wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_RIGHT]) < 100.0);
}

static void go_forward() {
	wb_motor_set_velocity(left_motor, MAX_SPEED);
	wb_motor_set_velocity(right_motor, MAX_SPEED);
}

static void go_backward() {
	wb_motor_set_velocity(left_motor, -HALF_SPEED);
	wb_motor_set_velocity(right_motor, -HALF_SPEED);
}

static void stop() {
	wb_motor_set_velocity(left_motor, -NULL_SPEED);
	wb_motor_set_velocity(right_motor, -NULL_SPEED);
}

static void passive_wait(double sec) {
	double start_time = wb_robot_get_time();
	do {
		step();
	} while (start_time + sec > wb_robot_get_time());
}

static double randdouble() {
	return rand() / ((double)RAND_MAX + 1);
}

static void turn(double angle) {
	lastAngle = angle;
	stop();
	double l_offset = wb_position_sensor_get_value(left_position_sensor);
	double r_offset = wb_position_sensor_get_value(right_position_sensor);
	step();
	double neg = (angle < 0.0) ? -1.0 : 1.0;
	wb_motor_set_velocity(left_motor, neg * HALF_SPEED);
	wb_motor_set_velocity(right_motor, -neg * HALF_SPEED);
	double orientation;
	do {
		double l = wb_position_sensor_get_value(left_position_sensor) - l_offset;
		double r = wb_position_sensor_get_value(right_position_sensor) - r_offset;
		double dl = l * WHEEL_RADIUS;                 // distance covered by left wheel in meter
		double dr = r * WHEEL_RADIUS;                 // distance covered by right wheel in meter
		orientation = neg * (dl - dr) / AXLE_LENGTH;  // delta orientation in radian
		step();
	} while (orientation < neg * angle);
	stop();
	step();
}

static void turnAwayFromObstacle(double turnValue) {
	go_backward();
	passive_wait(0.5);
	turn(turnValue);
}

static bool getLastDir() {
	return randdouble() > 0.5 ? true : false;
}

void storeWall(char side) {
	lastSide = side;
	oldCoordX = coordX;
	oldCoordY = coordY;
	walling = true;
	oldAngle = lastAngle;

	switch (lastSide) {
	case TOP:
		if (isLastDir) {
			turn(M_PI);
		}
		else {
			turn(0);
		}
		break;
	case BOTTOM:
		if (isLastDir) {
			turn(0);
		}
		else {
			turn(M_PI);
		}
		break;
	case LEFT:
		if (isLastDir) {
			turn(M_PI / 2);
		}
		else {
			turn(1.5 * M_PI);
		}
		break;
	case RIGHT:
		if (isLastDir) {
			turn(1.5 * M_PI);
		}
		else {
			turn(M_PI / 2);
		}
		break;
	default:
		break;
	}
}

void restart(newAngle) {
	isLastDir = !isLastDir;
	walling = false;
	turn(newAngle);
	oldAngle = 0;
}

bool isDoneWalling() {
	switch (lastSide) {
	case TOP:
		if (isLastDir && coordX + 2 * lastAngle < oldCoordX) {
			return true;
		}
		else if (coordX - 2 * lastAngle > oldCoordX) {
			return true;
		}
		break;
	case BOTTOM:
		if (isLastDir && coordX - 2 * lastAngle > oldCoordX) {
			return true;
		}
		else if (coordX + 2 * lastAngle < oldCoordX) {
			return true;
		}
		break;
	case LEFT:
		if (isLastDir && coordY - 2 * lastAngle > oldCoordY) {
			return true;
		}
		else if (coordY + 2 * lastAngle < oldCoordY) {
			return true;
		}
		break;
	case RIGHT:
		if (isLastDir &&  coordY + 2 * lastAngle < oldCoordY) {
			return true;
		}
		else if (coordY - 2 * lastAngle > oldCoordY) {
			return true;
		}
		break;
	default:
		return false;
	}
	return false;
}

void setReverse() {
	walling = false;
	isLastDir = !isLastDir;
	turn(oldAngle + M_PI);
}

void checkWallingToStoreOrRestart(side, newAngle) {
	if (!walling) {
		storeWall(side);
	}
	else {
		restart(newAngle);
	}
}

void move() {
	coordX += MAX_SPEED * cos(lastAngle);
	coordY += MAX_SPEED * sin(lastAngle);

	if (coordX + lastAngle > ROOM_WIDTH) {
		coordX = ROOM_WIDTH - lastAngle;
		checkWallingToStoreOrRestart(RIGHT, randdouble() * M_PI + M_PI / 2);
	}
	else if (coordX - lastAngle < 0) {
		coordX = lastAngle + 0.1;
		checkWallingToStoreOrRestart(LEFT, randdouble() * M_PI + 1.5 * M_PI);
	}
	else if (coordY - lastAngle < 0) {
		coordY = lastAngle;
		checkWallingToStoreOrRestart(TOP, randdouble() * M_PI + M_PI);
	}
	else if (coordY + lastAngle > ROOM_HEIGHT) {
		coordY = ROOM_HEIGHT - lastAngle;
		checkWallingToStoreOrRestart(BOTTOM, randdouble() * M_PI);
	}

	if (walling && isDoneWalling()) {
		oldCoordX = coordX;
		oldCoordY = coordY;
		setReverse();
	}
	fflush_ir_receiver();
	go_forward();
	// step();
}

/* main */
int main(int argc, char **argv) {
	wb_robot_init();
	isLastDir = getLastDir();
	coordX = -0.0193578;
	coordY = 3.69248;

	printf("Default controller of the iRobot Create robot started...\n");

	init_devices();
	srand(time(NULL));

	wb_led_set(leds[LED_ON], true);
	passive_wait(0.5);

	while (true) {
		if (is_there_a_virtual_wall()) {
			printf("Virtual wall detected\n");
			coordY = M_PI;
			turn(coordY);
		}
		else if (is_there_a_collision_at_left() || is_there_a_cliff_at_left()) {
			printf("Left obstacle detected\n"); 
			printf("coordY=%f coordX=%f", coordY, coordX);
			coordX = lastAngle + 0.1;
			turnAwayFromObstacle(coordX);
		}
		else if (is_there_a_collision_at_right() || is_there_a_cliff_at_right() || is_there_a_cliff_at_front()) {
			printf("Right obstacle detected\n");
			printf("coordY=%f coordX=%f", coordY, coordX);
			coordX = ROOM_WIDTH - lastAngle;
			turnAwayFromObstacle(-coordY);
		}
		else {
			printf("No obstacles detected, go forward.\n");
			printf("coordY=%f coordX=%f", coordY, coordX);
			go_forward();
		}
		fflush_ir_receiver();
		step();
	}

	return EXIT_SUCCESS;
}
