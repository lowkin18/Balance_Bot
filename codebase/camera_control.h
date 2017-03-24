/*
 * camera_control.h
 *
 *  Created on: Mar 16, 2017
 *      Author: c_ker
 */

#ifndef CAMERA_CONTROL_H_
#define CAMERA_CONTROL_H_

#include "project_types.h"



void level_mounts(float angle);
void init_levelers();

int distance_level_calc(float angle);
int camera_tilt_calc(float angle);


#endif /* CAMERA_CONTROL_H_ */
