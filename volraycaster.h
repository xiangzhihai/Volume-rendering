#pragma once
#include "ray.h"
#include "camera.h"
#include "vrlight.h"
#include "vrtransfer.h"
#include "stdint.h"
#include "vrvolume.h"


//Setters and getters for the different components
//of our modular volume render
void vrc_setvolume(VRVOL* vol);
void vrc_setcamera(CAMERA *cam);
void vrc_setlight(VRLIGHT *lt);
void vrc_settransfer(TFUNC *trans);

CAMERA* vrc_camera();
VRLIGHT* vrc_light();
TFUNC *vrc_transfer();
VRVOL *vrc_volume();

//turns phong lighting on and off
void vrc_phong(int use_phong);

/*
 * delta_t: lower value -> higher resoution, slower, less transparent
 * max_steps: maximum number of iterations along the ray, sets the worst case bound
 */
int vrc_setmarch(float delta_t, uint32_t max_steps);

/*
 * renders the scene using the current camera, light, transfer function, and volume
 * results are stored in out_rgba which should be 4*cam->frame_px*cam->frame_py
 */
int vrc_render(float *out_rgba);


/****************************************************************************************************
 * The code was developed by Garrett Aldrich for [ECS 277] Advanced Visualization at UC Davis.
 * Bugs and problems :'(
 * If you are in my class, please don't email me.... start a thread on canvas :-)
 * If you aren't in my class I apologize for the bad code, I probably will never fix it!
 *
 * It's free as in beer
 * (free free, do whatever you want with it)
 *
 * If you use a big chunk, please keep this code block somewhere in your code (the end is fine)
 * Or atleats a comment my name and where you got the code.
 *
 * If you found this useful please don't email me, (sorry I ignore way to many already),
 * feel free to send me a thanks on instagram or something like that (I might not read it for a
 * while, but eventually I will)
 *
 ****************************************************************************************************/
