#pragma once
#include "common.h"
#include "glm/glm.hpp"

//type of volume and interpolation
//note we treat thease as flags that
//and will be logically or'd together
//i.e. vol|interp == VRV_UINT8|IT_NEAREST
enum VOL_T {VRV_UINT8=0};
enum VINTERP_T {VI_NEAREST=1<<8};


/***NOTES****/
//You will need to modify vrvolume.h/c to implement
//the different interpolation methods, see my nearest
//neighbor interpolation as a guide.

//This structure encapsulates the raw volume image
//For this project you only need to support volumes
//with byte sized input, but volumes in general can be
//16bit 32bit or larger, this structure hides those
//details from the ray caster. This way each ray can
//just look up the interpolated value at the point, and not
//care about the type of data.
//This could be futher extended to support non rectalinear grids
struct VRVOL{
    void *data;
    uint32_t gridx;
    uint32_t gridy;
    uint32_t gridz;
    glm::vec3 dim;
    glm::vec3 bb_p0;
    glm::vec3 bb_p1;
    glm::vec3 inv_size;
    VOL_T type;
    VINTERP_T interp;
};


//creates a new volume from raw input data,
//by default assumes 8bit integers.
int vrv_setvolume(VRVOL* vol,
                  void *data,
                  uint32_t grid_x,
                  uint32_t grid_y,
                  uint32_t grid_d,
                  glm::vec3 volumedim,
                  VINTERP_T interpolation = VI_NEAREST,
                  VOL_T data_type = VRV_UINT8);

//reads in a raw volume file from fn.  The file should not have any header
//information and just store each byte of the volume in row major order
//this is the format of for the volumes I included
VRVOL* readvol_ui8(const char *fn,size_t gridx,size_t gridy,size_t gridz, glm::vec3 voldim);

//returns the interpolated value and gradient at
float vrv_interpolate(const VRVOL* vol, glm::vec3 pt);
glm::vec3 vrv_gradient(const VRVOL* vol, glm::vec3 pt);


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
