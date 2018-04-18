#pragma once
#include "glm/glm.hpp"
#include "common.h"
#include "vrvolume.h"


//****NOTE*********/
//you should add methods for dynamically creating different color
//maps and allowing the user to create more complex transfer functions

//simple transfer function is represented by an array of
//"len" colors (rgba) such that rgb represents the relative
//color at each point in the array, and a represent the alpha
//transparency value.  Min and Max shoudl represent the domain
//represented by the colormap, generally 0.0 and 1.0
struct TFUNC{
    glm::vec4 *colors;
    int len;
    float min;
    float max;
};


//looks up color and alpha from the transfer function for a given value
//between tf->max and tf->min
inline glm::vec4 vrt_lookup(const TFUNC* tf, float val){
    float index = (tf->len-1.0)*clampf((val-tf->min)/(tf->max-tf->min),0,1.0);
    //might be better to interpolate, but just round for right now
    return tf->colors[(int)roundf(index)];
}


//creates a simple linear greyscale color map
int vrt_init(TFUNC* tf, int length, float min, float max);


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
