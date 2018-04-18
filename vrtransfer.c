#include "vrtransfer.h"


//creates a very simple transfer function that linearly
//interpolates color and alpha values between min and
//max such that min -> RGBA (0,0,0,0) and max -> RGBA(1,1,1,1);
int
vrt_init(TFUNC* tf, int length, float min, float max){
    if(max <= min || tf==NULL){
        return 0;
    }

    tf->colors = (glm::vec4*)malloc(length*sizeof(glm::vec4));
    if(!tf->colors){
        return 0;
    }

    tf->max = max;
    tf->min = min;
    tf->len = length;

    float inc = (max-min)/(length-1);
    for(int i =0; i < length; i++){
        //grey scale transfer function with linearly
        //ramping alpha values;
        tf->colors[i] = glm::vec4(inc*i);
    }
    return 1;
}

void
vrt_clean(TFUNC* tf){
    free(tf->colors);
    tf->colors = NULL;
    tf->len = 0;
    tf->min = MAXFLOAT;
    tf->max = -MAXFLOAT;
}

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
