#pragma once
#include "glm/vec3.hpp"

//location in space plus a unit directional vector
//dir should alwasy be unit length!!!
struct RAY{
    glm::vec3 orig;
    glm::vec3 dir;
};

/*
 * Intersect ray with axis alligned bounding box defined by 2 points (p0, p1)
 * returns 1 if ray intersects, and 0 otherwise
 * if ray intersects, t0 and t1 will be the intersection points along the ray
 */
int ray_int_aabb(const RAY* ray, glm::vec3 p0, glm::vec3 p1, float *t0, float *t1);


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
