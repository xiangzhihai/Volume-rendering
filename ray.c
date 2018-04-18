#include "ray.h"
#include "glm/glm.hpp"

/*
 * Intersect ray with axis alligned bounding box defined by 2 points (p0, p1)
 * returns 1 if ray intersects, and 0 otherwise
 * if ray intersects, t0 and t1 will be the intersection points along the ray
 *
 * This code I believe is based from an early Nvidia Cuda sample code
 * but has been slightly modified by over the years
 */
int
ray_int_aabb(const RAY* ray, glm::vec3 p0, glm::vec3 p1, float *t0, float *t1)
{

    //computes intersection of ray with all 6 bb planes
    //expand the vectors to see the math, uses the face that
    //the bounding box is axis alligned (and thus the normals)
    //to each plane are in the ordinal directions
    //each of the 6 values in tbot/ttop represent the intersection point
    //along
    glm::vec3 invR = glm::vec3(1.0)/ray->dir;
    glm::vec3 tbot = invR * (p0 - ray->orig);
    glm::vec3 ttop = invR * (p1 - ray->orig);

    //reorder the intersections to find smalleds and larges on each axis
    glm::vec3 tmin = glm::min(ttop,tbot);
    glm::vec3 tmax = glm::max(ttop,tbot);

    //find the largest tmin and the smallest tmax which
    //are the intersection points for the box;
    *t0 = fmaxf(fmaxf(tmin.x,tmin.y),tmin.z);
    *t1 = fminf(fminf(tmax.x,tmax.y),tmax.z);

    //start ray atleast a little bit in front of the camera eye
    if(*t0 < .001)
        *t0 = .001;

    //if t1 < t0 then ray does not intersect
    return *t1 > *t0;
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
