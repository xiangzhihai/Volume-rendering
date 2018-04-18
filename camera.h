#pragma once
#include "glm/glm.hpp"
#include "glm/vec3.hpp"
#include "ray.h"

struct CAMERA{
    //all vectors should all be unit length, defines
    //the camera position, direction and orientation
    glm::vec3 eye;
    glm::vec3 dir;
    glm::vec3 up;

    //frame basis vectors
    //should be unit length
    glm::vec3 frame_u; //should always be dir x up
    glm::vec3 frame_v; //should always be === up

    //frame dimensions;
    //larger dimensions will "zoom out" and smaller dimensions will
    //zoom into the scene.  For most cameras this should be calculated from the
    //field of view, but for orthogonal cameras we need to set this
    //explicitly
    float frame_w;
    float frame_h;

    //number of pixels in each dimension
    int frame_px;
    int frame_py;

};


//creates a basic camera pointing in the z direction
void cam_init(CAMERA *cam, int pixels_x, int pixels_y);

//given a location in pixel space
//i.e. (5.5, 5.5) is the middle of pixel (5,5)
//returns a ray for the orthogonal projection (all rays are parallel to
//the camera view "dir"
RAY cam_ortho_getray(const CAMERA* cam, float point_x, float point_y);

//dim_x/dim_y should be pixels_x/pixels_y otherwise the rendering will be
//stretched in one dimension or another.  This controls the resolution and
//size of the render frame... larger dimensions will zoom out in the scene
//while smaller dimensions will zoom in
void cam_update_frame(CAMERA *cam, int pixels_x, int pixels_y, float dim_x, float dim_y);

//updates the view, normalizes vectors and sets up frame basis
//you should call this rather than setting the camera components
//directly.
void cam_update_view(CAMERA *cam, glm::vec3 dir, glm::vec3 eye, glm::vec3 up);

//rotates cam from one point on a sphere to another
void
cam_rotate_sphere(CAMERA *cam, glm::vec3 start, glm::vec3 end);

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
