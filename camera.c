#include "camera.h"
#include "common.h"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

RAY
cam_ortho_getray(const CAMERA* cam, float point_x, float point_y)
{
    RAY r;
    r.dir = cam->dir;

    //get relative position of pixels from the center pixel
    float dx = (point_x/cam->frame_px-.5f)*cam->frame_w;
    float dy = (point_y/cam->frame_py-.5f)*cam->frame_h;

    r.orig = cam->eye+dy*cam->frame_v+dx*cam->frame_u;
    return r;
}

void
cam_init(CAMERA *cam, int pixels_x, int pixels_y)
{
    //create camera basis
    //put camera at 0,0,-1 looking
    //at center of volume and with a
    //standard up vector (+y)
    cam->dir = glm::vec3(0,0,1);
    cam->eye = glm::vec3(0,0,-1);
    cam->up = glm::vec3(0,1,0);

    //create frame that is width/height pixels
    //render frame should be (-.5,-.5) x (.5, .5) initially
    //frame basis defined by camera basis
    cam->frame_px = pixels_x;
    cam->frame_py = pixels_y;
    cam->frame_h = 1.0;
    cam->frame_w = 1.0;
    cam->frame_u = glm::cross(cam->dir,cam->up);
    cam->frame_v = cam->up;

}


void
cam_update_frame(CAMERA *cam, int pixels_x, int pixels_y, float dim_x, float dim_y)
{
    //create frame that is width/height pixels
    //render frame should be (-dim_x/2,-dim_y/2),(dim_x/2,dim_y/2)
    //frame basis defined by camera basis
    cam->frame_px = pixels_x;
    cam->frame_py = pixels_y;
    cam->frame_w = dim_x;
    cam->frame_h = dim_y;
}


void
cam_update_view(CAMERA *cam, glm::vec3 dir, glm::vec3 eye, glm::vec3 up){
    cam->dir = glm::normalize(dir);
    cam->eye = eye;
    cam->up = glm::normalize(up);

    cam->frame_u = glm::cross(cam->dir,cam->up);
    cam->frame_v = cam->up;
}


/*
 * Uses quaternion rotation to rotate the camera around a sphere
 * This is really buggy, but I haven't taken the time to fix it...
 * eventually this causes an error and the camera gets screwed up
 * I'll give extra credit to whoever figures it out and posts an
 * answer to canvas.
 */
void
cam_rotate_sphere(CAMERA *cam, glm::vec3 start, glm::vec3 end){
    //two points on a sphere (or two unit vectors)
    start = glm::normalize(start);
    end = glm::normalize(end);

    //get angle and axis
    float rad = acosf(glm::dot(start,end));
    if(fabs(rad) < .001) return;
    glm::vec3 axis = glm::normalize(glm::cross(start,end));

    glm::quat rot = glm::angleAxis(rad,axis);
    glm::quat rot_c = glm::conjugate(rot);
    glm::vec3 eye = glm::normalize(rot*cam->eye*rot_c);

    //new view is eye - <0,0,0>
    //eye point is the rotated eye, normalized
    //up is the rotated up
    cam_update_view(cam,-eye,eye,rot*cam->up*rot_c);
    return;
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
