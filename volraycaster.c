#include "volraycaster.h"
#include "glm/glm.hpp"
#include "glm/vec3.hpp"
#include "common.h"
#include "vrvolume.h"
#include <math.h>
#include <iostream>
#include <vector>
//this just used for the simple ray caster that intersects
//a dense part of the volume and returns that color, rather
//than accumulating the color properly
#define ISO_THRESHOLD 0.025
#define END .9
#define LIGHTMOVE 0.5
#define RANGEMOVE 0.05
#define RNAGEDIFF 0.1
using namespace std;
//ray casting variables
CAMERA *camera = NULL;
VRLIGHT *light = NULL;
TFUNC *transfer = NULL;
VRVOL *volume = NULL;

//ray marching
float dt = .01;
float maxsteps = 1000;
int mode = 0;
//use blinn phong shading to render
int rendphong = 0;
float low = 0.0, high = 1.0;
void light_right()
{
    light->pos.x += LIGHTMOVE;
}
void light_left()
{
    light->pos.x -= LIGHTMOVE;
}

void light_up()
{
    light->pos.y += LIGHTMOVE;
}
void light_down()
{
    light->pos.y -= LIGHTMOVE;
}

void light_in()
{
    light->pos.z += LIGHTMOVE;
}
void light_out()
{
    light->pos.z -= LIGHTMOVE;
}

void vrc_phong(int use_phong)
{
    rendphong = use_phong;
}

void change_mode()
{
    if (mode == 0)
    {
        mode = 1;
        std::cout << "linear" << std::endl;
    }
    else if (mode == 1)
    {
        mode = 2;
        std::cout << "cubic" << std::endl;
    }
    else if (mode == 2)
    {
        mode = 3;
        std::cout << "vrv" << std::endl;
    } 
    else
    {
        mode = 0;
        std::cout << "com" << std::endl;
    }
        
}

bool inRange(float val)
{
    return low < val && val < high;
}

void Range_low_up()
{
    if (low + RANGEMOVE + RNAGEDIFF <= high)
        low += RANGEMOVE;
}

void Range_low_down()
{
    if (low - RANGEMOVE > 0)
        low -= RANGEMOVE;
}

void Range_high_up()
{
    if (high + RANGEMOVE < 1)
        high += RANGEMOVE;
}

void Range_high_down()
{
    if (low + RNAGEDIFF <= high - RANGEMOVE)
        high -= RANGEMOVE;
}

VRVOL *vrc_volume()
{
    return volume;
}

CAMERA *vrc_camera()
{
    return camera;
}

VRLIGHT *vrc_light()
{
    return light;
}

TFUNC *vrc_transfer()
{
    return transfer;
}

void vrc_setvolume(VRVOL *vol)
{
    volume = vol;
}

void vrc_setcamera(CAMERA *cam)
{
    camera = cam;
}

void vrc_setlight(VRLIGHT *lt)
{
    light = lt;
}

void vrc_settransfer(TFUNC *trans)
{
    transfer = trans;
}

int vrc_setmarch(float delta_t, uint32_t max_steps)
{
    if (delta_t <= 0 || max_steps <= 0)
        return 0;
    dt = delta_t;
    maxsteps = max_steps;
    return 1;
}

/*
 *
 */
glm::vec3 blinn_phong(glm::vec3 wpt, glm::vec3 view, glm::vec3 norm, glm::vec3 col)
{
    glm::vec3 ldir = glm::normalize(light->pos - wpt);
    norm = glm::faceforward(norm, view, norm);
    //ambient + diffuse componenets
    glm::vec3 res = glm::clamp(light->ambient + glm::dot(ldir, norm) * light->diffuse, glm::vec3(0.0), glm::vec3(1.0));
    if (light->exponent > 0)
    {
        //add specular component to make things shiny
        float spec = glm::pow(clampf(dot(norm, glm::normalize(ldir + view)), 0.0, 1.0), light->exponent);
        return res * col + spec * light->specular;
    }
    else //no specular (which should be the default for volume rendering)
        return res * col;
}

glm::vec3 linear_gradient(float *mesh, glm::vec3 pt)
{
    float x = pt.x - floor(pt.x), y = pt.y - floor(pt.y), z = pt.z - floor(pt.z);

    float dx = (1 - y) * (1 - z) * (-mesh[0]);
    dx += (-mesh[1]) * z * (1 - y) - mesh[2] * y * (1 - z) - mesh[3] * y * z;
    dx += mesh[4] * (1 - y) * (1 - z) + mesh[5] * z * (1 - y) + mesh[6] * y * (1 - z) + mesh[7] * y * z;

    float dy = (1 - x) * (1 - z) * (-mesh[0]);
    dy += (-mesh[1]) * z * (1 - x) + mesh[2] * (1 - x) * (1 - z) + mesh[3] * (1 - x) * z;
    dy += (-mesh[4]) * x * (1 - z) - mesh[5] * z * x + mesh[6] * x * (1 - z) + mesh[7] * x * z;

    float dz = (1 - x) * (1 - y) * (-mesh[0]);
    dz += mesh[1] * (1 - x) * (1 - y) - mesh[2] * y * (1 - x) + mesh[3] * y * (1 - x);
    dz += (-mesh[4]) * x * (1 - y) + mesh[5] * x * (1 - y) - mesh[6] * x * y + mesh[7] * x * y;

    return glm::vec3(dx, dy, dz);
}

float linear_interpolation(float *mesh, glm::vec3 pt)
{
    float x = pt.x - floor(pt.x), y = pt.y - floor(pt.y), z = pt.z - floor(pt.z);

    //now interpolate
    float R = (1 - x) * (1 - y) * (1 - z) * mesh[0];
    R += (1 - x) * (1 - y) * z * mesh[1];
    R += (1 - x) * y * (1 - z) * mesh[2];
    R += (1 - x) * y * z * mesh[3];
    R += x * (1 - y) * (1 - z) * mesh[4];
    R += x * (1 - y) * (z)*mesh[5];
    R += x * y * (1 - z) * mesh[6];
    R += x * y * z * mesh[7];

    return R;
}

void get_mesh(glm::vec3 pt, float *mesh)
{
    //create an array
    free(mesh);
    mesh = (float *)malloc(8 * sizeof(float));

    //first get all 8 points
    mesh[0] = get_pt(volume, glm::vec3(floor(pt.x), floor(pt.y), floor(pt.z)));
    mesh[1] = get_pt(volume, glm::vec3(floor(pt.x), floor(pt.y), ceil(pt.z)));
    mesh[2] = get_pt(volume, glm::vec3(floor(pt.x), ceil(pt.y), floor(pt.z)));
    mesh[3] = get_pt(volume, glm::vec3(floor(pt.x), ceil(pt.y), ceil(pt.z)));
    mesh[4] = get_pt(volume, glm::vec3(ceil(pt.x), floor(pt.y), floor(pt.z)));
    mesh[5] = get_pt(volume, glm::vec3(ceil(pt.x), floor(pt.y), ceil(pt.z)));
    mesh[6] = get_pt(volume, glm::vec3(ceil(pt.x), ceil(pt.y), floor(pt.z)));
    mesh[7] = get_pt(volume, glm::vec3(ceil(pt.x), ceil(pt.y), ceil(pt.z)));
}

glm::vec3 get_direvative(glm::vec3 pt)
{   
    //result direvative
    glm::vec3 Dir;
    float val = get_pt(volume, pt);
    if (pt.x == 0) //when in grid min
    {
        float x1 = get_pt(volume, glm::vec3(pt.x + 1, pt.y, pt.z));
        Dir.x = (x1 - val) / 2;
    }
    else if (pt.x == volume->gridz) //when in grid max
    {
        float x_1 = get_pt(volume, glm::vec3(pt.x - 1, pt.y, pt.z));
        Dir.x = (val - x_1) / 2;
    }
    else { //in middle
        float x1 = get_pt(volume, glm::vec3(pt.x + 1, pt.y, pt.z));
        float x_1 = get_pt(volume, glm::vec3(pt.x - 1, pt.y, pt.z));
        Dir.x = (x1 - x_1) / 2;
    }

    if (pt.y == 0)
    {
        float y1 = get_pt(volume, glm::vec3(pt.x, pt.y + 1, pt.z));
        Dir.y = (y1 - val) / 2;
    }
    else if (pt.y == volume->gridy) //when in grid max
    {
        float y_1 = get_pt(volume, glm::vec3(pt.x, pt.y - 1, pt.z));
        Dir.y = (val - y_1) / 2;
    }
    else
    { //in middle
        float y1 = get_pt(volume, glm::vec3(pt.x, pt.y + 1, pt.z));
        float y_1 = get_pt(volume, glm::vec3(pt.x, pt.y - 1, pt.z));
        Dir.y = (y1 - y_1) / 2;
    }

    if (pt.z == 0)
    {
        float z1 = get_pt(volume, glm::vec3(pt.x, pt.y, pt.z + 1));
        Dir.z = (z1 - val) / 2;
    }
    else if (pt.z == volume->gridz) //when in grid max
    {
        float z_1 = get_pt(volume, glm::vec3(pt.x, pt.y, pt.z - 1));
        Dir.z = (val - z_1) / 2;
    }
    else
    { //in middle
        float z1 = get_pt(volume, glm::vec3(pt.x, pt.y, pt.z + 1));
        float z_1 = get_pt(volume, glm::vec3(pt.x, pt.y, pt.z - 1));
        Dir.z = (z1 - z_1) / 2;
    }
    return Dir;
}

vector<vector<vector<float > > > get_bezier(glm::vec3 pt, float *mesh)
{
    //get all g and m points into a 3d vector, 8 elements in total 2 * 2 * 2
    vector<vector<vector<glm::vec3> > > g(2, vector<vector<glm::vec3> >(2, vector<glm::vec3>(2)));
    vector<vector<vector<float> > > m(2, vector<vector<float> >(2, vector<float>(2)));
    vector<vector<vector<float> > > b(4, vector<vector<float> >(4, vector<float>(4)));
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++)
            {
                g[i][j][k] = get_direvative(glm::vec3(i ? ceil(pt.x) : floor(pt.x),
                                                      j ? ceil(pt.y) : floor(pt.y),
                                                      k ? ceil(pt.z) : floor(pt.z)));
                m[i][j][k] = mesh[i * 4 + j * 2 + i];
            }
                
    //try to get all b points, 32 elements in total 4 * 4 * 4
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
            {
                float x = i < 2 ? 0 : 1, y = j < 2 ? 0 : 1, z = k < 2 ? 0 : 1;
                b[i][j][k] = m[x][y][z];
                if (i == 2 || i == 1) //in middle
                    b[i][j][k] += (i == 1) ? (g[x][y][z].x / 3) : (-g[x][y][z].x / 3);
                if (j == 2 || j == 1) //in middle
                    b[i][j][k] += (i == 1) ? (g[x][y][z].y / 3) : (-g[x][y][z].y / 3);
                if (k == 2 || k == 1) //in middle
                    b[i][j][k] += (i == 1) ? (g[x][y][z].z / 3) : (-g[x][y][z].z / 3);
            }
    return b;
}

float Bezier3(int i, int j, int k, glm::vec3 pt)
{
    float px = pt.x - floor(pt.x), py = pt.y - floor(pt.y), pz = pt.z - floor(pt.z);
    float x = pow(1 - px, 3 - i) * pow(px, i) * ((i == 2 || i == 1) ? 3 : 1);
    float y = pow(1 - py, 3 - j) * pow(py, j) * ((j == 2 || j == 1) ? 3 : 1);
    float z = pow(1 - pz, 3 - k) * pow(pz, k) * ((k == 2 || k == 1) ? 3 : 1);
    return x * y * z;
}

float cubic_interpolation(glm::vec3 pt, vector<vector<vector<float> > > b)
{
    float res = 0;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                res += b[i][j][k] * Bezier3(i, j, k, pt);
    return res;
}

float burn2(int d, float pt)
{
    return pow(1 - pt, 2 - d) * pow(pt, d) * ((d == 1) ? 2 : 1);
}

float burn3(int d, float pt)
{
    return pow(1 - pt, 3 - d) * pow(pt, d) * ((d == 2 || d == 1) ? 3 : 1);
}

glm::vec3 cubic_gradient(glm::vec3 pt, vector<vector<vector<float> > > b)
{
    float px = pt.x - floor(pt.x), py = pt.y - floor(pt.y), pz = pt.z - floor(pt.z);
    float dx = 0, dy = 0, dz = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                dx += (b[i + 1][j][k] - b[i][j][k]) * burn2(i, px) * burn3(j, py) * burn3(k, pz);

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 4; k++)
                dy += (b[i][j + 1][k] - b[i][j][k]) * burn3(i, px) * burn2(j, py) * burn3(k, pz);

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 3; k++)
                dz += (b[i][j][k + 1] - b[i][j][k]) * burn3(i, px) * burn3(j, py) * burn2(k, pz);

    return glm::vec3(dx, dy, dz);
}

/*
 *
 */
void vrc_accumulate(const RAY *ray, float t0, float t1, float *out)
{
    
    float R = 0, G = 0, B = 0, A = 0;
    //march ray for at most maxsteps
    for (int i = 0; i < maxsteps && t0 <= t1; i++)
    {
        //get point along ray
        glm::vec3 pt = ray->orig + ray->dir * t0;
        //get location of point in volume
        pt = volume->inv_size * (pt - volume->bb_p0);
        //scale to volumes dimensions
        pt = pt * glm::vec3(volume->gridx, volume->gridy, volume->gridz);
        //check boundaries
        if (pt.x < 0 || pt.x > volume->gridx - 1 ||
            pt.y < 0 || pt.y > volume->gridy - 1 ||
            pt.z < 0 || pt.z > volume->gridz - 1)
        {
            t0 += dt;
            continue;
        }

        //std::cout << pt.x << " " << pt.y << " " << pt.z << std::endl;
        //interpolate value at point from data
        /*** This is where you need to implement several
         *** interpolation methods, the user should be able
         *** to select which interpolation method to use
         ***/
        float val;
        float *mesh;
        vector<vector<vector<float> > > b;
        if (mode == 0 || mode == 3)
             val = vrv_interpolate(volume,pt);
        else 
        {
            mesh = (float *)malloc(8 * sizeof(float));
            get_mesh(pt, mesh);
            if (mode == 1)
                val = linear_interpolation(mesh, pt);
            else if (mode == 2)
            {
                b = get_bezier(pt, mesh);
                val = cubic_interpolation(pt, b);
            }
            
        }
        
        /*** This is where you would accumulate colors based on
         *** the opacity values returned from the transfer function.
         *** For testing purposes I just return the color for the first
         *** value over a threshold that the ray hits.
         *** With a good interpolation function, this should aproximate the
         *** isosurface at the threshold value.
         ***/

            //assumes interpolate returns values between 1.0 and 0;
            /*if(val > END)
            break;*/

        if (inRange(val))
        {
            if (mode == 3)
            {
                mesh = (float *)malloc(8 * sizeof(float));
                get_mesh(pt, mesh);
                b = get_bezier(pt, mesh);
                float v1 = linear_interpolation(mesh, pt);
                float v2 = cubic_interpolation(pt, b);
                val = abs(v1 - v2);
            }
            glm::vec4 tcol = vrt_lookup(transfer, val);
            if (rendphong)
            {
                glm::vec3 grad;
                //gradient is the normal to the iso-surface at this
                //location, so it makes sense to use for lighting
                if (mode == 0 || mode == 3)
                    grad = vrv_gradient(volume, pt);
                else if (mode == 1)
                    grad = linear_gradient(mesh, pt);
                else
                    grad = cubic_gradient(pt, b);
                //glm::vec3 grad = linear_gradient(mesh, pt);
               
                grad = glm::normalize(grad);
                //turn gradient (normal) so its always facing the light
                glm::vec3 col = blinn_phong(ray->orig + ray->dir * t0, ray->dir, grad, glm::vec3(tcol));
                tcol = glm::vec4(col, tcol.w);
            }
            //should accumulate colors here based on opacity values
            //just return the first value for now.
            
            R = val * tcol.x + (1 - val) * R;
            G = val * tcol.y + (1 - val) * G;
            B = val * tcol.z + (1 - val) * B;
            A = val * tcol.w + (1 - val) * A;
        }
        //march along ray by delta t
        t0 += dt;
        
    }
    
    out[0] = R;
    out[1] = G;
    out[2] = B;
    out[3] = A;
    return;
}

/*
 *
 */
int vrc_render(float *out_rgba)
{
    if (!camera || !light || !transfer || !volume)
    {
        EPRINT("Error renderer called before everything is set up\n");
        return 0;
    }

    //clear image
    memset(out_rgba, 0, sizeof(float) * camera->frame_px * camera->frame_py);
    vrt_init(transfer, 1000, low, high);
    vrc_settransfer(transfer);
    //iterate over the entire output frame
    //this pragma will parallelize this loop if compiled corectly
    //but comment it out while debugging
    //note this probably won't work in os x, unless the latests
    //versions of clang support it, i'll have to check and update
    //the code
    //#pragma omp parallel for
    for (int y = 0; y < camera->frame_py; y++)
    {
        for (int x = 0; x < camera->frame_px; x++)
        {
            //get inex into output array
            int index = 4 * (x + y * camera->frame_px);

            //get ray for this pixel
            //for raycasting we generally use a orthogonal
            //infite perspective camera so all rays are
            //parallel, but in some cases a pinhole or perspective
            //camera might be used
            //we add the .5 so that we are shooting a ray through the center
            //of each pixel
            RAY r = cam_ortho_getray(camera, x + .5, y + .5);

            //intersection points t0 to t1
            float t0, t1;
            //intersect ray with the bounding box for this volume
            //which is centered at <0,0,0> in our virtual "world space"
            if (!ray_int_aabb(&r, volume->bb_p0, volume->bb_p1, &t0, &t1) || fabsf(t1 - t0) < 0.00001)
                continue; //this ray doesn't hit the box (or just touches it)

            //march along ray and accumulate values
            vrc_accumulate(&r, t0, t1, out_rgba + index);
        }
    }
    return 1;
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
