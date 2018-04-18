#include "vrvolume.h"

#define MAX_UI8 255.0

/*
 *  Returns the value of the nearest grid point to "pt"
 */
float
interpolate_nearest_ui8(const VRVOL* vol, glm::vec3 pt)
{
    if(pt.x < 0 || pt.y < 0 || pt.z < 0 ||
       pt.x >= vol->gridx || pt.y >= vol->gridy || pt.z >= vol->gridz)
    {
        return 0.0; //this treats everything outside the volume as 0
                    //there are other choices, but this will work for
                    //most cases.
    }

    int ind = (int)roundf(pt.x);
    ind += (int)roundf(pt.y)*vol->gridx;
    ind += (int)roundf(pt.z)*vol->gridx*vol->gridy;
    float val =  ((uint8_t*)vol->data)[ind]/MAX_UI8;
    return val;
}

/*
 * Returns the gradient at pt, this is not the most efficient
 * way to implement this function, but it will work.
 */
glm::vec3
gradient_nearest_ui8(const VRVOL*vol, glm::vec3 pt)
{
    //we can do this more effieinctly by explicitly looking up the data, but
    //this is a better representation.
    glm::vec3 grad;
    grad.x = interpolate_nearest_ui8(vol,pt+glm::vec3(1,0,0))-
                interpolate_nearest_ui8(vol,pt-glm::vec3(1,0,0));
    grad.y = interpolate_nearest_ui8(vol,pt+glm::vec3(0,1,0))-
                interpolate_nearest_ui8(vol,pt-glm::vec3(0,1,0));
    grad.z = interpolate_nearest_ui8(vol,pt+glm::vec3(0,0,1))-
                interpolate_nearest_ui8(vol,pt-glm::vec3(0,0,1));
    return grad;
}


int
vrv_initvolume(VRVOL *vol,
               void *data,
               uint32_t grid_x,
               uint32_t grid_y,
               uint32_t grid_z,
               glm::vec3 vdim,
               VINTERP_T interpolation,
               VOL_T data_type)
{
    if(!data)
    {
        EPRINT("ERROR: Null Data passed to vrc_setvolume\n");
        return 0;
    }
    if(vdim.x <= 0|| vdim.y <=0||vdim.z<=0)
    {
        EPRINT("ERROR: Bad Volume Dimensions w: %f h: %f d: %f\n",vdim.x,vdim.y,vdim.z);
        return 0;
    }

    //compute bounding box
    vol->bb_p0 = glm::vec3(0.0,0.0,0.0);
    vol->bb_p1 = vdim;
    if(vdim.x > 1|| vdim.y > 1 || vdim.z > 1)
    {
        EPRINT("WARNING: Bad Volume Dimensions w: %f h: %f d: %f\n",vdim.x,vdim.y,vdim.z);
        vol->bb_p1 = glm::normalize(vol->bb_p1);
        EPRINT("WARNING: Normalizing to w: %f h: %f d: %f\n",vol->bb_p1.x,vol->bb_p1.y,vol->bb_p1.z);
    }

    //center at <0>
    vol->bb_p0 = -vol->bb_p1* .5f;
    vol->bb_p1 = vol->bb_p1* .5f;

    vol->inv_size = 1.0f/(vol->bb_p1-vol->bb_p0);

    //save data information
    vol->data = data;
    vol->gridx = grid_x;
    vol->gridy = grid_y;
    vol->gridz = grid_z;

    vol->type = data_type;
    vol->interp = interpolation;

    return 1;

}

VRVOL* readvol_ui8(const char *fn, size_t gridx, size_t gridy, size_t gridz, glm::vec3 voldim)
{
    FILE *fd;
    fd = fopen(fn,"rb");
    if(!fd){
        EPRINT("Error opening file %s\n",fn);
    }

    uint8_t *in = (uint8_t *)malloc(gridx*gridy*gridz);

    if(fread(in,1,gridx*gridy*gridz,fd) < gridx*gridy*gridz){
        free(in);
        EPRINT("Error reading in volume %s\n",fn);
    }
    fclose(fd);

    VRVOL *vol = (VRVOL*)malloc(sizeof(VRVOL));
    if(!vrv_initvolume(vol,in,gridx,gridy,gridz,voldim,VI_NEAREST,VRV_UINT8))
    {
        EPRINT("Error creating volume structure\n");
        free(in);
        free(vol);
        return NULL;
    }

    return vol;
}


//ADD a new interpolation scheme by adding it to the
//VINTERP_T enum, for example add VI_LINEAR which will
//get the value 1<<8+1.  Then you can add that combination
//to the switch statment i.e.
//case VRV_UINT8 | VI_LINEAR
//
//Note if you want to support other volume types, say float
//you will need to implement an interpolation function for each
//interpolation type i.e.
//VRV_FLOAT32 | VI_NEAREST and VRV_FLOAT32 | VRV_LINEAR ... etc
//Most people will not need to do this, and it is not required for
//the class
float vrv_interpolate(const VRVOL* vol, glm::vec3 pt)
{
    switch(vol->type|vol->interp){
        case VRV_UINT8 | VI_NEAREST:
            return interpolate_nearest_ui8(vol,pt);
        default:
            EPRINT("BAD VOLUME TYPE or INTERPOLATION METHOD\n");
            return 0;
    }
}

glm::vec3 vrv_gradient(const VRVOL* vol, glm::vec3 pt){
    switch(vol->type|vol->interp){
        case VRV_UINT8|VI_NEAREST:
            return gradient_nearest_ui8(vol,pt);
        default:
            EPRINT("BAD VOLUME IN TYPE or INTERPOLATION Combonation in Volume");
            return glm::vec3(0);
    }
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
