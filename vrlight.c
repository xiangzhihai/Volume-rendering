#include "vrlight.h"
#define INIT_AMB_INTENSITY .4
#define INIT_DIFF_INTENSITY .6
#define BAD_EXP_VAL -1

//Initializes a really simple light with no specular
//component (we don't generally expect shiny things when
//volume rendering)
void
vrl_init(VRLIGHT *l, glm::vec3 light_pos)
{
    l->ambient = glm::vec3(INIT_AMB_INTENSITY);
    l->diffuse = glm::vec3(INIT_DIFF_INTENSITY);
    l->specular = glm::vec3(0.0);
    l->exponent = BAD_EXP_VAL;

    l->pos = light_pos;
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
