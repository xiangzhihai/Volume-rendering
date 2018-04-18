#pragma once
#include "glm/vec3.hpp"
#include "glm/vec4.hpp"

//Light plus the basic phong components
struct VRLIGHT{
    glm::vec3 pos;

    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;
    float exponent;
};


/*
 *  Creates a a basic light for volume rendering.
 */
void vrl_init(VRLIGHT *l, glm::vec3 light_pos);


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
