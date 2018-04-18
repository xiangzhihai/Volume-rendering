#pragma once
/*
 * Simple glut demo that can be used as a template for
 * other projects by Garrett Aldrich
 */

//GLUT, OpenGL, and GLExtension includes
//I haven't tested windows in a long time :-/
#ifdef WIN32
#include <windows.h>
#endif
//For Macs
//compile with -framework OpenGL -framework GLUT
#if defined (__APPLE__) || defined(MACOSX)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else //For Linux (maybe windows also :-/)
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>
#endif


//GLUT VIEWER Constants
#define INIT_WIN_WIDTH 512
#define INIT_WIN_HEIGHT 512


/*
 *  Called after glut creates a new context.
 *  Sets any global variables, and initializes the OpenGL state.
 *  Also initializes any other modules 
 *
 * returns 0 on error, otherwise 1;
 */
int initialize();

/***GLUT FUNCTIONS***/
/* For full description see https://www.opengl.org/resources/libraries/glut/spec3/spec3.html */

/*
 *  When the glut rendering loop has nothing else to do, the idle function
 *  is called repeatedly.  
 *  For progressive or hierarchal rendering this is a great place
 *  to increase the resolution.  
 *  For in-situ visualiztion this is a great place to incrament the simulation. 
 */
void idle();

/*
 *  This functions is where the rendering calls should be made and 
 *  information is drawn to the "screen" 
 */
void display();

/*
 * This function is called when the screen size changes.  Usually any output
 * or render buffers need to be updated so that there size matches the new size
 * and aspect ratio matches the current screen size.  
 */ 
void reshape(int width, int height);

/*
 * This function is called whenever the keyboard is used, allows for basic
 * user interface functionality.
 */
void key(unsigned char ch, int x, int y);

/* 
 * This function handles button press/release from the mouse.
 */
void mouse(int button, int state, int x, int y);

/*
 *  This function handles mouse movement when a button is pressed.
 */
void motion(int x, int y);


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
