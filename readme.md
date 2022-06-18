#Building Project

Simple Makefile should compile on mac and linux, see makefile to comment the
appropriate lines out.  On linux make sure the freeglut libraries are installed.
This should also compile on windows, but you will need the to download the glut
library.  Any glut/opengl tutorial on the web should help you get this to compile
on windows.  I suggest if you haven't written any opengl code, that you go through
an online tutorial [http://www.lighthouse3d.com/tutorials/glut-tutorial/] will work.

You can easily speed up the code by using openmp, however atleast on my older mac
laptop, openmp is not supported, and the compiler commands are ignored.  However,
on linux, or using gcc on mac, you should be able to get parrallel rendering by
compiling with openmp and uncommenting the `#pragma openmp parallel for` in vrc_render.
While debugging I suggest only using a single thread.


#Running sample application
raycaster volume_file grid_x grid_y grid_z <dim_x dim_y dim_z>
volume_file - binary file of bytes that represent the volume, in row major order
grid_[x,y,z] - the number of points in the volume
dim_[x,y,z] - optional, for rectangular grids can give there relative dimensions
            - for example the default is a cube 1 1 1 but if the z dimension is
            - twice as long as the x and ydimension, then use .5 .5 1

##Example included in project
./raycaster fuel_8_64.raw 64 64 64


#OUTLINE

##glut_viewer
Contains all the code for displaying the rendered image, and user interface
Also intializes the renderer and each of its componenets.
Uses GLUT, please see the spec or go through a tutorial if you need help
[https://www.opengl.org/resources/libraries/glut/spec3/spec3.html]

###IMPORTANT NOTE
You will need to update several functions so that you can manipulate the
lights, change to different views, show multiple views, manipulate the transfer
functions, etc.  Creating multiple windows is not difficult, you can also use
keyboard and mouse combinations

##volraycaster
Renders a volume using ray casting.
Rays are generated from a virtual camera, and intersected with the volumes
bounding box.  We march along the ray, testing values in small incraments *dt*.
Currently we ignore values lower than a threshold value.  Once a density greater
than the value is intersected by our ray, we lookup that density value in a transfer
function to get the color and alpha (transparency).  We then return that color.
Rays are generated for each pixel in the image, so that the entire volume should
be rendered (depending on camera set up).

###IMPORTANT NOTE
You need to update these functions so that they do full volume rendering and
accumulate values along the ray instead of picking the first value above a threshold.

##vrvolume
This is a volume wrapper that holds the raw data and handles interpolation.
I have implemented nearest neighbor interpolation for volumes of type uint8_t (unsigned char).
Important to note, interpolated values return between 0 and 1.  This makes dealing with
different volume types much easier!!!


###IMPORTANT NOTE
You shouldn't need to implement other volume types (although doing so is trivial), but
you need to implement different interpolation methods.  I use a simple flag system based
on enumurated types.  If you have trouble understanding whats happening, please see me
in office hours or start a thread on canvas.  You should create a interpolation functions
and gradient functions for each interpolation type.

##vrtransfer
Transfer function, fills an array of rgba values and then maps values between a min
and max (usueally 0 and 1).  RGBA values represent "red green blue" (colors) and
alpha (transparency)

Right now a simple grey scale (R=G=B) colormap is used and transparency is directly
related to the (a).

###Notes
You will need to implement functions that allow users to create or set different colormaps.
You might want to store the color portions (rgb) in a file and have the user only manipulate
the transfer function (alpha values) in real time.  Your transfer function can be piecewise
linear, or any other type.  You may also look into 2D and 3D transfer functions.

For generating color maps.  Please everyone go to [http://sciviscolor.org/home/colormoves/]
This is an excellent resource (made by friends of mine) that is the best resource avaialble
for learning how to use color with datascience.  Feel free to use any of there colormaps in
your projects.

##Camera
Simple camera with very little implemented.  However it will generate orthogonal
rays for the image, and I included a broken rotation function for a simple track ball.
There is an issue that eventually it becomes unstable and the rotations stop being
consistant.  Also I think sometimes the view direction gets flipped, or something else
strange happens because I get a bad camera.  Feel free to fix this for me, but it should be
more than enough to get you started.

###IMPORTATNT NOTE
At minimum you should add methods for changing the view into the three ordinal directions.
Also you probably want a function that "zooms" the camera in and out.  I suggest just changing
the cameras frame width and height from 1.0 to something larger or smaller to zoom our or in
respectively.

##ray
Very simple ray class
Includes code to intersect a axis alligned bounding box, which we use to intersect our volume.

##vrlight
Simple light class that is used for phong shading in the volume raycaster

###IMPORTANT NOTE
You will need to implement functions for moving the lights location, ideally you should
move it with the mouse or arrow keys, please don't use the command line

