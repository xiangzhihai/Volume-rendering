#include "glut_viewer.h"

#include "common.h"
#include "volraycaster.h"

#define INITAL_DELTA_T .01
#define INITIAL_MAX_STEPS 1000
#define INITIAL_MINTRANS .01
#define INIT_RENDER_RATIO .25
#define TF_LENGTH 1000
#define MIN_TF_VAL 0.0
#define MAX_TF_VAL 1.0
//window title
const char *win_title = "Garrett's Sample Viewer";

//current window dimensions
int win_width = INIT_WIN_WIDTH;
int win_height = INIT_WIN_HEIGHT;
float render_ratio = INIT_RENDER_RATIO;
float view_width;
float view_height;

//render buffer for current window size*render_ratio
float *render_buff = NULL;
size_t render_width;
size_t render_height;

//mouse details
//stores the mouses relative position in the viewport
float mousex = 0;
float mousey = 0;
//stores the mouse button, if its currently pressed
int mouseb = 0;

//global ui variables
float ray_dt = INITAL_DELTA_T;
float ray_maxsteps = INITIAL_MAX_STEPS;
float rend_mintransparency = INITIAL_MINTRANS;

//output texture
GLuint render_tex;
GLuint colormap_tex;

//volume information
char *volfile;
size_t volgridx,volgridy,volgridz;
glm::vec3 voldim(1.0);

void checkGL()
{
    GLenum err = glGetError();
    if(err != GL_NO_ERROR)
    {
        EPRINT("There was an error %s\n",gluErrorString(err));
        exit(1);
    }
}

int
initialize()
{

//GLEW not needed on Mac Osx
#ifndef __APPLE__
    GLenum err = glewInit();
    if (GLEW_OK != err)
        EPRINT("Error initializing glew\n");
#endif

    //sets the default background to black
    //won't matter because we are completely overwriting the screen
    //with the volume rendered texture.
    glClearColor(0.0,0.0,0.0,0.0);

    //create output texture
    //Don't do anything fancy with the render texture, don't repeat texture
    //and no resampling.  The texture will end up having the same number of pixels
    //as the viewport (display).
    //don't turn on Linear filtering when testing your volume renderer... it can cause
    //unpredictable effects which will alter the results when comparing different interpolation
    //methods.
    glGenTextures(1,&render_tex);
    glBindTexture(GL_TEXTURE_2D,render_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D,0);

    //Initialize the raycaster
    vrc_setmarch(ray_dt,ray_maxsteps);

    //Initialize camera
    CAMERA *cam = (CAMERA*) malloc(sizeof(CAMERA));
    cam_init(cam,win_width*render_ratio,win_height*render_ratio);
    vrc_setcamera(cam);

    //shouldn't be necesary as init should only be called once,
    //but just in case
    if(render_buff)free(render_buff);
    //allocate rgba floating point framebuffer and intiailize to 0;
    render_buff = (float*)calloc(vrc_camera()->frame_px*vrc_camera()->frame_py,sizeof(float)*4);
    render_width = vrc_camera()->frame_px;
    render_height = vrc_camera()->frame_py;

    //Initiailize volume
    VRVOL *vol = readvol_ui8(volfile,volgridx,volgridy,volgridz,voldim);
    vrc_setvolume(vol);

    //Iintiailize light
    //position light above the center of the volume
    VRLIGHT *light = (VRLIGHT *)malloc(sizeof(VRLIGHT));
    vrl_init(light,(vol->bb_p1+vol->bb_p0)*.5f+glm::vec3(0,2.0,0));
    vrc_setlight(light);

    //Initialize transfer
    TFUNC *trans = (TFUNC *)malloc(sizeof(TFUNC));
    if(!vrt_init(trans,TF_LENGTH,MIN_TF_VAL,MAX_TF_VAL))
    {
        EPRINT("ERROR creating transfer functions\n");
        return 0;
    }
    vrc_settransfer(trans);

    //create texture for colormap
    glGenTextures(1,&colormap_tex);
    glBindTexture(GL_TEXTURE_1D,colormap_tex);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S,GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glBindTexture(GL_TEXTURE_1D,0);

    checkGL();
    return 1;
}



int
parse_cmdline(int argc, char **argv)
{
    if(argc < 5)
    {
        EPRINT("ERROR: Bad command line, how to use:\n");
        EPRINT("%s volume_file grid_x grid_y grid_z <dim_x dim_y dim_z>\n",argv[0]);
        EPRINT("volume_file - Raw volume of bytes\n");
        EPRINT("grid_[x,y,z] - Number of bytes in volume\n");
        EPRINT("dim_[x,y,z] - dimensions of volume (should be 0 < dim <= 1\n");
        return 0;
    }

    volfile = argv[1];
    volgridx = atoi(argv[2]);
    volgridy = atoi(argv[3]);
    volgridz = atoi(argv[4]);

    if(argc <= 5)
        return 1;

    if(argc > 5 && argc < 8)
    {
        EPRINT("Warning: Not enough of volume dimension information\n");
        return 1;
    }
    voldim = glm::vec3(atof(argv[5]),atof(argv[6]),atof(argv[7]));

    return 1;
}


/*
 *  Starts the glut main loop :-)
 *  For more complex programs just comment this out and
 *  make your own main.
 */
int
main(int argc, char **argv)
{
    //Initialize GLUT (extract glut arguments from command line)
    glutInit(&argc,argv);
    //Double buffered window, with Red Green Blue Alpha and Depth
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(win_width,win_height);
    glutCreateWindow(win_title);

    if(!parse_cmdline(argc,argv)){
        EPRINT("Error parsing commandline\n");
        exit(0);
    }

    if(!initialize()){
        EPRINT("Error initializing volume raycaster");
        exit(0);
    }

    //set callback functions for GLUT rendering loop
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(key);
    glutIdleFunc(idle);


    //starts the main rendering loop
    glutMainLoop();

    //should free the unused memory and clean
    //data structures but since we are exiting
    //we can be lazy.
    return 0;
}

void idle(){
    //for now do nothing;
    ;
}


//Remaps the screens coordinates and draws a rectangle the size of the screen
//these include texture coordinates, so that we can map a texture onto
//that quad which renders the entire
void render_fs_texture()
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-win_width/2.0,win_width/2.0,-win_height/2.0,win_height/2.0,1,20);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor4f(1,0,1,1);
    glTranslated(0,0,-1);
    glBegin(GL_QUADS);
    glTexCoord2d(0,0);glVertex3f(-win_width/2.0,-win_height/2.0,0);
    glTexCoord2d(1,0);glVertex3f(win_width/2.0,-win_height/2.0,0);
    glTexCoord2d(1,1);glVertex3f(win_width/2.0,win_height/2.0,0);
    glTexCoord2d(0,1);glVertex3f(-win_width/2.0,win_height/2.0,0);
    glEnd();
    glDisable(GL_TEXTURE_2D);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void render_tex_to_rec(float x0,float y0,float x1,float y1)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0,win_width,0,win_height,-1,1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor4f(1,0,1,1);
    glTranslated(0,0,-1);
    glBegin(GL_QUADS);
    glTexCoord2d(0,0);glVertex3f(x0,y0,0);
    glTexCoord2d(1,0);glVertex3f(x1,y0,0);
    glTexCoord2d(1,1);glVertex3f(x1,y1,0);
    glTexCoord2d(0,1);glVertex3f(x0,y1,0);
    glEnd();
    glDisable(GL_TEXTURE_2D);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

//rendering function
//here we render the image
//copy to a texture
//and draw to the screen
void display()
{
    //render texture to full screen quad
    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
    glLoadIdentity();

    if(!render_buff)
    {
        EPRINT("ERROR: render buffer is NULL\n");
        exit(0);
    }

    memset(render_buff,0,sizeof(float)*4*render_width*render_height);

    //render scene
    if(!vrc_render(render_buff))
    {
        EPRINT("Error rendering volume\n");
        exit(0);
    }

    //Enable textures, and copy render buffer to texture
    glEnable (GL_TEXTURE_2D);
    glTexEnvf (GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
    glBindTexture(GL_TEXTURE_2D,render_tex);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,render_width,render_height,
                 0,GL_RGBA,GL_FLOAT,render_buff);

    //render texture to screen
    render_fs_texture();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    //render the color bar
    glEnable( GL_TEXTURE_1D);
    glTexEnvf (GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
    glBindTexture(GL_TEXTURE_1D,colormap_tex);
    glTexImage1D(GL_TEXTURE_1D,0,GL_RGBA,vrc_transfer()->len,
                 0,GL_RGBA,GL_FLOAT,vrc_transfer()->colors);


    //this is a very simple example of drawing the transfer function
    //as a colormap and white line, this is terrible, and in a terrible location,
    //please make your own user interface for both displaying and interacting with
    //the transfer function.

    //render texture to screen
    render_tex_to_rec(0,0,win_width,10);
    glBindTexture(GL_TEXTURE_1D, 0);
    glDisable(GL_TEXTURE_1D);

    glBegin(GL_LINES);
    glColor3f(1.0,1.0,1.0);
    //render the opacity function
    float df = 2.0/vrc_transfer()->len;
    for(int i = 1; i < vrc_transfer()->len; i++){
        //get colors from tranfer function and draw
        //curve in the middle of the screen
        glm::vec4 ca1 = vrc_transfer()->colors[i-1];
        glm::vec4 ca2 = vrc_transfer()->colors[i];
        //sorry for the really crappy code, I shouldn't
        //use all these magic numbers and everything
        //but its supposed to be just a quick example
        //(note the bottom right of the screen is -1,-1
        //  only if the viewport is square see reshape()!!!)
        glVertex2d(-1.0+(i-1)*df,-1+.1*ca1.a);
        glVertex2d(-1.0+i*df,-1+.1*ca2.a);

    }
    glEnd();

    //swap buffers actually draws what we just rendered
    //to the screen
    glutSwapBuffers();
    checkGL();
}

void reshape(int width, int height)
{
    //update viewport match aspect ratio
    /*set up projection matrix to define the view port*/
    win_width = width;
    win_height = height;
    view_width = width*render_ratio;
    view_height = height*render_ratio;

    //keep the aspect ratio from changing
    float w = 1.0,h = 1.0;
    if(width>height)
    {
        w = (float) width/ (float) height;
        h = 1.0;
    }
    if(height>width)
    {
        w = 1.0;
        h = (float) height/ (float) width;
    }
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-w,w,-h,h,-10,10);
    glMatrixMode(GL_MODELVIEW);
    checkGL();

    //update camera
    cam_update_frame(vrc_camera(),view_width,view_height,w,h);

    //update render_buff
    if(render_buff)free(render_buff);
    //allocate rgba floating point framebuffer and intiailize to 0;
    render_buff = (float*)calloc(vrc_camera()->frame_px*vrc_camera()->frame_py,sizeof(float)*4);
    render_width = vrc_camera()->frame_px;
    render_height = vrc_camera()->frame_py;


}

void key(unsigned char ch, int x, int y){
    switch(ch)
    {
    case 'l':
        vrc_phong(0);
        glutPostRedisplay();
        break;
    case 'L':
        vrc_phong(1);
        glutPostRedisplay();
        break;

    default:
        EPRINT("Unknown keypress: %c\n",ch);
        break;
    };
}

//given a point 0<=(x,y)<= 1
//returns the point on a unit
//sphere, good for implementing
//a trackball camera
//however this is not ideal, see discussion in:
//https://www.khronos.org/opengl/wiki/Object_Mouse_Trackball
glm::vec3 spheremap(float x, float y)
{
    float z = 1-(x*x+y*y);
    z = z>0?sqrtf(z):0;
    return glm::vec3(x,y,z);
}


//Handles button press/release by mouse
void mouse(int button, int state, int x, int y)
{
    mousex = (x/(float)win_width*2.0-1.0f);
    mousey = (y/(float)win_height*2.0-1.0f);
    if(state !=GLUT_DOWN)
        mouseb = 0;
    else
        mouseb = button;
    switch(button)
    {
        case GLUT_LEFT_BUTTON:
            if(state == GLUT_DOWN)
            {
                ;
            }
            break;
        case GLUT_RIGHT_BUTTON:
            if(state == GLUT_DOWN)
            {
                ;
            }
            break;
        default:
            break;
    }
}

//handles mouse movement when a button is pressed
void motion(int x, int y)
{
    float mx = x;
    float my = y;

    mx = (mx/win_width*2.0f-1.0f);
    my = (my/win_height*2.0f-1.0f);


    switch(mouseb)
    {
        case GLUT_LEFT_BUTTON:
            cam_rotate_sphere(vrc_camera(),spheremap(mousex,mousey),spheremap(mx,my));
            break;
        case GLUT_RIGHT_BUTTON:
            break;
        default:
            break;
    }

    mousex = mx;
    mousey = my;
    glutPostRedisplay();
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
