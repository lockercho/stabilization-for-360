#ifndef _WIN32
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <GLUT/glut.h>
#include <OpenGL/gl3.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
#endif

#include "Equirect2Cubic.h"
#include "Tracker.h"
#include "PlainModel.h"
//#include "SphereModel.h"
#include "VideoHandler.h"




//#include "res_texture.c"

using namespace std;
using namespace cv;

static int width = 1920;
static int height = 1080;
static int nVertex = 0;

GLuint vbo_sprite_vertices, vbo_sprite_texcoords;
GLuint program;
GLuint texture_id;
GLint attribute_v_coord, attribute_v_texcoord;
GLint uniform_mvp, uniform_mytexture;

bool doubleBuffer;



/* ARGSUSED1 */
static void Key(unsigned char key, int x, int y) {
    switch (key) {
        case 27:
            exit(0);
    }
}

/**
 * Store all the file's contents in memory, useful to pass shaders
 * source code to OpenGL
 */
char* file_read(const char* filename)
{
    FILE* in = fopen(filename, "rb");
    if (in == NULL) return NULL;
    
    int res_size = BUFSIZ;
    char* res = (char*)malloc(res_size);
    int nb_read_total = 0;
    
    while (!feof(in) && !ferror(in)) {
        if (nb_read_total + BUFSIZ > res_size) {
            if (res_size > 10*1024*1024) break;
            res_size = res_size * 2;
            res = (char*)realloc(res, res_size);
        }
        char* p_res = res + nb_read_total;
        nb_read_total += fread(p_res, 1, BUFSIZ, in);
    }
    
    fclose(in);
    res = (char*)realloc(res, nb_read_total + 1);
    res[nb_read_total] = '\0';
    return res;
}

/**
 * Display compilation errors from the OpenGL shader compiler
 */
void print_log(GLuint object)
{
    GLint log_length = 0;
    if (glIsShader(object))
        glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
    else if (glIsProgram(object))
        glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
    else {
        fprintf(stderr, "printlog: Not a shader or a program\n");
        return;
    }
    
    char* log = (char*)malloc(log_length);
    
    if (glIsShader(object))
        glGetShaderInfoLog(object, log_length, NULL, log);
    else if (glIsProgram(object))
        glGetProgramInfoLog(object, log_length, NULL, log);
    
    fprintf(stderr, "%s", log);
    free(log);
}
/**
 * Compile the shader from file 'filename', with error handling
 */
GLuint create_shader(const char* filename, GLenum type)
{
    const GLchar* source = file_read(filename);
    if (source == NULL) {
        fprintf(stderr, "Error opening %s: ", filename); perror("");
        return 0;
    }
    GLuint res = glCreateShader(type);
    const GLchar* sources[] = {
        // Define GLSL version
#ifdef GL_ES_VERSION_2_0
        "#version 100\n"  // OpenGL ES 2.0
#else
        "#version 120\n"  // OpenGL 2.1
#endif
        ,
        // GLES2 precision specifiers
#ifdef GL_ES_VERSION_2_0
        // Define default float precision for fragment shaders:
        (type == GL_FRAGMENT_SHADER) ?
        "#ifdef GL_FRAGMENT_PRECISION_HIGH\n"
        "precision highp float;           \n"
        "#else                            \n"
        "precision mediump float;         \n"
        "#endif                           \n"
        : ""
        // Note: OpenGL ES automatically defines this:
        // #define GL_ES
#else
        // Ignore GLES 2 precision specifiers:
        "#define lowp   \n"
        "#define mediump\n"
        "#define highp  \n"
#endif
        ,
        source };
    
    fprintf(stderr, "\n====SHADER====\n\n%s\n", source);
    glShaderSource(res, 3, sources, NULL);
    free((void*)source);
    
    glCompileShader(res);
    GLint compile_ok = GL_FALSE;
    glGetShaderiv(res, GL_COMPILE_STATUS, &compile_ok);
    if (compile_ok == GL_FALSE) {
        fprintf(stderr, "%s:", filename);
        print_log(res);
        glDeleteShader(res);
        return 0;
    }
    
    return res;
}

void writeVideo(FILE * out)
{
    
    GLint vp[4];
    glGetIntegerv( GL_VIEWPORT, vp );
    
    int x,y, w,h;
    x = vp[0];
    y = vp[1];
    w = vp[2];
    h = vp[3];
    
    int j;
    
    unsigned char *bottomup_pixel = (unsigned char *) malloc( w*h*3*sizeof(unsigned char) );
    unsigned char *topdown_pixel = (unsigned char *) malloc( w*h*3*sizeof(unsigned char) );
    
    
    //Byte alignment (that is, no alignment)
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    
    glReadPixels( x, y, w, h, GL_RGB, GL_UNSIGNED_BYTE, bottomup_pixel);
    for( j=0; j<h; j++ )
        memcpy( &topdown_pixel[j*w*3], &bottomup_pixel[(h-j-1)*w*3], w*3*sizeof(unsigned char) );
    
    
    if(out==NULL )
    {
        printf( "[Error] : SaveScreen()\n");
//        exit(-1);
        return;
    }
    
//    fprintf( f0, "P6\n%d %d\n255\n", w, h);
    fwrite( topdown_pixel, sizeof(unsigned char), w*h*3, out);
//    fclose( f0 );
    
    free(bottomup_pixel);
    free(topdown_pixel);
}

void save_screen( const char *spath )
{
    
    GLint vp[4];
    glGetIntegerv( GL_VIEWPORT, vp );
    
    int x,y, w,h;
    x = vp[0];
    y = vp[1];
    w = vp[2];
    h = vp[3];
    
    int j;
    
    unsigned char *bottomup_pixel = (unsigned char *) malloc( w*h*3*sizeof(unsigned char) );
    unsigned char *topdown_pixel = (unsigned char *) malloc( w*h*3*sizeof(unsigned char) );
    
    
    //Byte alignment (that is, no alignment)
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    
    glReadPixels( x, y, w, h, GL_RGB, GL_UNSIGNED_BYTE, bottomup_pixel);
    for( j=0; j<h; j++ )
        memcpy( &topdown_pixel[j*w*3], &bottomup_pixel[(h-j-1)*w*3], w*3*sizeof(unsigned char) );
    
    
    FILE *f0 = fopen( spath, "wb" );
    if( f0==NULL )
    {
        printf( "[Error] : SaveScreen(), Cannot open %s for writting.\n", spath );
        exit(-1);
    }
    
    fprintf( f0, "P6\n%d %d\n255\n", w, h);
    fwrite( topdown_pixel, sizeof(unsigned char), w*h*3, f0);
    fclose( f0 );
    
    free(bottomup_pixel);
    free(topdown_pixel);
}

std::string filename = "walk_short.mp4";
VideoHandler videoHandler(filename);



VideoCapture capture;
bool opened_capture = false;


int nFrames = 0;
cv::Mat RGB, YUV;
Tracker tracker;

FILE * result_video = NULL;

// fake rotation matrix
//float R = {
//    1.0, 0.0, 3.0,
//    0.0, 1.0, 3.0,
//    0.0, 0.0, 1.0,
//};

void videoProcessThread() {
    videoHandler.start();
}


void printRotations(std::vector<cv::Mat> R) {
    std::cout << std::endl << "printRotations: " << std::endl;
    for(int k=0 ; k<6 ; k++) {
        std::cout << k << ":" << std::endl;
        for(int i=0 ; i<9 ; i++) {
            std::cout << R[k].at<double>(i/3, i%3) << ", ";
            if((i+1) % 3 == 0) {
                std::cout << std::endl;
            }
        }
        std::cout << std::endl;
        std::cout << std::endl;
    }
}

Equirect2Cubic myTransForm(1920, 1080, 256, 256);

void applyRotation(float x , float y, float &nx, float &ny, cv::Mat * trans) {
//    if(x < 0 || x >=1 || y < 0 || y >= 1) {
//        std::cout << "XY: "<< x << ", " << y << std::endl;
//    }
    // src = [x, y, 1]
    // trans = 1x9 vector
    std::vector<float> res;
    nx = x * trans->at<double>(0, 0) + y * trans->at<double>(1, 0) + 1.0 * trans->at<double>(2, 0);
    ny = x * trans->at<double>(0, 1) + y * trans->at<double>(1, 1) + 1.0 * trans->at<double>(2, 1);
}

// https://en.wikipedia.org/wiki/Great-circle_distance
float getGreatCircalDistance(float u1, float v1, float u2, float v2) {
    // u = [0, 1] represents [0, 2PI]
    // v = [0, 1] represents [0, PI]
    // let r = 1.0
    float r = 1.0;
    u1 *= 2 * M_PI;
    u2 *= 2 * M_PI;
    v1 *= M_PI;
    v2 *= M_PI;
    float d = r * acos(sin(v1)*sin(v2)+cos(v1)*cos(v2)*cos(u1-u2));
    return d;
}

int Rmap[8][3] = {
    {0, 1, 4},
    {1, 2, 4},
    {2, 3, 4},
    {3, 0, 4},
    {0, 1, 5},
    {1, 2, 5},
    {2, 3, 5},
    {3, 0, 5},
};

cv::Mat getBlendedRotation(std::vector<cv::Mat> * R, float u, float v) {
    // 6 rotations cut sphere into 8 areas, 2x4
    int Rmap_index = (int) (u*4) + (v>0.5 ? 4 : 0);
    if(Rmap_index >=8) Rmap_index-=1;
    // get blended
    cv::Mat res = cv::Mat_<double>::zeros(3,3);
    float d = 1000;
    for(int i=0 ;i<3 ; i++) {
        int r_index = Rmap[Rmap_index][i];
        float Ru = myTransForm.cubeFacing[r_index][0] / 2 / M_PI + 0.5;
        float Rv = myTransForm.cubeFacing[r_index][1] / M_PI + 0.5;
        float dis = getGreatCircalDistance(u, v, Ru, Rv);
        if(d > dis && r_index <4) {
            res = R->at(r_index);
            d = dis;
        }
//        res += dis * R->at(r_index);
//        d += dis;
    }
    return res; // d;
}

int display_index = 0;
void
TexFunc(void)
{
    cv::Mat * RGB;
    std::vector<cv::Mat> R;
    fprintf(stderr, "display: %d\n", display_index);
    
    if(videoHandler.isFrameOk(display_index)) {
        RGB = videoHandler.getFrame(display_index);
        R = videoHandler.getRotation(display_index);
        display_index++;
    } else {
        return ;
    }
    
    glEnable(GL_TEXTURE_2D);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    
    
    nVertex = sizeof(model_vertices);

    // model definition
    glGenBuffers(1, &vbo_sprite_vertices);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_sprite_vertices);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model_vertices), model_vertices, GL_STATIC_DRAW);
    
    // texture coordinate definition
    // apply rotations to texcoord
    int nTexs =sizeof(model_texcoords) / sizeof(float);
    GLfloat coords[nTexs];

    for(int i=0 ; i<nTexs /2 ; i+=1) {
        float tmpx, tmpy;
//        int fid;
        float u = model_texcoords[i*2];
        float v = model_texcoords[i*2+1];
        cv::Mat B = getBlendedRotation(&R, u, v);
        // get faceId
        // myTransForm.uv2xy(model_texcoords[i*2], model_texcoords[i*2+1], 256, 256, fid, tmpx, tmpy);
        
        // get regional u,v by faceid
        if(v >= 0.25 && v <= 0.75) {
            // middle points
            int fid = (int)(u * 4);
            // 0 ~ 1
//            float uc = u * 4 - fid; // 0.0 ~ 0.25 -> 0.0 ~ 1.0
//            float vc = (v - 0.25) / 0.5; // 0.25 ~ 0.75 -> 0.0 ~ 1.0
//            float tmpu, tmpv;
//            applyRotation(uc, vc, tmpu, tmpv, &B);
//            coords[i*2] = (tmpu + (int) (u*4)) / 4.0;
//            coords[i*2+1] = tmpv * 0.5 + 0.25;  // 0.0 ~1.0 -> 0.25 ~ 0.75
//            
            // -1 ~ 1
            float uc = (u * 4 - fid) * 2 - 1.0; // 0.0 ~ 0.25 -> 0.0 ~ 1.0
            float vc = ((v - 0.25) / 0.5) * 2 - 1.0; // 0.25 ~ 0.75 -> 0.0 ~ 1.0
            float tmpu, tmpv;
            applyRotation(uc, vc, tmpu, tmpv, &B);
            coords[i*2] = ((tmpu + 1.0) / 2 + fid) / 4.0;
            coords[i*2+1] = (tmpv + 1.0) / 2 * 0.5 + 0.25;  // 0.0 ~1.0 -> 0.25 ~ 0.75
        } else {
            coords[i*2] = model_texcoords[i*2];
            coords[i*2+1] = model_texcoords[i*2+1];
        }

//        std::cout << "coorX: "<< model_texcoords[i*2]<<", coorY: " << model_texcoords[i*2+1]<< ", tmpx: " << tmpx << ", tmpy: " << tmpy << std::endl;
//        
    }
    
    glGenBuffers(1, &vbo_sprite_texcoords);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_sprite_texcoords);
    glBufferData(GL_ARRAY_BUFFER, sizeof(coords), coords, GL_STATIC_DRAW);

    glActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    
    glTexImage2D(GL_TEXTURE_2D, // target
                 0,  // level, 0 = base, no minimap,
                 GL_RGB, // internalformat
                 width,  // width
                 height,  // height
                 0,  // border, always 0 in OpenGL ES
                 GL_RGB,  // format
                 GL_UNSIGNED_BYTE, // type
                 RGB->data);
    
//    free(coords);
    
//    if(result_video == NULL) {
//        GLint vp[4];
//        glGetIntegerv( GL_VIEWPORT, vp );
//        
//        int x,y, w,h;
//        x = vp[0];
//        y = vp[1];
//        w = vp[2];
//        h = vp[3];
//        char ffmpeg_comm[300];
//        snprintf(ffmpeg_comm, 300,"/usr/local/bin/ffmpeg -v warning -vcodec rawvideo -f rawvideo -pix_fmt rgb24 -s %dx%d -i pipe:0 -vcodec h264 -r 60 -y /Users/lockercho/workspace/GPU/final_project/stabilization-for-360/out.avi", w, h);
//        result_video = popen(ffmpeg_comm, "w");
//    }
//    
//    GLint vp[4];
//    glGetIntegerv( GL_VIEWPORT, vp );
//    
//    int x,y, w,h;
//    x = vp[0];
//    y = vp[1];
//    w = vp[2];
//    h = vp[3];
//    
//    int j;
    
//    unsigned char *bottomup_pixel = (unsigned char *) malloc( w*h*3*sizeof(unsigned char) );
//    unsigned char *topdown_pixel = (unsigned char *) malloc( w*h*3*sizeof(unsigned char) );
//    
//    
//    //Byte alignment (that is, no alignment)
//    glPixelStorei(GL_PACK_ALIGNMENT, 1);
//    glReadBuffer(GL_FRONT);
//    glReadPixels( x, y, w, h, GL_RGB, GL_UNSIGNED_BYTE, bottomup_pixel);
//    for( j=0; j<h; j++ )
//        memcpy( &topdown_pixel[j*w*3], &bottomup_pixel[(h-j-1)*w*3], w*3*sizeof(unsigned char) );
//    
//    
//    if(result_video==NULL )
//    {
//        printf( "[Error] : SaveScreen()\n");
//        //        exit(-1);
//        return;
//    }
//    
//    fwrite( topdown_pixel, sizeof(unsigned char), w*h*3, result_video);
//
//    
//    free(bottomup_pixel);
//    free(topdown_pixel);

}


int init_resources() {
    
    GLint link_ok = GL_FALSE;
    
    GLuint vs, fs;
    if ((vs = create_shader("sprites.v.glsl", GL_VERTEX_SHADER))   == 0) return 0;
    if ((fs = create_shader("sprites.f.glsl", GL_FRAGMENT_SHADER)) == 0) return 0;
    
    program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &link_ok);
    if (!link_ok) {
        fprintf(stderr, "glLinkProgram:");
        print_log(program);
        return 0;
    }
    
    const char* attribute_name;
    attribute_name = "v_coord";
    attribute_v_coord = glGetAttribLocation(program, attribute_name);
    if (attribute_v_coord == -1) {
        fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
        return 0;
    }
    attribute_name = "v_texcoord";
    attribute_v_texcoord = glGetAttribLocation(program, attribute_name);
    if (attribute_v_texcoord == -1) {
        fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
        return 0;
    }
    const char* uniform_name;
    uniform_name = "mvp";
    uniform_mvp = glGetUniformLocation(program, uniform_name);
    if (uniform_mvp == -1) {
        fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
        return 0;
    }
    uniform_name = "mytexture";
    uniform_mytexture = glGetUniformLocation(program, uniform_name);
    if (uniform_mytexture == -1) {
        fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
        return 0;
    }
    
    return 1;
}

static void Init(void) {
    init_resources();
    return;
}

static void Draw(void) {
    glUseProgram(program);
    
    glUniform1i(uniform_mytexture, /*GL_TEXTURE*/ 0);
    glClearColor(0, 0, 0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    TexFunc();
    
    glEnableVertexAttribArray(attribute_v_coord);
    // Describe our vertices array to OpenGL (it can't guess its format automatically)
    glBindBuffer(GL_ARRAY_BUFFER, vbo_sprite_vertices);
    glVertexAttribPointer(attribute_v_coord, // attribute
                          3,                 // number of elements per vertex, here (x,y,z)
                          GL_FLOAT,          // the type of each element
                          GL_FALSE,          // take our values as-is
                          0,                 // no extra data between each position
                          0                  // offset of first element
                          );
    
    glEnableVertexAttribArray(attribute_v_texcoord);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_sprite_texcoords);
    glVertexAttribPointer(
                          attribute_v_texcoord, // attribute
                          2,                  // number of elements per vertex, here (x,y)
                          GL_FLOAT,           // the type of each element
                          GL_FALSE,           // take our values as-is
                          0,                  // no extra data between each position
                          0                   // offset of first element
                          );
    
    /* Push each element in buffer_vertices to the vertex shader */
    glDrawArrays(GL_TRIANGLES, 0, nVertex);
    
    glDisableVertexAttribArray(attribute_v_coord);
    glDisableVertexAttribArray(attribute_v_texcoord);
    
    if (doubleBuffer) {
        glutSwapBuffers();
    } else {
        glFlush();
    }
}

static void Args(int argc, char **argv) {
    GLint i;
    
    doubleBuffer = GL_TRUE;
    
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-sb") == 0) {
            doubleBuffer = GL_FALSE;
        } else if (strcmp(argv[i], "-db") == 0) {
            doubleBuffer = GL_TRUE;
        }
    }
}

void printVersion() {
    const char *GLVersionString = (const char *) glGetString(GL_VERSION);
    fprintf(stderr, "OpenGL Version: %s\n", GLVersionString);
}

void timer( int value ) {
    glutPostRedisplay();
    glutTimerFunc( 16, timer, 0 );
}

void onIdle() {
    glUseProgram(program);
    float mvp[16] = {0};
    mvp[0] = mvp[5] = mvp[10] = mvp[15] = 1;
    glUniformMatrix4fv(uniform_mvp, 1, GL_FALSE, &mvp[0]);
    glutPostRedisplay();
}

//bool initPthreadAttr(pthread_attr_t* attr) {
//    int stackSize = 1024 * 512;
//    return (pthread_attr_init(attr) == 0 && pthread_attr_setstacksize(attr, stackSize) == 0);
//}
//
//bool releasePthreadAttr(pthread_attr_t* attr) {
//    return pthread_attr_destroy(attr) == 0;
//}

int main(int argc, char **argv) {
    GLenum type;
    
    // init video processing thread
//    pthread_attr_t attr;
//    initPthreadAttr(&attr);
//    
//    // init websocket
//    pthread_t videoThread;
//    pthread_create(&videoThread, &attr, videoProcessThread, (void*)NULL);
//    
//    releasePthreadAttr(&attr);
    
    // execute thread
    thread mThread(videoProcessThread);
    
    glutInit(&argc, argv);
    
    Args(argc, argv);
    
    type = GLUT_RGB;
    type |= (doubleBuffer) ? GLUT_DOUBLE : GLUT_SINGLE;
    glutInitDisplayMode(type);
    glutInitWindowSize(width/2, height/2);
    glutCreateWindow("360 video");

#ifdef _WIN32

	glewExperimental = GL_TRUE;
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		// Problem: glewInit failed, something is seriously wrong.
		cout << "glewInit failed: " << glewGetErrorString(err) << endl;
		exit(1);
	}

#endif

    if (!glutExtensionSupported("GL_EXT_abgr")) {
        printf("Couldn't find abgr extension.\n");
        exit(0);
    }
    
    printVersion();

#if !GL_EXT_abgr
    printf("WARNING: client-side OpenGL has no ABGR extension support!\n");
    printf("         Drawing only RGBA (and not ABGR) images and textures.\n");
#endif
    Init();

    glutKeyboardFunc(Key);
    glutIdleFunc(onIdle);
    glutTimerFunc(0, timer, 0);
    glutDisplayFunc(Draw);
    glutMainLoop();
    return 0;             /* ANSI C requires main to return int. */
}
