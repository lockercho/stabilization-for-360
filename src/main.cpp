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
#endif

#include "Equirect2Cubic.h"
#include "Tracker.h"
#include "PlainModel.h"
//#include "SphereModel.h"
#include "Equirect2Cubic.h"
#include "Tracker.h"
#include "res_texture.c"

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

std::string filename = "walk_short.mp4";
VideoCapture capture;
bool opened_capture = false;

cv::Mat RGB, YUV;
Tracker tracker;

void
TexFunc(void)
{
    if(!opened_capture) {
        if(!capture.open(filename))
        {
            cout<<"Video Not Found"<<endl;
            return ;
        }
        opened_capture = true;
    }
    
    capture >> RGB;  //Read a frame from the video
    
    // Check if the frame has been read correctly or not
    if(RGB.empty()) {
        cout<<"Capture Finished"<<endl;
    }
    
	auto cube_face_size = 256;

	Equirect2Cubic myTransForm(RGB.cols, RGB.rows, cube_face_size, cube_face_size);

	Mat result[6];

	for (int i = 0; i<6; i++)
	{
		result[i] = Mat(cube_face_size, cube_face_size, RGB.type());
		myTransForm.remapWithMap(RGB, result[i], i);
	}

	tracker.Track(result);
    
    glEnable(GL_TEXTURE_2D);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

    cvtColor(RGB, RGB, CV_BGR2RGB);

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
                 RGB.data);
    
}


int init_resources() {
    
    nVertex = sizeof(model_vertices);
    glGenBuffers(1, &vbo_sprite_vertices);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_sprite_vertices);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model_vertices), model_vertices, GL_STATIC_DRAW);
    
    glGenBuffers(1, &vbo_sprite_texcoords);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_sprite_texcoords);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model_texcoords), model_texcoords, GL_STATIC_DRAW);
    
    glActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, // target
                 0,  // level, 0 = base, no minimap,
                 GL_RGB, // internalformat
                 res_texture.width,  // width
                 res_texture.height,  // height
                 0,  // border, always 0 in OpenGL ES
                 GL_RGB,  // format
                 GL_UNSIGNED_BYTE, // type
                 res_texture.pixel_data);
    
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

int main(int argc, char **argv) {
    GLenum type;
    
    glutInit(&argc, argv);
    
    Args(argc, argv);
    
    type = GLUT_RGB;
    type |= (doubleBuffer) ? GLUT_DOUBLE : GLUT_SINGLE;
    glutInitDisplayMode(type);
    glutInitWindowSize(width/2, height/2);
    glutCreateWindow("360 video");
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
