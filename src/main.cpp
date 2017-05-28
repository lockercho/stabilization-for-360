#include <iostream>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <OpenGL/gl.h>
#include <OpenGl/glu.h>
#include <GLUT/glut.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "RgbImage.h"
#include "Lock.h"

using namespace cv ;
using namespace std ;

__LOCK_TYPE__ buffer_lock;
static int win_width, win_height;
bool on_animation = true;
float animation_time = 0.0f;

int _baseYLoc, _baseULoc, _baseVLoc;
GLuint _baseYTexId, _baseUTexId, _baseVTexId;
int _posLoc, _texCoordLoc, _baseTextureMatrixLoc;


static float   camera_yaw = 0.0f;
static float   camera_pitch = -20.0f;
static float   camera_distance = 5.0f;

char vShaderStr[] =
"attribute vec4 a_position;                     \n"
"attribute vec2 a_texCoord;                     \n"
"uniform mat4 TextureMatrix;                    \n"
"varying vec2 v_texCoord;                       \n"
"void main()                                    \n"
"{                                              \n"
"   gl_Position = TextureMatrix * a_position;   \n"
"   v_texCoord = a_texCoord;                    \n"
"}                                              \n";

char fShaderStr[] =
"varying vec2 v_texCoord;                            \n"
"uniform sampler2D s_YMap;                           \n"
"uniform sampler2D s_UMap;                           \n"
"uniform sampler2D s_VMap;                           \n"
"void main()                                         \n"
"{                                                   \n"
"  float nx, ny, r, g, b, y, u, v;                   \n"
"  nx = v_texCoord.x;                                \n"
"  ny = v_texCoord.y;                                \n"
"  y = texture2D(s_YMap, vec2(nx, ny)).r;            \n"
"  u = texture2D(s_UMap, vec2(nx, ny)).r;            \n"
"  v = texture2D(s_VMap, vec2(nx, ny)).r;            \n"
"  if (0.0 < u)                                      \n"
"  {                                                 \n"
"     u = u - 0.5;                                   \n"
"  }                                                 \n"
"  if (0.0 < v)                                      \n"
"  {                                                 \n"
"     v = v - 0.5;                                   \n"
"  }                                                 \n"
"  r = y + 1.402 * v;                                \n"
"  g = y - 0.344 * u - 0.714 * v;                    \n"
"  b = y + 1.772 * u;                                \n"
"  gl_FragColor = vec4(r, g, b, 1.0);                \n"
"}                                                   \n";

GLuint loadShader(GLenum type, const char *shaderSrc) {
    GLuint shader;
    GLint compiled;
    
    // Create the shader object
    shader = glCreateShader(type);
    
    if (shader == 0)
        return 0;
    
    // Load the shader source
    glShaderSource(shader, 1, &shaderSrc, NULL);
    
    // Compile the shader
    glCompileShader(shader);
    
    // Check the compile status
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    
    if (!compiled) {
        GLint infoLen = 0;
        
        glGetShaderiv ( shader, GL_INFO_LOG_LENGTH, &infoLen);
        
        if (infoLen > 1) {
            char* infoLog = (char*) malloc(sizeof(char) * infoLen);
            glGetShaderInfoLog(shader, infoLen, NULL, infoLog);
            printf("Error compiling shader:\n%s\n", infoLog);
            free(infoLog);
        }
        
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

GLuint loadProgram(const char *vertShaderSrc, const char *fragShaderSrc) {
    GLuint vertexShader;
    GLuint fragmentShader;
    GLuint programObject;
    GLint linked;
    
    // Load the vertex/fragment shaders
    vertexShader = loadShader(GL_VERTEX_SHADER, vertShaderSrc);
    if (vertexShader == 0)
        return 0;
    
    fragmentShader = loadShader(GL_FRAGMENT_SHADER, fragShaderSrc);
    if (fragmentShader == 0) {
        glDeleteShader(vertexShader);
        return 0;
    }
    
    // Create the program object
    programObject = glCreateProgram();
    
    if (programObject == 0)
        return 0;
    
    glAttachShader(programObject, vertexShader);
    glAttachShader(programObject, fragmentShader);
    
    // Link the program
    glLinkProgram(programObject);
    
    // Check the link status
    glGetProgramiv(programObject, GL_LINK_STATUS, &linked);
    
    if (!linked) {
        GLint infoLen = 0;
        
        glGetProgramiv(programObject, GL_INFO_LOG_LENGTH, &infoLen);
        
        if (infoLen > 1) {
            char* infoLog = (char*) malloc(sizeof(char) * infoLen);
            glGetShaderInfoLog(programObject, infoLen, NULL, infoLog);
            printf("Error compiling shader:\n%s\n", infoLog);
            free(infoLog);
        }
        glDeleteProgram(programObject);
        return 0;
    }
    
    // Free up no longer needed shader resources
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    
    return programObject;
}

GLuint loadTexture() {
    GLuint texture;
    
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, 0, 0, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, NULL);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    return texture;
}


void  reshape( int w, int h ) {
    glViewport(0, 0, w, h);
    
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( 45, (double)w/h, 1, 500 );
    
    win_width = w;
    win_height = h;
}


void testUpdateTexture() {
    std::string filename = "/Users/lockercho/workspace/GPU/final_project/walk_short.mp4";
    
    VideoCapture capture;
    cv::Mat RGB, YUV;
    
    if(!capture.open(filename))
    {
        cout<<"Video Not Found"<<endl;
        return ;
    }
    
    capture >> RGB;  //Read a frame from the video
        
    // Check if the frame has been read correctly or not
    if(RGB.empty()) {
        cout<<"Capture Finished"<<endl;
//        break;
    }
        
    cv::cvtColor(RGB, YUV, CV_BGR2YCrCb);
    
    Mat chan[3];
    split(YUV,chan);
    
    Mat y  = chan[0];
    Mat u = chan[1];
    Mat v = chan[2];
    
    namedWindow("Y", CV_WINDOW_AUTOSIZE );
    imshow("Y",y);
    
    namedWindow("U", CV_WINDOW_AUTOSIZE );
    imshow("U",u);
    
    namedWindow("V", CV_WINDOW_AUTOSIZE );
    imshow("V",v);
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _baseYTexId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, RGB.cols, RGB.rows, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE,
                 (uint8_t*)y.data);
    //        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, strideY, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, textureY);
    glUniform1i(_baseYLoc, 0);
    
    // Bind the U map
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, _baseUTexId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, RGB.cols, RGB.rows / 2, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, (uint8_t*)u.data);
    glUniform1i(_baseULoc, 1);
    
    // Bind the V map
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, _baseVTexId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, RGB.cols, RGB.rows / 2, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, (uint8_t*)v.data);
    glUniform1i(_baseVLoc, 2);
    
    capture.release();
}

void  display() {
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
    
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    glTranslatef( 0.0, 0.0, - camera_distance );

//    char  message[ 64 ];
//    sprintf( message, "Press 'L' key to Load a BVH file" );
//    drawMessage( 0, message );
//    fprintf(stderr, "qwe");
    
    glFlush();
    glDisable(GL_TEXTURE_2D);
    glutSwapBuffers();
}


int main(int argc, char ** argv) {
    
    __INIT_LOCK__(&buffer_lock);

    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL );
    glutInitWindowSize( 640, 640 );
    glutInitWindowPosition( 0, 0 );
    glutCreateWindow("Player");

    GLuint shader = glCreateShader(GL_VERTEX_SHADER);

    GLuint program = loadProgram(vShaderStr, fShaderStr);
    
    _baseYLoc = glGetUniformLocation(program, "s_YMap");
    _baseULoc = glGetUniformLocation(program, "s_UMap");
    _baseVLoc = glGetUniformLocation(program, "s_VMap");
//
    // Load the textures
    _baseYTexId = loadTexture();
    _baseUTexId = loadTexture();
    _baseVTexId = loadTexture();
    
    // Get the attribute locations
    _posLoc = glGetAttribLocation(program, "a_position");
    _texCoordLoc = glGetAttribLocation(program, "a_texCoord");
    _baseTextureMatrixLoc = glGetUniformLocation(program, "TextureMatrix");
    
    testUpdateTexture();
    
    // Use the program object
    glUseProgram(program);
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    
    glutMainLoop();

    return 0;
}
