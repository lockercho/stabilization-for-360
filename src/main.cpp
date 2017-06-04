
/* Copyright (c) Mark J. Kilgard, 1994. */

/**
 * (c) Copyright 1993, Silicon Graphics, Inc.
 * ALL RIGHTS RESERVED
 * Permission to use, copy, modify, and distribute this software for
 * any purpose and without fee is hereby granted, provided that the above
 * copyright notice appear in all copies and that both the copyright notice
 * and this permission notice appear in supporting documentation, and that
 * the name of Silicon Graphics, Inc. not be used in advertising
 * or publicity pertaining to distribution of the software without specific,
 * written prior permission.
 *
 * THE MATERIAL EMBODIED ON THIS SOFTWARE IS PROVIDED TO YOU "AS-IS"
 * AND WITHOUT WARRANTY OF ANY KIND, EXPRESS, IMPLIED OR OTHERWISE,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY OR
 * FITNESS FOR A PARTICULAR PURPOSE.  IN NO EVENT SHALL SILICON
 * GRAPHICS, INC.  BE LIABLE TO YOU OR ANYONE ELSE FOR ANY DIRECT,
 * SPECIAL, INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY
 * KIND, OR ANY DAMAGES WHATSOEVER, INCLUDING WITHOUT LIMITATION,
 * LOSS OF PROFIT, LOSS OF USE, SAVINGS OR REVENUE, OR THE CLAIMS OF
 * THIRD PARTIES, WHETHER OR NOT SILICON GRAPHICS, INC.  HAS BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH LOSS, HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, ARISING OUT OF OR IN CONNECTION WITH THE
 * POSSESSION, USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * US Government Users Restricted Rights
 * Use, duplication, or disclosure by the Government is subject to
 * restrictions set forth in FAR 52.227.19(c)(2) or subparagraph
 * (c)(1)(ii) of the Rights in Technical Data and Computer Software
 * clause at DFARS 252.227-7013 and/or in similar or successor
 * clauses in the FAR or the DOD or NASA FAR Supplement.
 * Unpublished-- rights reserved under the copyright laws of the
 * United States.  Contractor/manufacturer is Silicon Graphics,
 * Inc., 2011 N.  Shoreline Blvd., Mountain View, CA 94039-7311.
 *
 * OpenGL(TM) is a trademark of Silicon Graphics, Inc.
 */

/* abgr.c - Demonstrates the use of the extension EXT_abgr.
 
 The same image data is used for both ABGR and RGBA formats
 in glDrawPixels and glTexImage2D.  The left side uses ABGR,
 the right side RGBA.  The top polygon demonstrates use of texture,
 and the bottom image is drawn with glDrawPixels.
 
 Note that the textures are defined as 3 component, so the alpha
 value is not used in applying the DECAL environment.  */

#ifndef _WIN32
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/gl3.h>
#include <iostream>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

using namespace std;
using namespace cv;

static int width = 1920;
static int height = 1080;

GLenum doubleBuffer;
GLubyte ubImage[65536];


char vs_str[] =
"vec3 vertex_position;\n"
"vec2 vt;\n"
"uniform mat4 view, proj;\n"
"vec2 texture_coordinates;\n"
"void main() {\n"
"    texture_coordinates = vt;\n"
"    gl_Position = proj * view * vec4 (vertex_position, 1.0);\n"
"}\n";

char fs_str[] =
"vec3 colour;\n"
"vec4 frag_colour;\n"
"void main() {\n"
"    frag_colour = vec4(colour, 1.0);\n"
"}\n";

GLuint shader_program;
GLuint vao;

bool is_programme_valid( GLuint sp ) {
    glValidateProgram( sp );
    GLint params = -1;
    glGetProgramiv( sp, GL_VALIDATE_STATUS, &params );
    if ( GL_TRUE != params ) {
        fprintf(stderr, "program %i GL_VALIDATE_STATUS = GL_FALSE\n", sp );
//        print_programme_info_log( sp );
        return false;
    }
    fprintf(stderr, "program %i GL_VALIDATE_STATUS = GL_TRUE\n", sp );
    return true;
}


GLuint createProgram(char * vShaderStr, char * fShaderStr) {
    GLuint vertShader, fragShader, program;
    
    // init vertex shader
    vertShader = glCreateShader(GL_VERTEX_SHADER);
    const GLchar *p = (const GLchar *) vShaderStr;
    glShaderSource( vertShader, 1, &p, NULL );
    glCompileShader(vertShader);
    // check for compile errors
    int params = -1;
    glGetShaderiv(vertShader, GL_COMPILE_STATUS, &params );
    if ( GL_TRUE != params ) {
        fprintf(stderr, "ERROR: GL shader index %i did not compile\n", vertShader);
//        print_shader_info_log( vertShader);
        int max_length = 2048;
        int actual_length = 0;
        char log[2048];
        glGetShaderInfoLog(vertShader, max_length, &actual_length, log );
        fprintf(stderr, "GL index %i:\n%s\n", vertShader, log );
        return false; // or exit or something
    }
    fprintf(stderr, "shader compiled. index %i\n", vertShader);
    
    // init vertex shader
    fragShader = glCreateShader(GL_FRAGMENT_SHADER);
    p = (const GLchar *) fShaderStr;
    glShaderSource(fragShader, 1, &p, NULL );
    glCompileShader(fragShader);
    // check for compile errors
    params = -1;
    glGetShaderiv(fragShader, GL_COMPILE_STATUS, &params );
    if ( GL_TRUE != params ) {
        fprintf(stderr, "ERROR: GL shader index %i did not compile\n", fragShader);
        int max_length = 2048;
        int actual_length = 0;
        char log[2048];
        glGetShaderInfoLog(fragShader, max_length, &actual_length, log );
        fprintf(stderr, "GL index %i:\n%s\n", fragShader, log );
        //        print_shader_info_log( vertShader);
        return false; // or exit or something
    }
    fprintf(stderr, "shader compiled. index %i\n", fragShader);
    
    program = glCreateProgram();
    fprintf(stderr, "created programme %u. attaching shaders %u and %u...\n", program, vertShader, fragShader);
    glAttachShader(program, vertShader);
    glAttachShader(program, fragShader);
    // link the shader programme. if binding input attributes do that before link
    glLinkProgram(program);
    params = -1;
    glGetProgramiv(program, GL_LINK_STATUS, &params );
    if ( GL_TRUE != params ) {
        fprintf(stderr, "ERROR: could not link shader programme GL index %u\n",
                   program);
//        print_programme_info_log(program);
        return false;
    }
    is_programme_valid(program);
    // delete shaders here to free memory
    glDeleteShader( vertShader );
    glDeleteShader( fragShader );
    
    return program;
}

static void
Init(void)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 1.0, 0.1, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glDisable(GL_DITHER);
    
    
    /* OTHER STUFF GOES HERE NEXT */
    GLfloat points[] = { -0.5f, -0.5f, -2.0f, 0.5f,	-0.5f, -2.0f, 0.5f,	0.5f, -2.0f,
        0.5f,	0.5f,-2.0f, -0.5f, 0.5f,	-2.0f, -0.5f, -0.5f, -2.0f};
    
    // 2^16 = 65536
    GLfloat texcoords[] = { 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f };
    
    GLuint points_vbo;
    glGenBuffers( 1, &points_vbo );
    glBindBuffer( GL_ARRAY_BUFFER, points_vbo );
    glBufferData( GL_ARRAY_BUFFER, 18 * sizeof( GLfloat ), points, GL_STATIC_DRAW );
    
    GLuint texcoords_vbo;
    glGenBuffers( 1, &texcoords_vbo );
    glBindBuffer( GL_ARRAY_BUFFER, texcoords_vbo );
    glBufferData( GL_ARRAY_BUFFER, 12 * sizeof( GLfloat ), texcoords,
                 GL_STATIC_DRAW );
    
//    GLuint vao;
    glGenVertexArrays( 1, &vao );
    glBindVertexArray( vao );
    glBindBuffer( GL_ARRAY_BUFFER, points_vbo );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, NULL );
    glBindBuffer( GL_ARRAY_BUFFER, texcoords_vbo );
    glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, 0, NULL ); // normalise!
    glEnableVertexAttribArray( 0 );
    glEnableVertexAttribArray( 1 );
    
    shader_program = createProgram(vs_str, fs_str);
    
    return;
}

/* ARGSUSED1 */
static void
Key(unsigned char key, int x, int y)
{
    switch (key) {
        case 27:
            exit(0);
    }
}

std::string filename = "walk_short.mp4";
VideoCapture capture;
bool opened_capture = false;

cv::Mat RGB, YUV;

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
        //        break;
    }
    
//    cv::cvtColor(RGB, YUV, CV_BGR2YCrCb);
    
//    Mat chan[3];
//    split(YUV,chan);
//    
//    Mat y  = chan[0];
//    Mat u = chan[1];
//    Mat v = chan[2];
    
//        namedWindow("Y", CV_WINDOW_AUTOSIZE );
//        imshow("Y",RGB);
    //
    //    namedWindow("U", CV_WINDOW_AUTOSIZE );
    //    imshow("U",u);
    //
    //    namedWindow("V", CV_WINDOW_AUTOSIZE );
    //    imshow("V",v);
    
    glEnable(GL_TEXTURE_2D);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    
//    glTexImage2D(GL_TEXTURE_2D, 0, 3, 128, 128, 0, GL_RGBA,
//                 GL_UNSIGNED_BYTE, ubImage);
    cvtColor(RGB, RGB, CV_BGR2RGB);
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, (uint8_t*) RGB.data);
    
    
//    glBegin(GL_POLYGON);
//    glTexCoord2f(1.0, 1.0);
//    glVertex3f(0.8, 0.8, -2.0);
//    glTexCoord2f(0.0, 1.0);
//    glVertex3f(0.2, 0.8, -100.0);
//    glTexCoord2f(0.0, 0.0);
//    glVertex3f(0.2, 0.2, -100.0);
//    glTexCoord2f(1.0, 0.0);
//    glVertex3f(0.8, 0.2, -2.0);
//    glEnd();
    
    glBegin(GL_POLYGON);
    
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1, 1, -2.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1, 1, -2.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1, -1, -2.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1, -1, -2.0);
    glEnd();
    
    glDisable(GL_TEXTURE_2D);
}

static void
Draw(void)
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    
    glUseProgram( shader_program);
    glBindVertexArray( vao );
    // draw points 0-3 from the currently bound VAO with current in-use shader
    glDrawArrays( GL_TRIANGLES, 0, 6 );
    
//    TexFunc();
    
    if (doubleBuffer) {
        glutSwapBuffers();
    } else {
        glFlush();
    }
}

static void
Args(int argc, char **argv)
{
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

void timer( int value )
{
    glutPostRedisplay();
    glutTimerFunc( 16, timer, 0 );
}

int
main(int argc, char **argv) {
    GLenum type;
    
    glutInit(&argc, argv);
    Args(argc, argv);
    
    type = GLUT_RGB;
    type |= (doubleBuffer) ? GLUT_DOUBLE : GLUT_SINGLE;
    glutInitDisplayMode(type);
    glutInitWindowSize(width/2, height/2);
    glutCreateWindow("ABGR extension");
    if (!glutExtensionSupported("GL_EXT_abgr")) {
        printf("Couldn't find abgr extension.\n");
        exit(0);
    }
#if !GL_EXT_abgr
    printf("WARNING: client-side OpenGL has no ABGR extension support!\n");
    printf("         Drawing only RGBA (and not ABGR) images and textures.\n");
#endif
    Init();
    glutKeyboardFunc(Key);
    glutTimerFunc( 0, timer, 0 );
    glutDisplayFunc(Draw);
    glutMainLoop();
    return 0;             /* ANSI C requires main to return int. */
}
