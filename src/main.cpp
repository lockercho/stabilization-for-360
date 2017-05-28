
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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <GLUT/glut.h>
#include <iostream>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

static int width = 1920;
static int height = 1080;

GLenum doubleBuffer;
GLubyte ubImage[65536];

static void
Init(void)
{
    int j;
    GLubyte *img;
    GLsizei imgWidth = 128;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 1.0, 0.1, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glDisable(GL_DITHER);
    return;
    
    /* Create image */
    img = ubImage;
    for (j = 0; j < 32 * imgWidth; j++) {
        *img++ = 0xff;
        *img++ = 0x00;
        *img++ = 0x00;
        *img++ = 0xff;
    }
    for (j = 0; j < 32 * imgWidth; j++) {
        *img++ = 0xff;
        *img++ = 0x00;
        *img++ = 0xff;
        *img++ = 0x00;
    }
    for (j = 0; j < 32 * imgWidth; j++) {
        *img++ = 0xff;
        *img++ = 0xff;
        *img++ = 0x00;
        *img++ = 0x00;
    }
    for (j = 0; j < 32 * imgWidth; j++) {
        *img++ = 0x00;
        *img++ = 0xff;
        *img++ = 0x00;
        *img++ = 0xff;
    }
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

std::string filename = "/Users/lockercho/workspace/GPU/final_project/walk_short.mp4";
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
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1, 1, -2.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1, 1, -2.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1, -1, -2.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1, -1, -2.0);
    glEnd();
    
    glDisable(GL_TEXTURE_2D);
}

static void
Draw(void)
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    
    TexFunc();
    
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
