#ifndef Renderer_h
#define Renderer_h

#include <OpenGL/gl.h>
#include <OpenGl/glu.h>
#include <GLUT/glut.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

//class Renderer {
//public:
//    Renderer(void* object, int model_id, bool can_sphere);
//    virtual ~Renderer();
//    
//    // init
//    void resetBuffer();
//    
//    // event
//    virtual void onSurfaceChanged(int width, int height);
//    virtual void onDoubleTap(float x, float y);
//    
//    // control
//    virtual void setRotationMatrix(float*);
//    virtual void setTranslation(float, float);
//    virtual void setDisplayOrientation(int n);
//    
//    // render
//    virtual void draw(int width, int height, uint8_t* textureY, int strideY, uint8_t* textureU, int strideU, uint8_t* textureV, int strideV);
//    virtual void redraw();
//    
//protected:
//    // loader
//    virtual GLuint loadProgram(const char *vertShaderSrc, const char *fragShaderSrc);
//    virtual GLuint loadShader(GLenum type, const char *shaderSrc);
//    virtual GLuint loadTexture();
//    
//    // render
//    virtual bool drawBeforeCheck();
//    virtual void updateTexture(int width, int height, uint8_t* textureY, int strideY, uint8_t* textureU, int strideU, uint8_t* textureV, int strideV);
//    virtual void drawHelp();
//    
//    // callback interface
//    void* _object;
//    
//    // shader str
//    static char _vShaderStr[];
//    static char _fShaderStr[];
//    static float _reCoordMatrix[4][16];
//    
//    // opengl params
//    void* _context;
//    GLuint _program;
//    GLuint _viewRenderbuffer, _viewFramebuffer, _depthRenderbuffer;
//    GLint _backingWidth, _backingHeight;
//    
//    int _baseYLoc, _baseULoc, _baseVLoc;
//    GLuint _baseYTexId, _baseUTexId, _baseVTexId;
//    int _posLoc, _texCoordLoc, _baseTextureMatrixLoc;
//    
//    // model
//    Model* _model;
//    
//    int _width, _height;
//    int _orientation;
//    
//    int _model_id;
//    bool _can_sphere;
//    bool _sphere;
//    bool _draw;
//    
//    // lock
//    boost::mutex _lock;
//};

#endif /* Renderer_hpp */
