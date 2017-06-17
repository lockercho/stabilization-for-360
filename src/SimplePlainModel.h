#ifndef PlainModel_h
#define PlainModel_h
//     0           1
//   -1,-1------ 1,-1
//    |      /     |
//    |    /       |
//  -1, 1 ------ 1,1
//    3            2

GLfloat model_vertices[] = {
    -1, -1, 0,
    -1, 1, 0,
    1, 1, 0,
    -1, -1, 0,
    1, -1, 0,
    1, 1, 0,
};

GLfloat model_texcoords[] = {
    0, 1.0,
    0, 0.0,
    1.0, 0.0,
    0, 1.0,
    1, 1,
    1.0, 0.0
};

#endif 
