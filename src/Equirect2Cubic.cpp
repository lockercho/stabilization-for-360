#include "Equirect2Cubic.h"
#include <stdio.h>


Equirect2Cubic::Equirect2Cubic(int sourceWidth, int sourceHeight, int width, int height)
{
  // initial the cubeFacing angle
  /**    4
   * 0 1 2 3
   *     5
   * */
    // theta = -PI/2 ~ PI/2
    // phi = -PI/2 ~ PI/2
  cubeFacing[0][0] = M_PI / 2; // theta
  cubeFacing[0][1] = 0;        // phi
  cubeFacing[1][0] = M_PI;
  cubeFacing[1][1] = 0;
  cubeFacing[2][0] = -M_PI/2;
  cubeFacing[2][1] = 0;
  cubeFacing[3][0] = 0;
  cubeFacing[3][1] = 0;
  cubeFacing[4][0] = 0;
  cubeFacing[4][1] = -M_PI/2;
  cubeFacing[5][0] = 0;
  cubeFacing[5][1] = M_PI/2;
    
    
//    cubeFacing[0][0] = -M_PI / 4; // theta
//    cubeFacing[0][1] = 0;        // phi
//    cubeFacing[1][0] = M_PI / 4;
//    cubeFacing[1][1] = 0;
//    cubeFacing[2][0] = M_PI * 3 / 4;
//    cubeFacing[2][1] = 0;
//    cubeFacing[3][0] = - M_PI * 3 / 4;
//    cubeFacing[3][1] = 0;
//    cubeFacing[4][0] = 0;
//    cubeFacing[4][1] = -M_PI/2;
//    cubeFacing[5][0] = 0;
//    cubeFacing[5][1] = M_PI/2;

  //initial the mapx and mapy for each face
  for (int faceId=0;faceId<6;faceId++)
  {
    this->mapX[faceId] = cv::Mat(height, width, CV_32F);
    this->mapY[faceId] = cv::Mat(height, width, CV_32F);
    
    // calculate mapX and mapY
    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            float u, v;
            xy2uv(faceId, x, y, sourceWidth, sourceHeight, width, height, u, v);
            
            // Save the result for this pixel in map
            this->mapX[faceId].at<float>(x, y) = u * (sourceWidth - 1);
            this->mapY[faceId].at<float>(x, y) = v * (sourceHeight - 1);
        }
    }
  }
}

void Equirect2Cubic::xy2uv(int faceId, int x, int y, int sourceWidth, int sourceHeight, int width, int height, float &u, float &v) {
    const float ftu = this->cubeFacing[faceId][0]; // theta
    const float ftv = this->cubeFacing[faceId][1]; // phi
    
    float nx = (float)y / (float)height - 0.5f;
    float ny = (float)x / (float)width - 0.5f;
    
    nx *= 2;
    ny *= 2;
    
    nx *= an;
    ny *= an;
    
    // Project from plane to sphere surface.
    if(ftv == 0) {
        // Center faces
        u = atan2(nx, ak);
        v = atan2(ny * cos(u), ak);
        u += ftu;
    } else if(ftv > 0) {
        // Bottom face
        float d = sqrt(nx * nx + ny * ny);
        v = M_PI / 2 - atan2(d, ak);
        u = atan2(ny, nx);
    } else {
        // Top face            // Map face pixel coordinates to [-1, 1] on plane
        
        float d = sqrt(nx * nx + ny * ny);
        v = -M_PI / 2 + atan2(d, ak);
        u = atan2(-ny, nx);
    }
    
    // Map from angular coordinates to [-1, 1], respectively.
    u = u / (M_PI);
    v = v / (M_PI / 2);
    
    // Warp around, if our coordinates are out of bounds.
    while (v < -1) {
        v += 2;
        u += 1;
    }
    while (v > 1) {
        v -= 2;
        u += 1;
    }
    
    while(u < -1) {
        u += 2;
    }
    while(u > 1) {
        u -= 2;
    }
    
    // Map from [-1, 1] to in texture space [0, 1]
    u = u / 2.0f + 0.5f;
    v = v / 2.0f + 0.5f;
}

std::vector<float> transform(std::vector<float> src, std::vector<float> trans) {
    // src = [x, y, 1]
    // trans = 1x9 vector
    std::vector<float> res;
    for (int i = 0; i<3 ; i++) {
        res.push_back(src[0] * trans[i*3] + src[1] * trans[i*3+1] + src[2] * trans[i*3+2]);
    }
    return res;
}

void Equirect2Cubic::uv2xy(float u, float v, int width, int height, int &faceId, float &x, float &y) {
//    [u, v, 1]
//    u, v -> x, y, z
//    
    // u,v [0, 1] -> [-1, 1]
    u = (u) * 2;
    v = (v - 0.5) * 2;
    
    std::cout << "U2: " << u << ", V2: " << v <<std::endl;

    u *= M_PI;     // 180 degree
//    v *= M_PI / 2; // 90 degree
    v = asin(v);
    
    // get face id
    
    // top face
    if(v < -M_PI / 4) {
        faceId = 4;
        
//        float d = sqrt(nx * nx + ny * ny);
//        v = -M_PI / 2 + atan2(d, ak);
//        u = atan2(-ny, nx);
        
    // bottom face
    } else if(v > M_PI / 4) {
        faceId = 5;
    } else {
        faceId = (int)((u + 1.0) * 2);
    }
    
    const float ftu = this->cubeFacing[faceId][0];
    const float ftv = this->cubeFacing[faceId][1];
    
    
//    u += ftu; // should be 0 ~ PI / 4
//    v += ftv;
    float len = sqrt((width/2)*(width/2) + (height/2)*(height/2));
//    x = tan(u) * len * sin(M_PI/4);
//    y = tan(v) * len * sin(M_PI/4);
//    x = tan(u) * width / 2 + width/2;
//    y = tan(v) * width / 2 + width/2;
    
    x = cos(v) * cos(u) * len;
    y = cos(v) * sin(u) * len;
//    z = cos(phi) sin(theta)
    
//    if(u < 0) {
//        y =
//    }
//    
//    x = u / (M_PI / 4) * width;
//    y = v / (M_PI / 4) * height;
    
//    while(x > width) {
//        x -= width;
//    }
//    
//    while(y > height) {
//        y -= height;
//    }
//    
//    while(x < 0) {
//        x += width/2;
//    }
    
//    while(y < 0) {
//        y += height/2;
//    }
//
    
//    //apply tranform to x, y
//    std::vector<float> src;
//    src.push_back(x);
//    src.push_back(y);
//    src.push_back(1);
//    std::vector<float> res = transform(src, trans);
//    
//    // convert new x, y to uv
//    
    
    
}

void Equirect2Cubic::remapWithMap(cv::Mat& in, cv::Mat& out, int faceId)
{
  // call opencv remap function with self calculated mapX and mapY
  cv::remap(in, out, this->mapX[faceId], this->mapY[faceId],
            CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  return;
}
