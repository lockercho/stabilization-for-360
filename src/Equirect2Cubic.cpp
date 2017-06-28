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
    
    // save xy and uv map
    bool shouldWrite = fopen("/tmp/xyuv", "r") == NULL;
    FILE* file;
    if(shouldWrite) file = fopen("/tmp/xyuv", "w");
    
  for (int faceId=0;faceId<6;faceId++)
  {
    this->mapX[faceId] = cv::Mat(height, width, CV_32F);
    this->mapY[faceId] = cv::Mat(height, width, CV_32F);
    
    // calculate mapX and mapY
    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            float u, v;
            xy2uv(faceId, x, y, sourceWidth, sourceHeight, width, height, u, v);
            
            if(shouldWrite && x % 8 == 0 && y % 8 == 0)
                fprintf(file, "%d %d %d %f %f\n", faceId, x, y, u, v);
            
            
            // Save the result for this pixel in map
            this->mapX[faceId].at<float>(x, y) = u * (sourceWidth - 1);
            this->mapY[faceId].at<float>(x, y) = v * (sourceHeight - 1);
        }
    }
  }
    if(shouldWrite){
    fflush(file);
    fclose(file);
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

void Equirect2Cubic::remapWithMap(cv::Mat& in, cv::Mat& out, int faceId)
{
  // call opencv remap function with self calculated mapX and mapY
  cv::remap(in, out, this->mapX[faceId], this->mapY[faceId],
            CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  return;
}
