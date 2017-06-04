#ifndef EQUIRECT2CUBIC_H
#define EQUIRECT2CUBIC_H
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief The Equirect2Cubic class
 * usage example:
 *

 // 512  as desire out put mat size
  Equirect2Cubic myTransForm(inputImage.cols,inputImage.rows,512,512);

  for (int i=0;i<6;i++)
  {
    result[i] = Mat(512,512,image.type());
    myTransForm.remapWithMap(image,result[i],i);
  }

 */


class Equirect2Cubic
{
  public:
    Equirect2Cubic(int sourceWidth, int sourceHeight,
                   int outWidth, int outHeight);

    // private member
  private:
    int sourceWidth;
    int sourceHeight;
    int outWidth;
    int outHeight;
    /**    4
     * 0 1 2 3
     *     5
     * */
    float cubeFacing[6][2];
    cv::Mat mapX[6];
    cv::Mat mapY[6];

    // public function
  public:
    void remapWithMap(cv::Mat &in, cv::Mat &out, int faceId);

};

#endif // EQUIRECT2CUBIC_H
