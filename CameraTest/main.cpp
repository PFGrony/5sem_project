#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main()
{
    Mat file=Mat::zeros(200,200,CV_8UC1)*255;
    namedWindow("1",WINDOW_AUTOSIZE);
    imshow("1",file);
    cout << "Hello World!" << endl<<endl;
    waitKey(0);
    return 0;
}
