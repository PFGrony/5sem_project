#include "pathPlanner.h"

pathPlanner::pathPlanner()
{

}

void pathPlanner::doBrushfire()
{
    // map
        cv::Mat smallMap;

        smallMap = cv::imread("floor_plan.png",CV_LOAD_IMAGE_GRAYSCALE);

        cv::imshow("test1",smallMap);

        char charMap[smallMap.rows][smallMap.cols];

        std::cout << "Antal rows "<< smallMap.rows << "Antal cols " << smallMap.cols << std::endl;

        std::cout << smallMap.rows << ":" << smallMap.cols << std::endl;

        for (int i = 0;i<smallMap.rows;i++)
        {
            for (int j = 0;j<smallMap.cols;j++)
            {
                if (smallMap.at<uchar>(i,j) == 0)
                    charMap[i][j] = '#';
                else
                    charMap[i][j] = ' ';
            }
        }
}
