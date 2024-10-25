#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#define POINTCOUNT 4

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    using namespace cv;
    // TODO: Implement de Casteljau's algorithm

    std::vector<Point2f> newPoint;

    for(size_t i = 0; i < control_points.size() - 1; ++i){
        Point2f point = control_points[i] * (1 - t) +control_points[i+1]*t;
        newPoint.push_back(point);
    }
    if(control_points.size() == 1){
        return control_points[0];
    }
    else{
        return recursive_bezier(newPoint, t);
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    using namespace cv;
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(double t = 0.0; t <= 1.0; t += 0.0001){
        auto point = recursive_bezier(control_points, t);

#define ANTIALIASING 1
#if ANTIALIASING
        Point2f center(std::floor(point.x) + 0.5f, std::floor(point.y) + 0.5f);
        for(float i = -1;i <= 1; ++i){
            for(float j= -1;j <= 1; ++j){
                Point2f nearPoint(center.x + i,center.y + j);
                Point2f vector = nearPoint - point;
                double distance = sqrt(pow(vector.x,2)+pow(vector.y,2));
                float color =(1 - distance / 1.5f * sqrt(2)) * 255.0f;
                float preColor = window.at<cv::Vec3b>(center.y+j, center.x+i)[1];
                window.at<cv::Vec3b>(center.y+j, center.x+i)[1] = max<float>(color,preColor);
            }
        }
#else
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
#endif
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() >= POINTCOUNT) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(20);
        }
        else{
            cv::imshow("Bezier Curve", window);
            key = cv::waitKey(20);
        }
    }

    return 0;
}
