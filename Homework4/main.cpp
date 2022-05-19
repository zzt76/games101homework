#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
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

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, double t) 
{
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> new_points;
    for(int i = 0; i+1 < control_points.size(); i++)
    {
        cv::Point2f p1 = control_points[i];
        cv::Point2f p2 = control_points[i+1];
        new_points.push_back(t * p1 + (1-t) * p2);
    }
    if(new_points.size() > 1){
        return recursive_bezier(new_points, t);
    }
    else
    {
        return new_points[0];
    }

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        cv::Point2f point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
        float x = std::floor(point.x) + 0.5;
        float y = std::floor(point.y) + 0.5;
        std::vector<cv::Point2f> near_points = 
        {
            cv::Point2f(x-1, y), cv::Point2f(x+1, y), //left right
            cv::Point2f(x, y+1), cv::Point2f(x, y-1), //top bottom
        };
        for(int i = 0; i < near_points.size(); i++)
        {
            float dist = std::sqrt(std::pow((near_points[i].x - point.x), 2) 
            + std::pow((near_points[i].y - point.y), 2));
            if(dist < 2)
            {
                float scale = (2-dist)/2;
                cv::Vec3b color;
                window.at<cv::Vec3b>(near_points[i].y, near_points[i].x) = cv::Vec3b(0, 255*scale, 255*scale);
            }
        }

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

        if (control_points.size() == 4) 
        {
//            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
