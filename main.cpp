/*     ____  ____________    __ ____   __     _   __________________  _  __
 __ / / / / / __/_  __/___/ // / /  / / ___| | / /  _/ __/  _/ __ \/ |/ /
/ // / /_/ /\ \  / / /___/ _  / /__/ /_/___/ |/ // /_\ \_/ // /_/ /    /
\___/\____/___/ /_/     /_//_/____/____/   |___/___/___/___/\____/_/|_/
*/
/**
 * @file        Buff detector
 * @details     Windows-10 + Qt Creator-5.14.2 + OpenCV-4.1.1
 * @author      zhaoqicheng zhaoqicheng2023@163.com/623146682@qq.com
 * @version     1.0
 * @date        2023/3/14
 * @copyright   2023 JUST-HLL.All rights reserved
 */
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

/*参数声明--------------------------------------------------------------------------------------------*/
const string video_path = "F:\\hll-code\\video\\R.mp4";  // 设置视频文件路径
cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));    // 设置内核1
cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(25,25));  // 设置内核2
/*函数声明--------------------------------------------------------------------------------------------*/
void ImageReprocessing(cv::Mat video_frame,vector<cv::Mat> channels,cv::Mat element1,cv::Mat element2);  // 图像预处理
void IdentifyBUFF(cv::Mat video_frame,cv::Mat frame_plot,
                  vector<vector<cv::Point> > contours,vector<cv::Vec4i> hierarchy,
                  int area[],vector<cv::Point2d> points) ;  // 识别 BUFF
int LeastSquaresCircleFitting(vector<cv::Point2d> &m_Points, cv::Point2d &Centroid, double &dRadius);  // 拟合函数
/*主函数----------------------------------------------------------------------------------------------*/
int main() {
    cv::VideoCapture video(video_path);  // 读取视频
    // 判断视频是否成功读取
    if (video.isOpened())
        cout << "[INFO] Video read successfully!" << endl;
    else
        cout << "[WARN] Video read failure!" << endl;

    cv::Mat video_frame,frame_plot;  // 创建一个 Mat 对象用于将 VideoCapture 对象转换为 Mat 对象
    vector<cv::Point2d> points;
    while(true) {
        video >> video_frame;  // 将 VideoCapture 对象转换为 Mat 对象用于图形预处理
        frame_plot = video_frame.clone();  // 克隆该 Mat 对象用于最后的绘图
        if (video_frame.empty())
            break;
        // 图片预处理
        vector<cv::Mat> channels;  // 定义存放 Mat 对象的向量，用于保存图片分离后各个通道的图像
        ImageReprocessing(video_frame,channels,element1,element2);
        // 计算轮廓面积
        vector<vector<cv::Point> > contours;
        vector<cv::Vec4i> hierarchy;
        int area[25] = {0};
        // 能量机关识别
        IdentifyBUFF(video_frame,frame_plot,contours,hierarchy,area,points);

        cv::namedWindow("video_frame",cv::WINDOW_NORMAL);
        cv::imshow("video_frame",frame_plot);
        cv::waitKey(1);
    }

    return 0;
}

/*函数定义---------------------------------------------------------------------------------------------*/
/**
 * @brief 图像预处理
 * @param video_frame 图像
 * @param channels    分离后的通道
 * @param element1    内核1
 * @param element2    内核2
 */
void ImageReprocessing(cv::Mat video_frame,vector<cv::Mat> channels,cv::Mat element1,cv::Mat element2) {
    cv::split(video_frame,channels);  // 分离通道
    cv::threshold(channels.at(2) - channels.at(0),video_frame,100,255,cv::THRESH_BINARY_INV);  // 二值化图像
    cv::morphologyEx(video_frame,video_frame,cv::MORPH_OPEN,element1);   // 开运算
    // 能不能在这里写一句闭运算将外侧装甲板轮廓闭合？
//    cv::namedWindow("test1",cv::WINDOW_NORMAL);
//    cv::imshow("test1",video_frame);
    cv::floodFill(video_frame,cv::Point(0,0),cv::Scalar(0));  // 漫水法
//    cv::namedWindow("test2",cv::WINDOW_NORMAL);
//    cv::imshow("test2",video_frame);
    cv::morphologyEx(video_frame,video_frame,cv::MORPH_CLOSE,element2);  // 闭运算
//    cv::namedWindow("test3",cv::WINDOW_NORMAL);
//    cv::imshow("test3",video_frame);
}

//findContours(video1, contours, hierarchy);//找轮廓
//int area[25] = { 0 };
void IdentifyBUFF(cv::Mat video_frame,cv::Mat frame_plot,
                  vector<vector<cv::Point> > contours,vector<cv::Vec4i> hierarchy,
                  int area[],vector<cv::Point2d> points) {
    cv::findContours(video_frame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);  //找轮廓
    for (int i = 0; i != int(hierarchy.size()); ++i) {
        area[i] = cv::contourArea(contours[i]);  //计算轮廓面积
        if (area[i] < 1000 ) {
            cv::Point2f rect[4];  // 用于保存装甲板最小外接矩阵的四个顶点坐标
            cv::RotatedRect box1 = minAreaRect(cv::Mat(contours[i])); //获取最小外接矩阵
            circle(frame_plot,cv::Point(box1.center.x, box1.center.y), 5, cv::Scalar(255, 0, 0), -1, 8);  //绘制最小外接矩形的中心点
            box1.points(rect);  //把最小外接矩形四个端点复制给rect数组
            for (int j = 0; j != 4; ++j) {
                cv::line(frame_plot, rect[j], rect[(j + 1) % 4], cv::Scalar(0, 255, 0), 2, 8);  //绘制最小外接矩形每条边
            }
            points.push_back(box1.center);//储存最小外接矩形中心点
            cv::Point2d c;  //圆心坐标
            double r = 0;   //半径
            LeastSquaresCircleFitting(points, c, r);//拟合圆
            cv::circle(frame_plot, c, r, cv::Scalar(0, 0, 255), 2, 8);//绘制圆
            cv::circle(frame_plot, c, 5, cv::Scalar(255, 0, 0), -1, 8);//绘制圆心

        }
    }
}

/**
 * @brief LeastSquaresCircleFitting
 * @param m_Points
 * @param Centroid
 * @param dRadius
 * @return
 */
int LeastSquaresCircleFitting(vector<cv::Point2d> &m_Points, cv::Point2d &Centroid, double &dRadius) {//拟合圆函数(三个参数依次为输入点集，圆心，半径)
    if (!m_Points.empty()) {
        int iNum = (int)m_Points.size();
        if (iNum < 3)	return 1;
        double X1 = 0.0;double Y1 = 0.0;
        double X2 = 0.0;double Y2 = 0.0;
        double X3 = 0.0;double Y3 = 0.0;
        double X1Y1 = 0.0;double X1Y2 = 0.0;double X2Y1 = 0.0;

        vector<cv::Point2d>::iterator iter;
        vector<cv::Point2d>::iterator end = m_Points.end();

        for (iter = m_Points.begin(); iter != end; ++iter) {
            X1 = X1 + (*iter).x; Y1 = Y1 + (*iter).y;
            X2 = X2 + (*iter).x * (*iter).x; Y2 = Y2 + (*iter).y * (*iter).y;
            X3 = X3 + (*iter).x * (*iter).x * (*iter).x; Y3 = Y3 + (*iter).y * (*iter).y * (*iter).y;
            X1Y1 = X1Y1 + (*iter).x * (*iter).y;
            X1Y2 = X1Y2 + (*iter).x * (*iter).y * (*iter).y;
            X2Y1 = X2Y1 + (*iter).x * (*iter).x * (*iter).y;
        }
        double C = 0.0;
        double D = 0.0;
        double E = 0.0;
        double G = 0.0;
        double H = 0.0;
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;
        C = iNum * X2 - X1 * X1;
        D = iNum * X1Y1 - X1 * Y1;
        E = iNum * X3 + iNum * X1Y2 - (X2 + Y2) * X1;
        G = iNum * Y2 - Y1 * Y1;
        H = iNum * X2Y1 + iNum * Y3 - (X2 + Y2) * Y1;
        a = (H * D - E * G) / (C * G - D * D);
        b = (H * C - E * D) / (D * D - G * C);
        c = -(a * X1 + b * Y1 + X2 + Y2) / iNum;
        double A = 0.0;
        double B = 0.0;
        double R = 0.0;
        A = a / (-2);
        B = b / (-2);
        R = double(sqrt(a * a + b * b - 4 * c) / 2);
        Centroid.x = A;
        Centroid.y = B;
        dRadius = R;
        return 0;
    }
    else
        return 1;
    return 0;
}


