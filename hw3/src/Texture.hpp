//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cstdlib>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBiLinear(float u, float v){
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto u4 =  std::round(u_img); //右上角的纹理坐标
        auto v4 = std::round(v_img);

        auto u1 = u4 > 0? u4 - 1: 0;//左下角
        auto v1 = v4 > 0? v4 - 1: 0; 
        auto u2 = u4;               //右下角
        auto v2 = v4 > 0? v4 - 1: 0; 
        auto u3 = u4 > 0? u4 - 1: 0;//左上角
        auto v3 = v4;   

        auto color1 = image_data.at<cv::Vec3b>(v1, u1);
        auto color2 = image_data.at<cv::Vec3b>(v2, u2);
        auto color3 = image_data.at<cv::Vec3b>(v3, u3);
        auto color4 = image_data.at<cv::Vec3b>(v4, u4);

        float tu = (u_img - u1)/ 2.f;
        float tv = (v_img - v1)/ 2.f;


        auto i_color1 = color1 * tu + color2 * (1-tu); //水平方向插值两次
        auto i_color2 = color3 * tu + color4 * (1-tu);

        auto color = i_color1 * tv + i_color2 * (1-tv); //竖直方向插值一次
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
