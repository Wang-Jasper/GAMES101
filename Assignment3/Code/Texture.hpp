//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
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

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        // ensure within [0, 1].
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);

        // Convert normalized coordinates to image pixel coordinates.
        float xCoord = u * width;
        float yCoord = (1 - v) * height;

        float xFloor = std::floor(xCoord);
        float xCeil = std::min(static_cast<float>(width), std::ceil(xCoord));
        float yFloor = std::floor(yCoord);
        float yCeil = std::min(static_cast<float>(height), std::ceil(yCoord));

        auto colorTopLeft = image_data.at<cv::Vec3b>(yFloor, xFloor);
        auto colorTopRight = image_data.at<cv::Vec3b>(yFloor, xCeil);
        auto colorBottomLeft = image_data.at<cv::Vec3b>(yCeil, xFloor);
        auto colorBottomRight = image_data.at<cv::Vec3b>(yCeil, xCeil);

        float xAlpha = (xCoord - xFloor) / (xCeil - xFloor);
        float yAlpha = (yCoord - yCeil) / (yFloor - yCeil);

        auto interpolatedBottom = (1 - xAlpha) * colorBottomLeft + xAlpha * colorBottomRight;
        auto interpolatedTop = (1 - xAlpha) * colorTopLeft + xAlpha * colorTopRight;

        // Interpolate
        auto interpolatedColor = (1 - yAlpha) * interpolatedBottom + yAlpha * interpolatedTop;

        return Eigen::Vector3f(interpolatedColor[0], interpolatedColor[1], interpolatedColor[2]);
    }
};
#endif // RASTERIZER_TEXTURE_H
