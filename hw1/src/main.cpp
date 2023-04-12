#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

enum Axis{
    X, Y, Z
};
Eigen::Matrix4f get_rotation_matrix(Axis axis, float rotation_angle){
    Eigen::Matrix4f rot_mat = Eigen::Matrix4f::Identity();

    float radians = (rotation_angle) / 180.f * MY_PI;
    float cos_theta = std::cos(radians);
    float sin_theta = std::sin(radians);
    switch (axis)
    {
    case Axis::X:
        rot_mat.block(1, 1, 2, 2) << cos_theta, -sin_theta,
                                     sin_theta,  cos_theta;
        
        break;
    case Axis::Y:
        rot_mat(0, 0) = cos_theta;
        rot_mat(2, 0) = -sin_theta;
        rot_mat(0, 2) = sin_theta;
        rot_mat(2, 2) = cos_theta;
        break;
    case Axis::Z:
        rot_mat.block(0,0,2,2) << cos_theta, -sin_theta,
                                  sin_theta,  cos_theta;
        
        break;
    default:
        break;
    }
    return rot_mat;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    model = get_rotation_matrix(Axis::Z, rotation_angle);

    return model;
}

Eigen::Matrix4f get_model_matrix(float angle_x, float angle_y, float angle_z)
{
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    auto Rx = get_rotation_matrix(Axis::X, angle_x);
    auto Ry = get_rotation_matrix(Axis::Y, angle_y);
    auto Rz = get_rotation_matrix(Axis::Z, angle_z);        

   // return Rx * Ry * Rz;
   return Rz * Ry * Rx;
}


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    //近平面height = tan(eye_fov / 2) * zNear * 2;
    //width = height * aspect_ratio

    float radins = eye_fov / 180.f * MY_PI;
    float height = std::tan(radins / 2) * zNear * 2;
    float width = height * aspect_ratio;
    float r = width / 2, l = -r;
    float t = height / 2, b = -t;
    float n = zNear, f = zFar;

    Eigen::Matrix4f m_orth = Eigen::Matrix4f::Identity();
    m_orth<< 2 / (r-l), 0, 0, -(r+l)/(r-l),
             0, 2/(t-b), 0, -(t+b)/(t-b),
             0, 0, 2/(n-f), -(n+f)/(n-f),
             0, 0, 0, 1;
    Eigen::Matrix4f m_per = Eigen::Matrix4f::Identity();
    m_per<<n, 0, 0, 0,
           0, n, 0, 0,
           0, 0, n+f, -n*f,
           0, 0, 1, 0;
    projection = m_orth * m_per;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{1, 0, -1}, {0, 1, -1}, {-1, 0, -1}}; //顶点位置

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};  //顶点索引，多个三角形公用顶点

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;


    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    float angle_x=0.f, angle_y=0.f, angle_z = 0.f;
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_model_matrix(angle_x, angle_y, angle_z));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        //std::cout << "frame count: " << frame_count++ << '\n';

        // if (key == 'a') {
        //     angle += 10;
        // }
        // else if (key == 'd') {
        //     angle -= 10;
        // }

        switch (key)
        {
        case 'q':
            angle_x += 10;
            break;
        case 'w':
            angle_x -= 10;
            break;
        case 'a':
            angle_y += 10;
            break;
        case 's':
            angle_y -= 10;
            break;
        case 'x':
            angle_z += 10;
            break;
        case 'z':
            angle_z -= 10;
            break;
        case ' ':
            std::cout<<get_model_matrix(angle_x, angle_y, angle_z)<<std::endl;
            break;
        default:
            break;
        }
    }

    return 0;
}
