// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <tuple>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    float z = 0.f;
    Eigen::Vector3f v1 = {_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0.f};
    Eigen::Vector3f v2 =  {x-_v[0].x(), y-_v[0].y(), 0.f};
    z = v1.cross(v2).z();

    v1 = {_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0.f};
    v2 = {x-_v[1].x(), y-_v[1].y(), 0.f};
    z *= v1.cross(v2).z();
    v1 = {_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0.f};
    v2 = {x-_v[2].x(), y-_v[2].y(), 0.f};
    z *= v1.cross(v2).z();
    return z > 0.f;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            //vert.z() = vert.z() * f1 + f2;
            vert.z() = -vert.z() * f1 + f2;

        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

static auto bounding_box(const Triangle& t)->decltype(auto){
    float max_x = std::max(t.v[0].x(), t.v[1].x());
    max_x = std::max(max_x, t.v[2].x());

    float max_y =  std::max(t.v[0].y(), t.v[1].y());
    max_y = std::max(max_y, t.v[2].y());

    float min_x = std::min(t.v[0].x(), t.v[1].x());
    min_x = std::min(min_x, t.v[2].x());

    float min_y =  std::min(t.v[0].y(), t.v[1].y());
    min_y = std::min(min_y, t.v[2].y());
    return std::make_tuple(min_x, min_y, max_x, max_y);
    //return std::pair<std::pair<float, float>, std::pair<float, float>>({min_x, min_y}, {max_x, max_y});
}
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end, Eigen::Vector3f line_color = {255, 255, 255})
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();



    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    //1. 查找bbox
    auto[min_x, min_y, max_x, max_y] = bounding_box(t);  
    //2. 遍历bbox中的每一像素，判断是否在在三角形内部
    //  2.1 若不在则继续
    //  2.2 若在则根据中心坐标插值颜色值
    const float EPSILON = 10e-6;
   for(int y = min_y; y < max_y; ++y){
        for(int x = min_x; x < max_x; ++x){
            if(msaa){
                float dt = 1.f / freq; //一个像素中采样的频率，采样freq^2个采样点
                float start_x = x + dt * 0.5, start_y = y + dt * 0.5;
                Eigen::Vector3f color = Eigen::Vector3f::Zero();
                bool is_view = false;
                for(int i = 0; i < freq; ++i){
                    float sx = start_x + i * dt;
                    for(int j = 0; j < freq; ++j){
                        float sy = start_y + j * dt;
                        if(insideTriangle(sx, sy, t.v)){
                            //1.计算重心坐标
                            auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                            //2.投影矫正，
                            float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            alpha *= Z / v[0].w(); beta *= Z/v[1].w(); gamma *= Z / v[2].w();
                            float zp = alpha * v[0].z() + beta * v[1].z() + gamma * v[2].z();
                
                            auto ind = get_index(x, y); //同一个像素中的采样点，深度值对应的深度缓冲中的位置应该是一样的
                            if(zp <= depth_buf[ind] + EPSILON){
                                depth_buf[ind] = zp;
                                is_view = true;
                                color += t.getColor();
                            }
                        }
                    }
                }
                
                if(is_view){
                    set_pixel({x, y, 0.f}, color/(freq * freq));
                }
                
            }
            else if(insideTriangle(x+0.5, y+0.5, t.v)){
                //1.计算重心坐标
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                //2.投影矫正，
                float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                alpha *= Z / v[0].w(); beta *= Z/v[1].w(); gamma *= Z / v[2].w();
                float zp = alpha * v[0].z() + beta * v[1].z() + gamma * v[2].z();
                //从相机可见
                auto ind = get_index(x, y);
                if(zp <= depth_buf[ind] + EPSILON){
                    depth_buf[ind] = zp;
                    auto color = t.getColor(); //本应该获取三角形的各个顶点颜色然后插值计算出来，但是每个三角形只有一个颜色，所以内部颜色和顶点颜色相等
                    set_pixel({x, y, 0.f}, color);
                }
            }
        }
    }  
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on