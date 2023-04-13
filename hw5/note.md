1. 计算cast ray
    ![pres](resource/1.png)
    计算cast ray就是光栅化中投影的步骤倒着走：首先Mview_port的逆变换，其次将经典立方体的**近平面投射到原来位置即可**，因为光线投射出去只需要知道两点就能确定一条光线(相机位置(起点)和光线打到近平面上的一点)
    1. view_port逆变换就是将frame_buffer中的像素位置，转换回对应的近平面的位置
        ```C++
        float scale = std::tan(deg2rad(scene.fov * 0.5f));
        float imageAspectRatio = scene.width / (float)scene.height;
        for (int j = 0; j < scene.height; ++j){
            for (int i = 0; i < scene.width; ++i){
                //计算从像素点(i,j)打出去的光线在近平面的位置坐标x，y
                float x = ((i+0.5)/ scene.width * 2) - 1; //屏幕坐标 i，j 转换回在经典立方体中近平面的坐标x，y
                float y = ((j+0.5)/scene.height * 2) - 1;
                //投影回相机的视锥中的长宽比
                x *= imageAspectRatio;   // 此时 [-1, 1]^2 投影回视锥时x比y多一个缩放比例imageAspectRatio
                x *= scale * zNear;
                y *= scale * zNear;      //scale * zNear是整个截面在视锥中一个缩放倍数，随着截面上的坐标x，y也随截面进行缩放，所以需要乘上
            }
        }
        ```
2. TODO
   三角形求交，moller公式![moller](resource/2.png)