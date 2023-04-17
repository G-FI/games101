# 1.problems
1. 半球上采样一点的均匀采样一点的概率为什么是 1 / 2*pi
   ```c++
    float Material::pdf(const Vector3f &wi, const Vector3f &wo, const    Vector3f &N){
        switch(m_type){
            case DIFFUSE:
            {
                // uniform sample probability 1 / (2 * PI)
                if (dotProduct(wo, N) > 0.0f)
                    return 0.5f / M_PI;
                else
                    return 0.0f;
                break;
            }
        }   
    }
   ```
2. sampleLight(pos, pdf) 应该是对光源采样，可是为什呢它里面是对所有的发光物体采样，而所有的物体都不是发光的
    因为他将光源作为发光体，也即场景中的一个object，使用材质标识具体物体是否发光Material::hasEmission()，而不是使用单独的光源light
3. 对于光源为黑色
   1. 如果从相机打出来的trace ray直接打到光源上时，此时对光源采样这一步都正常，但是对光源的采样点p'与tarce_ray与光源的相交点p在同一个平面上，此时由于数值精度问题，从p到p'的射线可能就会与光源本身相交，所以总会使得采样点p'处于阴影里。
   2. 解决方法：在castRay(即shade函数)中trace ray打到的点是否是光源，是的话直接返回光源的颜色
   ```C++
        Intersection inter = intersect(ray);
        if(inter.obj->hasEmit()){
            return inter.m->getEmission();
    }
   ```
4. 流程
   1. 判断相交点p是否为光源，如上3
   2. `sampleLight`对光源进行采样，传入参数Intersection只设置坐标coords和normal，并不设置hasInersection表示相交，本身就是在光源上采样，hasIntersecion字段在这里就是没有意义的。也并不设置distance，所以需要自己计算出distance来与shadow_inter.distance(p点向p'打的一条光线与场景中物体的交点)以此判断p点是否在阴影中。因为向p'方向打光线，至少也会与p'点相交，由于数值精度问题(两个distance特别近)，需要添加一个误差项EPOSILON
    ```C++
     //对直接光源进行采样
    Vector3f L_dir{0.f,0.f,0.f};

    Intersection inter_light;
    float light_pdf;
    sampleLight(inter_light, light_pdf); //返回的inter_light只会设置coord和normal两个属性
    Vector3f p_prime = inter_light.coords;
    inter_light.distance = (p_prime - p).norm();
    
    Vector3f ws = (p_prime - p).normalized();
    Ray trace_light_ray(p, ws);
    Intersection shadow_inter = intersect(trace_light_ray);
    Vector3f NN = shadow_inter.normal;
    
    if(shadow_inter.distance >= inter_light.distance - EPSILON){
        //l_dir = Li * f_r(p, wi, wo) * cosθ * cosθ' /(distance(p, p_prime))
        Vector3f f_r = inter.m->eval(ws, wo, N);
        float cos_theta = dotProduct(ws, N);
        float cos_theta_prime = dotProduct(-ws, NN);
        float dist = (p-p_prime).norm();
        L_dir = inter_light.emit * f_r * cos_theta * cos_theta_prime / (dist*dist) / light_pdf;
    }
    ```
    3. 轮盘赌判断是否继续
    4. 对间接光照进行采样, 结果除以rr概率，对p点，wi方向入射光采样时期望值是Li
    ```C++
    //采样间接光照

    Vector3f L_indir(0.f, 0.f, 0.f);
    Vector3f wi =  inter.m->sample(wo, N).normalized();
    Ray trace_ray(p, wi);
    Intersection inter_indir = intersect(trace_ray);

    //反射光线打到其他物体，并且该物体不发光
    if(inter_indir.happened && !inter_indir.obj->hasEmit()){
        float pdf_hemi = inter.m->pdf(wi, wo, N);
        Vector3f f_r = inter.m->eval(wi, wo, N);
        float cos_theta = dotProduct(wo, N);
        L_indir = castRay(trace_ray, depth+1) * f_r * cos_theta / pdf_hemi / RussianRoulette; 
    }
    ```
5. 多线程
    1. 因为每个像素点的raytracing都是相互不以来的，所以可以进行多线程：设置`workers_number`个线程，每个线程处理至多`std::ceil(scene.height / static_cast<float>(workers_number))`行的像素
   

6. 
# 2. 启发
1. 最好重新设置变量名与公式中的相对应，make life easier
# 2. 关于光追的思考
1. 有没有可能会爆栈(很小概率的情况下) 