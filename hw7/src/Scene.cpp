//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

//开启BVH时，使用这个函数进行求交
Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){ 
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){ 
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

//没有BVH时使用的，它会遍历场景中的所有物体，计算是否相交
bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject) const
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    //从相机打出的trace ray与场景中物体的交点
    Intersection inter = intersect(ray);
    //光线不与场景中的物体相交，返回
    if(!inter.happened){
        return backgroundColor;
    }

    //如果是打到光源了，直接返回
    if(inter.obj->hasEmit()){
        return inter.m->getEmission();
    }

    //交点位置
    Vector3f p = inter.coords;
    Vector3f wo = -ray.direction;
    Vector3f N = inter.normal;

    //对光源进行采样
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

    /*if(inter_light.happened){
        Vector3f p_prime = inter_light.coords;
        Vector3f ws = (p_prime - p).normalized();
        Ray trace_ray(p, ws);
        Intersection shadow_inter = intersect(trace_ray);
        Vector3f NN = shadow_inter.normal;

        //p点不在阴影中，计算p点的直接光照项
        if(shadow_inter.distance >= inter_light.distance + EPSILON){
            //l_dir = Li * f_r(p, wi, wo) * cosθ * cosθ' /(distance(p, p_prime))
            Vector3f f_r = inter.m->eval(ws, wo, N);
            float cos_theta = dotProduct(ws, N);
            float cos_theta_prime = dotProduct(-ws, NN);
            float dist = (p-p_prime).norm();
            L_dir = inter_light.emit * f_r * cos_theta * cos_theta_prime / (dist*dist) / light_pdf;
        }

    }*/

    /*if(inter_light.happened){
        Vector3f p_prime = inter_light.coords;
        float distance = std::numeric_limits<float>::max();
        Object* hit_obj;
        uint32_t index;
        //采样光源的tracing ray
        Ray trace_ray(p, p_prime - p);
        //TODO 会不会打出光线加上epsilon
        bool hit = trace(trace_ray, objects, distance, index, &hit_obj);
        
        //认为着色点与光源之间没有障碍
        if(!hit || distance >= inter_light.distance + EPSILON){
            //cos_theta cos_theat_prime 
            //l_dir = Li * f_r(p, wi, wo) * cosθ * cosθ' /(distance(p, p_prime))
            Vector3f f_r = inter.m->eval(ray.direction, trace_ray.direction, inter.normal);
            float cos_theta = dotProduct(trace_ray.direction, inter.normal);
            float cos_theta_prime = dotProduct(-trace_ray.direction, inter_light.normal);
            float dist = (p-p_prime).norm();
            L_dir = inter_light.emit * f_r * cos_theta * cos_theta_prime / (dist*dist) / light_pdf;
        }
    }
    */

    //RR判断是否进行递归
    if (get_random_float() > RussianRoulette)
        return L_dir;
    
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
    
    return L_dir + L_indir;
}