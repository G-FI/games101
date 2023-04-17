#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    // for (int i = 0; i < objects.size(); ++i)
    //     bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        //将重心的作为BBox的对角线上的点来进行合并
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        //选择最长的轴进行划分
        int dim = centroidBounds.maxExtent();
        //对物体进行排序
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                    f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                    f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                    f2->getBounds().Centroid().z;
            });
            break;
        }
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        bool group = false;
        if(splitMethod == SplitMethod::SAH && group){
            auto S = centroidBounds.SurfaceArea();
            float min_cost = std::numeric_limits<float>::max();
            int partation = 0;
            Bounds3 left_bound;
            int n = objects.size();
            int groups = 10;
            int group_size = n / groups;
            for(int i = 0; i < groups; ++i){
                int mid = (i + 1) * group_size;
                for(int j = i * group_size ; j < mid; ++j){ 
                    left_bound = Union(left_bound, objects[j]->getBounds().Centroid());  //将重心的作为BBox的对角线上的点来进行合并
                }
               
                Bounds3 right_bound;
                for(int j = mid; j < n; ++j){
                    right_bound = Union(right_bound, objects[j]->getBounds().Centroid());
                }

                auto SA = left_bound.SurfaceArea();
                auto SB = right_bound.SurfaceArea();
                auto cost = SA/S * mid + SB/S * (n - mid);
                if(cost < min_cost){
                    min_cost = cost;
                    partation = mid;
                }
            }
            middling = objects.begin() + partation + 1;
        }
        else if(splitMethod == SplitMethod::SAH && !group){
            auto S = centroidBounds.SurfaceArea();
            float min_cost = std::numeric_limits<float>::max();
            int partation = 0;
            Bounds3 left_bound;
            int n = objects.size();
            for(int i = 0; i < n - 1; ++i){
                left_bound = Union(left_bound, objects[i]->getBounds().Centroid());  //将重心的作为BBox的对角线上的点来进行合并
                Bounds3 right_bound;
                for(int j = i + 1; j < n; ++j){
                    right_bound = Union(right_bound, objects[j]->getBounds().Centroid());
                }

                auto SA = left_bound.SurfaceArea();
                auto SB = right_bound.SurfaceArea();
                auto cost = SA/S * (i + 1)  + SB/S * (n - i - 1);
                if(cost < min_cost){
                    min_cost = cost;
                    partation = i;
                }
            }
            middling = objects.begin() + partation + 1;
        }

        //将物体划分为两部分
        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));
        //递归进行构建
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        //父节点的BBox为子节点的BBox的并
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection isect;
    //BVH节点为空或者不与光线相交，则返回默认Intersection
    if(node == nullptr || !node->bounds.IntersectP(ray, ray.direction_inv, {}))
        return isect;

    //当前为也叶子节点，计算光线与物体相交，该bvh加速结构中叶子节点中只有一个物体
    if(node->left == nullptr && node->right == nullptr){
        isect = node->object->getIntersection(ray);
        return isect;
    }
    
    //当前节点为非叶子节点，递归
    Intersection isect1, isect2;
    isect1 = getIntersection(node->left, ray);
    isect2 = getIntersection(node->right, ray);

    //若是与两棵子树都有交点，返回距离更近的交点
    if(isect1.happened && isect2.happened){
        return isect1.distance < isect2.distance? isect1: isect2;
    }
    else if(isect1.happened){
        return isect1;
    }
    else{
        return isect2;
    }
}