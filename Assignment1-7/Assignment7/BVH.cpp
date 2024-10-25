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
    if (0) {
        root = recursiveBuildBySVH(primitives);
    }
    else {
        root = recursiveBuild(primitives);
    }


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
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
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

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }

    return node;
}



// ���������ʽ����
BVHBuildNode* BVHAccel::recursiveBuildBySVH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());   // ��ȡ����Χ�� 
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildBySVH(std::vector{ objects[0] });
        node->right = recursiveBuildBySVH(std::vector{ objects[1] });

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)    // ��ȡ����Χ��
            centroidBounds =
            Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();   // ���ı߸����ʹ��ı߲�
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

        // Ѱ����ѻ���
        int bucketSize = 32;    // ���ֳɶ��Ͱ
        double boundsArea = bounds.SurfaceArea();  // �����,ͨ�������ø���
        double minCost = std::numeric_limits<double>::max();
        int pos = 0;
        for (int i = 1; i < bucketSize; ++i) {
            auto mid = objects.begin() + objects.size() * i / bucketSize;
            std::vector<Object*> left(objects.begin(), mid);
            std::vector<Object*> right(mid, objects.end());
            // ��ȡ��Χ�����Χ�е����
            Bounds3 leftBound, rightBound;
            for (std::size_t i = 0; i < left.size(); ++i) {
                leftBound = Union(leftBound, objects[i]->getBounds());   // ��ȡ����Χ�� 
            }
            for (std::size_t i = 0; i < right.size(); ++i) {
                rightBound = Union(rightBound, objects[i]->getBounds());   // ��ȡ����Χ�� 
            }
            double pLeft = leftBound.SurfaceArea() / boundsArea;
            double pRight = rightBound.SurfaceArea() / boundsArea;
            // ���������ÿ�������ཻ��ʱ����ȣ���ʹ����������� n ����Ա�ʾ������ð�Χ���ཻ�ĺ�ʱ
            double cost = pLeft * left.size() + pRight * right.size();
            if (cost < minCost) {
                minCost = cost;
                pos = i;
            }
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + objects.size() * pos / bucketSize;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildBySVH(leftshapes);
        node->right = recursiveBuildBySVH(rightshapes);

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
    // �����߲����Χ���ཻ
    std::array<int, 3> dirIsNeg = { ray.direction.x > 0,ray.direction.y > 0 ,ray.direction.z > 0 };
    if (!node || !node->bounds.IntersectP(ray, Vector3f::Reciprocal(ray.direction), dirIsNeg)) {
        return Intersection();
    }
    // ��Ϊ�ӽڵ�
    if (!node->left && !node->right) {
        Intersection hit = node->object->getIntersection(ray);
        return hit;
    }
    // ������������
    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);

    // ��������ĵ�
    return hit1.distance < hit2.distance ? hit1 : hit2;
}

void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}