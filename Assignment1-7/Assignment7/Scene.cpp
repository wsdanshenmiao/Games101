//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
    return this->bvh->Intersect(ray);
}


void Scene::sampleLight(Intersection& pos, float& pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray& ray,
    const std::vector<Object*>& objects,
    float& tNear, uint32_t& index, Object** hitObject)
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
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // 递归最大深度
    if (depth > this->maxDepth)
        return Vector3f();

    // 寻找射线打中的物体
    Intersection objInter = intersect(ray); //打中的物体
    Material* material = objInter.m;
    const Vector3f& objNormal = objInter.normal;
    const Vector3f& rayDir = ray.direction;
    if (!objInter.happened)
        return Vector3f();

    // 打到光源
    if (objInter.m->hasEmission()) {
        return objInter.m->getEmission();
    }

    // 计算直接光照和间接光照
    Vector3f dirLight, indirLight;
    Intersection lightInter;    // 采样的光源点
    float lightPDF;
    sampleLight(lightInter, lightPDF);

    Vector3f objToLightDir(lightInter.coords - objInter.coords);
    Ray lightRay(objInter.coords, objToLightDir.normalized());  // 出射向量
    // 直接光照,这里需要判断光源和着色点之间是否有阻挡，同时需要预留浮点数的误差
    if (intersect(lightRay).distance - objToLightDir.norm() > -0.001) {
        dirLight = lightInter.emit
            * material->eval(rayDir, lightRay.direction, objNormal)
            * dotProduct(lightRay.direction, objNormal)
            * dotProduct(-lightRay.direction, lightInter.normal)
            / pow(objToLightDir.norm(), 2)
            / lightPDF;
    }

    // 间接光照
    if (get_random_float() < RussianRoulette) {
        Vector3f sampleDir = material->sample(rayDir, objNormal).normalized();  // 采样的向量
        Ray sampleRay(objInter.coords, sampleDir);
        Intersection sampleInter = intersect(sampleRay);    // 采样向量打到的点
        if (sampleInter.happened && !sampleInter.m->hasEmission()) {
            indirLight = castRay(sampleRay, depth + 1)
                * material->eval(rayDir, sampleDir, objNormal)
                * dotProduct(sampleDir, objNormal)
                / material->pdf(rayDir, sampleDir, objNormal)
                / RussianRoulette;
        }
    }

    return dirLight + indirLight;
}