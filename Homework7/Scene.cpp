//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

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

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
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
    // TO DO Implement Path Tracing Algorithm here
    Intersection p = intersect(ray);  //ray intersect with object
    Material *m = p.m;  //material of object
    if(!p.happened) return Vector3f(0);
    
    Vector3f light_dir(0);   //directional light
    Vector3f light_indir(0);   //indirectional light

    Vector3f wo = normalize(-ray.direction);
    Vector3f pos = p.coords;
    Vector3f n = normalize(p.normal);
        
    float pdf_light;
    Intersection light_insec;
    sampleLight(light_insec, pdf_light);  // initialize light_insec

    Vector3f x = light_insec.coords;  //pos of light sampling point
    Vector3f nn = normalize(light_insec.normal);  //normal of light sampling point
    Vector3f ws = normalize(x - pos); //direction of light sampling

    Ray lightray(pos, ws);
    Intersection light_ray = intersect(lightray);
    
    float dist = (x - pos).norm();
    if(light_ray.happened && (light_ray.coords - x).norm() < 0.1)
        light_dir = light_insec.emit * m->eval(wo, ws, n) * dotProduct(ws, n) * dotProduct(-ws, nn)
            / (dist * dist) / pdf_light;

    if(get_random_float() > RussianRoulette) return Vector3f(0);
        Vector3f wi = normalize(m->sample(wo, n));
        Ray objectray(pos, wi);
        Intersection object_ray = intersect(objectray);
        if(m->hasEmission()) return m->getEmission();
        light_indir = castRay(objectray, depth) * m->eval(wi, wo, n) * dotProduct(wi, n) / m->pdf(wo, wi, n) / RussianRoulette;
    
    return (light_dir + light_indir); 
}