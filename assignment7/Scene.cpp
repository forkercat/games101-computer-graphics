//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
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
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(const Ray& ray, const std::vector<Object*>& objects, float& tNear,
                  uint32_t& index, Object** hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float    tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
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
    if (depth > this->maxDepth)
    {
        return Vector3f(0.0f, 0.0f, 0.0f);
    }

    Intersection p = Scene::intersect(ray);

    if (!p.happened)  // not hit anything
    {
        return Vector3f(0.0f);
    }

    // hit light
    if (p.m->hasEmission())
    {
        return Vector3f(1.0f, 1.0f, 1.0f);
    }

    Vector3f wo = -ray.direction;
    Vector3f N = normalize(p.normal);

    // Sample light
    Intersection lightInter;
    float        pdf_light;
    sampleLight(lightInter, pdf_light);

    Vector3f wi = normalize(lightInter.coords - p.coords);
    Vector3f NN = normalize(lightInter.normal);

    // Shadow ray is blocked by objects
    Ray          shadowRay(p.coords, wi);
    Intersection shadowInter = intersect(shadowRay);

    // Direct Lighting
    Vector3f L_dir;
    if (shadowInter.happened && shadowInter.m->hasEmission())  // hit light source
    {
        if (pdf_light < EPSILON) pdf_light = EPSILON;

        Vector3f brdf = p.m->eval(wo, wi, N);
        float    inv = 1.0f / (shadowInter.distance * shadowInter.distance * pdf_light);
        L_dir = lightInter.emit * brdf * dotProduct(wi, N) * dotProduct(-wi, NN) * inv;
    }

    // Indirect Lighting
    Vector3f L_indir;
    if (get_random_float() <= RussianRoulette)  // RR Test
    {
        Vector3f wi_random = normalize(p.m->sample(wo, N));

        // Test if ray hits a non-emitting object
        Ray          ray_indir(p.coords, wi_random);
        Intersection inter_indir = intersect(ray_indir);
        if (inter_indir.happened && !inter_indir.m->hasEmission())
        {
            float pdf_hemi = p.m->pdf(wo, wi_random, N);

            if (pdf_hemi < EPSILON) pdf_hemi = EPSILON;

            Vector3f brdf = p.m->eval(wo, wi_random, N);
            float    inv = 1.0f / (pdf_hemi * RussianRoulette);
            L_indir = castRay(ray_indir, ++depth) * brdf * dotProduct(wi_random, N) * inv;
        }
    }

    return L_dir + L_indir;
}