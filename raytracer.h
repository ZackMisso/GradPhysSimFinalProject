#pragma once

#include "renderer.h"

class RayTracer : public Renderer
{
private:
public:
    RayTracer();
    ~RayTracer();

    virtual void render(SimParameters simparams, std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies);
    virtual void renderFromBake(std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies);
    virtual void constructGeom(std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies);
    virtual void bake();
    virtual void createLight();
    virtual void initialize();
};
