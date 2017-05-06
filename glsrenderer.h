#pragma once

#include "renderer.h"

class GLSRenderer : public Renderer
{
private:
public:
    GLSRenderer();
    ~GLSRenderer();

    virtual void render(SimParameters simparams, std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies);
    virtual void renderFromBake(std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies);
    virtual void constructGeom(std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies);
    virtual void bake();
    virtual void createLight();
    virtual void initialize();
};
