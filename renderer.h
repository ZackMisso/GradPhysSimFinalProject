#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include "simparameters.h"

class Renderer
{
public:
    Renderer();
    ~Renderer();

    virtual void render(SimParameters simparams, std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies) = 0x0;
    virtual void renderFromBake(std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies, int iteration) = 0x0;
    virtual void constructGeom(std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies) = 0x0;
    virtual void bake() = 0x0;
    virtual void createLight() = 0x0;
    virtual void initialize() = 0x0;
};
