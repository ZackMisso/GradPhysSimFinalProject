#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <QGLWidget>
#include <vector>
#include "simparameters.h"
#include "rigidbodyinstance.h"
#include "hairinstance.h"

class Renderer
{
public:
    Renderer();
    ~Renderer();

    virtual void render(SimParameters simparams, std::vector<HairInstance*> &guideHairs, std::vector<HairInstance*> &interpHairs, std::vector<RigidBodyInstance*> &bodies) = 0;
    virtual void renderFromBake(std::vector<HairInstance*> &guideHairs, std::vector<HairInstance*> &interpHairs, std::vector<RigidBodyInstance*> &bodies, int iteration) = 0;
    virtual void constructGeom(std::vector<HairInstance*> &guideHairs, std::vector<HairInstance*> &interpHairs, std::vector<RigidBodyInstance*> &bodies) = 0;
    virtual void bake() = 0;
    virtual void createLight() = 0;
    virtual void initialize() = 0;
};
