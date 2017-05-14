#pragma once

#include "renderer.h"

class GLRenderer : public Renderer
{
private:
public:
    GLRenderer();
    ~GLRenderer();

    virtual void render(SimParameters simparams, std::vector<HairInstance*> &guideHairs, std::vector<HairInstance*> &interpHairs, std::vector<RigidBodyInstance*> &bodies);
    virtual void renderFromBake(std::vector<HairInstance*> &guideHairs, std::vector<HairInstance*> &interpHairs, std::vector<RigidBodyInstance*> &bodies, int iteration);
    virtual void constructGeom(std::vector<HairInstance*> &guideHairs, std::vector<HairInstance*> &interpHairs, std::vector<RigidBodyInstance*> &bodies);
    virtual void bake();
    virtual void createLight();
    virtual void initialize();

    void renderHairStrand(HairInstance* hair);
    void renderHairCylinder(HairInstance* hair);
    void renderHairStrandSegment(HairInstance* hair, int segment, int maxSegment);
    void renderHairCylinderSegment(HairInstance* hair, int segment, int maxSegment);
};
