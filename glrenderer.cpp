#include "glrenderer.h"
#include <iostream>

using namespace std;

GLRenderer::GLRenderer()
{
    // to be implemented
}

GLRenderer::~GLRenderer()
{
    // to be implemented
}

void GLRenderer::render(SimParameters simparams, std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies)
{
    if (!simparams.showSegments)
    {
        for(int i = 0; i < guideHairs.size(); i++)
        {
            renderHairStrand(guideHairs[i]);
        }
        for(int i = 0; i < interpHairs.size(); i++)
        {
            renderHairStrand(interpHairs[i]);
        }
    }
    else
    {
        for(int i = 0; i < guideHairs.size(); i++)
        {
            for (int j = 0; j < guideHairs[i]->getNumberOfSegments(); j++)
            {
                renderHairStrandSegment(guideHairs[i], j, guideHairs[i]->getNumberOfSegments());
            }
        }
        for(int i = 0; i < interpHairs.size(); i++)
        {
            for (int j = 0; j < interpHairs[i]->getNumberOfSegments(); j++)
            {
                renderHairStrandSegment(interpHairs[i], j, interpHairs[i]->getNumberOfSegments());
            }
        }
    }
}

void GLRenderer::renderFromBake(std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies, int iteration)
{
    // to be implemented
}


void GLRenderer::constructGeom(std::vector<HairInstance*> guideHairs, std::vector<HairInstance*> interpHairs, std::vector<RigidBodyInstance*> bodies)
{
    // to be implemented
}

void GLRenderer::bake()
{
    // to be implemented
}

void GLRenderer::createLight()
{
    // to be implemented
}

void GLRenderer::initialize()
{
    // to be implemented
}

void GLRenderer::renderHairStrand(HairInstance* hair)
{
    //glShadeModel(GL_FLAT);
    //glEnable(GL_LIGHTING);
    //glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    //glEnable(GL_COLOR_MATERIAL);
    //glEnable(GL_NORMALIZE);

    glColor4d(hair->color_[0], hair->color_[1], hair->color_[2], 1.0);

    glLineWidth(2.0);

    glBegin(GL_LINES);
    for (int i = 0; i < hair->verts_.rows() - 1; i++)
    {
        glVertex3d(hair->verts_(i, 0), hair->verts_(i, 1), hair->verts_(i, 2));
        glVertex3d(hair->verts_(i + 1, 0), hair->verts_(i + 1, 1), hair->verts_(i + 1, 2));
    }
    glEnd();
}

void GLRenderer::renderHairCylinder(HairInstance* hair)
{
    // glColor4d(hair->color_[0], hair->color_[1], hair->color_[2], 1.0);

    // glLineWidth(2.0);

    // int cylinderRes = 20;

    // Eigen::Vector3d normOne = hair->normals_.row(1);
    // Eigen::Vector3d normTwo = hair->normals_.row(2);

    // glBegin(GL_QUADS);

    // for (int j = 1; j < hair->verts_.rows(); j++)
    // {
    //     for (int i = 0; i < cylinderRes; i++)
    //     {
    //         // to be implemented
    //         // BLAHAHAHSDFHAS
    //     }
    // }
}

void GLRenderer::renderHairStrandSegment(HairInstance* hair, int segment, int maxSegment)
{
    //glShadeModel(GL_FLAT);
    //glEnable(GL_LIGHTING);
    //glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    //glEnable(GL_COLOR_MATERIAL);
    //glEnable(GL_NORMALIZE);

    int eps = hair->getEdgesPerSegment();

    glColor4d(hair->color_[0] / (maxSegment / (segment + 1)) + 0.1, hair->color_[1] / (maxSegment / (segment + 1)) + 0.1, hair->color_[2] / (maxSegment / (segment + 1)) + 0.1, 1.0);

    glLineWidth(2.0);

    glBegin(GL_LINES);
    for (int i = segment * eps; i < (segment + 1) * (eps); i++)
    {
        glVertex3d(hair->verts_(i, 0), hair->verts_(i, 1), hair->verts_(i, 2));
        glVertex3d(hair->verts_(i + 1, 0), hair->verts_(i + 1, 1), hair->verts_(i + 1, 2));
    }
    glEnd();
}

void GLRenderer::renderHairCylinderSegment(HairInstance * hair, int segment, int maxSegment)
{
    // to be implemented
}
