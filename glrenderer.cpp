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
    for(int i = 0; i < guideHairs.size(); i++)
    {
        renderHairStrand(guideHairs[i]);
    }
    // for(int i = 0; i < interpHairs.size(); i++)
    // {
    //     renderHairStrand(interpHairs[i]);
    // }
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

    // cout << "WHAT THE FUCK" << endl;

    glColor4d(hair->color_[0], hair->color_[1], hair->color_[2], 1.0);

    glLineWidth(2.0);

    glBegin(GL_LINES);
    for (int i = 0; i < hair->verts_.rows() - 1; i++)
    {
        glVertex3d(hair->verts_(i, 0), hair->verts_(i, 1), hair->verts_(i, 2));
        glVertex3d(hair->verts_(i + 1, 0), hair->verts_(i + 1, 1), hair->verts_(i + 1, 2));
    }
    glEnd();

    // cout << "HELLS" << endl;
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

void GLRenderer::renderHairStrandSegment(HairInstance* hair, int segment)
{
    //glShadeModel(GL_FLAT);
    //glEnable(GL_LIGHTING);
    //glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    //glEnable(GL_COLOR_MATERIAL);
    //glEnable(GL_NORMALIZE);

    int eps = hair->getEdgesPerSegment();

    //glColor4d(hair->color_[0], hair->color_[1], hair->color_[2], 1.0);

    glLineWidth(2.0);

    glBegin(GL_LINES);
    for (int i = segment * eps; i < segment * (eps + 1); i++)
    {
        glVertex3d(hair->verts_(i, 0), hair->verts_(i, 1), hair->verts_(i, 2));
        glVertex3d(hair->verts_(i + 1, 0), hair->verts_(i + 1, 1), hair->verts_(i + 1, 2));
    }
    glEnd();
}

void GLRenderer::renderHairCylinderSegment(HairInstance * hair, int segment)
{
    // to be implemented
}
