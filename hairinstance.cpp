#include "hairinstance.h"
#include <QGLWidget>
#include <iostream>

using namespace std;
using namespace Eigen;

HairInstance::HairInstance()
{
    initializeLine(20, 1);
}

HairInstance::HairInstance(const Eigen::MatrixX3d &pos)
{
    initializeFromPositions(pos, 20, 1);
}

HairInstance::~HairInstance()
{
    // to be implemented
}

HairInstance::HairInstance(const HairInstance& other)
{
    // to be implemented
}

void HairInstance::initializeLine(int eps, int nos)
{
    edgesPerSegment_ = eps;
    numberOfSegments_ = nos;
    // to be implemented
}

void HairInstance::initializeFromPositions(const MatrixX3d positions, int eps, int nos)
{
    edgesPerSegment_ = eps;
    numberOfSegments_ = nos;
    // to be implemented
}

void HairInstance::initializeFromCurvatures(const MatrixX3d curves, int eps, int nos, int length)
{
    edgesPerSegment_ = eps;
    numberOfSegments_ = nos;
    // to be implemented
}

void HairInstance::reconstructHair()
{
    // to be implemented
}

void HairInstance::computeLength()
{
    // to be implemented
}

void HairInstance::computeSegments()
{
    // to be implemented
}

void HairInstance::computeNormals()
{
    // to be implemented
}

void HairInstance::render(double scale)
{
    glLineWidth(2.0);
    glColor3f(color[0], color[1], color[2]);

    glBegin(GL_LINES);
    for (int i = 0; i < pos.rows()-1; i++)
    {
        glVertex3d(pos(i, 0), pos(i, 1), pos(i, 2));
        glVertex3d(pos(i + 1, 0), pos(i + 1, 1), pos(i + 1, 2));
    }
    glEnd();
}
