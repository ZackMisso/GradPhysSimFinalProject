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
    initializeFromPositions(pos, 20, 4);
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

    template_verts_.resize(13, 3);
    int index = 0;

    for (double i = -2.0; i <= 2.0; i += 0.25)
    {
        template_verts_(index, 0) = i;
        template_verts_(index, 1) = 0.0;
        template_verts_(index, 2) = 0.0;

        index++;
    }

    computeLength();
    computeNormals();
    computeSegments();
}

void HairInstance::initializeFromPositions(const MatrixX3d positions, int eps, int nos)
{
    edgesPerSegment_ = eps;
    numberOfSegments_ = nos;

    // to be implemented

    computeLength();
    computeNormals();
    computeSegments();
}

void HairInstance::initializeFromCurvatures(const MatrixX3d curves, int eps, int nos, int length)
{
    edgesPerSegment_ = eps;
    numberOfSegments_ = nos;

    // to be implemented

    computeLength();
    computeNormals();
    computeSegments();
}

void HairInstance::reconstructHair()
{
    // to be implemented
}

// simple
void HairInstance::computeLength()
{
    length_ = 0.0;

    for (int i = 0; i < template_verts_.rows() - 1; i++)
    {
        Vector3d one = template_verts.row(i);
        Vector3d two = template_verts.row(i+1);

        length_ += (one-two).norm();
    }
}

void HairInstance::computeSegments()
{
    // this really is not needed for the first milestone
    assert(edgesPerSegment_ == normals_.size() / numberOfSegments_);

    int currentIndex = 0;

    for (int i = 0; i < numberOfSegments_; i++)
    {
        MatrixX3d segment;
        segment.resize(edgesPerSegment_, 3);

        for (int j = 0; j < edgesPerSegment_; j++)
        {
            segment.row(j) = template_verts_.row(currentIndex);
            currentIndex++;
        }
    }
}

// hardcode for now, fix later
void HairInstance::computeNormals()
{
    normals_.resize(template_verts_.size() - 1);

    for (int i = 0; i < normals_.size(); i++)
    {
        normals_.row(i) = Vector3d(0.0, 0.0, 1.0);
    }
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

int HairInstance::getNumberOfDofs()
{
    // maybe do more stuff here later
    return curvatures_.rows() * 3;
}
