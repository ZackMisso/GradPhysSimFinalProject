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

    // hardcoded stuff
    length_ = 10.0;
    lengthPerSegment_ = 10.0;
    lengthPerEdge_ = 1.0;

    template_verts_.resize(11, 3);
    int index = 0;

    for (double i = 0.0; i <= 10.0; i += 1)
    {
        template_verts_(index, 0) = i;
        template_verts_(index, 1) = 0.0;
        template_verts_(index, 2) = 0.0;

        index++;
    }

    // verts_.resize(template_verts_.size());
    // verts_ = template_verts_;
    //
    // verts_dot.resize(template_verts_.size());
    // verts_.setZero();

    initialCurvatures_.resize(numberOfSegments_, 3);
    initialCurvatures_.setZero();

    initialCurvatures_.row() = Vector3d(,,);

    curvatures_.resize(numberOfSegments_);
    curvatures_.setZero();
    curvatures_ = initialCurvatures_;

    prev_curvatures_.resize(numberOfSegments_);
    prev_curvatures_.setZero();
    prev_curvatures_ = initialCurvatures_;

    curvatures_dot_.resize(numberOfSegments_);
    curvatures_dot_.setZero();

    pos_ = template_verts_.row(0);

    reconstructHair();


    // computeLength();
    // computeNormals();
    // computeSegments();
}

void HairInstance::initializeFromPositions(const MatrixX3d positions, int eps, int nos)
{
    edgesPerSegment_ = eps;
    numberOfSegments_ = nos;

    // to be implemented for milestone 2

    computeLength();
    computeNormals();
    computeSegments();
}

void HairInstance::initializeFromCurvatures(const MatrixX3d curves, int eps, int nos, int length)
{
    edgesPerSegment_ = eps;
    numberOfSegments_ = nos;

    // to be implemented for milestone 2

    computeLength();
    computeNormals();
    computeSegments();
}

void HairInstance::reconstructHair()
{
    // L is in terms of the segment
    // s is distance between 0 and L for segment

    Vector3d lastPos = pos_;
    verts_.row(0) = lastPos;

    Vector3d n0 = normals_.row(0);
    Vector3d n1 = normals_.row(1);
    Vector3d n2 = normals_.row(2);

    for (int i = 0; i < numberOfSegments_; i++)
    {
        Vector3d curvature = curvatures_.row(i);

        Vector3d initialDarboux = curvature[0] * n0 + curvature[1] * n1 + curvature[2] * n2;
        double darbouxNorm = initialDarboux.norm();
        Vector3d omega = initialDarboux / darbouxNorm;

        for (int j = 0; j < edgesPerSegment_; j++)
        {
            Vector3d currentN0 = calculateNi(n0, n1, n2, omega, i, lengthPerEdge_*j+1, 0);

            Vector3d firstTerm = lastPos;
            Vector3d secondTerm = para(currentN0, omega) * s;
            Vector3d thirdTerm = perp(currentN0, omega) * ( sin(darbouxNorm * s) / darbouxNorm );
            Vector3d fourthTerm = omega.cross(perp(currentN0, omega)) * ( ( 1 - cos(darbouxNorm * s) ) / darbouxNorm );
            Vector3d nextPos = firstTerm + secondTerm + thirdTerm + fourthTerm;

            verts_.row(i * edgesPerSegment_ + j) = nextPos;
        }

        // will need for more than one strand
        // n0 = calculateNi(n0, n1, n2, omega, i, lengthPerEdge_, 0);
        // n1 = calculateNi(n0, n1, n2, omega, i, lengthPerEdge_, 0);
        // n2 = calculateNi(n0, n1, n2, omega, i, lengthPerEdge_, 0);
    }
}

Vector3d HairInstance::calculateNi(Vector3d n0, Vector3d n1, Vector3d n2, Vector3d omega, int segment, double s, int i)
{
    Vector3d firstTerm = para(n0, omega);
    Vector3d secondTerm = perp(n0, omega) * cos(darbouxNorm * s);
    Vector3d thirdTerm = omega.cross(perp(n0, omega)) * sin(darbouxNorm * s);
    return firstTerm + secondTerm + thirdTerm;
}

Vector3d HairInstance::para(Vector3d vec, Vector3d omega)
{
    return vec.dot(omega) * omgea;
}

Vector3d HairInstance::perp(Vector3d vec, Vector3d omega)
{
    return vec - para(vec, omega);
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
    return curvatures_.rows();
}
