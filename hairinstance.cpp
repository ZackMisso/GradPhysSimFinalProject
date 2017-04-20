#include "hairinstance.h"
#include <QGLWidget>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

HairInstance::HairInstance()
{
    initializeLine(5, 2);
}

HairInstance::HairInstance(const Eigen::MatrixX3d &pos)
{
    initializeFromPositions(pos, 10, 1);
}

HairInstance::~HairInstance()
{
    // to be implemented
}

HairInstance::HairInstance(const HairInstance& other)
{
    // to be implemented for milestone 2
}

void HairInstance::initializeLine(int eps, int nos)
{
    edgesPerSegment_ = eps;
    numberOfSegments_ = nos;

    // hardcoded stuff
    length_ = 1.0;
    lengthPerSegment_ = 0.5;
    lengthPerEdge_ = 0.1;

    color_ = Vector3d(0.0, 1.0, 0.0);

    template_verts_.resize(eps * nos + 1, 3);
    int index = 0;

    for (double i = -0.5; i <= 0.5; i += 1.0 / (eps * nos))
    {
        template_verts_(index, 0) = i;
        template_verts_(index, 1) = 0.0;
        template_verts_(index, 2) = 0.0;

        index++;
    }

    cout << template_verts_.rows() << endl;

    verts_.resize(template_verts_.rows(), 3);
    verts_ = template_verts_;

    // maybe use later ?
    // verts_dot.resize(template_verts_.size());
    // verts_dot.setZero();

    initialCurvatures_.resize(numberOfSegments_, 3);
    initialCurvatures_.setZero();

    initialCurvatures_.row(0) = Vector3d(0.0, 0.0, 1.0);
    initialCurvatures_.row(1) = Vector3d(0.0, 0.0, -2.0);

    curvatures_.resize(numberOfSegments_, 3);
    curvatures_.setZero();
    curvatures_ = initialCurvatures_;

    prev_curvatures_.resize(numberOfSegments_, 3);
    prev_curvatures_.setZero();
    prev_curvatures_ = initialCurvatures_;

    curvatures_dot_.resize(numberOfSegments_, 3);
    curvatures_dot_.setZero();

    pos_ = template_verts_.row(0);

    normals_.resize(3, 3);
    normals_.row(0) = Vector3d(1.0, 0.0, 0.0);
    normals_.row(1) = Vector3d(0.0, 1.0, 0.0);
    normals_.row(2) = Vector3d(0.0, 0.0, 1.0);

    reconstructHair();

    // use these for milestone 2
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

    Vector3d initialDarboux;
    double darbouxNorm;
    Vector3d omega;

    for (int i = 0; i < numberOfSegments_; i++)
    {
        Vector3d curvature = curvatures_.row(i);

        if (i != 0)
        {
            n0 = calculateNi(n0, n1, n2, omega, i, lengthPerSegment_, darbouxNorm, 0);
            n1 = calculateNi(n0, n1, n2, omega, i, lengthPerSegment_, darbouxNorm, 1);
            n2 = calculateNi(n0, n1, n2, omega, i, lengthPerSegment_, darbouxNorm, 2);
        }

        initialDarboux = curvature[0] * n0 + curvature[1] * n1 + curvature[2] * n2;
        darbouxNorm = initialDarboux.norm();
        omega = initialDarboux / darbouxNorm;

        Vector3d nextPos;

        for (int j = 0; j < edgesPerSegment_; j++)
        {
            double s = lengthPerEdge_*(j+1);
            Vector3d currentN0 = n0;

            Vector3d par = para(currentN0, omega);
            Vector3d per = perp(currentN0, omega);
            Vector3d cros = omega.cross(per);

            Vector3d firstTerm = lastPos;
            Vector3d secondTerm = par * s;
            Vector3d thirdTerm = per * ( sin(darbouxNorm * s) / darbouxNorm );
            Vector3d fourthTerm = cros * ( ( 1 - cos(darbouxNorm * s) ) / darbouxNorm );
            nextPos = firstTerm + secondTerm + thirdTerm + fourthTerm;

            verts_.row(i * edgesPerSegment_ + j + 1) = nextPos;
        }

        lastPos = nextPos;
    }

    for (int i = 1; i < verts_.rows(); i++)
    {
        Vector3d one = verts_.row(i-1);
        Vector3d two = verts_.row(i);
    }
}

Vector3d HairInstance::calculateNi(Vector3d n0, Vector3d n1, Vector3d n2, Vector3d omega, int segment, double s, double darbouxNorm, int i)
{
    if (i == 0)
    {
        Vector3d par = para(n0, omega);
        Vector3d per = perp(n0, omega);
        Vector3d cros = omega.cross(per);

        Vector3d firstTerm = par;
        Vector3d secondTerm = per * cos(darbouxNorm * s);
        Vector3d thirdTerm = cros * sin(darbouxNorm * s);

        return firstTerm + secondTerm + thirdTerm;
    }

    else if (i == 1)
    {
        Vector3d par = para(n1, omega);
        Vector3d per = perp(n1, omega);
        Vector3d cros = omega.cross(per);

        Vector3d firstTerm = par;
        Vector3d secondTerm = per * cos(darbouxNorm * s);
        Vector3d thirdTerm = cros * sin(darbouxNorm * s);

        return firstTerm + secondTerm + thirdTerm;
    }

    else
    {
        Vector3d par = para(n2, omega);
        Vector3d per = perp(n2, omega);
        Vector3d cros = omega.cross(per);

        Vector3d firstTerm = par;
        Vector3d secondTerm = per * cos(darbouxNorm * s);
        Vector3d thirdTerm = cros * sin(darbouxNorm * s);

        return firstTerm + secondTerm + thirdTerm;
    }

}

Vector3d HairInstance::para(Vector3d vec, Vector3d omega)
{
    return vec.dot(omega) * omega;
}

Vector3d HairInstance::perp(Vector3d vec, Vector3d omega)
{
    return vec - para(vec, omega);
}

// will be needed for milestone 2
void HairInstance::computeLength()
{
    // length_ = 0.0;
    //
    // for (int i = 0; i < template_verts_.rows() - 1; i++)
    // {
    //     Vector3d one = template_verts_.row(i);
    //     Vector3d two = template_verts_.row(i+1);
    //
    //     length_ += (one-two).norm();
    // }
}

void HairInstance::computeSegments()
{
    // this really is not needed for the first milestone

    // assert(edgesPerSegment_ == normals_.size() / numberOfSegments_);
    //
    // int currentIndex = 0;
    //
    // for (int i = 0; i < numberOfSegments_; i++)
    // {
    //     MatrixX3d segment;
    //     segment.resize(edgesPerSegment_, 3);
    //
    //     for (int j = 0; j < edgesPerSegment_; j++)
    //     {
    //         segment.row(j) = template_verts_.row(currentIndex);
    //         currentIndex++;
    //     }
    // }
}

// hardcode for now, use for milestone 2
void HairInstance::computeNormals()
{
    // normals_.resize(template_verts_.size() - 1);

    // for (int i = 0; i < normals_.size(); i++)
    // {
    //     normals_.row(i) = Vector3d(0.0, 0.0, 1.0);
}

void HairInstance::render2D(double scale)
{
    glLineWidth(2.0);
    glColor3f(color_[0], color_[1], color_[2]);

    glBegin(GL_LINES);
    for (int i = 0; i < verts_.rows()-1; i++)
    {
        glVertex3d(verts_(i, 0), verts_(i, 1), verts_(i, 2));
        glVertex3d(verts_(i + 1, 0), verts_(i + 1, 1), verts_(i + 1, 2));
    }
    glEnd();

    glColor3f(1.0, 0.0, 0.0);
    glPointSize(4.0);
    glBegin(GL_POINTS);
    for (int i = 0; i < verts_.rows() / 2; i++)
    {
        glVertex3d(verts_(i, 0), verts_(i, 1), verts_(i, 2));
    }
    glEnd();

    glColor3f(0.0, 0.0, 1.0);
    glPointSize(4.0);
    glBegin(GL_POINTS);
    for (int i = verts_.rows() / 2; i < verts_.rows(); i++)
    {
        glVertex3d(verts_(i, 0), verts_(i, 1), verts_(i, 2));
    }
    glEnd();
}

void HairInstance::render3D(double scale, double radius)
{
    // to be implemented
}

int HairInstance::getNumberOfDofs()
{
    // maybe do more stuff here later
    return curvatures_.rows();
}
