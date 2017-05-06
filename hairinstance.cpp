#include "hairinstance.h"
#include <QGLWidget>
#include <iostream>
#include <Eigen/Dense>
#include "vectormath.h"

using namespace std;
using namespace Eigen;

HairInstance::HairInstance()
{
    initializeLine(100, 2);
}

HairInstance::HairInstance(const Eigen::MatrixX3d &pos)
{
    initializeFromPositions(pos, 100, 2);
}

HairInstance::HairInstance(const Eigen::VectorXd &curves, Vector3d startPos, Matrix3d startNorm)
{
    initializeFromCurvatures(curves, 100, 2, 1.0, startPos, startNorm);
}

HairInstance::HairInstance(const Eigen::VectorXd &curves, Vector3d startPos, Matrix3d startNorm, int eps, int nos, double length)
{
    initializeFromCurvatures(curves, eps, nos, length, startPos, startNorm);
}

HairInstance::HairInstance(HairInstance* one, HairInstance* two, double alpha, double beta) : guideOne(one), guideTwo(two), baryOne(alpha), baryTwo(beta)
{
    verts_.resize(one->verts_.rows(), 3);
    curvatures_.resize(one->curvatures_.size());
    curvatures_.setZero();
    verts_.setZero();
    numberOfSegments_ = one->numberOfSegments_;
    edgesPerSegment_ = one->edgesPerSegment_;
    lengthPerEdge_ = one->lengthPerEdge_;
    lengthPerSegment_ = one->lengthPerSegment_;
    length_ = one->length_;
    normals_ = one->normals_; // THIS NEEDS TO BE CHANGED

    interpolateTwo();
    reconstructHair();
}

HairInstance::HairInstance(HairInstance* one, HairInstance* two, HairInstance* three, double alpha, double beta, double gamma) : guideOne(one), guideTwo(two), guideThree(three), baryOne(alpha), baryTwo(beta), baryThree(gamma)
{
    verts_.resize(one->verts_.rows(), 3);
    numberOfSegments_ = one->numberOfSegments_;
    edgesPerSegment_ = one->edgesPerSegment_;
    lengthPerEdge_ = one->lengthPerEdge_;
    lengthPerSegment_ = one->lengthPerSegment_;
    length_ = one->length_;

    interpolateThree();
    reconstructHair();
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
    lengthPerEdge_ = 0.005;

    color_ = Vector3d(0.74, 0.19, 0.19);

    template_verts_.resize((eps * nos + 1), 3);
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

    initialCurvatures_.resize(numberOfSegments_ * 3);
    initialCurvatures_.setZero();

    initialCurvatures_.segment<3>(0) = Vector3d(0.0, 0.0, 1.0);
    initialCurvatures_.segment<3>(3) = Vector3d(0.0, 0.0, -2.0);

    curvatures_.resize(numberOfSegments_ * 3);
    curvatures_.setZero();
    curvatures_ = initialCurvatures_;

    prev_curvatures_.resize(numberOfSegments_ * 3);
    prev_curvatures_.setZero();
    prev_curvatures_ = initialCurvatures_;

    curvatures_dot_.resize(numberOfSegments_ * 3);
    curvatures_dot_.setZero();

    pos_ = template_verts_.row(0);

    normals_.row(0) = Vector3d(1.0, 0.0, 0.0);
    normals_.row(1) = Vector3d(0.0, 1.0, 0.0);
    normals_.row(2) = Vector3d(0.0, 0.0, 1.0);

    reconstructHair();
}

void HairInstance::initializeFromPositions(const MatrixX3d positions, int eps, int nos)
{
    edgesPerSegment_ = eps;
    numberOfSegments_ = nos;

    // to be implemented for fast rendering
}

void HairInstance::initializeFromCurvatures(const VectorXd curves, int eps, int nos, double length, Vector3d startPos, Matrix3d startNorm)
{
    edgesPerSegment_ = eps;
    numberOfSegments_ = nos;

    length_ = length;
    lengthPerSegment_ = length / nos;
    lengthPerEdge_ = length / (eps * nos);

    color_ = Vector3d(0.74, 0.19, 0.19);

    template_verts_.resize(eps * nos + 1, 3);
    template_verts_.setZero();

    verts_.resize(template_verts_.rows(), 3);

    initialCurvatures_.resize(numberOfSegments_ * 3);
    initialCurvatures_.setZero();

    initialCurvatures_ = curves;

    curvatures_.resize(numberOfSegments_ * 3);
    curvatures_.setZero();
    curvatures_ = initialCurvatures_;

    prev_curvatures_.resize(numberOfSegments_ * 3);
    prev_curvatures_.setZero();
    prev_curvatures_ = initialCurvatures_;

    curvatures_dot_.resize(numberOfSegments_ * 3);
    curvatures_dot_.setZero();

    pos_ = startPos;

    normals_ = startNorm;

    reconstructHair();

    for (int i = 0; i < verts_.rows(); i++)
    {
        template_verts_.row(i) = verts_.row(i);
    }
}

void HairInstance::buildConfiguration(VectorXd &q, VectorXd &qprev, VectorXd &v, int &index)
{
    int ndofs = getNumberOfDofs();

    for (int i = 0; i < ndofs / 3; i++)
    {
        q.segment<3>(3 * (index + i)) = curvatures_.segment<3>(i * 3);
        qprev.segment<3>(3 * (index + i)) = prev_curvatures_.segment<3>(i * 3);
        v.segment<3>(3 * (index + i)) = curvatures_dot_.segment<3>(i * 3);
    }
    index += ndofs / 3;
}

void HairInstance::unbuildConfiguration(const VectorXd &q, const VectorXd &v, int &index)
{
    int ndofs = getNumberOfDofs();
    prev_curvatures_ = curvatures_;

    for (int i = 0; i < ndofs / 3; i++)
    {
        curvatures_.segment<3>(i * 3) = q.segment<3>(3 * (index + i));
        curvatures_dot_.segment<3>(i * 3) = v.segment<3>(3 * (index + i));
    }
    index += ndofs / 3;
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
        Vector3d curvature = curvatures_.segment<3>(i * 3);

        if (i != 0)
        {
            Vector3d newN0 = calculateNi(n0, n1, n2, omega, i, lengthPerSegment_, darbouxNorm, 0);
            Vector3d newN1 = calculateNi(n0, n1, n2, omega, i, lengthPerSegment_, darbouxNorm, 1);
            Vector3d newN2 = calculateNi(n0, n1, n2, omega, i, lengthPerSegment_, darbouxNorm, 2);

            n0 = newN0;
            n1 = newN1;
            n2 = newN2;
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
}

void HairInstance::calculateNewInitialConditions(Vector3d q, Vector3d start, Matrix3d n, Vector3d &newStart, Matrix3d &newNorms)
{
    newStart.setZero();
    newNorms.setZero();

    Vector3d n0 = n.row(0);
    Vector3d n1 = n.row(1);
    Vector3d n2 = n.row(2);

    Vector3d initialDarboux;
    double darbouxNorm;
    Vector3d omega;

    initialDarboux = q[0] * n0 + q[1] * n1 + q[2] * n2;
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

        Vector3d firstTerm = newStart;
        Vector3d secondTerm = par * s;
        Vector3d thirdTerm = per * ( sin(darbouxNorm * s) / darbouxNorm );
        Vector3d fourthTerm = cros * ( ( 1 - cos(darbouxNorm * s) ) / darbouxNorm );
        nextPos = firstTerm + secondTerm + thirdTerm + fourthTerm;
    }

    Vector3d newN0 = calculateNi(n0, n1, n2, omega, -1, lengthPerSegment_, darbouxNorm, 0);
    Vector3d newN1 = calculateNi(n0, n1, n2, omega, -1, lengthPerSegment_, darbouxNorm, 1);
    Vector3d newN2 = calculateNi(n0, n1, n2, omega, -1, lengthPerSegment_, darbouxNorm, 2);

    newStart = nextPos;
    newNorms.row(0) = newN0;
    newNorms.row(1) = newN1;
    newNorms.row(2) = newN2;
}

Vector3d HairInstance::rsh(double s, Vector3d q, Vector3d start, Matrix3d n)
{
    Vector3d r;
    r.setZero();

    Vector3d n0 = n.row(0);
    Vector3d n1 = n.row(1);
    Vector3d n2 = n.row(2);

    Vector3d initialDarboux = q[0] * n0 + q[1] * n1 + q[2] * n2;
    double darbouxNorm = initialDarboux.norm();
    Vector3d omega = initialDarboux / darbouxNorm;

    Vector3d par = para(n0, omega);
    Vector3d per = perp(n0, omega);
    Vector3d cros = omega.cross(per);

    Vector3d firstTerm = start;
    Vector3d secondTerm = par * s;
    Vector3d thirdTerm = per * ( sin(darbouxNorm * s) / darbouxNorm );
    Vector3d fourthTerm = cros * ( ( 1 - cos(darbouxNorm * s) ) / darbouxNorm );

    r = firstTerm + secondTerm + thirdTerm + fourthTerm;

    return r;
}

Matrix3d HairInstance::drsh(double s, Vector3d q, Vector3d start, Matrix3d n)
{
    Matrix3d dr;
    dr.setZero();

    Vector3d n0 = n.row(0);
    Vector3d n1 = n.row(1);
    Vector3d n2 = n.row(2);

    Vector3d initialDarboux = q[0] * n0 + q[1] * n1 + q[2] * n2;
    double darbouxNorm = initialDarboux.norm();
    Vector3d omega = initialDarboux / darbouxNorm;

    Vector3d par = para(n0, omega);
    Vector3d per = perp(n0, omega);
    Vector3d cros = omega.cross(per);

    Matrix3d firstTerm;
    Matrix3d secondTerm;
    Matrix3d thirdTerm;

    firstTerm.setZero();
    secondTerm.setZero();
    thirdTerm.setZero();

    firstTerm = per * (darbouxNorm * cos(darbouxNorm * s) * s * q.transpose() / darbouxNorm - sin(darbouxNorm * s) * q.transpose() / darbouxNorm) / (darbouxNorm * darbouxNorm);

    secondTerm = VectorMath::crossProductMatrix(per * ((1 - cos(darbouxNorm * s)) / darbouxNorm)) * (n.transpose() / darbouxNorm - initialDarboux * q.transpose() / (darbouxNorm * darbouxNorm * darbouxNorm));

    thirdTerm = VectorMath::crossProductMatrix(omega) * per * ( ( darbouxNorm * sin(darbouxNorm * s) * s * q.transpose() / darbouxNorm - (1 - cos(darbouxNorm * s)) * q.transpose() / darbouxNorm) / (darbouxNorm * darbouxNorm) );

    dr = firstTerm - secondTerm + thirdTerm;

    return dr;
}

Vector3d HairInstance::hairF(int j, Vector3d qip1, Vector3d qi, Vector3d qim1, Vector3d start, Matrix3d norms, SimParameters params)
{
    Vector3d f;
    f.setZero();

    double M = 0.000001;
    double G = params.gravity;
    double h = params.timeStep;
    double k = params.stiffness;

    Vector3d firstTerm;
    Vector3d secondTerm;
    Vector3d thirdTerm;
    Vector3d fourthTerm;

    firstTerm.setZero();
    secondTerm.setZero();
    thirdTerm.setZero();
    fourthTerm.setZero();

    for (int i = 0; i < edgesPerSegment_; i++)
    {
        double s = lengthPerEdge_*(i+1);

        firstTerm += M * ((rsh(s, qip1, start, norms) - rsh(s, qi, start, norms)) / h).transpose() * drsh(s, qi, start, norms);

        fourthTerm += M * ((rsh(s, qi, start, norms) - rsh(s, qim1, start, norms)) / h).transpose() * drsh(s, qi, start, norms);

        thirdTerm += M * G * drsh(s, qi, start, norms).row(1);
    }

    secondTerm = (qi - initialCurvatures_.segment<3>(j * 3)) * k;

    f = fourthTerm - firstTerm - secondTerm - thirdTerm;

    f /= edgesPerSegment_;

    return f;
}

Matrix3d HairInstance::hairdF(int j, Vector3d qip1, Vector3d qi, Vector3d qim1, Vector3d start, Matrix3d norms, SimParameters params)
{
    Matrix3d df;
    df.setZero();

    double M = 0.000001;
    double h = params.timeStep;

    for (int i = 0; i < edgesPerSegment_; i++)
    {
        double s = lengthPerEdge_*(i+1);

        df += (drsh(s, qip1, start, norms) / h).transpose() * drsh(s, qi, start, norms);
    }

    df /= edgesPerSegment_;

    return -df;
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

int HairInstance::getNumberOfDofs()
{
    // maybe do more stuff here later
    return curvatures_.size();
}

void HairInstance::preprocessRendering()
{
    // to be implemented
}

void HairInstance::interpolateTwo(HairInstance* one, HairInstance* two, double alpha, double beta)
{
    HairInstance* oldOne = guideOne;
    HairInstance* oldTwo = guideTwo;
    double oldBaryOne = baryOne;
    double oldBaryTwo = baryTwo;

    guideOne = one;
    guideTwo = two;
    baryOne = alpha;
    baryTwo = beta;

    interpolateTwo();

    guideOne = oldOne;
    guideTwo = oldTwo;
    baryOne = oldBaryOne;
    baryTwo = oldBaryTwo;
}

void HairInstance::interpolateThree(HairInstance* one, HairInstance* two, HairInstance* three, double alpha, double beta, double gamma)
{
    HairInstance* oldOne = guideOne;
    HairInstance* oldTwo = guideTwo;
    HairInstance* oldThree = guideThree;
    double oldBaryOne = baryOne;
    double oldBaryTwo = baryTwo;
    double oldBaryThree = baryThree;

    guideOne = one;
    guideTwo = two;
    guideThree = three;
    baryOne = alpha;
    baryTwo = beta;
    baryThree = gamma;

    interpolateThree();

    guideOne = oldOne;
    guideTwo = oldTwo;
    guideThree = oldThree;
    baryOne = oldBaryOne;
    baryTwo = oldBaryTwo;
    baryThree = oldBaryThree;
}

void HairInstance::interpolateTwo()
{
    assert(guideOne->verts_.rows() == guideTwo->verts_.rows());
    assert((baryOne + baryTwo) == 1.0);

    pos_.setZero();
    pos_ = guideOne->pos_ * baryOne + guideTwo->pos_ * baryTwo;
    color_ = guideOne->color_ * baryOne + guideTwo->color_ * baryTwo;

    curvatures_ = guideOne->curvatures_ * baryOne + guideTwo->curvatures_ * baryTwo;
}

void HairInstance::interpolateThree()
{
    assert(guideOne->verts_.rows() == guideTwo->verts_.rows() && guideOne->verts_.rows() == guideThree->verts_.rows());
    assert((baryOne + baryTwo + baryThree) == 1.0);

    pos_ = guideOne->pos_ * baryOne + guideTwo->pos_ * baryTwo + guideThree->pos_ * baryTwo;
    color_ = guideOne->color_ * baryOne + guideTwo->color_ * baryTwo + guideThree->color_ * baryThree;

    curvatures_ = guideOne->curvatures_ * baryOne + guideTwo->curvatures_ * baryTwo + guideThree->curvatures_ * baryThree;
}
