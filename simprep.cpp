#include "simprep.h"

using namespace Eigen;
using namespace std;

void SimPrep::setupInterpExample(vector<HairInstance*>& guideStrands)
{
    int eps = 100;
    int nos = 2;
    double length = 1.0;
    Vector3d startPos = Vector3d(-0.5, 0.6, 0.0);
    Matrix3d normals;
    normals.row(0) = Vector3d(1.0, 0.0, 0.0);
    normals.row(1) = Vector3d(0.0, 1.0, 0.0);
    normals.row(2) = Vector3d(0.0, 0.0, 1.0);
    VectorXd curves;
    curves.resize(6);
    curves.segment<3>(0) = Vector3d(0.0, 0.0, 1.0);
    curves.segment<3>(3) = Vector3d(0.0, 0.0, -2.0);

    HairInstance* one = new HairInstance(curves, startPos, normals, eps, nos, length);
    one->initialCurvatures_.resize(6);
    one->initialCurvatures_.segment<3>(0) = Vector3d(0.0, 0.0, -0.5);
    one->initialCurvatures_.segment<3>(3) = Vector3d(0.0, 0.0, -1.0);

    guideStrands.push_back(one);

    int eps2 = 100;
    int nos2 = 2;
    double length2 = 1.0;
    Vector3d startPos2 = Vector3d(-0.5, -0.0, 0.0);
    Matrix3d normals2;
    normals2.row(0) = Vector3d(1.0, 0.0, 0.0);
    normals2.row(1) = Vector3d(0.0, 1.0, 0.0);
    normals2.row(2) = Vector3d(0.0, 0.0, 1.0);
    VectorXd curves2;
    curves2.resize(6);
    curves2.segment<3>(0) = Vector3d(0.0, 0.0, -1.0);
    curves2.segment<3>(3) = Vector3d(0.0, 0.0, 1.0);

    HairInstance* two = new HairInstance(curves2, startPos2, normals2, eps2, nos2, length2);
    two->initialCurvatures_.resize(6);
    two->initialCurvatures_.segment<3>(0) = Vector3d(0.0, 0.0, -2.0);
    two->initialCurvatures_.segment<3>(3) = Vector3d(0.0, 0.0, 1.0);

    guideStrands.push_back(two);
}

void SimPrep::setupSingleStrandExample(vector<HairInstance*>& guideStrands)
{
    int eps = 100;
    int nos = 2;
    double length = 1.0;
    Vector3d startPos = Vector3d(-0.5, 0.4, 0.0);
    Matrix3d normals;
    normals.row(0) = Vector3d(1.0, 0.0, 0.0);
    normals.row(1) = Vector3d(0.0, 1.0, 0.0);
    normals.row(2) = Vector3d(0.0, 0.0, 1.0);
    VectorXd curves;
    curves.resize(6);
    curves.segment<3>(0) = Vector3d(0.0, 0.0, 1.0);
    curves.segment<3>(3) = Vector3d(0.0, 0.0, -2.0);

    HairInstance* one = new HairInstance(curves, startPos, normals, eps, nos, length);
    one->initialCurvatures_.resize(6);
    one->initialCurvatures_.segment<3>(0) = Vector3d(0.0, 0.0, -3.0);
    one->initialCurvatures_.segment<3>(3) = Vector3d(0.0, 0.0, 1.0);

    guideStrands.push_back(one);
}

void SimPrep::setupBundleExample(std::vector<HairInstance*>& guideStrands)
{
    int eps = 100;
    int nos = 2;
    double length = 1.0;
    Vector3d startPos = Vector3d(-0.5, 0.3, 0.0);
    Matrix3d normals;
    normals.row(0) = Vector3d(1.0, 0.0, 0.0);
    normals.row(1) = Vector3d(0.0, 1.0, 0.0);
    normals.row(2) = Vector3d(0.0, 0.0, 1.0);
    VectorXd curves;
    curves.resize(6);
    curves.segment<3>(0) = Vector3d(0.0, 0.0, 1.0);
    curves.segment<3>(3) = Vector3d(0.0, 0.0, -2.0);

    HairInstance* one = new HairInstance(curves, startPos, normals, eps, nos, length);
    one->initialCurvatures_.resize(6);
    one->initialCurvatures_.segment<3>(0) = Vector3d(0.0, 0.0, -0.5);
    one->initialCurvatures_.segment<3>(3) = Vector3d(0.0, 0.0, -1.0);

    guideStrands.push_back(one);

    int eps2 = 100;
    int nos2 = 2;
    double length2 = 1.0;
    Vector3d startPos2 = Vector3d(-0.5, 0.3, 0.0);
    Matrix3d normals2;
    normals2.row(0) = Vector3d(1.0, 0.0, 0.0);
    normals2.row(1) = Vector3d(0.0, 1.0, 0.0);
    normals2.row(2) = Vector3d(0.0, 0.0, 1.0);
    VectorXd curves2;
    curves2.resize(6);
    curves2.segment<3>(0) = Vector3d(0.0, 0.0, -1.0);
    curves2.segment<3>(3) = Vector3d(0.0, 0.0, -3.0);

    HairInstance* two = new HairInstance(curves2, startPos2, normals2, eps2, nos2, length2);
    two->initialCurvatures_.resize(6);
    two->initialCurvatures_.segment<3>(0) = Vector3d(0.0, 0.0, -2.0);
    two->initialCurvatures_.segment<3>(3) = Vector3d(0.0, 0.0, -4.0);

    guideStrands.push_back(two);
}

void SimPrep::setupSphereExample(vector<HairInstance*>& guideStrands)
{
    // to be implemented
}

void SimPrep::setupHeadExample(vector<HairInstance*>& guideStrands)
{
    // to be implemented
}
