#pragma once

#include "bakedhair.h"
#include <string>
#include <Eigen/Core>
#include <vector>
#include "simparameters.h"

class HairInstance
{
public:
    HairInstance();
    HairInstance(const Eigen::MatrixX3d &verts);
    HairInstance(const Eigen::VectorXd &curves, Eigen::Vector3d startPos, Eigen::Matrix3d startNorm);
    HairInstance(const Eigen::VectorXd &curves, Eigen::Vector3d startPos, Eigen::Matrix3d startNorm, int eps, int nos, double length);
    HairInstance(HairInstance* one, HairInstance* two, double alpha, double beta);
    HairInstance(HairInstance* one, HairInstance* two, HairInstance* three, double alpha, double beta, double gamma);

    ~HairInstance();

    double getLength() const { return length_; }
    const Eigen::MatrixX3d &getVerts() const { return verts_; }
    const Eigen::MatrixX3d &getSegment(int seg) { return segments_[seg]; };

    void reconstructHair();

    int getNumberOfDofs();
    int getNumberOfSegments() { return numberOfSegments_; }
    int getEdgesPerSegment() { return edgesPerSegment_; }

    // simulation info
    Eigen::Vector3d pos_; // location of first vert
    Eigen::VectorXd initialCurvatures_;
    Eigen::VectorXd curvatures_;
    Eigen::VectorXd prev_curvatures_;
    Eigen::VectorXd curvatures_dot_;
    Eigen::Vector3d color_;

    Eigen::MatrixX3d verts_;
    Eigen::Matrix3d normals_;

    void calculateNewInitialConditions(Eigen::Vector3d q, Eigen::Vector3d start, Eigen::Matrix3d n, Eigen::Vector3d &newStart, Eigen::Matrix3d &newNorms);
    Eigen::Vector3d rsh(double s, Eigen::Vector3d q, Eigen::Vector3d start, Eigen::Matrix3d n);
    Eigen::Matrix3d drsh(double s, Eigen::Vector3d q, Eigen::Vector3d start, Eigen::Matrix3d n);
    Eigen::Vector3d hairF(int j, Eigen::Vector3d qip1, Eigen::Vector3d qi, Eigen::Vector3d qim1, Eigen::Vector3d start, Eigen::Matrix3d norms, SimParameters params);
    Eigen::Matrix3d hairdF(int j, Eigen::Vector3d qip1, Eigen::Vector3d qi, Eigen::Vector3d qim1, Eigen::Vector3d start, Eigen::Matrix3d norms, SimParameters params);

    int getNumberOfDofs() const;
    int getNumberOfSegments() const { return numberOfSegments_; }

    void buildConfiguration(Eigen::VectorXd &q, Eigen::VectorXd &qprev, Eigen::VectorXd &v, int &index);
    void unbuildConfiguration(const Eigen::VectorXd &q, const Eigen::VectorXd &v, int &index);

    void interpolateTwo(HairInstance* one, HairInstance* two, double alpha, double beta);
    void interpolateThree(HairInstance* one, HairInstance* two, HairInstance* three, double alpha, double beta, double gamma);
    void interpolateTwo();
    void interpolateThree();
    void bake();

private:
    HairInstance(const HairInstance &other);
    HairInstance &operator=(const HairInstance);

    void initializeLine(int eps, int nos);
    void initializeFromPositions(const Eigen::MatrixX3d pos, int eps, int nos);
    void initializeFromCurvatures(const Eigen::VectorXd curves, int eps, int nos, double length, Eigen::Vector3d startPos, Eigen::Matrix3d startNorm);

    void preprocessRendering(); // maybe implement later

    Eigen::Vector3d para(Eigen::Vector3d vec, Eigen::Vector3d omega);
    Eigen::Vector3d perp(Eigen::Vector3d vec, Eigen::Vector3d omega);

    Eigen::Vector3d calculateNi(Eigen::Vector3d n0, Eigen::Vector3d n1, Eigen::Vector3d n2, Eigen::Vector3d omega, int segment, double s, double darbouxNorm, int i);

    // render info
    BakedHair* bakedVersion;
    HairInstance* guideOne;
    HairInstance* guideTwo;
    HairInstance* guideThree;
    double baryOne;
    double baryTwo;
    double baryThree;

    // template info
    Eigen::MatrixX3d template_verts_;
    std::vector<Eigen::MatrixX3d> segments_;
    std::vector<double> segmentLengths_;
    double length_;
    double lengthPerSegment_;
    double lengthPerEdge_;
    int edgesPerSegment_;
    int numberOfSegments_;
};
