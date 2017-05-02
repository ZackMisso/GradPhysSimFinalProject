#pragma once

#include "bakedhair.h"
#include <string>
#include <Eigen/Core>
#include <vector>

class HairInstance
{
public:
    HairInstance();
    HairInstance(const Eigen::MatrixX3d &verts);
    HairInstance(const Eigen::VectorXd &curves, Eigen::Vector3d startPos, Eigen::Matrix3d startNorm);
    HairInstance(const Eigen::VectorXd &curves, Eigen::Vector3d startPos, Eigen::Matrix3d startNorm, int eps, int nos, double length);

    ~HairInstance();

    double getLength() const { return length_; }
    const Eigen::MatrixX3d &getVerts() const { return verts_; }
    const Eigen::MatrixX3d &getSegment(int seg) { return segments_[seg]; };

    void render2D(double scale);
    void render3D(double scale, double radius);

    void reconstructHair();

    int getNumberOfDofs();
    int getNumberOfSegments() { return numberOfSegments_; }

    void interpolateTwo(const HairInstance* one, const HairInstance* two, double alpha, double beta);
    void interpolateThree(const HairInstance* one, const HairInstance* two, const HairInstance* three, double alpha, double beta, double gamma);
    void interpolateTwo();
    void interpolateThree();
    void bake();

    // simulation info
    Eigen::Vector3d pos_; // location of first vert
    Eigen::VectorXd curvatures_;
    Eigen::VectorXd prev_curvatures_;
    Eigen::VectorXd curvatures_dot_;
    // use the bottom two for rendering later
    Eigen::MatrixX3d verts_;
    // Eigen::MatrixX3d verts_dot;

private:
    HairInstance(const HairInstance &other);
    HairInstance &operator=(const HairInstance);

    void initializeLine(int eps, int nos);
    void initializeFromPositions(const Eigen::MatrixX3d pos, int eps, int nos);
    void initializeFromCurvatures(const Eigen::VectorXd curves, int eps, int nos, double length, Eigen::Vector3d startPos, Eigen::Matrix3d startNorm);

    void computeLength();
    void computeSegments();
    void computeNormals();
    void preprocessRendering();

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
    Eigen::Vector3d color_;
    int index_; // use in milestone 2

    // template info
    Eigen::MatrixX3d template_verts_;
    Eigen::VectorXd initialCurvatures_;
    Eigen::Matrix3d normals_;
    std::vector<Eigen::MatrixX3d> segments_;
    std::vector<double> segmentLengths_;
    double length_;
    double lengthPerSegment_;
    double lengthPerEdge_;
    int edgesPerSegment_;
    int numberOfSegments_;
};
