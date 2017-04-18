#pragma once

#include <string>
#include <Eigen/Core>
#include <vector>

class HairInstance
{
public:
    HairInstance();
    HairInstance(const Eigen::MatrixX3d &verts);
    HairInstance(const Eigen::MatrixX3d &verts, int eps, int nos);

    ~HairInstance();

    double getLength() const { return length_; }
    const Eigen::MatrixX3d &getVerts() const { return verts_; }
    const Eigen::MatrixX3d &getSegment(int seg) { return segments_[seg]; };

    void render(double scale);

private:
    HairInstance(const HairInstance &other);
    HairInstance &operator=(const HairInstance);

    void initializeLine(int eps, int nos);
    void initializeFromPositions(const Eigen::MatrixX3d positions, int eps, int nos);
    void initializeFromCurvatures(const Eigen::MatrixX3d curves, int eps, int nos, int length);

    void computeLength();
    void computeSegments();
    void computeNormals();

    // simulation info
    Eigen::MatrixX3d pos; // location of first vert
    Eigen::MatrixX3d cvel;

    // render info
    Eigen::Vector3d color;

    // template info
    Eigen::MatrixX3d verts_;
    Eigen::MatrixX3d curvatures_;
    Eigen::MatrixX3d initialCurvatures_;
    Eigen::MatrixX3d normals_;
    std::vector<Eigen::MatrixX3d> segments_;
    double length_;
    int edgesPerSegment_;
    int numberOfSegments_;
};
