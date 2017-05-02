#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <string>

class BakedHair
{
public:
    BakedHair(int val);
    BakedHair(int val, Eigen::Vector3d pos, Eigen::Matrix3d norms);
    ~BakedHair();

    int size;
    Eigen::Vector3d initialPosition;
    Eigen::Matrix3d initialNormals;
    std::vector<Eigen::VectorXd> curves;

    void readPositionsFromLine(std::string line);
    str::string writePositionsToLine();
};
