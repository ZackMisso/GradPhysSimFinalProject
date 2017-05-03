#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include "hairinstance.h"

class SimPrep
{
public:
    static void setupInterpExample(std::vector<HairInstance*>& guideStrands);
    static void setupSingleStrandExample(std::vector<HairInstance*>& guideStrands);
    static void setupSphereExample(std::vector<HairInstance*>& guideStrands);
    static void setupHeadExample(std::vector<HairInstance*>& guideStrands);
};
