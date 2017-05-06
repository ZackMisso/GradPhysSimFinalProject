#ifndef SIMPARAMETERS_H
#define SIMPARAMETERS_H

struct SimParameters
{
    SimParameters();

    // enum ConstraintHandler {CH_PENALTY_FORCE, CH_STEP_AND_PROJECT, CH_LAGRANGE_MULTIPLIER};
    enum ClickMode {CM_ADDPARTICLE, CM_ADDSAW};
    // enum ConnectorType {CT_SPRING, CT_RIGID_ROD, CT_FLEXIBLE_ROD};

    // const static int F_GRAVITY = 1;
    // const static int F_SPRINGS = 2;
    // const static int F_FLOOR   = 4;
    // const static int F_DAMPING = 8;
    // const static int F_BENDING = 16;


    bool simRunning;
    // ConstraintHandler constraintHandler;
    double timeStep;
    double NewtonTolerance;
    int NewtonMaxIters;
    // double penaltyStiffness;

    bool singleStrandExample;
    bool interpolationExample;
    bool bundleExample;
    bool sphereExample;
    bool headExample;

    bool reset;

    double gravity;
    double stiffness;
    double hairLength;
    int segments;
    int subsegments;

    // int activeForces;
    // double gravityG;
    // double springStiffness;
    // double maxSpringStrain;
    // double dampingStiffness;

    ClickMode clickMode;
    // double particleMass;
    // double maxSpringDist;
    // bool particleFixed;
    // double sawRadius;

    // ConnectorType connector;
    // double rodDensity;
    // double rodStretchingStiffness;
    // double rodBendingStiffness;
    // int rodSegments;

    double artificialScale;

    bool glRendering;
    bool glsRendering;
    bool raytracing;

    bool showSegments;
    bool show3D;
};

#endif // SIMPARAMETERS_H
