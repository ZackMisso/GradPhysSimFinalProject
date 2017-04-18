#ifndef SIMPARAMETERS_H
#define SIMPARAMETERS_H

struct SimParameters
{
    SimParameters();

    enum ConstraintHandler {CH_PENALTY_FORCE, CH_STEP_AND_PROJECT, CH_LAGRANGE_MULTIPLIER};
    enum ClickMode {CM_ADDPARTICLE, CM_ADDSAW};
    enum ConnectorType {CT_SPRING, CT_RIGID_ROD, CT_FLEXIBLE_ROD};

    const static int F_GRAVITY = 1;
    const static int F_SPRINGS = 2;
    const static int F_FLOOR   = 4;
    const static int F_DAMPING = 8;
    const static int F_BENDING = 16;


    bool simRunning;
    ConstraintHandler constraintHandler;
    double timeStep;
    double NewtonTolerance;
    int NewtonMaxIters;
    double penaltyStiffness;

    int activeForces;
    double gravityG;
    double springStiffness;
    double maxSpringStrain;
    double dampingStiffness;

    ClickMode clickMode;
    double particleMass;
    double maxSpringDist;
    bool particleFixed;
    double sawRadius;

    ConnectorType connector;
    double rodDensity;
    double rodStretchingStiffness;
    double rodBendingStiffness;
    int rodSegments;
};

#endif // SIMPARAMETERS_H
