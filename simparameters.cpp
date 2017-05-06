#include "simparameters.h"

SimParameters::SimParameters()
{
    simRunning = false;
    // constraintHandler = CH_PENALTY_FORCE;
    timeStep = 0.001;
    NewtonMaxIters = 20;
    NewtonTolerance = 1e-8;
    // penaltyStiffness = 1e5;

    // activeForces = F_GRAVITY | F_SPRINGS | F_FLOOR | F_DAMPING | F_BENDING;
    // gravityG = -9.8;
    // springStiffness = 100;
    // maxSpringStrain = 0.2;
    // dampingStiffness = 1.0;

    clickMode = CM_ADDPARTICLE;

    singleStrandExample = true;
    interpolationExample = false;
    bundleExample = false;
    sphereExample = false;
    headExample = false;

    reset = false;

    gravity = 9.81;
    stiffness = 0.1;
    hairLength = 1.0;
    segments = 2;
    subsegments = 100;

    // particleMass = 1.0;
    // maxSpringDist = 0.25;
    // particleFixed = false;

    // sawRadius= 0.1;

    // connector = CT_SPRING;
    // rodDensity = 2.0;
    // rodStretchingStiffness = 100.0;
    // rodBendingStiffness = 0.05;
    // rodSegments = 5;

    artificialScale = 1.0;
}
