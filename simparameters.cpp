#include "simparameters.h"

SimParameters::SimParameters()
{
    simRunning = false;
    timeStep = 0.001;
    NewtonMaxIters = 20;
    NewtonTolerance = 1e-8;

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

    artificialScale = 1.0;

    glRendering = true;
    glsRendering = false;
    raytracing = false;

    showSegments = false;
    show3D = false;
}
