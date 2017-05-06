#ifndef SIMPARAMETERS_H
#define SIMPARAMETERS_H

struct SimParameters
{
    SimParameters();

    enum ClickMode {CM_ADDPARTICLE, CM_ADDSAW};

    bool simRunning;
    double timeStep;
    double NewtonTolerance;
    int NewtonMaxIters;

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

    ClickMode clickMode;

    double artificialScale;

    bool glRendering;
    bool glsRendering;
    bool raytracing;

    bool showSegments;
    bool show3D;
};

#endif // SIMPARAMETERS_H
