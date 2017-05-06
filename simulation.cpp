#include "simulation.h"
#include <QGLWidget>
#include "simparameters.h"
#include "hairinstance.h"
#include "glrenderer.h"
#include "glsrenderer.h"
#include "raytracer.h"
#include "simprep.h"
#include <iostream>
#include <Eigen/Dense>

const double PI = 3.1415926535898;

using namespace Eigen;
using namespace std;

Simulation::Simulation(const SimParameters &params) : params_(params), time_(0)
{
    clearScene();
}

void Simulation::render(bool is3D)
{
    if (renderLock_.tryLock())
    {
        renderer->render(params_, hairs_, interpHairs_, bodies_);
        renderLock_.unlock();
    }
}

void Simulation::takeSimulationStep()
{
    VectorXd q, qprev, v;

    buildConfiguration(q, qprev, v);

    numericalIntegration(q, qprev, v);

    unbuildConfiguration(q, v);
    reconstruction();

    cleanInterpolations();
    createInterpolations();

    time_ += params_.timeStep;
}

void Simulation::numericalIntegration(Eigen::VectorXd &q, Eigen::VectorXd &qprev, Eigen::VectorXd &v)
{
    VectorXd guessFull = q;

    int total = 0;

    for (int i = 0; i < hairs_.size(); i++)
    {
        // numericalIntegration on each individual curvature
        Matrix3d norms = hairs_[i]->normals_;
        Vector3d start = hairs_[i]->pos_;

        for (int j = 0; j < hairs_[i]->getNumberOfSegments(); j++)
        {
            Vector3d qi = hairs_[i]->curvatures_.segment<3>(j * 3);
            Vector3d qim1 = hairs_[i]->prev_curvatures_.segment<3>(j * 3);
            Vector3d qip1 = qi;

            BiCGSTAB<Matrix3d > solver;
            Vector3d dq;
            dq.setZero();
            Matrix3d B;
            B.setZero();

            int iterations = 0;

            Vector3d f = hairs_[i]->hairF(j, qip1, qi, qim1, start, norms, params_);

            while (f.norm() > params_.NewtonTolerance && iterations < params_.NewtonMaxIters)
            {
                B = hairs_[i]->hairdF(j, qip1, qi, qim1, start, norms, params_);
                B(0, 0) += 1.0;
                B(1, 1) += 1.0;
                B(2, 2) += 1.0;

                solver.compute(B);
                dq = solver.solve(f);
                // cout << "dq in newton " << dq.norm() << endl;
                qip1 = qip1 - dq;

                f = hairs_[i]->hairF(j, qip1, qi, qim1, start, norms, params_);

                iterations++;
            }

            hairs_[i]->calculateNewInitialConditions(qi, start, norms, start, norms);

            guessFull.segment<3>(total) = qip1;

            total += 3;
        }
    }

    q = guessFull;
}

void Simulation::reconstruction()
{
    for (int i = 0; i < hairs_.size(); i++)
    {
        hairs_[i]->reconstructHair();
    }
}

void Simulation::addParticle(double x, double y)
{
    renderLock_.lock();
    // blah
    renderLock_.unlock();
}

void Simulation::addSaw(double x, double y)
{
    renderLock_.lock();
    // blah
    renderLock_.unlock();
}

void Simulation::clearScene()
{
    renderLock_.lock();

    for (vector<HairInstance *>::iterator it = hairs_.begin(); it != hairs_.end(); ++it)
    {
        delete *it;
    }

    bodies_.clear();

    renderer = new GLRenderer();

    for (vector<HairInstance *>::iterator it = interpHairs_.begin(); it != interpHairs_.end(); ++it)
    {
        delete *it;
    }

    hairs_.clear();
    interpHairs_.clear();

    if (params_.singleStrandExample)
    {
        SimPrep::setupSingleStrandExample(hairs_);
    }
    if (params_.interpolationExample)
    {
        SimPrep::setupInterpExample(hairs_);
    }
    if (params_.bundleExample)
    {
        SimPrep::setupBundleExample(hairs_);
    }
    if (params_.sphereExample)
    {
        SimPrep::setupSphereExample(hairs_);
    }
    if (params_.headExample)
    {
        SimPrep::setupHeadExample(hairs_);
    }
    createInterpolations();

    renderLock_.unlock();
}

void Simulation::buildConfiguration(VectorXd &q, VectorXd &qprev, VectorXd &v)
{
    int ndofs = 0;

    for (int i = 0; i < hairs_.size(); i++)
    {
        ndofs += hairs_[i]->getNumberOfDofs();
    }

    q.resize(ndofs);
    qprev.resize(ndofs);
    v.resize(ndofs);

    int currentIndex = 0;

    for (int i = 0; i < hairs_.size(); i++)
    {
        hairs_[i]->buildConfiguration(q, qprev, v, currentIndex);
    }
}

void Simulation::unbuildConfiguration(const VectorXd &q, const VectorXd &v)
{
    int ndofs = q.size();

    int currentIndex = 0;

    for (int i = 0; i < hairs_.size(); i++)
    {
        hairs_[i]->unbuildConfiguration(q, v, currentIndex);
    }
}

void Simulation::cleanInterpolations()
{
    for (vector<HairInstance *>::iterator it = interpHairs_.begin(); it != interpHairs_.end(); ++it)
    {
        delete *it;
    }

    interpHairs_.clear();
}

void Simulation::createInterpolations()
{
    if (hairs_.size() == 2)
    {
        for (double i = 0.05; i < 1.0; i += 0.05)
        {
            interpHairs_.push_back(new HairInstance(hairs_[0], hairs_[1], i, 1.0 - i));
        }
    }
}

void Simulation::bakeHairs()
{
    // to be implemented
}
