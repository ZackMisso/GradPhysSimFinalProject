#include "simulation.h"
#include <QGLWidget>
#include "simparameters.h"
#include "hairinstance.h"
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
        for (int i = 0; i < hairs_.size(); i++)
        {
            if (is3D)
            {
                // replace with actual radius later
                hairs_[i]->render3D(params_.artificialScale, 1.0);
            }
            else
            {
                hairs_[i]->render2D(params_.artificialScale);
            }
        }
        for (int i = 0; i < interpHairs_.size(); i++)
        {
            if (is3D)
            {
                // replace with actual radius later
                interpHairs_[i]->render3D(params_.artificialScale, 1.0);
            }
            else
            {
                interpHairs_[i]->render2D(params_.artificialScale);
            }
        }
        renderLock_.unlock();
    }
}

void Simulation::takeSimulationStep()
{
    VectorXd q, qprev, v;

    buildConfiguration(q, qprev, v);

    q[0] = 4 * cos(time_ / 2.0 + 0.5);
    q[1] = 2 * sin(time_) - 3 * cos(time_);
    q[2] = sin(time_ + 3.14 / 6);

    q[3] = 3 * cos(time_) + 3 * cos(time_ / 3.14);
    q[4] = -3 * cos(time_ / 2.0 + 0.5);;
    q[5] = 5 - 3 * sin(time_);

    // numericalIntegration(q, qprev, v);
    unbuildConfiguration(q, v);
    reconstruction();

    cleanInterpolations();
    createInterpolations();

    time_ += params_.timeStep;
}

void Simulation::numericalIntegration(Eigen::VectorXd &q, Eigen::VectorXd &qprev, Eigen::VectorXd &v)
{
    VectorXd F;
    SparseMatrix<double> H;
    SparseMatrix<double> Minv;

    computeMassInverse(Minv); // change for hair

    // so we can explicitly constrain length during reconstruction
    // so all we have to do is calculate the change in curvature then reconstruct

    computeForceAndHessian(q, qprev, F, H);

    // for (int i = 0; i < hairs_.size(); i++)
    // {
    //     // numericalIntegration on each individual curvature
    // }

    VectorXd guess = q;

    // stuff could go here

    v = (guess - q) / params_.timeStep;
    q = guess;
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

    hairs_.clear();

    // initialize the hairs for the test here
    HairInstance* singleStrand = new HairInstance();
    hairs_.push_back(singleStrand);

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

    for (int i = 0; i < hairs_.size(); i++)
    {
        q.segment<3>(3*i) = Vector3d(hairs_[i]->curvatures_(i, 0), hairs_[i]->curvatures_(i, 1), hairs_[i]->curvatures_(i, 2));
        qprev.segment<3>(3*i) = Vector3d(hairs_[i]->prev_curvatures_(i, 0), hairs_[i]->prev_curvatures_(i, 1), hairs_[i]->prev_curvatures_(i, 2));
        v.segment<3>(3*i) = Vector3d(hairs_[i]->curvatures_dot_(i, 0), hairs_[i]->curvatures_dot_(i, 1), hairs_[i]->curvatures_dot_(i, 2));
    }
}

void Simulation::unbuildConfiguration(const VectorXd &q, const VectorXd &v)
{
    int ndofs = q.size();

    for (int j = 0; j < hairs_[0]->getNumberOfSegments(); j++)
    {
        // FIX LATER
        hairs_[0]->prev_curvatures_(j, 0) = hairs_[0]->curvatures_(j, 0);
        hairs_[0]->prev_curvatures_(j, 1) = hairs_[0]->curvatures_(j, 1);
        hairs_[0]->prev_curvatures_(j, 2) = hairs_[0]->curvatures_(j, 2);

        hairs_[0]->curvatures_(j, 0) = q(j * 3);
        hairs_[0]->curvatures_(j, 1) = q(j * 3 + 1);
        hairs_[0]->curvatures_(j, 2) = q(j * 3 + 2);

        hairs_[0]->curvatures_dot_(j, 0) = v(j * 3);
        hairs_[0]->curvatures_dot_(j, 1) = v(j * 3 + 1);
        hairs_[0]->curvatures_dot_(j, 1) = v(j * 3 + 2);
    }
}

void Simulation::computeForceAndHessian(const VectorXd &q, const VectorXd &qprev, Eigen::VectorXd &F, SparseMatrix<double> &H)
{

    F.resize(q.size());
    F.setZero();
    H.resize(q.size(), q.size());
    H.setZero();

    vector<Tr> Hcoeffs;

    // do stuffs

    H.setFromTriplets(Hcoeffs.begin(), Hcoeffs.end());
}

void Simulation::computeMassInverse(Eigen::SparseMatrix<double> &Minv)
{
    // to be implemented, and renamed
}

void Simulation::cleanInterpolations()
{
    // to be implemented
}

void Simulation::createInterpolations()
{
    // to be implemented
}

void Simulation::bakeHairs()
{
    // to be implemented
}
