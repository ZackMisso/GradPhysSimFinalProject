#include "simulation.h"
#include <QGLWidget>
#include "simparameters.h"
#include "hairinstance.h"
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

    for (vector<HairInstance *>::iterator it = interpHairs_.begin(); it != interpHairs_.end(); ++it)
    {
        delete *it;
    }

    hairs_.clear();
    interpHairs_.clear();

    // initialize the hairs for the test here
    // cout << "Making Hair Strand" << endl;

    // HairInstance* singleStrand = new HairInstance();
    // // cout << "WHHHAAATTT" << endl;
    // hairs_.push_back(singleStrand);

    // SimPrep::setupSingleStrandExample(hairs_);
    SimPrep::setupInterpExample(hairs_);
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
