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
    // cout << "In Render" << endl;
    // glLineWidth(2.0);
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
        renderLock_.unlock();
    }
}

void Simulation::takeSimulationStep()
{
    VectorXd q, qprev, v;

    // cout << "Before build" << endl;
    buildConfiguration(q, qprev, v);
    // cout << "Setting Vals" << endl;

    q[0] = 4 * cos(time_ / 2.0 + 0.5);
    q[1] = 2 * sin(time_) - 3 * cos(time_);
    q[2] = sin(time_ + 3.14 / 6);

    q[3] = 3 * cos(time_) + 3 * cos(time_ / 3.14);
    q[4] = -3 * cos(time_ / 2.0 + 0.5);;
    q[5] = 5 - 3 * sin(time_);

    // numericalIntegration(q, qprev, v);
    // cout << "Unbuild" << endl;
    unbuildConfiguration(q, v);
    // cout << "Reconstruct" << endl;
    reconstruction();

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

    // THIS STUFF IS JUST COPIED AND PASTED. NOT ACTUALLY IN USE
    for (int i = 0; i < params_.NewtonMaxIters; i++)
    {
        VectorXd fval = guess - q; // terms go here

        if (fval.norm() < params_.NewtonTolerance)
        {
            SparseMatrix<double> I(q.size(), q.size());
            I.setIdentity();
            //SparseMatrix<double> Hf = I + params_.timeStep*params_.timeStep*Minv*H;
            SparseMatrix<double> Hf = I + params_.timeStep * H; // terms go here
            BiCGSTAB<SparseMatrix<double> > solver;
            solver.compute(Hf);
            VectorXd deltaguess = solver.solve(-fval);
            guess += deltaguess;
        }
    }

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
    cout << "X: " << x << " Y: " << y << endl;
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

    // cout << "NDOFS" << 

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

    // cout << q << endl;

    for (int i = 0; i < ndofs / 3; i++)
    {
        for (int j = 0; j < hairs_[i]->getNumberOfSegments(); j++)
        {
            // cout << "SETTING" << endl;
            // cout << "SEGMENTS:L " << hairs_[i]->getNumberOfSegments() << endl;
            hairs_[i]->prev_curvatures_(j, 0) = hairs_[j]->curvatures_(j, 0);
            hairs_[i]->prev_curvatures_(j, 1) = hairs_[j]->curvatures_(j, 1);
            hairs_[i]->prev_curvatures_(j, 2) = hairs_[j]->curvatures_(j, 2);

            hairs_[i]->curvatures_(j, 0) = q(j * 3);
            hairs_[i]->curvatures_(j, 1) = q(j * 3 + 1);
            hairs_[i]->curvatures_(j, 2) = q(j * 3 + 2);

            hairs_[i]->curvatures_dot_(j, 0) = v(j * 3);
            hairs_[i]->curvatures_dot_(j, 1) = v(j * 3 + 1);
            hairs_[i]->curvatures_dot_(j, 1) = v(j * 3 + 2);

            // i++;
        }
    }
}

void Simulation::computeForceAndHessian(const VectorXd &q, const VectorXd &qprev, Eigen::VectorXd &F, SparseMatrix<double> &H)
{

    F.resize(q.size());
    F.setZero();
    H.resize(q.size(), q.size());
    H.setZero();

    vector<Tr> Hcoeffs;

    if(params_.activeForces & SimParameters::F_GRAVITY)
    {
        // remap this functionality
    }
    if(params_.activeForces & SimParameters::F_SPRINGS)
    {
        // remap this functionality
    }
    if(params_.activeForces & SimParameters::F_DAMPING)
    {
        // remap this functionality
    }
    if(params_.activeForces & SimParameters::F_FLOOR)
    {
        // remap this functionality
    }
    if(params_.activeForces & SimParameters::F_BENDING)
    {
        // remap this functionality
    }

    H.setFromTriplets(Hcoeffs.begin(), Hcoeffs.end());
}

void Simulation::computeMassInverse(Eigen::SparseMatrix<double> &Minv)
{
    // to be implemented, and renamed
}
