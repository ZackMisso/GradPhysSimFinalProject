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
        cout << "RENDER" << endl;
        renderer->render(params_, hairs_, interpHairs_, bodies_);
        // for (int i = 0; i < hairs_.size(); i++)
        // {
        //     if (is3D)
        //     {
        //         // replace with actual radius later
        //         hairs_[i]->render3D(params_.artificialScale, 1.0);
        //     }
        //     else
        //     {
        //         hairs_[i]->render2D(params_.artificialScale);
        //     }
        // }
        // // cout << "RENDERING" << endl;
        // for (int i = 0; i < interpHairs_.size(); i++)
        // {
        //     // cout << "RENDERING INTERP" << endl;
        //     if (is3D)
        //     {
        //         // replace with actual radius later
        //         interpHairs_[i]->render3D(params_.artificialScale, 1.0);
        //     }
        //     else
        //     {
        //         interpHairs_[i]->render2D(params_.artificialScale);
        //     }
        // }
        renderLock_.unlock();
    }
}

void Simulation::takeSimulationStep()
{
    VectorXd q, qprev, v;

    buildConfiguration(q, qprev, v);

    // q[0] = 4 * cos(time_ / 2.0 + 0.5);
    // q[1] = 2 * sin(time_) - 3 * cos(time_);
    // q[2] = sin(time_ + 3.14 / 6);

    // q[3] = 3 * cos(time_) + 3 * cos(time_ / 3.14);
    // q[4] = -3 * cos(time_ / 2.0 + 0.5);;
    // q[5] = 5 - 3 * sin(time_);

    numericalIntegration(q, qprev, v);

    // cout << "HAIR " << hairs_[0]->verts_.row(10);

    // renderLock_.lock();
    // {
        unbuildConfiguration(q, v);
        reconstruction();

        cleanInterpolations();
        createInterpolations();
    // }
    // renderLock_.unlock();
    // render(true);

    time_ += params_.timeStep;
}

void Simulation::numericalIntegration(Eigen::VectorXd &q, Eigen::VectorXd &qprev, Eigen::VectorXd &v)
{
    VectorXd F;
    SparseMatrix<double> H;
    SparseMatrix<double> Minv;

    computeMassInverse(Minv); // change for hair
    // cout << "START" << endl;

    // so we can explicitly constrain length during reconstruction
    // so all we have to do is calculate the change in curvature then reconstruct

    computeForceAndHessian(q, qprev, F, H);

    VectorXd guessFull = q;
    // cout << "STARTING numericalIntegration" << endl;

    int total = 0;

    // cout << "HAIRS:" << endl;
    // cout << hairs_[0]->curvatures_ << endl;

    for (int i = 0; i < hairs_.size(); i++)
    // for (int i = 0; i < 1; i++)
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

            // cout << "FNORM: " << f.norm() << endl;
            // cout << "F:" << endl;
            // cout << f << endl;
            // cout << "NEWMAX " << params_.NewtonMaxIters << endl;

            while (f.norm() > params_.NewtonTolerance && iterations < params_.NewtonMaxIters)
            {
                // cout << "Setting B" << endl;
                // if (iterations == 2)
                // {
                //     exit(1);
                // }
                B = hairs_[i]->hairdF(j, qip1, qi, qim1, start, norms, params_);
                B(0, 0) += 1.0;
                B(1, 1) += 1.0;
                B(2, 2) += 1.0;
                // cout << "QI:" << endl;
                // cout << qi << endl;
                // cout << "QIP1:" << endl;
                // cout << qip1 << endl;
                // cout << "F:" << endl;
                // cout << f << endl;
                // cout << "dF:" << endl;
                // cout << B << endl;
                // cout << "Before Compute Solver" << endl;
                solver.compute(B);
                dq = solver.solve(f);
                cout << "dq in newton " << dq.norm() << endl;
                qip1 = qip1 - dq;

                f = hairs_[i]->hairF(j, qip1, qi, qim1, start, norms, params_);

                iterations++;
            }

            // Vector3d n0 = norms.row(0);
            // Vector3d n1 = norms.row(1);
            // Vector3d n2 = norms.row(2);

            // norms.row(0) = calculateNi(n0, n1, n2, omega, i, lengthPerSegment_, darbouxNorm, 0);
            // norms.row(1) = calculateNi(n0, n1, n2, omega, i, lengthPerSegment_, darbouxNorm, 1);
            // nroms.row(2) = calculateNi(n0, n1, n2, omega, i, lengthPerSegment_, darbouxNorm, 2);

            hairs_[i]->calculateNewInitialConditions(qi, start, norms, start, norms);

            // exit(1);

            guessFull.segment<3>(total) = qip1;

            total += 3;
        }
    }

    // cout << "STOP" << endl;

    cout << "FINSIEDH" << endl;

    VectorXd guess = q;

    // stuff could go here

    v = (guess - q) / params_.timeStep;
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

    // initialize the hairs for the test here
    // cout << "Making Hair Strand" << endl;

    // HairInstance* singleStrand = new HairInstance();
    // // cout << "WHHHAAATTT" << endl;
    // hairs_.push_back(singleStrand);

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
    // cout << "HAIRSIZE: " << hairs_.size() << endl;
    if (hairs_.size() == 2)
    {
        for (double i = 0.05; i < 1.0; i += 0.05)
        {
            // cout << "Before CREATE" << endl;
            interpHairs_.push_back(new HairInstance(hairs_[0], hairs_[1], i, 1.0 - i));
            // cout << "AFTER CREATE" << endl;
        }
    }
}

void Simulation::bakeHairs()
{
    // to be implemented
}
