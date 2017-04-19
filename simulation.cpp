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

void Simulation::render()
{
    // glLineWidth(2.0);
    if (renderLock_.tryLock())
    {
        for (int i = 0; i < hairs_.size(); i++)
        {
            hairs_[i]->render(params_.artificialScale);
            // hairs_[i]->render();
        }
        renderLock_.unlock();
    }

    // double baseradius = 0.02;
    // double pulsefactor = 0.1;
    // double pulsespeed = 50.0;
    //
    // int sawteeth = 20;
    // double sawdepth = 0.1;
    // double sawangspeed = 10.0;
    //
    // double baselinewidth = 0.5;
    //
    // int numcirclewedges = 20;
    //
    // if(params_.activeForces & SimParameters::F_FLOOR)
    // {
    //     glBegin(GL_TRIANGLES);
    //     {
    //         glColor3f(0.3, 1.0, 0.3);
    //
    //         glVertex2f(-1, -0.5);
    //         glVertex2f(1, -0.5);
    //         glVertex2f(-1, -1);
    //
    //         glVertex2f(-1, -1);
    //         glVertex2f(1, -0.5);
    //         glVertex2f(1, -1);
    //     }
    //     glEnd();
    // }
    //
    // renderLock_.lock();
    // {
    //     int springIndex = -1;
    //     for(vector<Spring>::iterator it = springs_.begin(); it != springs_.end(); ++it)
    //     {
    //         glColor3f(0.0, 0.0, 1.0);
    //         springIndex++;
    //
    //         for(vector<BendingHinge>::iterator bit = hinges_.begin(); bit != hinges_.end(); ++bit)
    //         {
    //             if (bit->s1 == springIndex || bit->s2==springIndex)
    //             {
    //                 glColor3f(0.7, 0.0, 0.34);
    //             }
    //         }
    //
    //         Vector2d sourcepos = particles_[it->p1].pos;
    //         Vector2d destpos   = particles_[it->p2].pos;
    //
    //         double dist = (sourcepos-destpos).norm();
    //
    //         glLineWidth(baselinewidth/dist);
    //
    //         glBegin(GL_LINES);
    //         glVertex2f(sourcepos[0], sourcepos[1]);
    //         glVertex2f(destpos[0], destpos[1]);
    //         glEnd();
    //     }
    //
    //     for(vector<RigidRod>::iterator it = rigids_.begin(); it != rigids_.end(); ++it)
    //     {
    //         glColor3f(1.0, 0.0, 1.0);
    //         Vector2d sourcepos = particles_[it->p1].pos;
    //         Vector2d destpos   = particles_[it->p2].pos;
    //
    //         double dist = (sourcepos-destpos).norm();
    //
    //         glLineWidth(baselinewidth/dist);
    //
    //         glBegin(GL_LINES);
    //         glVertex2f(sourcepos[0], sourcepos[1]);
    //         glVertex2f(destpos[0], destpos[1]);
    //         glEnd();
    //     }
    //
    //     for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    //     {
    //         double radius = baseradius*sqrt(it->mass);
    //         radius *= (1.0 + pulsefactor*sin(pulsespeed*time_));
    //
    //         glColor3f(0,0,0);
    //
    //         if(it->fixed)
    //         {
    //             radius = baseradius;
    //             glColor3f(1.0,0,0);
    //         }
    //
    //         glBegin(GL_TRIANGLE_FAN);
    //         {
    //             glVertex2f(it->pos[0], it->pos[1]);
    //             for(int i=0; i<=numcirclewedges; i++)
    //             {
    //                 glVertex2f(it->pos[0] + radius * cos(2*PI*i/numcirclewedges),
    //                            it->pos[1] + radius * sin(2*PI*i/numcirclewedges));
    //             }
    //         }
    //         glEnd();
    //     }
    //
    //     for(vector<Saw>::iterator it = saws_.begin(); it != saws_.end(); ++it)
    //     {
    //         double outerradius = it->radius;
    //         double innerradius = (1.0-sawdepth)*outerradius;
    //
    //         glColor3f(0.5,0.5,0.5);
    //
    //         glBegin(GL_TRIANGLE_FAN);
    //         {
    //             glVertex2f(it->pos[0], it->pos[1]);
    //             int spokes = 2*sawteeth;
    //             for(int i=0; i<=spokes; i++)
    //             {
    //                 double radius = (i%2==0) ? innerradius : outerradius;
    //                 glVertex2f(it->pos[0] + radius * cos(2*PI*i/spokes + sawangspeed*time_),
    //                            it->pos[1] + radius * sin(2*PI*i/spokes + sawangspeed*time_));
    //             }
    //         }
    //         glEnd();
    //     }
    // }
    // renderLock_.unlock();
}

void Simulation::takeSimulationStep()
{
    VectorXd q, qprev, v;

    buildConfiguration(q, qprev, v);
    // numericalIntegration(q, qprev, v);
    unbuildConfiguration(q, v);
    reconstruction();

    time_ += params_.timeStep;
}

void Simulation::void numericalIntegration(Eigen::VectorXd &q, Eigen::VectorXd &qprev, Eigen::VectorXd &v)
{
    VectorXd F;
    SparseMatrix<double> H;
    SparseMatrix<double> Minv;

    computeMassInverse(Minv); // change for hair

    // so we can explicitly constrain length during reconstruction
    // so all we have to do is calculate the change in curvature then reconstruct

    computeForceAndHessian(q, oldq, F, H);

    // for (int i = 0; i < hairs_.size(); i++)
    // {
    //     // numericalIntegration on each individual curvature
    // }

    VectorXd guess = q;

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
    // {
    //     Vector2d newpos(x,y);
    //
    //     double mass = params_.particleMass;
    //     if(params_.particleFixed)
    //         mass = std::numeric_limits<double>::infinity();
    //
    //     particles_.push_back(Particle(newpos, mass, params_.particleFixed, false));
    //     int indexOfNew = particles_.size() - 1;
    //
    //     for(int i=0; i<(int)particles_.size(); i++)
    //     {
    //         if (!particles_[i].inert)
    //         {
    //             Vector2d pos = particles_[i].pos;
    //             double dist = (pos-newpos).norm();
    //             if(dist <= params_.maxSpringDist && dist != 0.0)
    //             {
    //                 if (params_.connector == SimParameters::CT_SPRING)
    //                     springs_.push_back(Spring(particles_.size() - 1, i, params_.springStiffness/dist, dist, false, 0.0));
    //                 else if (params_.connector == SimParameters::CT_RIGID_ROD)
    //                     rigids_.push_back(RigidRod(particles_.size() - 1, i, dist));
    //                 else if (params_.connector == SimParameters::CT_FLEXIBLE_ROD)
    //                     createFlexibleRod(newpos, i, indexOfNew, mass);
    //             }
    //         }
    //     }
    // }
    renderLock_.unlock();
}

void Simulation::createFlexibleRod(Vector2d newpos, int indexOfOther, int indexOfNew, double mass) {
    // int rodSegments = params_.rodSegments > 2 ? params_.rodSegments : 2;
    // Vector2d otherPosition = particles_[indexOfOther].pos;
    // Vector2d difference = newpos - otherPosition;
    // Vector2d distToNextSegment = difference / (rodSegments);
    // double restLen = distToNextSegment.norm();
    // Vector2d nextPos = otherPosition;
    //
    // if (rodSegments == 2)
    // {
    //     nextPos += distToNextSegment;
    //     particles_.push_back(Particle(nextPos, 0.0, false, true));
    //     springs_.push_back(Spring(particles_.size()-1, indexOfOther, params_.rodStretchingStiffness / restLen, restLen, true, restLen * params_.rodDensity));
    //     springs_.push_back(Spring(particles_.size()-1, indexOfNew, params_.rodStretchingStiffness / restLen, restLen, true, restLen * params_.rodDensity));
    //     hinges_.push_back(BendingHinge(springs_.size()-2, springs_.size()-1, 2*params_.rodBendingStiffness / (springs_[springs_.size()-2].restlen + springs_[springs_.size()-1].restlen)));
    //     particles_[particles_.size()-1].mass += springs_[springs_.size()-1].mass / 2.0;
    //     particles_[indexOfOther].mass += springs_[springs_.size()-2].mass / 2.0;
    //     particles_[particles_.size()-1].mass += springs_[springs_.size()-2].mass / 2.0;
    //     particles_[indexOfNew].mass += springs_[springs_.size()-1].mass / 2.0;
    // }
    // else
    // {
    //     for (int i = 1; i < rodSegments; i++)
    //     {
    //         nextPos += distToNextSegment;
    //         particles_.push_back(Particle(nextPos, 0.0, false, true));
    //         if (i == 1)
    //         {
    //             springs_.push_back(Spring(particles_.size()-1, indexOfOther, params_.rodStretchingStiffness / restLen, restLen, true, restLen * params_.rodDensity));
    //             // increment mass of particles from spring
    //             particles_[particles_.size()-1].mass += springs_[springs_.size()-1].mass / 2.0;
    //             particles_[indexOfOther].mass += springs_[springs_.size()-1].mass / 2.0;
    //         }
    //         else if (i == rodSegments - 1)
    //         {
    //             springs_.push_back(Spring(particles_.size()-1, particles_.size()-2, params_.rodStretchingStiffness / restLen, restLen, true, restLen * params_.rodDensity));
    //             springs_.push_back(Spring(particles_.size()-1, indexOfNew, params_.rodStretchingStiffness / restLen, restLen, true, restLen * params_.rodDensity));
    //             // increment mass of particles from springs
    //             particles_[particles_.size()-1].mass += springs_[springs_.size()-1].mass / 2.0;
    //             particles_[particles_.size()-1].mass += springs_[springs_.size()-2].mass / 2.0;
    //             particles_[indexOfNew].mass += springs_[springs_.size()-2].mass / 2.0;
    //             particles_[particles_.size()-2].mass += springs_[springs_.size()-1].mass / 2.0;
    //             // create hinge connecting both last inert and previous inert
    //             hinges_.push_back(BendingHinge(springs_.size()-3, springs_.size()-2, 2*params_.rodBendingStiffness / (springs_[springs_.size()-3].restlen + springs_[springs_.size()-2].restlen)));
    //             hinges_.push_back(BendingHinge(springs_.size()-2, springs_.size()-1, 2*params_.rodBendingStiffness / (springs_[springs_.size()-2].restlen + springs_[springs_.size()-1].restlen)));
    //         }
    //         else
    //         {
    //             springs_.push_back(Spring(particles_.size()-1, particles_.size()-2, params_.rodStretchingStiffness / restLen, restLen, true, restLen * params_.rodDensity));
    //             // increment mass of particles from spring
    //             particles_[particles_.size()-2].mass += springs_[springs_.size()-1].mass / 2.0;
    //             particles_[particles_.size()-1].mass += springs_[springs_.size()-1].mass / 2.0;
    //             // create hinge connecting current spring and last spring
    //             hinges_.push_back(BendingHinge(springs_.size()-2, springs_.size()-1, 2*params_.rodBendingStiffness / (springs_[springs_.size()-2].restlen + springs_[springs_.size()-1].restlen)));
    //         }
    //     }
    // }
}

void Simulation::addSaw(double x, double y)
{
    renderLock_.lock();
        // saws_.push_back(Saw(Vector2d(x,y), params_.sawRadius));
    renderLock_.unlock();
}

void Simulation::clearScene()
{
    renderLock_.lock();

    for (vector<HairInstance *>::iterator it = hairs_.begin(); it != hairs_.end(); ++i)
    {
        delete *it;
    }

    hairs_.clear();

    // initialize the hairs for the test here
    HairInstance* singleStrand = new HairInstance();
    hairs_.push_back(singleStrand());

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
        q.segment<3>(3*i) = Vector3d(hairs_[i].getCurvatures()(i, 0), hairs_[i].getCurvatures()(i, 1), hairs_[i].getCurvatures()(i, 2));
        qprev.segment<3>(3*i) = Vector3d(hairs_[i].getPrevCurvatures()(i, 0), hairs_[i].getPrevCurvatures()(i, 1), hairs_[i].getPrevCurvatures()(i, 2));
        v.segment<3>(3*i) = Vector3d(hairs_[i].getCDot()(i, 0), hairs_[i].getCDot()(i, 1), hairs_[i].getCDot()(i, 2));
    }
}

void Simulation::unbuildConfiguration(const VectorXd &q, const VectorXd &v)
{
    int ndofs = q.size();

    for (int i = 0; i < ndofs / 3; i++)
    {
        for (int j = 0; j < hairs_[i].getNumberOfSegments(); j++)
        {
            hairs_[j]->prev_urvatures_(j, 0) = hairs_[j]->curvatures_(j, 0);
            hairs_[j]->prev_curvatures_(j, 1) = hairs_[j]->curvatures_(j, 1);
            hairs_[j]->prev_curvatures_(j, 2) = hairs_[j]->curvatures_(j, 2);

            hairs_[j]->curvatures_(j, 0) = q(i * 3);
            hairs_[j]->curvatures_(j, 1) = q(i * 3 + 1);
            hairs_[j]->curvatures_(j, 2) = q(i * 3 + 2);

            hairs_[j]->cdot_(j, 0) = v(i * 3);
            hairs_[j]->cdot_(j, 1) = v(i * 3 + 1);
            hairs_[j]->cdot_(j, 1) = v(i * 3 + 2);

            i++;
        }
        i--;
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
        processGravityForce(F);
    if(params_.activeForces & SimParameters::F_SPRINGS)
        processSpringForce(q, F, Hcoeffs);
    if(params_.activeForces & SimParameters::F_DAMPING)
        processDampingForce(q, qprev, F, Hcoeffs);
    if(params_.activeForces & SimParameters::F_FLOOR)
        processFloorForce(q, qprev, F, Hcoeffs);
    if(params_.activeForces & SimParameters::F_BENDING)
        processBendingForce(q, F);

    H.setFromTriplets(Hcoeffs.begin(), Hcoeffs.end());
}

void Simulation::processBendingForce(const VectorXd &q, VectorXd &F)
{
    // int nhinges = (int)hinges_.size();
    //
    // for (int i = 0; i < nhinges; i++)
    // {
    //     int s1i = hinges_[i].s1;
    //     int s2i = hinges_[i].s2;
    //     Spring s1 = springs_[s1i];
    //     Spring s2 = springs_[s2i];
    //
    //     Vector2d pi, pj, pk;
    //     int pii = 0;
    //     int pji = 0;
    //     int pki = 0;
    //
    //     if (s1.p1 == s2.p1)
    //     {
    //         pj = q.segment<2>(2*s1.p1);
    //         pi = q.segment<2>(2*s1.p2);
    //         pk = q.segment<2>(2*s2.p2);
    //         pji = s1.p1;
    //         pii = s1.p2;
    //         pki = s2.p2;
    //     }
    //     else if (s1.p1 == s2.p2)
    //     {
    //         pj = q.segment<2>(2*s1.p1);
    //         pi = q.segment<2>(2*s1.p2);
    //         pk = q.segment<2>(2*s2.p1);
    //         pji = s1.p1;
    //         pii = s1.p2;
    //         pki = s2.p1;
    //     }
    //     else if (s1.p2 == s2.p1)
    //     {
    //         pj = q.segment<2>(2*s1.p2);
    //         pi = q.segment<2>(2*s1.p1);
    //         pk = q.segment<2>(2*s2.p2);
    //         pji = s1.p2;
    //         pii = s1.p1;
    //         pki = s2.p2;
    //     }
    //     else if (s1.p2 == s2.p2)
    //     {
    //         pj = q.segment<2>(2*s1.p2);
    //         pi = q.segment<2>(2*s1.p1);
    //         pk = q.segment<2>(2*s2.p1);
    //         pji = s1.p2;
    //         pii = s1.p1;
    //         pki = s2.p1;
    //     }
    //
    //     Vector2d jmi = pj - pi;
    //     Vector2d kmj = pk - pj;
    //
    //     Vector3d jmit = Vector3d(jmi[0], jmi[1], 0.0);
    //     Vector3d kmjt = Vector3d(kmj[0], kmj[1], 0.0);
    //     Vector3d zHat = Vector3d(0.0, 0.0, 1.0);
    //
    //     double atan2ParamOne = (jmit.cross(kmjt)).dot(zHat);
    //     double atan2ParamTwo = jmi.norm() * kmj.norm() + jmi.dot(kmj);
    //
    //     double theta = 2.0 * atan2(atan2ParamOne, atan2ParamTwo);
    //
    //     Vector3d Fit = hinges_[i].stiffness * theta * (jmit.cross(zHat)) / (jmit.squaredNorm());
    //     Vector3d Fkt = hinges_[i].stiffness * theta * (kmjt.cross(zHat)) / (kmjt.squaredNorm());
    //     Vector3d Fjt = -Fit - Fkt;
    //
    //     Vector2d Fi = Vector2d(Fit[0], Fit[1]);
    //     Vector2d Fk = Vector2d(Fkt[0], Fkt[1]);
    //     Vector2d Fj = Vector2d(Fjt[0], Fjt[1]);
    //
    //     F.segment<2>(2*pii) += Fi;
    //     F.segment<2>(2*pki) += Fk;
    //     F.segment<2>(2*pji) += Fj;
    // }
}

void Simulation::processGravityForce(VectorXd &F)
{
    // int nparticles = (int)particles_.size();
    // for(int i=0; i<nparticles; i++)
    // {
    //     if(!particles_[i].fixed)
    //     {
    //         F[2*i+1] += params_.gravityG*particles_[i].mass;
    //     }
    // }
}

void Simulation::processSpringForce(const VectorXd &q, VectorXd &F, std::vector<Tr> &H)
{
    // int nsprings = (int)springs_.size();
    //
    // for(int i=0; i<nsprings; i++)
    // {
    //     Vector2d p1 = q.segment<2>(2*springs_[i].p1);
    //     Vector2d p2 = q.segment<2>(2*springs_[i].p2);
    //     double dist = (p2-p1).norm();
    //     Vector2d localF = springs_[i].stiffness*(dist-springs_[i].restlen)/dist * (p2-p1);
    //     F.segment<2>(2*springs_[i].p1) += localF;
    //     F.segment<2>(2*springs_[i].p2) -= localF;
    //
    //     Matrix2d I;
    //     I << 1, 0, 0, 1;
    //     Matrix2d localH = springs_[i].stiffness * (1.0 - springs_[i].restlen/dist)*I;
    //     localH += springs_[i].stiffness*springs_[i].restlen*(p2-p1)*(p2-p1).transpose()/dist/dist/dist;
    //
    //     for(int j=0; j<2; j++)
    //         for(int k=0; k<2;k++)
    //         {
    //             H.push_back(Tr(2*springs_[i].p1+j, 2*springs_[i].p1+k, localH.coeff(j,k)));
    //             H.push_back(Tr(2*springs_[i].p2+j, 2*springs_[i].p2+k, localH.coeff(j,k)));
    //             H.push_back(Tr(2*springs_[i].p1+j, 2*springs_[i].p2+k, -localH.coeff(j,k)));
    //             H.push_back(Tr(2*springs_[i].p2+j, 2*springs_[i].p1+k, -localH.coeff(j,k)));
    //         }
    // }
}

void Simulation::processDampingForce(const VectorXd &q, const VectorXd &qprev, VectorXd &F, std::vector<Tr> &H)
{
    // int nsprings = (int)springs_.size();
    //
    // for(int i=0; i<nsprings; i++)
    // {
    //     Vector2d p1 = q.segment<2>(2*springs_[i].p1);
    //     Vector2d p2 = q.segment<2>(2*springs_[i].p2);
    //     Vector2d p1prev = qprev.segment<2>(2*springs_[i].p1);
    //     Vector2d p2prev = qprev.segment<2>(2*springs_[i].p2);
    //
    //     Vector2d relvel = (p2 - p2prev)/params_.timeStep - (p1 - p1prev)/params_.timeStep;
    //     Vector2d localF = params_.dampingStiffness*relvel;
    //     F.segment<2>(2*springs_[i].p1) += localF;
    //     F.segment<2>(2*springs_[i].p2) -= localF;
    //
    //     Matrix2d I;
    //     I << 1, 0, 0, 1;
    //     Matrix2d localH = params_.dampingStiffness*I/params_.timeStep;
    //
    //     for(int j=0; j<2; j++)
    //         for(int k=0; k<2;k++)
    //         {
    //             H.push_back(Tr(2*springs_[i].p1+j, 2*springs_[i].p1+k, localH.coeff(j,k)));
    //             H.push_back(Tr(2*springs_[i].p2+j, 2*springs_[i].p2+k, localH.coeff(j,k)));
    //             H.push_back(Tr(2*springs_[i].p1+j, 2*springs_[i].p2+k, -localH.coeff(j,k)));
    //             H.push_back(Tr(2*springs_[i].p2+j, 2*springs_[i].p1+k, -localH.coeff(j,k)));
    //         }
    // }
}

void Simulation::processFloorForce(const VectorXd &q, const VectorXd &qprev, VectorXd &F, std::vector<Tr> &H)
{
    // int nparticles = particles_.size();
    //
    // double basestiffness = 10000;
    // double basedrag = 1000.0;
    //
    // for(int i=0; i<nparticles; i++)
    // {
    //     if(q[2*i+1] < -0.5 && ! particles_[i].fixed)
    //     {
    //         double vel = (q[2*i+1]-qprev[2*i+1])/params_.timeStep;
    //         double dist = -0.5 - q[2*i+1];
    //
    //         F[2*i+1] += basestiffness*dist - basedrag*dist*vel;
    //
    //         H.push_back(Tr(2*i+1, 2*i+1, basestiffness
    //                        - 0.5*basedrag/params_.timeStep
    //                        + basedrag*qprev[2*i+1]/params_.timeStep
    //                     - 2.0*basedrag*q[2*i+1]/params_.timeStep));
    //     }
    // }
}

void Simulation::processPenaltyForce(const VectorXd &q, VectorXd &F)
{
    // int nrigids = (int)rigids_.size();
    //
    // for(int i=0; i<nrigids; i++)
    // {
    //     Vector2d p1 = q.segment<2>(2*rigids_[i].p1);
    //     Vector2d p2 = q.segment<2>(2*rigids_[i].p2);
    //     double dist = (p2-p1).norm();
    //     Vector2d localF = 4.0 * params_.penaltyStiffness * (dist * dist - rigids_[i].len * rigids_[i].len) * (p2-p1);
    //     F.segment<2>(2*rigids_[i].p1) += localF;
    //     F.segment<2>(2*rigids_[i].p2) -= localF;
    // }
}

void Simulation::computeMassInverse(Eigen::SparseMatrix<double> &Minv)
{
    // int ndofs = 2*int(particles_.size());
    //
    // Minv.resize(ndofs, ndofs);
    // Minv.setZero();
    //
    // vector<Tr> Minvcoeffs;
    // for(int i=0; i<ndofs/2; i++)
    // {
    //     double mass = particles_[i].fixed ? 0.0 : 1.0 / particles_[i].mass;
    //
    //     Minvcoeffs.push_back(Tr(2*i,   2*i,   mass));
    //     Minvcoeffs.push_back(Tr(2*i+1, 2*i+1, mass));
    // }
    //
    // Minv.setFromTriplets(Minvcoeffs.begin(), Minvcoeffs.end());
}

// void Simulation::computeConstraintFunction(const VectorXd &lambda, const Eigen::SparseMatrix<double> &Minv, const VectorXd &q,
//                                            const VectorXd &unconstrainedq, VectorXd &fval)
// {
//     int n = int(q.size());
//     int m = int(rigids_.size());
//     fval.setZero();
//
//     VectorXd dgT(n);
//     dgT.setZero();
//
//     VectorXd g(m);
//     g.setZero();
//
//     for(int i = 0; i < m; i++)
//     {
//         int ip1 = 2*rigids_[i].p1;
//         int ip2 = 2*rigids_[i].p2;
//
//         Vector2d p1 = q.segment<2>(ip1);
//         Vector2d p2 = q.segment<2>(ip2);
//
//         // Sum dgT
//         double lambda2 = lambda[i] * 2;
//         dgT[ip1]   += lambda2 * (p1[0] - p2[0]);
//         dgT[ip1+1] += lambda2 * (p1[1] - p2[1]);
//         dgT[ip2]   += lambda2 * (p2[0] - p1[0]);
//         dgT[ip2+1] += lambda2 * (p2[1] - p1[1]);
//
//         // Sum g
//         double dist = (p1-p2).norm();
//         g[i] += (dist * dist) - (rigids_[i].len * rigids_[i].len);
//     }
//
//     fval.segment(0, n) = (q - unconstrainedq) + Minv * dgT;
//     fval.segment(n, m) = g;
// }
//
// void Simulation::computeConstraintDifferential(const VectorXd &lambda, const Eigen::SparseMatrix<double> &Minv, const VectorXd &q,
//             Eigen::SparseMatrix<double> &dfval)
// {
//     int n = int(q.size());
//     int m = int(rigids_.size());
//     dfval.setZero();
//
//     SparseMatrix<double> Hg(n, n);
//     SparseMatrix<double> dgT(n, m);
//     SparseMatrix<double> I(n, n);
//     I.setIdentity();
//
//     vector<Tr> dgTcoeffs;
//     for(int i = 0; i < m; i++)
//     {
//         int ip1 = 2*rigids_[i].p1;
//         int ip2 = 2*rigids_[i].p2;
//
//         Vector2d p1 = q.segment<2>(ip1);
//         Vector2d p2 = q.segment<2>(ip2);
//
//         // Sum Hgi
//         double localHg = lambda[i] * 2;
//         Hg.coeffRef(ip1,   ip1)   += localHg;
//         Hg.coeffRef(ip1+1, ip1+1) += localHg;
//         Hg.coeffRef(ip2,   ip2)   += localHg;
//         Hg.coeffRef(ip2+1, ip2+1) += localHg;
//         Hg.coeffRef(ip1,   ip2)   -= localHg;
//         Hg.coeffRef(ip1+1, ip2+1) -= localHg;
//         Hg.coeffRef(ip2,   ip1)   -= localHg;
//         Hg.coeffRef(ip2+1, ip1+1) -= localHg;
//
//         // Set dgT
//         dgTcoeffs.push_back(Tr(ip1,   i, 2 * (p1[0] - p2[0])));
//         dgTcoeffs.push_back(Tr(ip1+1, i, 2 * (p1[1] - p2[1])));
//         dgTcoeffs.push_back(Tr(ip2,   i, 2 * (p2[0] - p1[0])));
//         dgTcoeffs.push_back(Tr(ip2+1, i, 2 * (p2[1] - p1[1])));
//     }
//     dgT.setFromTriplets(dgTcoeffs.begin(), dgTcoeffs.end());
//     dgT = Minv * dgT;
//
//     SparseMatrix<double> upperNxN(n, n);
//     upperNxN = I + Minv * Hg;
//
//     // Set the upper nxn matrix, dgT, and dg
//     vector<Tr> dfcoeffs;
//     for(int i = 0; i < n/2; i++)
//     {
//         dfcoeffs.push_back(Tr(2*i,   2*i,   upperNxN.coeff(2*i, 2*i)));
//         dfcoeffs.push_back(Tr(2*i+1, 2*i+1, upperNxN.coeff(2*i+1, 2*i+1)));
//     }
//     for(int i=0; i<n; i++)
//     {
//         for(int j=0; j<m; j++)
//         {
//             dfcoeffs.push_back(Tr(n+j, i,   dgT.coeff(i, j))); // dgT
//             dfcoeffs.push_back(Tr(i,   n+j, dgT.coeff(i, j))); // dg
//         }
//     }
//     dfval.setFromTriplets(dfcoeffs.begin(), dfcoeffs.end());
// }

// void Simulation::numericalIntegrationOld(VectorXd &q, VectorXd &qprev, VectorXd &v)
// {
//     VectorXd F;
//     SparseMatrix<double> H;
//     SparseMatrix<double> Minv;
//
//     computeMassInverse(Minv);
//
//     switch(params_.constraintHandler)
//     {
//         case SimParameters::CH_PENALTY_FORCE:
//         {
//             VectorXd oldq = q;
//             q += params_.timeStep*v;
//             computeForceAndHessian(q, oldq, F, H);
//             processPenaltyForce(q, F);
//             v += params_.timeStep*Minv*F;
//             break;
//         }
//
//         case SimParameters::CH_STEP_AND_PROJECT:
//         {
//             // Take unconstrained step
//             VectorXd unconstrainedq = q + params_.timeStep*v;
//             computeForceAndHessian(unconstrainedq, q, F, H);
//             v+=params_.timeStep*Minv*F;
//
//             int n = int(q.size());
//             int m = int(rigids_.size());
//
//             VectorXd qguess = unconstrainedq;
//
//             VectorXd lambda(m);
//             lambda.setZero();
//
//             VectorXd fval(n + m);
//             SparseMatrix<double> dfval(n + m, n + m);
//
//             int iter = 0;
//             for(iter = 0; iter < params_.NewtonMaxIters; iter++)
//             {
//                 computeConstraintFunction(lambda, Minv, qguess, unconstrainedq, fval);
//                 if(fval.norm() < params_.NewtonTolerance)
//                 {
//                     break;
//                 }
//                 computeConstraintDifferential(lambda, Minv, qguess, dfval);
//                 SparseQR<SparseMatrix<double>, COLAMDOrdering<int> > solver;
//                 dfval.makeCompressed();
//                 solver.compute(dfval);
//                 VectorXd deltaguess = solver.solve(-fval);
//                 // Decompose deltaguess into q and lambda
//                 qguess += deltaguess.head(n);
//                 lambda += deltaguess.tail(m);
//             }
//             if (iter >= params_.NewtonMaxIters)
//             {
//                 cout << "Newton's Method was unable to converge in " << params_.NewtonMaxIters << " step(s)!!!" << endl;
//                 exit(-1);
//             }
//             v += (qguess - unconstrainedq) / params_.timeStep;
//             q = qguess;
//             break;
//         }
//
//         case SimParameters::CH_LAGRANGE_MULTIPLIER:
//         {
//             VectorXd oldq = q;
//             q += params_.timeStep*v;
//             computeForceAndHessian(q, oldq, F, H);
//
//             int n = int(q.size());
//             int m = int(rigids_.size());
//
//             // Set dgT(q^(i+1))
//             SparseMatrix<double> dgT(n, m);
//             {
//                 vector<Tr> dgTcoeffs;
//                 for(int i = 0; i < m; i++)
//                 {
//                     int ip1 = 2*rigids_[i].p1;
//                     int ip2 = 2*rigids_[i].p2;
//
//                     Vector2d p1 = q.segment<2>(ip1);
//                     Vector2d p2 = q.segment<2>(ip2);
//
//                     // Set dgT
//                     dgTcoeffs.push_back(Tr(ip1,   i, 2 * (p1[0] - p2[0])));
//                     dgTcoeffs.push_back(Tr(ip1+1, i, 2 * (p1[1] - p2[1])));
//                     dgTcoeffs.push_back(Tr(ip2,   i, 2 * (p2[0] - p1[0])));
//                     dgTcoeffs.push_back(Tr(ip2+1, i, 2 * (p2[1] - p1[1])));
//                 }
//                 dgT.setFromTriplets(dgTcoeffs.begin(), dgTcoeffs.end());
//             }
//
//             VectorXd lambda(m);
//             lambda.setZero();
//
//             VectorXd fval(m);
//             SparseMatrix<double> dfval(m, m);
//
//             int iter = 0;
//             for(iter = 0; iter < params_.NewtonMaxIters; iter++)
//             {
//                 fval.setZero();
//                 VectorXd tmp = q+params_.timeStep*v+params_.timeStep*params_.timeStep*Minv*F+params_.timeStep*params_.timeStep*Minv*dgT*lambda;
//                 // Compute fval
//                 {
//                     for (int i = 0; i < m; i++)
//                     {
//                         int ip1 = 2*rigids_[i].p1;
//                         int ip2 = 2*rigids_[i].p2;
//
//                         Vector2d p1 = tmp.segment<2>(ip1);
//                         Vector2d p2 = tmp.segment<2>(ip2);
//
//                         // Sum g
//                         double dist = (p1-p2).norm();
//                         fval[i] += (dist * dist) - (rigids_[i].len * rigids_[i].len);
//                     }
//                 }
//                 if(fval.norm() < params_.NewtonTolerance)
//                 {
//                     break;
//                 }
//                 // Set dgT(tmp) then transpose to get dg(tmp)
//                 SparseMatrix<double> dgT2(n, m);
//                 {
//                     vector<Tr> dgT2coeffs;
//                     for(int i = 0; i < m; i++)
//                     {
//                         int ip1 = 2*rigids_[i].p1;
//                         int ip2 = 2*rigids_[i].p2;
//
//                         Vector2d p1 = tmp.segment<2>(ip1);
//                         Vector2d p2 = tmp.segment<2>(ip2);
//
//                         // Set dgT
//                         dgT2coeffs.push_back(Tr(ip1,   i, 2 * (p1[0] - p2[0])));
//                         dgT2coeffs.push_back(Tr(ip1+1, i, 2 * (p1[1] - p2[1])));
//                         dgT2coeffs.push_back(Tr(ip2,   i, 2 * (p2[0] - p1[0])));
//                         dgT2coeffs.push_back(Tr(ip2+1, i, 2 * (p2[1] - p1[1])));
//                     }
//                     dgT2.setFromTriplets(dgT2coeffs.begin(), dgT2coeffs.end());
//                 }
//                 dfval = dgT2.transpose()*params_.timeStep*params_.timeStep*Minv*dgT;
//
//                 SparseQR<SparseMatrix<double>, COLAMDOrdering<int> > solver;
//                 dfval.makeCompressed();
//                 solver.compute(dfval);
//                 lambda += solver.solve(-fval);
//             }
//             if (iter >= params_.NewtonMaxIters)
//             {
//                 cout << "Newton's Method was unable to converge in " << params_.NewtonMaxIters << " step(s)!!!" << endl;
//                 exit(-1);
//             }
//             computeForceAndHessian(q, oldq, F, H);
//             v += params_.timeStep*Minv*F+params_.timeStep*Minv*dgT*lambda;
//             break;
//         }
//     }
// }
