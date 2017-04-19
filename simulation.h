#ifndef SIMULATION_H
#define SIMULATION_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <set>
#include <QMutex>
// #include "hairinstance.h"

typedef Eigen::Triplet<double> Tr;

struct SimParameters;
class HairInstance;

// struct Particle
// {
// public:
//     Particle(Eigen::Vector2d pos, double mass, bool isFixed, bool isInert) : pos(pos), mass(mass), fixed(isFixed), inert(isInert)
//     {
//         vel.setZero();
//         prevpos = pos;
//     }
//
//     Eigen::Vector2d pos;
//     Eigen::Vector2d prevpos;
//     Eigen::Vector2d vel;
//     double mass;
//     bool fixed;
//     bool inert;
// };
//
// struct Spring
// {
// public:
//     Spring(int p1, int p2, double stiffness, double restlen, bool unsnappable, double mass) : p1(p1), p2(p2), stiffness(stiffness), restlen(restlen), unsnappable(unsnappable), mass(mass) {}
//
//     int p1;
//     int p2;
//     double stiffness;
//     double restlen;
//     bool unsnappable;
//     double mass;
// };
//
// struct RigidRod
// {
// public:
//     RigidRod(int p1, int p2, double len) : p1(p1), p2(p2), len(len) {}
//
//     int p1;
//     int p2;
//     double len;
// };
//
// struct BendingHinge
// {
// public:
//     BendingHinge(int s1, int s2, double stiffness) : s1(s1), s2(s2), stiffness(stiffness) {}
//
//     int s1;
//     int s2;
//     double stiffness;
// };
//
// struct Saw
// {
// public:
//     Saw(Eigen::Vector2d pos, double radius) : pos(pos), radius(radius) {}
//
//     Eigen::Vector2d pos;
//     double radius;
// };

class Simulation
{
public:
    Simulation(const SimParameters &params);

    void addParticle(double x, double y);
    void addSaw(double x, double y);

    void takeSimulationStep();
    void render();
    void clearScene();

private:
    const SimParameters &params_;
    QMutex renderLock_;

    double time_;
    std::vector<HairInstance*> hairs_;

    // std::vector<Particle> particles_;
    // std::vector<Spring> springs_;
    // std::vector<RigidRod> rigids_;
    // std::vector<BendingHinge> hinges_;
    // std::vector<Saw> saws_;

    void buildConfiguration(Eigen::VectorXd &q, Eigen::VectorXd &qprev, Eigen::VectorXd &v);
    void unbuildConfiguration(const Eigen::VectorXd &q, const Eigen::VectorXd &v);

    void computeForceAndHessian(const Eigen::VectorXd &q, const Eigen::VectorXd &qprev, Eigen::VectorXd &F, Eigen::SparseMatrix<double> &H);
    void processGravityForce(Eigen::VectorXd &F);
    void processSpringForce(const Eigen::VectorXd &q, Eigen::VectorXd &F, std::vector<Tr> &H);
    void processDampingForce(const Eigen::VectorXd &q, const Eigen::VectorXd &qprev, Eigen::VectorXd &F, std::vector<Tr> &H);
    void processFloorForce(const Eigen::VectorXd &q, const Eigen::VectorXd &qprev, Eigen::VectorXd &F, std::vector<Tr> &H);
    void processPenaltyForce(const Eigen::VectorXd &q, Eigen::VectorXd &F);
    void processBendingForce(const Eigen::VectorXd &q, Eigen::VectorXd &F);

    // void computeConstraintFunction(const Eigen::VectorXd &lambda, const Eigen::SparseMatrix<double> &Minv, const Eigen::VectorXd &q,
    //             const Eigen::VectorXd &initialq, Eigen::VectorXd &fval);
    // void computeConstraintDifferential(const Eigen::VectorXd &lambda, const Eigen::SparseMatrix<double> &Minv, const Eigen::VectorXd &q,
    //         Eigen::SparseMatrix<double> &dfval);

    void computeMassInverse(Eigen::SparseMatrix<double> &Minv);

    void createFlexibleRod(Eigen::Vector2d newpos, int indexOfOther, int indexOfNew, double mass);

    // void numericalIntegrationOld(Eigen::VectorXd &q, Eigen::VectorXd &qprev, Eigen::VectorXd &v);

    void numericalIntegration(Eigen::VectorXd &q, Eigen::VectorXd &qprev, Eigen::VectorXd &v);
    void reconstruction();
};

#endif // SIMULATION_H
