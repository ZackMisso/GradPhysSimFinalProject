#ifndef SIMULATION_H
#define SIMULATION_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <set>
#include <QMutex>

typedef Eigen::Triplet<double> Tr;

struct SimParameters;
class HairInstance;

class Simulation
{
public:
    Simulation(const SimParameters &params);

    void addParticle(double x, double y);
    void addSaw(double x, double y);

    void takeSimulationStep();
    void render(bool is3D);
    void clearScene();

    std::vector<HairInstance*> hairs_;

    double getTime() { return time_; };

private:
    const SimParameters &params_;
    QMutex renderLock_;

    double time_;

    void buildConfiguration(Eigen::VectorXd &q, Eigen::VectorXd &qprev, Eigen::VectorXd &v);
    void unbuildConfiguration(const Eigen::VectorXd &q, const Eigen::VectorXd &v);

    void computeForceAndHessian(const Eigen::VectorXd &q, const Eigen::VectorXd &qprev, Eigen::VectorXd &F, Eigen::SparseMatrix<double> &H);

    void computeMassInverse(Eigen::SparseMatrix<double> &Minv);

    void numericalIntegration(Eigen::VectorXd &q, Eigen::VectorXd &qprev, Eigen::VectorXd &v);
    void reconstruction();
};

#endif // SIMULATION_H
