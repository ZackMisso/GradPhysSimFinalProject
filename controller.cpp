#include "controller.h"
#include "mainwindow.h"
#include "simulation.h"
#include "hairinstance.h"
#include <QDebug>

Controller::Controller(int fps) : QThread(), mw_(NULL), fps_(fps)
{
}

Controller::~Controller()
{
    delete simtimer_;
    delete sim_;
}

void Controller::initialize(MainWindow *mw)
{
    mw_ = mw;
    sim_ = new Simulation(params_);
    // sim_->initializeGL();
}

void Controller::run()
{
    simtimer_ = new QTimer(this);
    reset();
    connect(simtimer_, SIGNAL(timeout()), this, SLOT(simTick()));
    simtimer_->start(1000/fps_);
    exec();
}

void Controller::reset()
{
    params_ = SimParameters();
    QMetaObject::invokeMethod(mw_, "setUIFromParameters", Q_ARG(SimParameters, params_));
    clearScene();
}

void Controller::clearScene()
{
    sim_->clearScene();
}

void Controller::updateParameters(SimParameters params)
{
    params_ = params;
}

void Controller::render(bool is3D)
{
    if (params_.reset)
    {
        params_.simRunning = false;
        params_.reset = false;
        clearScene();
    }
    if (!params_.bakeRunning)
    {
        sim_->render(is3D);
    }
    else
    {
        sim->bakeRender(is3D);
    }
}

void Controller::mouseClicked(double x, double y)
{
    switch(params_.clickMode)
    {
    case SimParameters::CM_ADDPARTICLE:
        sim_->addParticle(x, y);
        break;
    case SimParameters::CM_ADDSAW:
        sim_->addSaw(x,y);
        break;
    }
}

void Controller::simTick()
{
    if (params_.reset)
    {
        params_.simRunning = false;
        params_.reset = false;
        clearScene();
    }
    if(params_.simRunning)
    {
        sim_->takeSimulationStep();
    }
    if(params_.baking)
    {
        sim_->takeBakeStep();
    }
}

void Controller::getCameraInfo(int hair, Eigen::Vector3d &center, double &scale)
{
    // FIX LATER

    // center = sim_->hairs_[0]->pos_;
    scale = 1.0;
}
