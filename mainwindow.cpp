#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "simparameters.h"
#include "controller.h"

#include <iostream>
using namespace std;

MainWindow::MainWindow(Controller &cont, int fps, QWidget *parent) :
    QMainWindow(parent),
    cont_(cont),
    ui(new Ui::MainWindow)
{
    singleStrandExample_ = true;
    interpolationExample_ = false;
    bundleExample_ = false;
    sphereExample_ = false;
    headExample_ = false;
    ui->setupUi(this);
    ui->GLWidget->setController(&cont);
    simRunning_ = false;
    connect(&renderTimer_, SIGNAL(timeout()), this, SLOT(updateGL()));
    renderTimer_.start(1000/fps);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionExit_triggered()
{
    close();
}

void MainWindow::setParametersFromUI()
{
    SimParameters params;

    params.simRunning = simRunning_;

//    if(ui->explicitEulerButton->isChecked())
//        params.integrator = SimParameters::TI_EXPLICIT_EULER;
//    else if(ui->implicitEulerButton->isChecked())
//        params.integrator = SimParameters::TI_IMPLICIT_EULER;
//    else if(ui->midpointButton->isChecked())
//        params.integrator = SimParameters::TI_IMPLICIT_MIDPOINT;
//    else if(ui->velocityVerletButton->isChecked())
//        params.integrator = SimParameters::TI_VELOCITY_VERLET;

    // if(ui->penaltyForceButton->isChecked())
    //     params.constraintHandler = SimParameters::CH_PENALTY_FORCE;
    // else if(ui->stepAndProjectButton->isChecked())
    //     params.constraintHandler = SimParameters::CH_STEP_AND_PROJECT;
    // else if(ui->lagrangeMultiplierButton->isChecked())
    //     params.constraintHandler = SimParameters::CH_LAGRANGE_MULTIPLIER;

    params.timeStep = ui->timeStepEdit->text().toDouble();
    params.NewtonTolerance = ui->newtonTolEdit->text().toDouble();
    params.NewtonMaxIters = ui->newtonMaxItersEdit->text().toInt();

    params.gravity = ui->gravityBox->text().toDouble();
    params.stiffness = ui->stiffnessBox->text().toDouble();
    params.hairLength = ui->hairLengthBox->text().toDouble();
    params.segments = ui->segmentsBox->text().toInt();
    params.subsegments = ui->subSegmentsBox->text().toInt();

    params.singleStrandExample = singleStrandExample_;
    params.interpolationExample = interpolationExample_;
    params.bundleExample = bundleExample_;
    params.sphereExample = sphereExample_;
    params.headExample = headExample_;
    params.reset = reset_;

    reset_ = false;

    // params.penaltyStiffness = ui->penaltyStiffnessEdit->text().toDouble();

    // params.activeForces = 0;
    // if(ui->gravityCheckBox->isChecked())
    //     params.activeForces |= SimParameters::F_GRAVITY;
    // if(ui->springsCheckBox->isChecked())
    //     params.activeForces |= SimParameters::F_SPRINGS;
    // if(ui->floorCheckBox->isChecked())
    //     params.activeForces |= SimParameters::F_FLOOR;
    // if(ui->dampingStiffnessCheckBox->isChecked())
    //     params.activeForces |= SimParameters::F_DAMPING;
    // if(ui->elasticBendingCheckBox->isChecked())
    //     params.activeForces |= SimParameters::F_BENDING;

    // params.gravityG = ui->gravityGEdit->text().toDouble();
    // params.springStiffness = ui->springStiffnessEdit->text().toDouble();
    // params.maxSpringStrain = ui->maxStrainEdit->text().toDouble();
    // params.dampingStiffness = ui->dampingStiffnessEdit->text().toDouble();

    // if(ui->addParticleButton->isChecked())
    //     params.clickMode = SimParameters::CM_ADDPARTICLE;
    // else if(ui->addSawButton->isChecked())
    //     params.clickMode = SimParameters::CM_ADDSAW;

    // params.particleMass = ui->massEdit->text().toDouble();
    // params.maxSpringDist = ui->maxSpringDistEdit->text().toDouble();
    // params.particleFixed = ui->isFixedCheckBox->isChecked();

    // params.sawRadius = ui->radiusEdit->text().toDouble();

    // if(ui->springButton->isChecked())
    //     params.connector = SimParameters::CT_SPRING;
    // else if(ui->rigidRodButton->isChecked())
    //     params.connector = SimParameters::CT_RIGID_ROD;
    // else if(ui->flexibleRodButton->isChecked())
    //     params.connector = SimParameters::CT_FLEXIBLE_ROD;

    // params.rodDensity = ui->densityEdit->text().toDouble();
    // params.rodStretchingStiffness = ui->stretchKEdit->text().toDouble();
    // params.rodBendingStiffness = ui->bendKEdit->text().toDouble();
    // params.rodSegments = ui->segmentsEdit->text().toInt();

    // cout << "HELLO" << endl;

    setUIFromParameters(params);
    // cout << "HERE" << endl;
    QMetaObject::invokeMethod(&cont_, "updateParameters", Q_ARG(SimParameters, params));
    // cout << "WHAT" << endl;
}

void MainWindow::setUIFromParameters(const SimParameters &params)
{
    if(params.simRunning)
    {
        ui->startSimulationButton->setText(QString("Pause Simulation"));
        simRunning_ = true;
    }
    else
    {
        ui->startSimulationButton->setText(QString("Start Simulation"));
        simRunning_ = false;
    }

//    switch(params.integrator)
//    {
//    case SimParameters::TI_EXPLICIT_EULER:
//        ui->explicitEulerButton->setChecked(true);
//        break;
//    case SimParameters::TI_IMPLICIT_EULER:
//        ui->implicitEulerButton->setChecked(true);
//        break;
//    case SimParameters::TI_IMPLICIT_MIDPOINT:
//        ui->midpointButton->setChecked(true);
//        break;
//    case SimParameters::TI_VELOCITY_VERLET:
//        ui->velocityVerletButton->setChecked(true);
//        break;
//    }

    // switch(params.constraintHandler)
    // {
    // case SimParameters::CH_PENALTY_FORCE:
    //     ui->penaltyForceButton->setChecked(true);
    //     break;
    // case SimParameters::CH_STEP_AND_PROJECT:
    //     ui->stepAndProjectButton->setChecked(true);
    //     break;
    // case SimParameters::CH_LAGRANGE_MULTIPLIER:
    //     ui->lagrangeMultiplierButton->setChecked(true);
    //     break;
    // }

    ui->timeStepEdit->setText(QString::number(params.timeStep));
    ui->newtonTolEdit->setText(QString::number(params.NewtonTolerance));
    ui->newtonMaxItersEdit->setText(QString::number(params.NewtonMaxIters));

    ui->gravityBox->setText(QString::number(params.gravity));
    ui->stiffnessBox->setText(QString::number(params.stiffness));
    ui->hairLengthBox->setText(QString::number(params.hairLength));
    ui->segmentsBox->setText(QString::number(params.segments));
    ui->subSegmentsBox->setText(QString::number(params.subsegments));

    // ui->penaltyStiffnessEdit->setText(QString::number(params.penaltyStiffness));

    // ui->gravityCheckBox->setChecked(params.activeForces & SimParameters::F_GRAVITY);
    // ui->springsCheckBox->setChecked(params.activeForces & SimParameters::F_SPRINGS);
    // ui->floorCheckBox->setChecked(params.activeForces & SimParameters::F_FLOOR);
    // ui->dampingStiffnessCheckBox->setChecked(params.activeForces & SimParameters::F_DAMPING);
    // ui->elasticBendingCheckBox->setChecked(params.activeForces & SimParameters::F_BENDING);

    // ui->gravityGEdit->setText(QString::number(params.gravityG));
    // ui->springStiffnessEdit->setText(QString::number(params.springStiffness));
    // ui->maxStrainEdit->setText(QString::number(params.maxSpringStrain));
    // ui->dampingStiffnessEdit->setText(QString::number(params.dampingStiffness));

    // if(params.clickMode == SimParameters::CM_ADDPARTICLE)
    //     ui->addParticleButton->setChecked(true);
    // else if(params.clickMode == SimParameters::CM_ADDSAW)
    //     ui->addSawButton->setChecked(true);

    // ui->massEdit->setText(QString::number(params.particleMass));
    // ui->maxSpringDistEdit->setText(QString::number(params.maxSpringDist));
    // ui->isFixedCheckBox->setChecked(params.particleFixed);
    // ui->radiusEdit->setText(QString::number(params.sawRadius));

    // switch(params.connector)
    // {
    // case SimParameters::CT_SPRING:
    //     ui->springButton->setChecked(true);
    //     break;
    // case SimParameters::CT_RIGID_ROD:
    //     ui->rigidRodButton->setChecked(true);
    //     break;
    // case SimParameters::CT_FLEXIBLE_ROD:
    //     ui->flexibleRodButton->setChecked(true);
    //     break;
    // }

    // ui->densityEdit->setText(QString::number(params.rodDensity));
    // ui->stretchKEdit->setText(QString::number(params.rodStretchingStiffness));
    // ui->bendKEdit->setText(QString::number(params.rodBendingStiffness));
    // ui->segmentsEdit->setText(QString::number(params.rodSegments));
}

void MainWindow::updateGL()
{
    ui->GLWidget->update();
}

void MainWindow::on_actionReset_Everything_triggered()
{
    QMetaObject::invokeMethod(&cont_, "reset");
}

void MainWindow::on_actionReset_triggered()
{
    QMetaObject::invokeMethod(&cont_, "clearScene");
}

void MainWindow::on_startSimulationButton_clicked()
{
    simRunning_ = !simRunning_;
    setParametersFromUI();
}

//void MainWindow::on_explicitEulerButton_clicked()
//{
//    setParametersFromUI();
//}

//void MainWindow::on_implicitEulerButton_clicked()
//{
//    setParametersFromUI();
//}

//void MainWindow::on_midpointButton_clicked()
//{
//    setParametersFromUI();
//}

//void MainWindow::on_velocityVerletButton_clicked()
//{
//    setParametersFromUI();
//}

void MainWindow::on_timeStepEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_newtonTolEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_newtonMaxItersEdit_editingFinished()
{
    setParametersFromUI();
}

// void MainWindow::on_gravityCheckBox_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_springsCheckBox_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_floorCheckBox_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_dampingStiffnessCheckBox_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_gravityGEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_springStiffnessEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_maxStrainEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_dampingStiffnessEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_addParticleButton_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_addSawButton_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_massEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_maxSpringDistEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_isFixedCheckBox_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_radiusEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_penaltyForceButton_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_stepAndProjectButton_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_lagrangeMultiplierButton_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_penaltyStiffnessEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_elasticBendingCheckBox_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_densityEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_stretchKEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_bendKEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_segmentsEdit_editingFinished()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_springButton_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_rigidRodButton_clicked()
// {
//     setParametersFromUI();
// }

// void MainWindow::on_flexibleRodButton_clicked()
// {
//     setParametersFromUI();
// }

void MainWindow::on_singleStrandExButton_clicked()
{
    singleStrandExample_ = true;
    interpolationExample_ = false;
    bundleExample_ = false;
    sphereExample_ = false;
    headExample_ = false;
    reset_ = true;
    setParametersFromUI();
}

void MainWindow::on_interpolationExampleButton_clicked()
{
    singleStrandExample_ = false;
    interpolationExample_ = true;
    bundleExample_ = false;
    sphereExample_ = false;
    headExample_ = false;
    reset_ = true;
    setParametersFromUI();
}

void MainWindow::on_headExampleButton_clicked()
{
    singleStrandExample_ = false;
    interpolationExample_ = false;
    bundleExample_ = false;
    sphereExample_ = false;
    headExample_ = true;
    reset_ = true;
    setParametersFromUI();
}

void MainWindow::on_bundleExampleButton_clicked()
{
    singleStrandExample_ = false;
    interpolationExample_ = false;
    bundleExample_ = true;
    sphereExample_ = false;
    headExample_ = false;
    reset_ = true;
    setParametersFromUI();
}

void MainWindow::on_sphereExampleButton_clicked()
{
    singleStrandExample_ = false;
    interpolationExample_ = false;
    bundleExample_ = false;
    sphereExample_ = true;
    headExample_ = false;
    reset_ = true;
    setParametersFromUI();
}

void MainWindow::on_subSegmentsBox_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_segmentsBox_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_hairLengthBox_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_stiffnessBox_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_gravityBox_editingFinished()
{
    setParametersFromUI();
}
