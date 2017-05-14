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

    params.baking = baking_;
    params.bakeRunning = runBake_;

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

    params.showSegments = renderingSegments_;

    setUIFromParameters(params);
    QMetaObject::invokeMethod(&cont_, "updateParameters", Q_ARG(SimParameters, params));
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

    ui->timeStepEdit->setText(QString::number(params.timeStep));
    ui->newtonTolEdit->setText(QString::number(params.NewtonTolerance));
    ui->newtonMaxItersEdit->setText(QString::number(params.NewtonMaxIters));

    ui->gravityBox->setText(QString::number(params.gravity));
    ui->stiffnessBox->setText(QString::number(params.stiffness));
    ui->hairLengthBox->setText(QString::number(params.hairLength));
    ui->segmentsBox->setText(QString::number(params.segments));
    ui->subSegmentsBox->setText(QString::number(params.subsegments));

    ui->showSegmentsCheck->setChecked(params.showSegments);
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

void MainWindow::on_showSegmentsCheck_toggled(bool checked)
{
    renderingSegments_ = checked;
    setParametersFromUI();
}

void MainWindow::on_bakeButton_clicked()
{
    simRunning_ = false;
    baking_ = true;
    setParametersFromUI();
}

void MainWindow::on_runBakedSim_clicked()
{
    simRunning_ = false;
    runBake_ = true;
    setParametersFromUI();
}
