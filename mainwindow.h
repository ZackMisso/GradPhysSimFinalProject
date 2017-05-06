#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

class Controller;
struct SimParameters;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(Controller &cont, int fps, QWidget *parent = 0);
    ~MainWindow();   

public slots:
    void setUIFromParameters(const SimParameters &params);

private slots:
    void updateGL();

    void on_actionExit_triggered();

    void on_actionReset_Everything_triggered();

    void on_actionReset_triggered();

    void on_startSimulationButton_clicked();

//    void on_explicitEulerButton_clicked();

//    void on_implicitEulerButton_clicked();

//    void on_midpointButton_clicked();

//    void on_velocityVerletButton_clicked();

    void on_timeStepEdit_editingFinished();

    void on_newtonTolEdit_editingFinished();

    void on_newtonMaxItersEdit_editingFinished();

    // void on_gravityCheckBox_clicked();

    // void on_springsCheckBox_clicked();

    // void on_floorCheckBox_clicked();

    // void on_dampingStiffnessCheckBox_clicked();

    // void on_gravityGEdit_editingFinished();

    // void on_springStiffnessEdit_editingFinished();

    // void on_maxStrainEdit_editingFinished();

    // void on_dampingStiffnessEdit_editingFinished();

    // void on_addParticleButton_clicked();

    // void on_addSawButton_clicked();

    // void on_massEdit_editingFinished();

    // void on_maxSpringDistEdit_editingFinished();

    // void on_isFixedCheckBox_clicked();

    // void on_radiusEdit_editingFinished();

    // void on_penaltyForceButton_clicked();

    // void on_stepAndProjectButton_clicked();

    // void on_lagrangeMultiplierButton_clicked();

    // void on_penaltyStiffnessEdit_editingFinished();

    // void on_elasticBendingCheckBox_clicked();

    // void on_densityEdit_editingFinished();

    // void on_stretchKEdit_editingFinished();

    // void on_bendKEdit_editingFinished();

    // void on_segmentsEdit_editingFinished();

    // void on_springButton_clicked();

    // void on_rigidRodButton_clicked();

    // void on_flexibleRodButton_clicked();

    void on_singleStrandExButton_clicked();

    void on_interpolationExampleButton_clicked();

    void on_headExampleButton_clicked();

    void on_bundleExampleButton_clicked();

    void on_sphereExampleButton_clicked();

    void on_subSegmentsBox_editingFinished();

    void on_segmentsBox_editingFinished();

    void on_hairLengthBox_editingFinished();

    void on_stiffnessBox_editingFinished();

    void on_gravityBox_editingFinished();

    void on_showSegmentsCheck_toggled(bool checked);

private:
    Controller &cont_;
    Ui::MainWindow *ui;
    bool simRunning_;
    QTimer renderTimer_;

    bool singleStrandExample_;
    bool interpolationExample_;
    bool bundleExample_;
    bool sphereExample_;
    bool headExample_;
    bool reset_;

    bool renderingSegments_;

    void setParametersFromUI();
};

#endif // MAINWINDOW_H
