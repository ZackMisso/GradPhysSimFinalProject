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

    void on_timeStepEdit_editingFinished();

    void on_newtonTolEdit_editingFinished();

    void on_newtonMaxItersEdit_editingFinished();

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

    void on_bakeButton_clicked();

    void on_runBakedSim_clicked();

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

    bool runBake_;
    bool baking_;

    bool renderingSegments_;

    void setParametersFromUI();
};

#endif // MAINWINDOW_H
