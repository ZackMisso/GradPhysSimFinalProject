#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "simparameters.h"
#include <QThread>
#include <QTimer>

class MainWindow;
class Simulation;

class Controller : public QThread
{
    Q_OBJECT

public:
    Controller(int fps);
    virtual ~Controller();
    void initialize(MainWindow *mw);
    void render(bool is3D);

public slots:
    void reset();
    void clearScene();
    void updateParameters(SimParameters params);
    void mouseClicked(double x, double y);

    void simTick();

protected:
    virtual void run();

private:
    MainWindow *mw_;
    Simulation *sim_;
    SimParameters params_;

    int fps_;
    QTimer *simtimer_;
};

#endif // CONTROLLER_H
