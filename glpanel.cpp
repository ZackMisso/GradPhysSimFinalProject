#include "glpanel.h"
#include "controller.h"
#include <iostream>
#include <QMouseEvent>
#include "hairinstance.h"
#include "simulation.h"

using namespace std;
using namespace Eigen;

GLPanel::GLPanel(QWidget *parent) :
    QGLWidget(parent)
{
    cont_ = NULL;
    is3D = true;
}

void GLPanel::setController(Controller *cont)
{
    cont_ = cont;
}

void GLPanel::resizeGL(int w, int h)
{
    if (is3D)
    {
        c_.setPerpective(60.0, 1.0);
        c_.setViewport(w, h);
        // c_.setViewport(2.0, 2.0);
    }
    else
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        glOrtho(-1, 1, -1, 1, 0, 1);

        glMatrixMode(GL_MODELVIEW);

        glDisable(GL_DEPTH_TEST);

        glClearColor(1.0, 1.0, 1.0, 0.0);
    }
}

void GLPanel::paintGL()
{
    if (is3D)
    {
        assert(cont_);
        glClearColor(1.0, 1.0, 1.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glColor3f(0.0, 0.0, 0.0);

        c_.applyViewport();
        c_.applyProjection();

        // Vector3d eye = Vector3d(sin(cont_->getSim()->getTime() / 100.0), 0, cos(cont_->getSim()->getTime() / 100.0));
        
        Vector3d eye = Vector3d(0.0, 0.0, 1.0);

        double scale = 1.0;
        cont_->getCameraInfo(cameraView_, eye, scale);

        eye *= scale;

        double dz = exp(scale_);

        // eye[2] = dz;
        c_.setEye(eye);
        eye[2] -= 1.0;
        // c_.setCenter(eye);
        c_.setCenter(Vector3d(0.0, 0.0, 0.0));
        Vector3d up(0,1,0);
        c_.setUp(up);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        c_.applyLookAt();
        glScaled(scale,scale,scale);

        // GLfloat lightColor0[] = {1.f, 1.f, 1.f, 1.0f};
        // GLfloat ambientColor0[] = {0.2f, 0.2f, 0.2f, 1.0f};
        // GLfloat lightPosition[4] = { (float)lightPos_[0], (float)lightPos_[1], (float)lightPos_[2], (float)lightPos_[3] };
        // glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor0);
        // glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor0);
        // glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
        // glLightf( GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1.0f );
        // glLightf( GL_LIGHT0, GL_LINEAR_ATTENUATION , 0.0f );
        // glLightf( GL_LIGHT0, GL_QUADRATIC_ATTENUATION , 0.0f );
        // glEnable(GL_LIGHTING);
        // glEnable(GL_LIGHT0);

        // cout << "EYE" << endl;
        // cout << c_.getEye() << endl;

        // cout << "View Center" << endl;
        // cout << c_.getCenter() << endl;

        cont_->render(is3D);
    }
    else
    {
        glClear(GL_COLOR_BUFFER_BIT);
        cont_->render(is3D);
    }
}

void GLPanel::mousePressEvent(QMouseEvent *me)
{
    if (is3D)
    {
        // to be implemented
    }
    else
    {
        if(me->button() == Qt::LeftButton)
        {
            double x = -1.0 + 2.0*double(me->x())/double(width());
            double y = 1.0 - 2.0*double(me->y())/double(height());
            QMetaObject::invokeMethod(cont_, "mouseClicked", Q_ARG(double, x), Q_ARG(double, y));
        }
    }
}

void GLPanel::initializeGL()
{
    // cout << "IS THIS CALLED ???" << endl;

    assert(cont_);
    glShadeModel(GL_FLAT);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    GLfloat lmodel_ambient[] = { 0.06, 0.06, 0.06, 1.0 };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    // add shaders here for milestone 2 and convert to glfw
}

void GLPanel::mouseMoveEvent(QMouseEvent *me)
{
    // std::cout << "MOUSE MOVING" << std::endl;

    if (is3D)
    {
        // to be implemented
    }
    else
    {
        // does nothing for milestone 1
    }
}

void GLPanel::mouseReleaseEvent(QMouseEvent *me)
{
    // std::cout << "MOUSE RELEASE" << std::endl;

    if (is3D)
    {
        // to be implemented
    }
    else
    {
        // does nothing for milestone 1
    }
}

void GLPanel::keyPressEvent(QKeyEvent *ke)
{
    if (is3D)
    {
        // to be implemented
    }
    else
    {
        // does nothing for milestone 1
    }
}

void GLPanel::keyReleaseEvent(QKeyEvent *ke)
{
    if (is3D)
    {
        // to be implemented
    }
    else
    {
        // does nothing for milestone 1
    }
}

void GLPanel::scaleMousePos(int x, int y, double &scaledx, double &scaledy) const
{
    if (is3D)
    {
        // to be implemented
    }
    else
    {
        // does nothing for milestone 1
    }
}
