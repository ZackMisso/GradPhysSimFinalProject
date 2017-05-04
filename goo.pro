#-------------------------------------------------
#
# Project created by QtCreator 2014-09-03T16:42:53
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = goo
TEMPLATE = app

INCLUDEPATH += $$_PRO_FILE_PWD_/eigen_3.2.2/

win32 {
    LIBS += -lopengl32
}

unix {
    LIBS += -lGLU
    QMAKE_CXXFLAGS += -std=c++11 -g
}

SOURCES += main.cpp\
        mainwindow.cpp \
    glpanel.cpp \
    controller.cpp \
    simulation.cpp \
    simparameters.cpp \
    camera.cpp \
    vectormath.cpp \
    hairinstance.cpp \
    renderer.cpp \
    glrenderer.cpp \
    glsrenderer.cpp \
    raytracer.cpp \
    rigidbodyinstance.cpp

HEADERS  += mainwindow.h \
    glpanel.h \
    controller.h \
    simulation.h \
    simparameters.h \
    camera.h \
    vectormath.h \
    hairinstance.h \
    renderer.h \
    glrenderer.h \
    glsrenderer.h \
    raytracer.h \
    rigidbodyinstance.h

FORMS    += mainwindow.ui

CONFIG += c++11
