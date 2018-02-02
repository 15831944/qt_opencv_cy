#-------------------------------------------------
#
# Project created by QtCreator 2017-01-22T20:53:43
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = qt_opencv_cy
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    cyproc.cpp \
    chongya.cpp \
    cy_algorithm.cpp \
    cy_performance.cpp

HEADERS  += mainwindow.h \
    cyproc.h \
    globalParam.h \
    chongya.h \
    cy_algorithm.h \
    cy_performance.h

FORMS    += mainwindow.ui


#INCLUDEPATH+=D:\opencv\opencv\build\include
#INCLUDEPATH+=D:\opencv\opencv\build\include\opencv
#INCLUDEPATH+=D:\opencv\opencv\build\include\opencv2

#LIBS+=D:\opencv\opencv\build\x64\vc14\lib\opencv_world310d.lib
#LIBS+=D:\opencv\opencv\build\x64\vc14\lib\opencv_world310.lib
INCLUDEPATH +=  C:\opencv\build\include \
                C:\opencv\build\include\opencv \
                C:\opencv\build\include\opencv2

LIBS += C:\opencv\build\x64\vc14\lib\opencv_world320d.lib

