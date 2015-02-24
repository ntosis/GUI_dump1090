#-------------------------------------------------
#
# Project created by QtCreator 2015-01-28T19:45:18
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TryToUseRTLSDRwithQT
TEMPLATE = app
CONFIG += thread
SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

INCLUDEPATH +=/usr/include/libusb-1.0
LIBS += -L/usr/lib \
 -lusb
INCLUDEPATH += /usr/include/
LIBS += -L/usr/lib \
-lrtlsdr
INCLUDEPATH += /usr/include/
LIBS += -L/usr/lib \
-lpthread
