TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    pathPlanner.cpp

#Gazebo
PKGCONFIG += gazebo

#OpenCV
QT_CONFIG -=no-pkg-config
CONFIG += link_pkgconfig
PKGCONFIG += opencv

#Fuzzylite
DISTFILES +=

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../fuzzylite/release/bin/release/ -lfuzzylite
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../fuzzylite/release/bin/debug/ -lfuzzylite
else:unix: LIBS += -L$$PWD/../../fuzzylite/release/bin/ -lfuzzylite

INCLUDEPATH += $$PWD/../../fuzzylite
DEPENDPATH += $$PWD/../../fuzzylite


HEADERS += \
    pathPlanner.h
