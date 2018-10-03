TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

DISTFILES += \
ObstacleAvoidance.fll


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/fuzzylite/release/bin/release/ -lfuzzylite
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/fuzzylite/release/bin/debug/ -lfuzzylite
else:unix: LIBS += -L$$PWD/fuzzylite/release/bin/ -lfuzzylite

INCLUDEPATH += $$PWD/fuzzylite
DEPENDPATH += $$PWD/fuzzylite
