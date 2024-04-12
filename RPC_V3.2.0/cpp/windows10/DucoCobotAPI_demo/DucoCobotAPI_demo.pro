TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp

HEADERS += \
    include/DucoCobot.h

win32: LIBS += -L$$PWD/lib/ -lDucoCobotAPI

INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD/include

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/lib/DucoCobotAPI.lib
else:win32-g++: PRE_TARGETDEPS += $$PWD/lib/libDucoCobotAPI.a
