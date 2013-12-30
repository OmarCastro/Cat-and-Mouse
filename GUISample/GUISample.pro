TEMPLATE	= app
TARGET		= GUISample
DEFINES         += CIBERQTAPP
CONFIG          += qt warn_on release thread
#CONFIG		+= qt warn_on debug thread
DEPENDPATH += $$PWD/../libRobSock
INCLUDEPATH += $$PWD/../libRobSock

win32 {
    DEFINES += MicWindows
    CONFIG(release, debug|release): LIBS += -L$$PWD/../libRobSock/release/ -lRobSock
    CONFIG(debug, debug|release): LIBS += -L$$PWD/../libRobSock/debug/ -lRobSock
}

symbian {
    LIBS += -lRobSock
}

unix {
    LIBS += -L$$PWD/../libRobSock/ -lRobSock
}

HEADERS		= robview.h sampapp.h \ #keyhandler.h
    tactics.h
SOURCES		= main.cpp robview.cpp \
    tactics.cpp
