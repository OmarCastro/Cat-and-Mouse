

all: makeSimulator makeViewer makeLogplayer makeLibRobSock makeGUISample makeRobsample

makeSimulator:
	(cd simulator; qmake-qt4 -makefile) 
	make -C simulator

makeViewer:
	(cd Viewer; qmake-qt4 -makefile) 
	make -C Viewer

makeLogplayer:
	(cd logplayer; qmake-qt4 -makefile) 
	make -C logplayer

makeLibRobSock:
	(cd libRobSock; qmake-qt4 -makefile) 
	make -C libRobSock

makeGUISample:
	(cd GUISample; qmake-qt4 -makefile) 
	make -C GUISample

makeRobsample:
	(cd mouseSample; qmake-qt4 -makefile) 
	make -C mouseSample

clean:
	make -C simulator clean
	make -C Viewer clean
	make -C logplayer clean
	make -C libRobSock clean
	make -C GUISample clean
	make -C mouseSample clean

distclean:
	make -C simulator distclean
	make -C Viewer distclean
	make -C logplayer distclean
	make -C libRobSock distclean
	make -C GUISample distclean
	make -C mouseSample distclean

