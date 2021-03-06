QMAKE = qmake-qt4


all: makeSimulator makeViewer makeLogplayer makeLibRobSock makeAI_code

makeSimulator:
	(cd simulator; $(QMAKE) -makefile) 
	make -C simulator

makeViewer:
	(cd Viewer; $(QMAKE) -makefile) 
	make -C Viewer

makeLogplayer:
	(cd logplayer; $(QMAKE) -makefile) 
	make -C logplayer

makeLibRobSock:
	(cd libRobSock; $(QMAKE) -makefile) 
	make -C libRobSock

makeAI_code:
	(cd AI_code; $(QMAKE) -makefile) 
	make -C AI_code


clean:
	make -C simulator clean
	make -C Viewer clean
	make -C logplayer clean
	make -C libRobSock clean
	make -C AI_code clean

distclean:
	make -C simulator distclean
	make -C Viewer distclean
	make -C logplayer distclean
	make -C libRobSock distclean
	make -C AI_code distclean

