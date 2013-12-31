

a game project based on robot simulator CyberMouse (sourceforge.net/projects/cpss)

It requires Qt library version 4.8 to compile

#Requirements

Qt 4, recommended 4.8, do not use Qt 5 as it is not backward compactible

#Compilation


run:
		qmake --version

and check if it uses Qt 4 or Qt 5, most recent linux versions use Qt 5 if its Qt 4
modify the line in the Makefile

		QMAKE = qmake-qt4

to

		QMAKE = qmake

run:

		make


