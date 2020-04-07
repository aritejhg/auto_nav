.PHONY : clean

all: scripts/auto_nav/libnavMapProc.so

scripts/auto_nav/libnavMapProc.so : src/navMapProc.o
	gcc -shared -Wl,-soname,libnavMapProc.so -o scripts/auto_nav/libnavMapProc.so src/navMapProc.o

src/navMapProc.o : src/navMapProc.c
	gcc -c -fPIC src/navMapProc.c -o src/navMapProc.o

clean :
	-rm -vf scripts/auto_nav/libnavMapProc.so src/navMapProc.o