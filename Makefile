.PHONY : clean

CC = g++
CFLAGS = -std=c++11

all: scripts/auto_nav/libnavMapProc.so

scripts/auto_nav/libnavMapProc.so : src/navMapProc.o
	$(CC) $(CFLAGS) -shared -Wl,-soname,libnavMapProc.so -o $@ $^

src/navMapProc.o : src/navMapProc.cpp
	$(CC) $(CFLAGS) -c -fPIC -o $@ $^

clean :
	-rm -vf scripts/auto_nav/libnavMapProc.so src/navMapProc.o scripts/auto_nav/navMapProc_wrp.pyc
