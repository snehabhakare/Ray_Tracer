CC=g++ 
flag=-std=c++11

OPENGLLIB= -lGL
GLULIB= -lGLU
GLUTLIB=-lglut
THREAD=-pthread
LIBS=$(OPENGLLIB) $(GLULIB) $(GLUTLIB) $(THREAD)
LDFLAGS=-L/usr/local/lib 
BIN=raytracer
SRC=main.cpp
INCLUDE=include/
LINK=tinyxml/tinyxml.h tinyxml/tinyxml.cpp tinyxml/tinystr.h tinyxml/tinystr.cpp tinyxml/tinyxmlerror.cpp tinyxml/tinyxmlparser.cpp include/lodepng.cpp

all: $(BIN)

$(BIN): $(SRC)
	g++ $(flag) -I $(INCLUDE) $(SRC) $(LINK) -o $(BIN) $(LDFLAGS) $(LIBS)

clean:
	rm -f *~ *.o $(BIN)
