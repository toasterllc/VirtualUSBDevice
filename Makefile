NAME=VirtualUSBDevice
OBJECTS=src/main.o

CXX      = g++
CXXFLAGS = -O0 -g3 -Wall -std=c++17 -iquote Lib
LFLAGS   = -ludev -lpthread

all: ${OBJECTS}
	$(CXX) $(CXXFLAGS) $? -o $(NAME) $(LFLAGS)

clean:
	rm -Rf Src/*.o $(NAME)
