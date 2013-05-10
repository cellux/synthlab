VPATH = build

CPPFLAGS = -O2 -I../include
LDFLAGS = -ljack

headers := ../include/synthlab.hpp \
	$(wildcard ../include/synthlab/*.hpp) \
	$(wildcard ../include/synthlab/generators/*.hpp) \
	$(wildcard ../include/synthlab/audioproviders/*.hpp)

sources := $(wildcard *.cpp)

.PHONY: ALL
ALL: $(sources:.cpp=)

%: %.cpp $(headers)
	mkdir -p build
	$(CXX) $(CPPFLAGS) -o build/$@ $(LDFLAGS) $<

.PHONY: clean
clean:
	rm -f build/*
