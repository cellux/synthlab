VPATH = build

CPPFLAGS = -O2 -I../include
LDFLAGS = -ljack

sources := $(wildcard *.cpp)

.PHONY: ALL
ALL: $(sources:.cpp=)

%: %.cpp ../include/synthlab.hpp
	mkdir -p build
	$(CXX) $(CPPFLAGS) -o build/$@ $(LDFLAGS) $<

.PHONY: clean
clean:
	rm -f build/*
