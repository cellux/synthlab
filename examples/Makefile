VPATH = build

CPPFLAGS = -O3 -I../include
LDFLAGS = -ljack

sources := $(wildcard *.cpp)

.PHONY: ALL
ALL: $(sources:.cpp=)

%: %.cpp
	$(CXX) $(CPPFLAGS) -o build/$@ $(LDFLAGS) $<
