# --------------------------------------------------------
# Makefile for HW5 - OpenStreetMap (updated assignment)
# --------------------------------------------------------

CXX       := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -g -fsanitize=address

# Source files (must all be in the same directory as this Makefile)
SRCS := Edge.cpp \
        Vertex.cpp \
        Graph.cpp \
        HW5.cpp \
        tinyxml2.cpp

# Produce executable named "hw5"
TARGET := hw5

OBJS := $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile each .cpp → .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
