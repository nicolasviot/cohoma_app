build_dir ?= build

# release and installed (brew, apt, pacman)
djnn-pkgconf ?= djnn-cpp
smala-pkgconf ?= smala
# devel version : djnn-cpp-dev and smala-dev

# or local sources, uncomment _all_ following lines
#djnn-pkgconf :=
#smala-pkgconf :=
#djnn_cpp_path ?= ../djnn-cpp
#smala_path ?= ../smala

#CXXFLAGS += -std=c++20
CXXFLAGS += -std=c++17	
CXXFLAGS += -MMD
#CXXFLAGS += -Wall
CXXFLAGS += -g
CXXFLAGS += -O0
SMALACFLAGS += -g

#CXXFLAGS += -fsanitize=address
#LDFLAGS += -fsanitize=address
#CXXFLAGS += -fsanitize=thread
#LDFLAGS += -fsanitize=thread

# no_ros := 0 if you happen to have ROS installed...
no_ros := 1
