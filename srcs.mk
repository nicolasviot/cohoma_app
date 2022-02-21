#relative path
src_dir := src
res_dir := res
exe_dir := .

exe := button

srcs_sma := src/Dispatcher.sma src/Strip.sma src/Slider.sma src/Animator.sma src/TextLineEdit.sma src/PanAndZoom.sma src/Waypoints.sma src/PixmapTile.sma src/MapLayer.sma src/Map.sma src/WidgetMap.sma src/main.sma
# or
#srcs_sma := $(shell find $(src_dir) -name "*.sma")

# native sources
srcs_other := src/cpp/coords-utils.cpp src/cpp/map_move.cpp src/cpp/tiles_manager.cpp src/ros_publisher.cpp src/ros_subscriber.cpp
# or
#srcs_other := $(shell find $(src_dir) -name "*.cpp")


CXXFLAGS += -std=c++17

# external libraries
# CXXFLAGS += $(shell pkg-config --cflags foo)
CXXFLAGS += -I/opt/ros/galactic/include -I./src -I./src/cpp -I/home/ubuntu/djnn_smala_install_debug/dev_ws/install/icare_interfaces/include

ros_libs_install_path ?= /opt/ros/galactic/lib
ros_libs := $(shell ls $(ros_libs_install_path)/lib*.so | xargs echo)
ros_libs := $(filter-out $(ros_libs_install_path)/librmw_cyclonedds_cpp.so, $(ros_libs))
ros_libs := $(patsubst $(ros_libs_install_path)/lib%.so,-l%,$(ros_libs))
icare_interfaces_libs_install_path := /home/ubuntu/djnn_smala_install_debug/dev_ws/install/icare_interfaces/lib
icare_libs := $(shell ls $(icare_interfaces_libs_install_path)/lib*.so | xargs echo)
icare_libs := $(patsubst $(icare_interfaces_libs_install_path)/lib%.so, -l%, $(icare_libs))

#LDFLAGS += 
LIBS += -lcurl
LIBS += -L$(ros_libs_install_path) $(ros_libs)
LIBS += -L$(icare_interfaces_libs_install_path) $(icare_libs) 
# or
# pkgdeps +=

# Joins elements of the list in arg 2 with the given separator.
#   1. Element separator.
#   2. The list.
join-with = $(subst $(space),$1,$(strip $2))


ld_library_path+=$(ros_libs_install_path):$(icare_interfaces_libs_install_path)
