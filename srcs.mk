#relative path
src_dir ?= src
res_dir ?= res
exe_dir ?= .

exe ?= cohoma

srcs_sma ?= src/Dispatcher.sma src/Strip.sma src/Slider.sma src/CheckBox.sma src/Animator.sma src/TextLineEdit.sma \
            src/Button.sma src/PanAndZoom.sma src/StatusSelector.sma src/Waypoints.sma src/Node.sma src/Edge.sma src/ManagerId.sma \
            src/NavGraph.sma src/PixmapTile.sma src/MapLayer.sma src/Map.sma src/WidgetMap.sma \
            src/GraphPannel.sma src/RosManager.sma src/UpperLeftMenu.sma src/main.sma
# or
#srcs_sma := $(shell find $(src_dir) -name "*.sma")

# native sources
srcs_other ?= src/cpp/coords-utils.cpp src/cpp/map_move.cpp src/cpp/tiles_manager.cpp \
            src/ros_publisher.cpp  src/cpp/iojson.cpp\
            src/ros_node.cpp src/cpp/navgraph.cpp 
            #src/ros_subscriber.cpp src/graph_subscriber.cpp
        
# or
#srcs_other := $(shell find $(src_dir) -name "*.cpp")

# external libraries
# CXXFLAGS += $(shell pkg-config --cflags foo)
# or
# pkgdeps +=


ros_install_path := /opt/ros/galactic
ros_include_path := $(ros_install_path)/include
ros_lib_path := $(ros_install_path)/lib

ros_libs := $(shell ls $(ros_lib_path)/lib*.so 2>/dev/null | xargs echo)
ros_libs := $(filter-out $(ros_lib_path)/librmw_cyclonedds_cpp.so, $(ros_libs))
ros_libs := $(patsubst $(ros_lib_path)/lib%.so,-l%,$(ros_libs))

# Nico pourrais-tu essayer ceci vv à la place de cela ^^ stp?

# rclcpp_lib_deps := $(shell ldd $(ros_lib_path)/librclcpp.so | awk '{print $1}' | sed -e 's/.so.*//' | sed -e 's:/lib.*::'| sed -e 's/lib/-l/' | xargs echo)
# ros_libs := $(rclcpp_lib_deps)
# ros_libs := $(filter -l%,$(ros_libs))


#PATH_TO_WORKSPACE = /home/ubuntu/djnn_smala_install_debug/dev_ws
icare_interfaces_libs_install_path := $(PATH_TO_WORKSPACE)/install/icare_interfaces/lib
icare_libs := $(shell ls $(icare_interfaces_libs_install_path)/lib*.so 2>/dev/null | xargs echo)
icare_libs := $(patsubst $(icare_interfaces_libs_install_path)/lib%.so, -l%, $(icare_libs))

CXXFLAGS += -I./src -I./src/cpp -I./src/include
CXXFLAGS += -I$(ros_include_path)
CXXFLAGS += -I$(PATH_TO_WORKSPACE)/install/icare_interfaces/include \
            -I$(PATH_TO_WORKSPACE)/install/lemon/include

#LDFLAGS += 
LIBS += -lcurl
LIBS += -L$(ros_lib_path) $(ros_libs)
LIBS += -L$(icare_interfaces_libs_install_path) $(icare_libs) 




ld_library_path+=$(ros_lib_path):$(icare_interfaces_libs_install_path)
