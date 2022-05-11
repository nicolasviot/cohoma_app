#relative path
src_dir ?= src
res_dir ?= res
exe_dir ?= .

exe ?= cohoma

srcs_sma ?= src/ClockComponent.sma src/Reticule.sma src/Strip.sma src/Slider.sma src/CheckBox.sma src/Animator.sma src/TextLineEdit.sma \
            src/Button.sma src/PanAndZoom.sma src/StatusSelector.sma src/GraphNode.sma src/Trap.sma src/TrapLayer.sma src/Node.sma src/Edge.sma src/ManagerId.sma \
            src/SafetyPilot.sma src/TaskAreaSummit.sma src/TaskArea.sma src/Lima.sma src/LimaLayer.sma src/TaskLayer.sma \
            src/TaskEdge.sma src/TaskTrap.sma src/TaskAreaSummit.sma src/TaskArea.sma src/Lima.sma src/LimaLayer.sma src/ExclusionArea.sma src/ExclusionAreaLayer.sma src/SiteLayer.sma src/TaskLayer.sma \
            src/NavGraph.sma src/Itineraries.sma src/PixmapTile.sma src/MapLayer.sma src/Map.sma src/WidgetMap.sma \
            src/TaskDescriptor.sma src/CandidateTaskFilter.sma \
            src/ItineraryDescriptor.sma src/ItineraryPannel.sma src/CompteRendu.sma src/RightPannel.sma src/RosManager.sma \
            src/Vehicule.sma src/UpperLeftMenu.sma src/StripContainer.sma src/main.sma
# or
#srcs_sma := $(shell find $(src_dir) -name "*.sma")

# native sources
srcs_other ?= src/cpp/coords-utils.cpp src/cpp/map_move.cpp src/cpp/tiles_manager.cpp \
             src/ros_node.cpp\

#build/src/ros_node.o: CXXFLAGS+=-fno-implicit-templates

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
ros_libs := $(patsubst $(ros_lib_path)/lib%.so,-l%,$(ros_libs))

ros_x86_libs := $(shell ls $(ros_lib_path)/x86_64-linux-gnu/lib*.so 2>/dev/null | xargs echo)
#ros_libs := $(filter-out $(ros_lib_path)/librmw_cyclonedds_cpp.so, $(ros_libs))
# Nico pourrais-tu essayer ceci vv à la place de cela ^^ stp?

# rclcpp_lib_deps := $(shell ldd $(ros_lib_path)/librclcpp.so | awk '{print $1}' | sed -e 's/.so.*//' | sed -e 's:/lib.*::'| sed -e 's/lib/-l/' | xargs echo)
# ros_libs := $(rclcpp_lib_deps)
# ros_libs := $(filter -l%,$(ros_libs))


PATH_TO_WORKSPACE = /home/lii/Documents/dev_ws
icare_interfaces_libs_install_path := $(PATH_TO_WORKSPACE)/install/icare_interfaces/lib
icare_libs := $(shell ls $(icare_interfaces_libs_install_path)/lib*.so 2>/dev/null | xargs echo)
icare_libs := $(patsubst $(icare_interfaces_libs_install_path)/lib%.so, -l%, $(icare_libs))

CXXFLAGS += -I./src -I./src/cpp -I./src/include
CXXFLAGS += -I$(ros_include_path)
CXXFLAGS += -I$(PATH_TO_WORKSPACE)/install/icare_interfaces/include \
            -I$(PATH_TO_WORKSPACE)/install/lemon/include

#LDFLAGS += 
LIBS += -lcurl
LIBS += -L$(ros_lib_path) $(ros_libs) -L$(ros_lib_path)/x86_64-linux-gnu $(ros_x86_libs)
LIBS += -L$(icare_interfaces_libs_install_path) $(icare_libs) 


ld_library_path+=$(ros_lib_path):$(ros_lib_path)/x86_64-linux-gnu:$(icare_interfaces_libs_install_path)
