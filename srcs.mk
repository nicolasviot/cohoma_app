#relative path
src_dir ?= src
res_dir ?= res
exe_dir ?= .

exe ?= cohoma

srcs_sma ?= src/widgets/CheckBox.sma src/widgets/Slider.sma src/widgets/Button.sma \
            src/CohomaContext.sma \
            src/model/NodeModel.sma src/model/VehiculeModel.sma src/model/SafetyPilotModel.sma src/model/TrapModel.sma src/model/ModelManager.sma \
            src/behavior/NotDraggableItem.sma src/behavior/DraggableItem.sma src/behavior/DraggableItemWithRadius.sma \
            src/ClockComponent.sma src/Reticule.sma \
            src/strip/Strip.sma src/strip/StripContainer.sma \
            src/graph/NodeStatusSelector.sma src/graph/Edge.sma src/graph/GraphNode.sma src/graph/Node.sma src/graph/NavGraph.sma \
            src/trap/TrapStatusSelector.sma src/trap/Trap.sma \
            src/task/TaskAreaSummit.sma src/task/TaskArea.sma src/task/TaskEdge.sma src/task/TaskTrap.sma \
            src/task/TaskLayer.sma src/trap/TrapLayer.sma \
            src/site/Lima.sma src/site/ExclusionArea.sma src/site/SiteLayer.sma \
            src/itinerary/Itineraries.sma src/itinerary/ItineraryDescriptor.sma src/itinerary/ItineraryPannel.sma \
            src/map/PanAndZoom.sma src/map/EnvMapLayer.sma src/map/PixmapTile.sma src/map/MapLayer.sma src/map/MapLayerSync.sma src/map/Map.sma \
            src/Console.sma src/RosManager.sma \
            src/Vehicule.sma src/SafetyPilot.sma \
            src/menu/RightPannel.sma src/menu/UpperLeftMenu.sma \
            src/main.sma
# or
#srcs_sma := $(shell find $(src_dir) -name "*.sma")

# native sources
srcs_other ?= src/cpp/coords-utils.cpp \
            src/cpp/tiles_manager.cpp \
            src/ros_node.cpp
srcs_other += src/cpp/map_move.cpp
srcs_other += src/cpp/map_move_sync.cpp

# or
#srcs_other := $(shell find $(src_dir) -name "*.cpp")


# ros-specific precoompiled header
ifeq ($(compiler),llvm)
pch_ext ?= .pch
endif
ifeq ($(compiler),gnu)
pch_ext ?= .gch
endif

$(build_dir)/src/ros_node.o: pch_file=src/ros_precompiled.h
$(build_dir)/src/ros_node.o: $(build_dir)/src/ros_precompiled.h$(pch_ext)


ros_install_path := /opt/ros/galactic
ros_include_path := $(ros_install_path)/include
ros_lib_path := $(ros_install_path)/lib

ros_libs := $(shell ls $(ros_lib_path)/lib*.so 2>/dev/null | xargs echo)
ros_libs := $(filter-out $(ros_lib_path)/librviz_common.so, $(ros_libs))
ros_libs := $(filter-out $(ros_lib_path)/librviz_rendering.so, $(ros_libs))
ros_libs := $(filter-out $(ros_lib_path)/librviz_default_plugins.so, $(ros_libs))

ros_libs := $(patsubst $(ros_lib_path)/lib%.so,-l%,$(ros_libs))

ros_x86_libs := $(shell ls $(ros_lib_path)/x86_64-linux-gnu/lib*.so 2>/dev/null | xargs echo)

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


# external libraries
# CXXFLAGS += $(shell pkg-config --cflags foo)
# or
# pkgdeps +=

