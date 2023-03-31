#relative path
src_dir ?= src
res_dir ?= res
exe_dir ?= .

exe ?= cohoma

# find src -name "*.sma" -path "src/widgets/impl" | xargs echo

srcs_sma ?= src/widgets/CheckBox.sma src/widgets/Slider.sma src/widgets/Button.sma src/widgets/CLabel.sma \
            src/widgets/impl/scrollbar/clamp.sma src/widgets/impl/scrollbar/inverse_transform.sma src/widgets/impl/scrollbar/paging.sma src/widgets/Scrollbar.sma \
            src/widgets/chat/Bubble.sma src/widgets/chat/VBoxTranslation.sma src/widgets/chat/Chat.sma \
            src/CohomaContext.sma \
            src/model/PointModel.sma src/model/ExclusionZoneModel.sma src/model/LimaModel.sma \
            src/model/NodeModel.sma src/model/EdgeModel.sma src/model/ItineraryModel.sma \
            src/model/TrapModel.sma \
            src/model/task/TaskEdgeModel.sma src/model/task/TaskAreaModel.sma src/model/task/TaskTrapModel.sma \
            src/model/team/OperatorModel.sma src/model/team/SafetyPilotModel.sma src/model/team/VehicleModel.sma \
            src/model/SubLayerModel.sma src/model/ModelManager.sma src/model/NoRosModelManager.sma \
            src/behavior/NotDraggableItem.sma src/behavior/DraggableItem.sma src/behavior/DraggableItemWithRadius.sma src/behavior/NotDraggableItemWithRadius.sma \
            src/SubLayer.sma src/SubLayerVisibilityMap.sma \
            src/Console.sma src/Reticule.sma \
            src/graph/NodeStatusSelector.sma src/graph/Edge.sma src/graph/Node.sma src/graph/SubLayerNavigGraph.sma src/graph/CGraph.sma \
            src/trap/TrapForm.sma src/trap/TrapStatusSelector.sma src/trap/Trap.sma src/trap/SubLayerTraps.sma \
            src/task/TaskArea.sma src/task/TaskEdge.sma src/task/TaskTrap.sma src/task/SubLayerTasks.sma \
            src/site/ExclusionArea.sma src/site/Lima.sma src/site/SubLayerSite.sma \
            src/itinerary/ItineraryEdge.sma src/itinerary/ItineraryOnMap.sma src/itinerary/SubLayerItineraries.sma \
            src/itinerary/ItineraryStrip.sma src/itinerary/ItineraryPanel.sma \
            src/map/PanAndZoom.sma src/map/PixmapTile.sma src/map/MapLayer.sma src/map/Map.sma \
            src/RosManager.sma \
            src/movable/Vehicle.sma src/movable/SubLayerVehicles.sma src/movable/SafetyPilot.sma src/movable/SubLayerSafetyPilots.sma \
            src/left_panel/Strip.sma src/left_panel/Operator.sma src/left_panel/OperatorsList.sma src/left_panel/LeftPanel.sma \
            src/panels/PositionedItemSelector.sma src/panels/TopBar.sma src/panels/RightPanel.sma src/panels/UpperLeftMenu.sma \
            src/main.sma
# or
#srcs_sma := $(shell find $(src_dir) -name "*.sma")

# native sources
srcs_other ?= src/cpp/coords-utils.cpp \
            src/cpp/tiles_manager.cpp \
            src/ros_node.cpp
srcs_other += src/cpp/map_move.cpp

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

ifneq ($(no_ros),1)
ros_install_path ?= /opt/ros/humble

ros_include_path := $(ros_install_path)/include
ros_include_paths := $(shell find $(ros_include_path) -maxdepth 1 -type d)
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
endif

PATH_TO_WORKSPACE = /home/lii/Documents/cohoma2_dev_ws
#PATH_TO_WORKSPACE = /home/achil/dev/COHOMA

icare_interfaces_libs_install_path := $(PATH_TO_WORKSPACE)/install/icare_interfaces/lib
icare_libs := $(shell ls $(icare_interfaces_libs_install_path)/lib*.so 2>/dev/null | xargs echo)
icare_libs := $(patsubst $(icare_interfaces_libs_install_path)/lib%.so, -l%, $(icare_libs))

ifeq ($(os),Linux)
boost_cflags ?= #-I/usr/local/include/boost/
endif

ifeq ($(os),Darwin)
boost_cflags ?= #-I$(shell brew --prefix)/include
boost_ldflags ?= #-L$(shell brew --prefix)/lib
endif

CXXFLAGS += -std=c++20

CXXFLAGS += -I./src -I./src/cpp -I./src/include
CXXFLAGS += $(boost_cflags)
CXXFLAGS += $(addprefix -I,$(ros_include_paths))
CXXFLAGS += -I$(PATH_TO_WORKSPACE)/install/icare_interfaces/include/icare_interfaces \
            -I$(PATH_TO_WORKSPACE)/install/lemon/include


pkg += libcurl

LIBS += $(boost_ldflags)

ifeq ($(no_ros),1)
CXXFLAGS += -DNO_ROS -DNO_LEMON
else
LIBS += -L$(ros_lib_path) $(ros_libs) -L$(ros_lib_path)/x86_64-linux-gnu $(ros_x86_libs)
LIBS += -L$(icare_interfaces_libs_install_path) $(icare_libs)
ld_library_path+=$(ros_lib_path):$(ros_lib_path)/x86_64-linux-gnu:$(icare_interfaces_libs_install_path)
endif

ifeq ($(os),Linux)
LIBS += #-lboost_thread -lpthread
endif
ifeq ($(os),Darwin)
LIBS += #-lboost_thread-mt
endif

# external libraries
# CXXFLAGS += $(shell pkg-config --cflags foo)
# or
# pkgdeps +=

#pkgdeps += libcurl # already in djnn
pkgdeps += boost
