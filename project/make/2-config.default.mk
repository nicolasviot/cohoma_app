exe ?= stand_alone

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

config.mk:
	cp $(project_dir)/1-config.default.mk config.mk
