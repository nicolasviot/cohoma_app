
# ---------------------------------------
# precompiled headers

# https://stackoverflow.com/questions/58841/precompiled-headers-with-gcc
# https://stackoverflow.com/questions/26755219/how-to-use-pch-with-clang

ifeq ($(compiler),llvm)
pch_ext ?= .pch
CXXFLAGS_PCH_GEN += -fpch-instantiate-templates -fpch-codegen -fpch-debuginfo
endif
ifeq ($(compiler),gnu)
pch_ext ?= .gch
endif

pch_file ?= $(src_dir)/precompiled.h
#pch_src ?= src/$(pch_file)
pch_dst ?= $(build_dir)/$(pch_file)

# SDL and other stuff define new variables for compiling, canceling the use of pch with gnu cc
# FIXME this is not safe as every other external lib may define something
ifeq ($(compiler),gnu)
# https://gitlab.gnome.org/GNOME/gnome-online-accounts/-/merge_requests/14
# Both GCC and Clang appear to expand -pthread to define _REENTRANT on their own
CXXFLAGS_PCH_DEF += -D_REENTRANT
ifeq ($(display),SDL)
CXXFLAGS_PCH_DEF += -Dmain=SDL_main
endif
endif

$(build_dir)/%$(pch_ext): %
	@mkdir -p $(dir $@)
ifeq ($V,max)
	$(CXX) -x c++-header $(CXXFLAGS) $(CXXFLAGS_PCH_GEN) $< -o $@
else
	@$(call rule_message,compiling to,$(stylized_target))
	@$(CXX) -x c++-header $(CXXFLAGS) $(CXXFLAGS_PCH_GEN) $< -o $@
endif

ifeq ($(compiler),llvm)
CXXFLAGS_PCH_INC += -include-pch $(pch_dst)$(pch_ext)
endif
ifeq ($(compiler),gnu)
# https://stackoverflow.com/a/3164874
CXXFLAGS_PCH_INC += -I$(dir $(pch_dst)) -include $(notdir $(pch_file)) -Winvalid-pch
#-fno-implicit-templates
#$(build_dir)/src/core/utils/build/external_template.o: CXXFLAGS += -fimplicit-templates
endif

$(build_dir)/%$(pch_ext): override CXXFLAGS = $(CXXFLAGS_CFG) $(CXXFLAGS_PCH_DEF) $(djnn_cflags) $(CXXFLAGS_COMMON) $(CXXFLAGS_CK)

pch: $(pch_dst)
clean_pch:
	rm -f $(pch_dst)


$(objs): $(pch_dst)$(pch_ext)
