
#----------------------------

$(objs): CXXFLAGS += $(CXXFLAGS_PCH_DEF) $(CXXFLAGS_PCH_INC) $(djnn_cflags) $(smala_cflags) -I$(src_dir) -I$(build_dir)/$(src_dir) -I$(build_dir)/lib\
	$(CXXFLAGS_COMMON) $(CXXFLAGS_CK)

#$(CXXFLAGS_CFG)


$(objs): $(pch_dst)

$(exe): LDFLAGS += $(djnn_ldflags) $(smala_ldflags)
$(exe): LIBS += $(app_libs)

$(exe): $(objs)
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(LD) $^ -o $@ $(LDFLAGS) $(LIBS)
else
	@$(call rule_message,linking to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(LD) $^ -o $@ $(LDFLAGS) $(LIBS)
endif

# .sma to .cpp, .c etc
$(build_dir)/%.cpp $(build_dir)/%.h: %.sma
ifeq ($V,max)
	@mkdir -p $(dir $@)
	@$(smalac) -cpp $^ -builddir $(dir $@)
else
	@$(call rule_message,compiling to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(smalac) -cpp $^ -builddir $(dir $@)
endif


# from .cpp user sources
$(build_dir)/%.o: %.cpp
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@
else
	@$(call rule_message,compiling to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) -c $< -o $@
endif


# for .cpp generated sources
$(build_dir)/%.o: $(build_dir)/%.cpp
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@
else
	@$(call rule_message,compiling to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) -c $< -o $@
endif


# dependencies
CXXFLAGS += -MMD

deps := $(objs:.o=.d)
ifneq ($(nodeps),1)
-include $(deps)
endif
