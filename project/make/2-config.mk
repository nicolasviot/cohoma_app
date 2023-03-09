# include user-specified config if present
-include config.mk

# default config, which should only use ?= (set a variable if it has not been set by the user-modified config.mk), because we don't want to replace user's configuration
include $(project_dir)/config.default.mk


config.mk:
	cp $(project_dir)/config.default.mk config.mk
