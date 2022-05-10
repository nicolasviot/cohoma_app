#relative path
src_dir ?= src
res_dir ?= res
exe_dir ?= .

exe ?= main

srcs_sma ?= src/main.sma
# or
#srcs_sma := $(shell find $(src_dir) -name "*.sma")

# native sources
#srcs_other ?= src/cpp/foo.cpp
