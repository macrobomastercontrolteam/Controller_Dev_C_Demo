.DEFAULT_GOAL := all
PROJECTS := $(patsubst %/Makefile,%,$(wildcard */Makefile))
PROGRAMMER ?= STM32_Programmer_CLI
ifeq ($(OS),Windows_NT)
SHELL := cmd.exe
.SHELLFLAGS := /c
endif

.PHONY: all clean list probe $(PROJECTS) $(addsuffix -clean,$(PROJECTS))
all: $(PROJECTS)

$(PROJECTS):
	+$(MAKE) -C "$@" all

list:
	@echo $(PROJECTS)

clean: $(addsuffix -clean,$(PROJECTS))

$(addsuffix -clean,$(PROJECTS)):
	+$(MAKE) -C "$(patsubst %-clean,%,$@)" clean

# Enumerate probes only: no target halt, reset, erase, or programming.
probe:
	"$(PROGRAMMER)" -l stlink
