BUILD ?= debug
BUILD_DIR := .build
ifdef UPLOAD
  UPLOAD_FLAGS := -t upload
endif
ifdef PORT
  UPLOAD_FLAGS := $(UPLOAD_FLAGS) --upload-port $(PORT)
endif
ifdef VERBOSE
  NINJA_FLAGS := -v
endif
ifeq ($(BUILD),release)
  #
else ifeq ($(BUILD),debug)
  #
else
  $(error BUILD must be either 'debug' or 'release')
endif
CMAKE := cmake

ifeq ($(OS),Windows_NT)
  HOST_OS := Windows
else
  HOST_OS := $(shell uname)
endif

ifeq ($(HOST_OS),Windows)
  HOST_CPU := x86_64
  OUTPUT_TARGET = $(HOST_CPU)-unknown-windows-msvc
else ifeq ($(HOST_OS),Linux)
  HOST_CPU := $(shell uname -m)
  OUTPUT_TARGET := $(HOST_CPU)-unknown-linux-gnu
else ifeq ($(HOST_OS),Darwin)
  HOST_CPU := $(shell uname -m)
  OUTPUT_TARGET ?= $(HOST_CPU)-apple-macosx
else
  $(error Unsupported host platform)
endif
HOST_OUTPUT_BUILD_DIR := $(OUTPUT_TARGET)
HOST_OUTPUT_DIR := $(BUILD_DIR)/$(HOST_OUTPUT_BUILD_DIR)/$(BUILD)

ifdef TOOLCHAIN
  # Cross compiling
  ifneq (,$(wildcard $(TOOLCHAIN)))
    # We expect to find these variables in the toolchain file:
    #  TARGET_PLATFORM_NAME
    #  TARGET_PLATFORM_SDK
    #  TARGET_PLATFORM_VERSION
    #  TARGET_PLATFORM_ARCH
    OUTPUT_TARGET := $(shell sed -n -e 's/set(TOOLCHAIN_TARGET \([^)]*\).*/\1/p' $(TOOLCHAIN))
    TARGET_PLATFORM_SDK := $(shell sed -n -e 's/set(TARGET_PLATFORM_SDK \([^)]*\).*/\1/p' $(TOOLCHAIN))
    TARGET_PLATFORM_NAME := $(shell sed -n -e 's/set(TARGET_PLATFORM_NAME \([^)]*\).*/\1/p' $(TOOLCHAIN))
    TARGET_PLATFORM_VERSION := $(shell sed -n -e 's/set(TARGET_PLATFORM_VERSION \([^)]*\).*/\1/p' $(TOOLCHAIN))
    TARGET_PLATFORM_ARCH := $(shell sed -n -e 's/set(TOOLCHAIN_TARGET_CPU \([^)]*\).*/\1/p' $(TOOLCHAIN))
    TARGET_PLATFORM_CPU := $(shell sed -n -e 's/set(CMAKE_SYSTEM_PROCESSOR \([^)]*\).*/\1/p' $(TOOLCHAIN))
    TARGET_PLATFORM_NAME_SDK := $(shell echo $(TARGET_PLATFORM_NAME) | tr '[:upper:]' '[:lower:]').sdk
    OUTPUT_TARGET_SUFFIX := $(findstring -linux-gnu, $(OUTPUT_TARGET))
    LINUX_SUFFIX_CHECK := $(if $(filter %linux-gnu, $(OUTPUT_TARGET_SUFFIX)),-linux-gnu,)
    ifeq ($(LINUX_SUFFIX_CHECK),-linux-gnu)
      TARGET_PLATFORM_TYPE := Linux
    endif

    OUTPUT_TARGET_DIR := $(TARGET_PLATFORM_SDK)
    $(info TARGET_PLATFORM_NAME:    $(TARGET_PLATFORM_NAME))
    $(info TARGET_PLATFORM_SDK:     $(TARGET_PLATFORM_SDK))
    $(info TARGET_PLATFORM_VERSION: $(TARGET_PLATFORM_VERSION))
    $(info TARGET_PLATFORM_ARCH:    $(TARGET_PLATFORM_ARCH))
    $(info TARGET_PLATFORM_CPU:     $(TARGET_PLATFORM_CPU))
    OUTPUT_DIR := $(BUILD_DIR)/$(TARGET_PLATFORM_SDK)/$(BUILD)
    CMAKE_TOOLCHAIN := -DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN)
  else
    $(error No such toolchain: $(TOOLCHAIN))
  endif
else ifeq ($(HOST_OS),Linux)
  OUTPUT_DIR = $(HOST_OUTPUT_DIR)
else ifeq ($(HOST_OS),Darwin)
  OUTPUT_DIR = $(HOST_OUTPUT_DIR)
else ifeq ($(HOST_OS),Windows)
  OUTPUT_DIR = $(HOST_OUTPUT_DIR)
endif

all: $(OUTPUT_DIR)/CMakeCache.txt
	ninja -C $(OUTPUT_DIR) $(NINJA_FLAGS)

$(OUTPUT_DIR)/CMakeCache.txt:
	@$(CMAKE) -E make_directory $(OUTPUT_DIR)
	$(CMAKE) -B "$(OUTPUT_DIR)" $(CMAKE_TOOLCHAIN) -DCMAKE_BUILD_TYPE=$(BUILD) -G Ninja

.PHONY: quackhead documentation all

quackhead:
	@pio run $(NINJA_FLAGS) -e quackhead-esp32-v1 $(UPLOAD_FLAGS)

documentation:
	@doxygen docs/QuackHead.doxygen ; \
	 if [ $$? -eq 0 ]; then \
	 	git status docs/html | grep "Changes not staged" ; \
		if [ $$? -eq 0 ]; then \
		 	cd docs ; \
		 	rm -rf quackhead-doc html/quackhead-doc.tgz ; \
		 	mkdir -p quackhead-doc ; \
		 	rsync -au html index.html quackhead-doc ; \
		 	tar zcf html/quackhead-doc.tgz quackhead-doc/* ; \
		 	rm -rf quackhead-doc ; \
		fi \
	 fi
	@google-chrome docs/html/index.html&

clean:
	@$(CMAKE) -E rm -rf '.build/'
	@$(CMAKE) -E rm -rf '.pio/'
