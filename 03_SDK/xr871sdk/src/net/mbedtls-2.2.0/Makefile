# Rules for building library

################################################################################
ROOT_PATH = ../../..

# Include the common make definitions.
include $(ROOT_PATH)/gcc.mk

################################################################################
LIBS = libmbedtls.a

#DIRS_IGNORE := ./programs
DIRS_ALL := $(shell find ./ -type d | grep -v programs)
DIRS := $(filter-out $(DIRS_IGNORE),$(DIRS_ALL))

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.c)))
OBJS := $(addsuffix .o,$(SRCS))
INCLUDE_PATHS += -I$(ROOT_PATH)/include/net/mbedtls/include \
	-I$(ROOT_PATH)/include/net/mbedtls/configs 


CC_FLAGS += -Werror
CC_FLAGS += -Wno-address	# suppress memory address compare warnnings
CC_FLAGS += -Wno-format	# suppress printf/scanf format warnnings

#CC_FLAGS +=  -DMBEDTLS_CONFIG_FILE='<config-xr-mini.h>'
#CC_FLAGS +=  -DMBEDTLS_CONFIG_FILE='<config-xr-mini-serv.h>'
CC_FLAGS +=  -DMBEDTLS_CONFIG_FILE='<config-xr-mini-cli.h>'
#CC_FLAGS +=  -DMBEDTLS_CONFIG_FILE='<config-xr-mini-cliserv.h>'
################################################################################
.PHONY: all install size clean

all: $(LIBS)

$(LIBS): $(OBJS)
	$(AR) -crs $@ $^

install: $(LIBS)
	$(CP) -t $(ROOT_PATH)/lib $^

size:
	$(SIZE) -t $(LIBS)

clean:
	-rm -f $(LIBS) $(OBJS) $(DEPS)


################################################################################
DEPS = $(OBJS:.o=.d)
-include $(DEPS)
