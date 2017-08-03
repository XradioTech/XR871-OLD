#
# Rules for building library
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# library and objects
# ----------------------------------------------------------------------------
LIBS := libcedarx.a

.PHONY: all install 

all: 
	echo "CedarX is already placed in ${ROOT_PATH}"

install:
	echo "CedarX is already placed in ${ROOT_PATH}"

size:
	$(Q)$(SIZE) -t $(LIBS)

clean:
	$(Q)-rm -f $(LIBS) $(OBJS) $(DEPS)

install_clean:
	$(Q)-rm -f $(INSTALL_PATH)/$(LIBS)
# ----------------------------------------------------------------------------
# dependent rules
# ----------------------------------------------------------------------------
DEPS = $(OBJS:.o=.d)
-include $(DEPS)
