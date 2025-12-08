C_SOURCES = camera_controller.c quirc/quirc.c quirc/decode.c quirc/identify.c quirc/version_db.c

INCLUDE = -Iquirc

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include