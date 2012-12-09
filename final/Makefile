# GDIR define
include Makefile.ext

# params
CC=gcc
CXX=g++
EXTRADEFS=-DLINUX_OS $(CE)
WFLAGSC=-Wall -Wunused -Wshadow
WFLAGSCPP=-Wall -Wunused -Wshadow -Weffc++
WFLAGSDCPP=

# Flags for DEVELOPEMENT phase
CFLAGS=-O0 -g3 $(WFLAGSC) $(INCLUDES) $(EXTRADEFS)
CPPFLAGS=-O0 -g3 $(WFLAGSCPP) $(INCLUDES) $(EXTRADEFS) `xml2-config --cflags`
DCPPFLAGS=-O0 -g3 $(WFLAGSDCPP) $(INCLUDES) $(EXTRADEFS)

LD=g++
LIBS=-lrt -lpthread -lssl -ljpeg -lpng -lSDL_gfx -lSDL_image -larmadillo \
		 -lSerialLinkClient -lGIMI -lGIMutils -lMaCI -lFSR2011OrderSystem \
			`xml2-config --libs` `sdl-config  --libs`


# Define COMPILE and LINK commands
COMPILEC=$(CC) $(CFLAGS) -c
COMPILECPP=$(CXX) $(CPPFLAGS) -c
COMPILEDCPP=$(CXX) $(DCPPFLAGS) -c
LINK=$(LD) $(LDFLAGS)

.PHONY: all

# Define targets
TARGETS=J2B2-UI-example J2B2-UI-example-simple J2B2-UI-example-OrderSystem

# All rule compiles all per default
all:	$(TARGETS)

###############################################
###############################################
###############################################
# J2B2-UI-example target, and dependencies
J2B2-UI-EXAMPLE_OBJECTS=J2B2-UI-example.opp J2B2Demo.opp astar/node.opp astar/pathplan2.opp odoEKF/odoEKF.opp ../simpleSLAM/slam.opp ../camera/camera.opp
J2B2-UI-example: GIMlibs $(J2B2-UI-EXAMPLE_OBJECTS)
	@$(LINK) -o $@ $(J2B2-UI-EXAMPLE_OBJECTS) $(LIBS) -lSDL -lSDL_gfx -lSDL_image
	@echo "      [LD] $@"
###############################################
###############################################
J2B2-UI-EXAMPLE-SIMPLE_OBJECTS=J2B2-UI-example-simple.opp
J2B2-UI-example-simple: GIMlibs $(J2B2-UI-EXAMPLE-SIMPLE_OBJECTS)
	@$(LINK) -o $@ $(J2B2-UI-EXAMPLE-SIMPLE_OBJECTS) $(LIBS)
	@echo "      [LD] $@"
###############################################
J2B2-UI-EXAMPLE-ORDERSYSTEM_OBJECTS=J2B2-UI-example-OrderSystem.opp
J2B2-UI-example-OrderSystem: GIMlibs $(J2B2-UI-EXAMPLE-ORDERSYSTEM_OBJECTS)
	@$(LINK) -o $@ $(J2B2-UI-EXAMPLE-ORDERSYSTEM_OBJECTS) $(LIBS)
	@echo "      [LD] $@"
###############################################
###############################################
###############################################
###############################################
PHONY: libGIMI.a libMaCI.a libGIMutils.a libSerialLinkClient.a libFSR2011OrderSystem.a
libGIMI.a:
	+@if [ -d $(GDIR)/GIMI ]; then cd $(GDIR)/GIMI; make libGIMI.a-dist; fi
libMaCI.a:
	+@if [ -d $(GDIR)/MaCI ]; then cd $(GDIR)/MaCI; make libMaCI.a-dist; fi
libGIMutils.a:
	+@if [ -d $(GDIR)/utils ]; then cd $(GDIR)/utils; make libGIMutils.a-dist; fi
libSerialLinkClient.a:
	+@if [ -d $(GDIR)/GIMnetApps/SerialLink ]; then cd $(GDIR)/GIMnetApps/SerialLink; make CE="-O2" libSerialLinkClient.a; fi
libFSR2011OrderSystem.a:
	+@if [ -d $(GDIR)/GIMnetApps/FSR2011-OrderSystem ]; then cd $(GDIR)/GIMnetApps/FSR2011-OrderSystem; make CE="-O2" libFSR2011OrderSystem.a-dist; fi
.PHONY: GIMlibs
GIMlibs: libGIMI.a libMaCI.a libGIMutils.a libSerialLinkClient.a libFSR2011OrderSystem.a
###############################################
.PHONY: clean
clean:
	@rm -f *.o *.opp *.dopp $(TARGETS) astar/*.opp odoEKF/*.opp ../simpleSLAM/*.opp ../camera/*.opp
	@echo "     [LOCAL-CLEAN]"

########### Compile rules ####################
# Compile rule for all .o (.c) file
%.o:	%.c
	@$(COMPILEC) -o $@ $<
	@echo "      [CC] $<"

# Compile rule for all .opp (.cpp) files
%.opp:	%.cpp
	@$(COMPILECPP) -o $@ $<
	@echo "      [CXX] $<"

# Compile rule for all .dopp (dirty .cpp) files
%.dopp:	%.cpp
	@$(COMPILEDCPP) -o $@ $<
	@echo "      [DIRTY-CXX] $<"

##############################################
