#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.2 2006/10/20 20:20:15 emarkus Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/apps/sample/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = astar-map.js
DBG_NAME = $(NAME)
REL_NAME = $(NAME)

ROOT = ../../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs_html/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs_html/$(NAME)/release
DBG_BINDIR = $(ROOT)/html/debug
REL_BINDIR = $(ROOT)/html/release

PROJ_CXXFLAGS = -I$(ROOT)/absmapalgorithms -I$(ROOT)/graphalgorithms -I$(ROOT)/guihtml -I$(ROOT)/gui -I$(ROOT)/simulation -I$(ROOT)/environments -I$(ROOT)/mapalgorithms -I$(ROOT)/algorithms -I$(ROOT)/generic -I$(ROOT)/utils -I$(ROOT)/graph -I$(ROOT)/search  -I$(ROOT)/grids 

PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS = -L$(DBG_BINDIR)
PROJ_REL_LNFLAGS = -L$(REL_BINDIR)

PROJ_DBG_LIB = -lenvironments -lmapalgorithms -lalgorithms -lgraphalgorithms -lgraph -lutils -lguihtml -lgui -lgrids -lSTUB
PROJ_REL_LIB = -lenvironments -lmapalgorithms -lalgorithms -lgraphalgorithms -lgraph -lutils -lguihtml -lgui -lgrids -lSTUB

PROJ_DBG_DEP = \
  $(DBG_BINDIR)/libutils.a \
  $(DBG_BINDIR)/libgraph.a \
  $(DBG_BINDIR)/libenvironments.a \
  $(DBG_BINDIR)/libmapalgorithms.a \
  $(DBG_BINDIR)/libgraphalgorithms.a \
  $(DBG_BINDIR)/libalgorithms.a \
  $(DBG_BINDIR)/libguihtml.a \
  $(DBG_BINDIR)/libgui.a \
  $(DBG_BINDIR)/libgrids.a \
  $(DBG_BINDIR)/libSTUB.a

PROJ_REL_DEP = \
  $(REL_BINDIR)/libutils.a \
  $(REL_BINDIR)/libgraph.a \
  $(REL_BINDIR)/libenvironments.a \
  $(REL_BINDIR)/libmapalgorithms.a \
  $(REL_BINDIR)/libgraphalgorithms.a \
  $(REL_BINDIR)/libalgorithms.a \
  $(REL_BINDIR)/libguihtml.a \
  $(REL_BINDIR)/libgui.a \
  $(REL_BINDIR)/libgrids.a \
  $(REL_BINDIR)/libSTUB.a

default : all

SRC_CPP = \
	demos/astar-map/Driver.cpp 
