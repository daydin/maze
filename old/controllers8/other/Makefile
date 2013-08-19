### Generic Makefile.include for Webots controllers, physics plugins, robot
### window libraries, remote control libraries and other libraries
### to be used with GNU make
###
### Platforms: Windows, Mac OS X, Linux
### Languages: C, C++, Java
###
### Authors: Olivier Michel, Yvan Bourquin, Fabien Rohrer
###          Edmund Ronald, Sergei Poskriakov
###
###-----------------------------------------------------------------------------
###
### This file is meant to be included from the Makefile files located in the
### Webots projects subdirectories. It is possible to set a number of variables
### to customize the build process, i.e., add source files, compilation flags,
### include paths, libraries, etc. These variables should be set in your local
### Makefile just before including this Makefile.include. This Makefile.include
### should never be modified.
###
### Here is a description of the variables you may set in your local Makefile:
###
### ---- C Sources ----
### if your program uses several C source files:
C_SOURCES = evolution-tut.c
###
### ---- C++ Sources ----
### if your program uses several C++ source files:
### CXX_SOURCES = my_plugin.cc my_clever_algo.cpp my_graphics.c++
###
### ---- Compilation options ----
### if special compilation flags are necessary:
CFLAGS = -Wall
###
### ---- Linked libraries ----
### if your program needs additional libraries:
INCLUDE = -I"C:/Webots/mingw/include" -I"C:/Webots/mingw/lib/gcc/mingw32/3.4.2/include" -I"C:/Webots/include -I ../glib-dev-2.14.5/include" -I "../glib-dev-2.14.5/include/glib-2.0" -I "../glib-dev-2.14.5/lib/glib-2.0/include" -DWIN32
LIBRARIES = -L"/path/to/my/library" -lmy_library -lmy_other_library
###
### --- Webots included libraries ---
### In your C++ program, if you want to use the C API, add
### USE_C_API = true
### or if you want to link with the Qt framework embedded in Webots:
### USE_QT = true
###
### ----- Java -----
### if your program uses several C source files:
### JAVA_SOURCES = my_plugin.java my_clever_algo.java
###
### if your Java controller needs additional libraries, you should define
### the CLASSPATH environment variable as explained in the Webots User Guide
### CLASSPATH=relative/mylib.jar
###
###-----------------------------------------------------------------------------

### Do not modify: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

