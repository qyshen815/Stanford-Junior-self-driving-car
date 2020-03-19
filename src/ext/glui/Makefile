include ../../Makefile.conf

CC = g++
LINK = g++

CFLAGS +=  
CXXFLAGS = -O3
IFLAGS += 
LFLAGS += -lglut -lGL -lGLU -L/usr/X11R6/lib -lX11 -lXmu -lXext


MODULE_NAME = GLUI
SOURCES = glui_add_controls.cpp glui.cpp glui_bitmap_img_data.cpp \
		glui_bitmaps.cpp glui_button.cpp glui_edittext.cpp \
		glui_checkbox.cpp glui_node.cpp glui_radio.cpp \
		glui_statictext.cpp glui_panel.cpp glui_separator.cpp \
		glui_spinner.cpp glui_control.cpp glui_column.cpp \
		glui_translation.cpp glui_rotation.cpp  \
		glui_mouse_iaction.cpp glui_listbox.cpp glui_list.cpp \
		glui_rollout.cpp arcball.cpp algebra3.cpp quaternion.cpp \
		glui_commandline.cpp glui_filebrowser.cpp glui_scrollbar.cpp \
		glui_string.cpp glui_textbox.cpp glui_translation.cpp \
		glui_tree.cpp glui_treepanel.cpp glui_window.cpp


PUBLIC_INCLUDES = glui.h
PUBLIC_LIBRARIES = libglui.a
PUBLIC_BINARIES = 

TARGETS = libglui.a example1 example2 example3 example4 example5 example6

# rules

libglui.a:	glui_add_controls.o glui.o glui_bitmap_img_data.o \
		glui_bitmaps.o glui_button.o glui_edittext.o \
		glui_checkbox.o glui_node.o glui_radio.o glui_statictext.o \
		glui_panel.o glui_separator.o glui_spinner.o \
		glui_control.o glui_column.o glui_translation.o \
		glui_rotation.o glui_mouse_iaction.o glui_listbox.o \
		glui_list.o glui_rollout.o arcball.o algebra3.o quaternion.o \
		glui_commandline.o glui_filebrowser.o glui_scrollbar.o \
		glui_string.o glui_textbox.o glui_translation.o glui_tree.o \
		glui_treepanel.o glui_window.o

example1:	example1.o libglui.a

example2:	example2.o libglui.a

example3:	example3.o libglui.a

example4:	example4.o libglui.a

example5:	example5.o libglui.a

example6:	example6.o libglui.a

include ../../Makefile.rules
