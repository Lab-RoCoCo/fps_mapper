ADD_LIBRARY(gl_helpers_library SHARED
  opengl_primitives.h opengl_primitives.cpp
  simple_viewer.h simple_viewer.cpp
)

TARGET_LINK_LIBRARIES(gl_helpers_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
)
