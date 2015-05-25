#pragma once
#include <qglviewer.h>
#include <qevent.h>

namespace GLHelpers {

/** Simple viewer class, derived fro QGLViewer.
    Allows to display a simple OpenGL scene through the
    overriding of the draw() method.
    It offers some basic frame manipulation via the mouse.
    It also captures the keyboard events and allows to inspect the last key pressed.
    <br>
    The typical usage is in an unrolled app loop<br>
    QApplication app;<br>
    SimpleViewer v;<br>
    v.show();<br>
    ..<br>
    ..<br>
    while(1){<br>
    app.processEvents();<br>
      QKeyEvent* e = v.lastKeyEvent();<br>
      if (v.lastKeyEvent() == something){<br>
         do something;<br>
      }<br>
      v.keyEventProcessed();<br>
    }<br>
 */
  class SimpleViewer: public QGLViewer{
  public:
    //! ctor
    SimpleViewer();
    //! init method, opens the gl viewport and sets the key bindings
    void init();
    //! callback invoked by the application on new key event. It saves the last event in
    //! a member variable
    virtual void keyPressEvent(QKeyEvent *e);

    //! returns the last key pressed since invoking keyEventProcessed();
    QKeyEvent* lastKeyEvent();

    //! call this to clear the events, after processing them
    void keyEventProcessed();
  protected:
    QKeyEvent _last_key_event;
    bool _last_key_event_processed;

  };
 
}
