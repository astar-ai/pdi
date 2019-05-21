// Copyright (c) Ethan Eade, https://bitbucket.org/ethaneade/glwindow

#include "glwindow.hpp"

#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <GL/glx.h>
#include <iostream>

using namespace glwindow;

std::vector<GLWindow*> GLWindow::all_windows;

void GLWindow::add_window(GLWindow *win)
{
    all_windows.push_back(win);
}

bool GLWindow::remove_window(GLWindow *win)
{
    for (size_t i=0; i<all_windows.size(); ++i) {
        if (all_windows[i] == win) {
            all_windows[i] = all_windows.back();
            all_windows.pop_back();
            return true;
        }
    }
    return false;
}

void GLWindow::handle_all_events()
{
    for (size_t i=0; i<all_windows.size(); ++i)
        all_windows[i]->handle_events();
}

GLWindow *GLWindow::active_context = 0;

bool GLWindow::push_context()
{
    prev_active = active_context;
    return make_current();
}

void GLWindow::pop_context()
{
    if (active_context != this)
        return;

    if (prev_active) {
        prev_active->make_current();
    } else {
        active_context = 0;
    }
}

void GLWindow::add_handler(EventHandler* handler)
{
    handlers.push_back(handler);
}

bool GLWindow::remove_handler(EventHandler* handler)
{
    std::vector<EventHandler*>::reverse_iterator it;
    for (it = handlers.rbegin(); it != handlers.rend(); ++it) {
        if (*it == handler) {
            handlers.erase(it.base());
            return true;
        }
    }
    return false;
}

bool EventDispatcher::on_key_down(GLWindow& win, int key) {
    for (int i=handlers.size()-1; i>=0; --i)
        if (handlers[i]->on_key_down(win, key))
            return true;
    return false;
}
bool EventDispatcher::on_key_up(GLWindow& win, int key) {
    for (int i=handlers.size()-1; i>=0; --i)
        if (handlers[i]->on_key_up(win, key))
            return true;
    return false;
}
bool EventDispatcher::on_text(GLWindow& win, const char *text, int len) {
    for (int i=handlers.size()-1; i>=0; --i)
        if (handlers[i]->on_text(win, text, len))
            return true;
    return false;
}

bool EventDispatcher::on_button_down(GLWindow& win, int btn, int state, int x, int y) {
    for (int i=handlers.size()-1; i>=0; --i)
        if (handlers[i]->on_button_down(win, btn, state, x, y))
            return true;
    return false;
}

bool EventDispatcher::on_button_up(GLWindow& win, int btn, int state, int x, int y) {
    for (int i=handlers.size()-1; i>=0; --i)
        if (handlers[i]->on_button_up(win, btn, state, x, y))
            return true;
    return false;
}

bool EventDispatcher::on_mouse_move(GLWindow& win, int state, int x, int y) {
    for (int i=handlers.size()-1; i>=0; --i)
        if (handlers[i]->on_mouse_move(win, state, x, y))
            return true;
    return false;
}

bool EventDispatcher::on_mouse_wheel(GLWindow& win, int state, int x, int y, int dx, int dy) {
    for (int i=handlers.size()-1; i>=0; --i)
        if (handlers[i]->on_mouse_wheel(win, state, x, y, dx, dy))
            return true;
    return false;
}

bool EventDispatcher::on_resize(GLWindow &win, int x, int y, int w, int h) {
    for (int i=handlers.size()-1; i>=0; --i)
        if (handlers[i]->on_resize(win, x, y, w, h))
            return true;
    return false;
}

bool EventDispatcher::on_close(GLWindow &win) {
    for (int i=handlers.size()-1; i>=0; --i)
        if (handlers[i]->on_close(win))
            return true;
    return false;
}

static XVisualInfo *makeVisualInfo(Display *display)
{
    int visualAttributes[] = {
        GLX_RED_SIZE,      8,
        GLX_GREEN_SIZE,    8,
        GLX_BLUE_SIZE,     8,
        GLX_DEPTH_SIZE,    16,
        GLX_STENCIL_SIZE,  8,
        GLX_RGBA,
        GLX_DOUBLEBUFFER,
        None
    };
    XVisualInfo *vi = glXChooseVisual(display, DefaultScreen(display), visualAttributes);
    return vi;
}

static Window makeWindow(Display *display, XVisualInfo *vi, int width, int height)
{
    Window rootWindow = RootWindow(display, vi->screen);
    
    XSetWindowAttributes attributes;
    attributes.border_pixel = 0;
    attributes.colormap = XCreateColormap(display, rootWindow, vi->visual, AllocNone);
    attributes.event_mask = (KeyPressMask | KeyReleaseMask |
                             ButtonPressMask | ButtonReleaseMask |
                             PointerMotionMask |
                             VisibilityChangeMask |
                             StructureNotifyMask |
                             ExposureMask);
    
    Window window = XCreateWindow(display,
                                  rootWindow,
                                  0, 0, width, height,
                                  0, vi->depth,
                                  InputOutput,
                                  vi->visual,
                                  CWBorderPixel | CWColormap | CWEventMask,
                                  &attributes);
    return window;
}

struct GLWindow::SystemState
{
    Display* display;
    Window window;
    GLXContext context;
    Atom delete_atom;
    Cursor cursor;

    int width, height;
    bool visible;   
    
    SystemState() {
        display = 0;
        window = 0;
        width = 0;
        height = 0;
        visible = false;
    }
    ~SystemState() {
        if (!display)
            return;

        if (context) {
            destroy();
            
            glXMakeCurrent(display, None, 0);
            glXDestroyContext(display, context);
        }

        XCloseDisplay(display);        
    }
    
    bool init(int w, int h, const char *title)
    {
	display = XOpenDisplay(0);
        if (!display)
            return false;        
        XVisualInfo *vi = makeVisualInfo(display);
        if (!vi)
            return false;
        
	context = glXCreateContext(display, vi, 0, True);
        if (!context)
            return false;

        width = w;
        height = h;
        window = makeWindow(display, vi, width, height);
        if (!window)
            return false;
        
        XStoreName(display, window, title);

        {
            XClassHint classHint;
            classHint.res_name = const_cast<char*>(title);
            char classname[] = "glwindow";
            classHint.res_class = classname;
            XSetClassHint(display, window, &classHint);
            XMapWindow(display, window);
        }
        
        XEvent ev;
        do {
            XNextEvent(display, &ev);
        } while (ev.type != MapNotify);

        visible = true;
        delete_atom = XInternAtom(display, "WM_DELETE_WINDOW", True);
        XSetWMProtocols(display, window, &delete_atom, 1);
        
        cursor = XCreateFontCursor(display, ' ');
        return true;
    }

    void destroy()
    {
        if (window) {
            XUnmapWindow(display, window);
            XDestroyWindow(display, window);
            window = 0;
        }
    }
    
    void swap_buffers()
    {
        if (window) {
            glXSwapBuffers(display, window);
        }
    }

    void set_title(const char *title)
    {
        if (window) {
            XStoreName(display, window, title);
        }
    }
    
    bool make_current()
    {
        if (!window)
            return false;
        glXMakeCurrent(display, window, context);
        return true;
    }
};

GLWindow::GLWindow(int w, int h, const char *title)
{
    sys_state = new SystemState();
    sys_state->init(w, h, title);
    all_windows.push_back(this);
}

GLWindow::~GLWindow()
{
    for (size_t i=0; i<all_windows.size(); ++i) {
        if (all_windows[i] == this) {
            all_windows[i] = all_windows.back();
            all_windows.pop_back();
            break;
        }
    }
    delete sys_state;
}

int GLWindow::width() const
{
    return sys_state->width;
}

int GLWindow::height() const
{
    return sys_state->height;
}

bool GLWindow::visible() const
{
    return sys_state->visible;
}

bool GLWindow::alive() const
{
    return 0 != sys_state->window;
}
            
bool GLWindow::make_current()
{
    if (!sys_state->make_current())
        return false;
    
    active_context = this;
    return true;
}

void GLWindow::swap_buffers()
{
    sys_state->swap_buffers();
}
        
void GLWindow::set_size(int w, int h)
{
    if (!alive())
        return;
        
    XWindowChanges c;
    c.width = w;
    c.height = h;
    XConfigureWindow(sys_state->display,
                     sys_state->window,
                     CWWidth | CWHeight,
                     &c);
}

void GLWindow::set_position(int x, int y)
{
    if (!alive())
        return;
    
    XWindowChanges c;
    c.x = x;
    c.y = y;
    XConfigureWindow(sys_state->display,
                     sys_state->window,
                     CWX | CWY,
                     &c);
}
        
void GLWindow::set_title(const char* title)
{
    if (!alive())
        return;
    
    sys_state->set_title(title);
}

static int convert_button_state(unsigned int state)
{
    int s = 0;
    if (state & Button1Mask) s |= ButtonEvent::LEFT;
    if (state & Button2Mask) s |= ButtonEvent::MIDDLE;
    if (state & Button3Mask) s |= ButtonEvent::RIGHT;
    if (state & ControlMask) s |= ButtonEvent::MODKEY_CTRL;
    if (state & ShiftMask) s |= ButtonEvent::MODKEY_SHIFT;
    return s;
}
    
static int convert_button(int button)
{
    switch (button) {
    case Button1: return ButtonEvent::LEFT;
    case Button2: return ButtonEvent::MIDDLE;
    case Button3: return ButtonEvent::RIGHT;
    default: return 0;
    }
}

static int convert_keycode(int key)
{
    switch (key) {
    case XK_BackSpace: return KeyCode::BACKSPACE;
    case XK_Tab: return KeyCode::TAB;
    case XK_Return: return KeyCode::ENTER;
    case XK_Shift_L: return KeyCode::SHIFT;
    case XK_Shift_R: return KeyCode::SHIFT;
    case XK_Control_L: return KeyCode::CTRL;
    case XK_Control_R: return KeyCode::CTRL;
    case XK_Alt_L: return KeyCode::ALT;
    case XK_Alt_R: return KeyCode::ALT;
    case XK_Super_L: return KeyCode::SUPER;
    case XK_Super_R: return KeyCode::SUPER;
    case XK_Caps_Lock: return KeyCode::CAPSLOCK;
    case XK_Delete: return KeyCode::DEL;
    case XK_Escape: return KeyCode::ESCAPE;
    case XK_Left: return KeyCode::LEFT;
    case XK_Up: return KeyCode::UP;
    case XK_Right: return KeyCode::RIGHT;
    case XK_Down: return KeyCode::DOWN;
    }
    return key;
}

void GLWindow::handle_events()
{
    if (!alive())
        return;
    
    XEvent event;
    KeySym key;
    const int text_size = 64;
    char text[text_size];
    int len;
    EventDispatcher dispatcher(handlers);

    int btn, state;
    
    while (XPending(sys_state->display))
    {           
        XNextEvent(sys_state->display, &event);
        //std::cerr << "event " << event.type << std::endl;
        switch (event.type) {
        case ButtonPress:
            state = convert_button_state(event.xbutton.state);
            if (event.xbutton.button == Button4) {
                // MouseWheel down
                dispatcher.on_mouse_wheel(*this, state, event.xbutton.x, event.xbutton.y, 0, 1);
            } else if (event.xbutton.button == Button5) {
                // MouseWheel up
                dispatcher.on_mouse_wheel(*this, state, event.xbutton.x, event.xbutton.y, 0, -1);
            } else {
                btn = convert_button(event.xbutton.button);
                dispatcher.on_button_down(*this, btn, state, event.xbutton.x, event.xbutton.y);
            }
            break;
            
        case ButtonRelease:
            if (event.xbutton.button == Button4 ||
                event.xbutton.button == Button5)
                break;            
            btn = convert_button(event.xbutton.button);            
            state = convert_button_state(event.xbutton.state);
            dispatcher.on_button_up(*this, btn, state, event.xbutton.x, event.xbutton.y);
            break;
            
        case MotionNotify:
            state = convert_button_state(event.xbutton.state);
            dispatcher.on_mouse_move(*this, state, event.xmotion.x, event.xmotion.y);
            break;
            
        case KeyPress:
            len = XLookupString(&event.xkey, text, text_size-1, &key, 0);
            dispatcher.on_key_down(*this, convert_keycode(key));
            if (len > 0) {
                text[len] = 0;
                dispatcher.on_text(*this, text, len);
            }
            break;
            
        case KeyRelease:
            XLookupString(&event.xkey, 0, 0, &key, 0);
            dispatcher.on_key_up(*this, convert_keycode(key));
            break;
            
        case ConfigureNotify:
            sys_state->width = event.xconfigure.width;
            sys_state->height = event.xconfigure.height;
            dispatcher.on_resize(*this, event.xconfigure.x, event.xconfigure.y,
                                 sys_state->width, sys_state->height);
            break;

        case VisibilityNotify:
            if (event.xvisibility.state == VisibilityFullyObscured)
                sys_state->visible = false;
            else
                sys_state->visible = true;
            break;

        case DestroyNotify:
            //std::cerr << "DestroyNotify" << std::endl;
            //sys_state->window = 0;
            break;
            
        case Expose:
            //std::cerr << "Expose" << std::endl;
            break;
            
        case ClientMessage:
            if (event.xclient.data.l[0] == (int)sys_state->delete_atom) {
                if (!dispatcher.on_close(*this))
                    destroy();
            }
            break;
            
        default:
            break;
        }
    }    
}

void GLWindow::destroy()
{
    sys_state->destroy();
}
