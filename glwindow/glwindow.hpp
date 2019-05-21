// Copyright (c) Ethan Eade, https://bitbucket.org/ethaneade/glwindow

#pragma once

#include <vector>

namespace glwindow {
    
    namespace ButtonEvent {
        enum Buttons {
            LEFT=1, MIDDLE=2, RIGHT=4, WHEEL=8,
            MODKEY_CTRL=16, MODKEY_SHIFT=32, 
        };
    };

    namespace KeyCode {
        enum Codes {
            BACKSPACE=0x8,
            TAB=0x9,
            ENTER=0xD,
            ESCAPE=0x1B,
            DEL=0x7F,
            
            SHIFT=0xFF00,
            CTRL,
            ALT,
            SUPER,
            CAPSLOCK,
            LEFT,
            UP,
            RIGHT,
            DOWN,            
        };
    };
    

    class GLWindow;
    
    struct EventHandler
    {
    public:
        virtual ~EventHandler() {}
        virtual bool on_key_down(GLWindow& win, int key) { return false; }
        virtual bool on_key_up(GLWindow& win, int key) { return false; }
        virtual bool on_text(GLWindow& win, const char *text, int len) { return false; }
        virtual bool on_button_down(GLWindow& win, int btn, int state, int x, int y) { return false; }
        virtual bool on_button_up(GLWindow& win, int btn, int state, int x, int y) { return false; }
        virtual bool on_mouse_move(GLWindow& win, int state, int x, int y) { return false; }
        virtual bool on_mouse_wheel(GLWindow& win, int state, int x, int y, int dx, int dy) { return false; }
        virtual bool on_resize(GLWindow& win, int x, int y, int w, int h) { return false; }
        virtual bool on_close(GLWindow& win) { return false; }
    };

    // Dispatches to each handler in reverse order until one returns true
    class EventDispatcher : public EventHandler
    {
    public:
        const std::vector<EventHandler*> &handlers;
        EventDispatcher(const std::vector<EventHandler*> &h) :
            handlers(h) {}
        bool on_key_down(GLWindow& win, int key);
        bool on_key_up(GLWindow& win, int key);
        bool on_text(GLWindow& win, const char *text, int len);
        bool on_button_down(GLWindow& win, int btn, int state, int x, int y);
        bool on_button_up(GLWindow& win, int btn, int state, int x, int y);
        bool on_mouse_move(GLWindow& win, int state, int x, int y);
        bool on_mouse_wheel(GLWindow& win, int state, int x, int y, int dx, int dy);
        bool on_resize(GLWindow& win, int x, int y, int w, int h);
        bool on_close(GLWindow& win);
    };
    
    class GLWindow
    {
    public:
        GLWindow(int w=-1, int h=-1, const char *title=0);
        virtual ~GLWindow();

        int width() const;
        int height() const;
        bool visible() const;
        bool alive() const;
        
        bool make_current();
        bool push_context();
        void pop_context();

        struct ScopedContext {
            GLWindow &win;
            ScopedContext(GLWindow &w) : win(w) {
                win.push_context();
            }           
            ~ScopedContext() {
                win.pop_context();
            }
        };
        
        void swap_buffers();
        
        void set_size(int w, int h);
        void set_position(int x, int y);
        
        void set_title(const char* title);
        
        void add_handler(EventHandler* handler);
        bool remove_handler(EventHandler *handler);
        void handle_events();
        static void handle_all_events();
        
        void destroy();

        void draw_text(double x, double y, const char *text, int xywh[4]=0);
    protected:
        struct SystemState;
        SystemState *sys_state;

        std::vector<EventHandler*> handlers;
        GLWindow *prev_active;

        static GLWindow *active_context;
        static std::vector<GLWindow*> all_windows;
        static void add_window(GLWindow *win);
        static bool remove_window(GLWindow *win);
    };
}
