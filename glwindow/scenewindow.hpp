// Copyright (c) Ethan Eade, https://bitbucket.org/ethaneade/glwindow

#pragma once

#include "glwindow.hpp"

namespace glwindow
{

    class SceneWindow : public EventHandler
    {
    public:

        struct Viewpoint
        {
            double target[3];
            double azimuth, elevation, distance;
            Viewpoint();
        };
        
        SceneWindow(int width, int height, const char *title);
        virtual ~SceneWindow();

        void update();

        bool start_draw();
        void finish_draw();

        GLWindow win;
        Viewpoint viewpoint;
        
    protected:
        bool on_key_down(GLWindow& win, int key);
        bool on_button_down(GLWindow& win, int btn, int state, int x, int y);
        bool on_button_up(GLWindow& win, int btn, int state, int x, int y);
        bool on_mouse_move(GLWindow& win, int state, int x, int y);
        bool on_mouse_wheel(GLWindow& win, int state, int x, int y, int dx, int dy);
        bool on_resize(GLWindow& win, int x, int y, int w, int h);        
        
        bool dragging;
        int drag_btn;
        int x0, y0;
        double inv_w0, inv_h0;
        Viewpoint vp0;

        bool drawing;
    };
    
}
