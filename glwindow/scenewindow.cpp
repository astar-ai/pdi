// Copyright (c) Ethan Eade, https://bitbucket.org/ethaneade/glwindow

#include "scenewindow.hpp"
#include <GL/gl.h>
#include <cmath>
#include <iostream>

using namespace glwindow;

SceneWindow::SceneWindow(int width, int height, const char *title)
    : win(width, height, title)
{
    dragging = false;
    drawing = false;
    win.add_handler(this);
}

SceneWindow::~SceneWindow()
{
}

void SceneWindow::update()
{
    win.handle_events();
}

SceneWindow::Viewpoint::Viewpoint()
{
    target[0] = -0.05;
    target[1] = -0.75;
    target[2] = 0.;
    azimuth   = 0.;
    elevation = 0.0;
    distance  = 8.0;
}

static void set_viewpoint(const SceneWindow::Viewpoint &vp)
{
    const double RAD_TO_DEG = 180.0 / 3.141592653589793;
    glTranslated(0,0,vp.distance);
    glRotated(vp.elevation * RAD_TO_DEG, 1, 0, 0);
    glRotated(vp.azimuth * RAD_TO_DEG, 0, 1, 0);
    glTranslated(-vp.target[0], -vp.target[1], -vp.target[2]);
}

bool SceneWindow::start_draw()
{
    if (!win.alive() || drawing)
        return false;

    drawing = true;

    win.push_context();
    glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT);
    
    glViewport(0, 0, win.width(), win.height());
    double aspect = (double)win.width() / (double)std::max(win.height(),1);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double znear = 0.01;
    double zfar = 100.0;
    double fy = 0.6 * znear;
    double fx = aspect * fy;
    glFrustum(-fx,fx,-fy,fy, znear, zfar);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScaled(1, -1, -1);
    
    set_viewpoint(viewpoint);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_LIGHTING);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    return true;
}


void SceneWindow::finish_draw()
{
    if (!drawing)
        return;

    glPopAttrib();
    glFlush();
    win.swap_buffers();
    win.handle_events();
    
    win.pop_context();
    drawing = false;
}

bool SceneWindow::on_key_down(GLWindow& win, int key)
{
    return true;
}

bool SceneWindow::on_button_down(GLWindow& win, int btn, int state, int x, int y)
{
    if (dragging)
        return false;

    //std::cerr << "down " << btn << std::endl;
    drag_btn = btn;
    x0 = x;
    y0 = y;
    vp0 = viewpoint;
    inv_w0 = 1.0 / win.width();
    inv_h0 = 1.0 / win.height();
    dragging = true;
    
    return true;
}

bool SceneWindow::on_button_up(GLWindow& win, int btn, int state, int x, int y)
{
    //std::cerr << "up " << btn << std::endl;
    dragging = false;
    return true;
}

bool SceneWindow::on_mouse_move(GLWindow& win, int state, int x, int y)
{
    int idx = x - x0;
    int idy = y - y0;
    double dx = idx * inv_w0;
    double dy = idy * inv_w0;

    //std::cerr << dx << ", " << dy << std::endl;

    if (!dragging)        
        return false;

    if (drag_btn == ButtonEvent::LEFT) {
        viewpoint.azimuth = vp0.azimuth - dx * 4.0;
        viewpoint.elevation = vp0.elevation + dy * 4.0;
        return true;
    } else if (drag_btn == ButtonEvent::RIGHT) {
        viewpoint.distance = ::exp(dy * 4.0) * vp0.distance;
        return true;
    } else if (drag_btn == ButtonEvent::MIDDLE) {
        double sa = ::sin(-vp0.azimuth);
        double ca = ::cos(-vp0.azimuth);
        double se = ::sin(vp0.elevation);
        double ce = ::cos(vp0.elevation);

        double tx = -idx * 0.003;
        double ty = -idy * 0.003;
        
        double dtx = ca * tx - se*sa*ty;
        double dty = ce * ty;
        double dtz = -sa*tx - se*ca*ty;
        double r = vp0.distance;
        
        viewpoint.target[0] = vp0.target[0] + r*dtx;
        viewpoint.target[1] = vp0.target[1] + r*dty;
        viewpoint.target[2] = vp0.target[2] + r*dtz;
        return true;
    }

    return false;
}

bool SceneWindow::on_mouse_wheel(GLWindow& win, int state, int x, int y, int dx, int dy)
{
    viewpoint.distance *= ::exp(dy * -0.08);
    return true;
}

bool SceneWindow::on_resize(GLWindow& win, int x, int y, int w, int h)
{
    return false;
}

