#include <math.h>
#ifdef __APPLE__
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glu.h>
#include <GL/glut.h>
#endif
#include <SDL.h>
#include "util/ThreadedObject.h"
#include "util/LogManagerBase.h"
#include "util/GLsceneBase.h"
#include "SDLUtil.h"

SDLwindow::SDLwindow(GLsceneBase* i_scene, LogManagerBase *i_log,
                     ThreadedObject* i_throbj) :
    scene(i_scene), log(i_log), throbj(i_throbj),
    width(640), height(480),
    aspect(((double)width)/height),
    pan(M_PI/4), tilt(M_PI/16), radius(5),
    isShiftPressed(false), isControlPressed(false),
    xCenter(0), yCenter(0), zCenter(0.8),
    showingHelp(false)
{
    helpcommand.push_back("h: help");
    instructions.push_back("q: quit");
    instructions.push_back("SPACE: play/stop");
    instructions.push_back("f: faster");
    instructions.push_back("s: slower");
    instructions.push_back("r: record movie");
    instructions.push_back("t: toggle robot state");
    if (throbj){
        instructions.push_back("p: pause/resume background thread");
    }
    scene->setMessages(helpcommand);
}

SDLwindow::~SDLwindow()
{
    SDL_Quit();
}

bool SDLwindow::init()
{
    if(SDL_Init(SDL_INIT_VIDEO)<0) {
        fprintf(stderr,"failed to initialize SDL.\n");
        return false;
    }

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER,1);
    SDL_GL_SetAttribute(SDL_GL_SWAP_CONTROL,1);
    SDL_Surface *screen;
    screen=SDL_SetVideoMode(width,height,32,SDL_HWSURFACE | SDL_GL_DOUBLEBUFFER | SDL_OPENGL | SDL_RESIZABLE);
    if(!screen) {
        fprintf(stderr,"failed to set video mode to %dx%dx32.\n",width,height);
        SDL_Quit();
        return false;
    }
    SDL_WM_SetCaption("sdlsim", NULL);
    SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY,SDL_DEFAULT_REPEAT_INTERVAL);
    return true;
}

double SDLwindow::sliderRatio(double x)
{
    double ratio = (x - SLIDER_SIDE_MARGIN)/(width - SLIDER_SIDE_MARGIN*2);
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    return ratio;
}

bool SDLwindow::processEvents()
{
    SDL_Event event;
    while(SDL_PollEvent(&event)){
        switch(event.type){
        case SDL_QUIT:
            return false;
        case SDL_KEYDOWN:
        {
            //printf("%d\n", event.key.keysym.sym);
            int delta = isShiftPressed ? 10 : 1;
            switch(event.key.keysym.sym){
            case SDLK_h:
                if (showingHelp){
                    scene->setMessages(helpcommand);
                }else{
                    scene->setMessages(instructions);
                }
                showingHelp = !showingHelp;
                break;
            case SDLK_q:
                return false;
            case SDLK_p:
                if (throbj){
                    if (throbj->isPausing()){
                        throbj->resume();
                    }else{
                        throbj->pause();
                    }
                }
                break;
            case SDLK_SPACE:
                log->play(); 
                break;
            case SDLK_t:
                scene->toggleRobotState(); 
                break;
            case SDLK_f:
                log->faster();
                break;
            case SDLK_s:
                log->slower();
                break;
            case SDLK_r:
                log->record();
                break;
            case SDLK_RIGHT:
                if (isControlPressed){
                    log->tail();
                }else{
                    log->next(delta);
                }
                break;
            case SDLK_LEFT:
                if (isControlPressed){
                    log->head();
                }else{
                    log->prev(delta);
                }
                break;
            case SDLK_LSHIFT:
            case SDLK_RSHIFT:
                isShiftPressed = true;
                break;
            case SDLK_LCTRL:
            case SDLK_RCTRL:
                isControlPressed = true;
                break;
            }
            break;
        }
        case SDL_KEYUP:
            switch(event.key.keysym.sym){
            case SDLK_LSHIFT:
            case SDLK_RSHIFT:
                    isShiftPressed = false;
                    break;
            case SDLK_LCTRL:
            case SDLK_RCTRL:
                isControlPressed = false;
                break;
            }
            break;
            
        case SDL_MOUSEBUTTONDOWN:
            switch(event.button.button){
            case SDL_BUTTON_LEFT:
                if (event.button.y > height-SLIDER_AREA_HEIGHT){
                    log->move(sliderRatio(event.button.x));
                    buttonPressedInSliderArea = true;
                }else{
                    buttonPressedInSliderArea = false;
                }
                break;
            case SDL_BUTTON_MIDDLE:
                break;
            case SDL_BUTTON_RIGHT:
                break;
            case SDL_BUTTON_WHEELUP:
                radius *= 1.1;
                break;
            case SDL_BUTTON_WHEELDOWN:
                radius *= 0.9;
                if (radius < 0.1) radius = 0.1; 
                break;
            }
        case SDL_MOUSEBUTTONUP:
            switch(event.button.button){
            case SDL_BUTTON_LEFT:
                break;
            case SDL_BUTTON_MIDDLE:
                break;
            case SDL_BUTTON_RIGHT:
                break;
            case SDL_BUTTON_WHEELUP:
                break;
            case SDL_BUTTON_WHEELDOWN:
                break;
            }
            break;
        case SDL_MOUSEMOTION:
        {
            int dx = event.motion.xrel;
            int dy = event.motion.yrel;
            if (event.motion.state&SDL_BUTTON(SDL_BUTTON_LEFT)){
                if (isShiftPressed){
                    radius *= (1+ 0.1*dy);
                    if (radius < 0.1) radius = 0.1; 
                }else{
                    if (buttonPressedInSliderArea){
                        log->move(sliderRatio(event.motion.x));
                    }else{
                        pan  -= 0.05*dx;
                        tilt += 0.05*dy;
                        if (tilt >  M_PI/2) tilt =  M_PI/2;
                        if (tilt < -M_PI/2) tilt = -M_PI/2;
                    }
                }
            }else if (event.motion.state&SDL_BUTTON(SDL_BUTTON_RIGHT)){
                xCenter += sin(pan)*dx*0.01;
                yCenter -= cos(pan)*dx*0.01;
                zCenter += dy*0.01;
            }
            scene->showSlider(event.motion.y > height-SLIDER_AREA_HEIGHT);
        }
        break;
        case SDL_VIDEORESIZE:
            width = event.resize.w;
            height = event.resize.h;
            SDL_SetVideoMode(width,height,32,SDL_HWSURFACE | SDL_GL_DOUBLEBUFFER | SDL_OPENGL | SDL_RESIZABLE);
            glViewport(0, 0, width, height);
            aspect = ((double)width)/height;
            scene->setScreenSize(width, height);
            break;
        }
    }
    return true;
}

static void drawString(const char *str)
{
    for (unsigned int i=0; i<strlen(str); i++){
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[i]);
    }
}

void SDLwindow::draw()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30,aspect, 0.1, 100);

    double xEye = xCenter + radius*cos(tilt)*cos(pan);
    double yEye = yCenter + radius*cos(tilt)*sin(pan);
    double zEye = zCenter + radius*sin(tilt);
    
    gluLookAt(xEye, yEye, zEye,
              xCenter, yCenter, zCenter,
              0,0,1);
    
    glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);

    scene->draw();
}

void SDLwindow::swapBuffers()
{
    SDL_GL_SwapBuffers();
}