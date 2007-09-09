//////////////////////////////////////////////////////////////////////////
// system.cpp
//////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <gl/glut.h>

#include "ChainApp.h"
#include "ab_types.h"
#include "ab_adapter.h"
#include "render.h"
#include "timer.h"
#include "processor.h"
#include "camera.h"
#include "inputer.h"


using namespace app;

void Application::_DisplayFunc(void)
{
    App._renderer->processRender();      
    return;
}

void Application::_IdleFunc(void)
{  
    App._timer->processTimer();
    glutPostRedisplay();
    return;
}

void Application::_KeyboardFunc(unsigned char c, int x, int y)
{  
    inp::InputParams inp_params;
    int sysKeyModif = glutGetModifiers();
    inp::Inputer::MapGLInputToApp(&c, NULL, NULL, &sysKeyModif, &x, &y, inp_params);
    App._inputer->processInput(inp_params);
    return;
}

void Application::_MouseFunc(int button, int state, int x, int y)
{  
    inp::InputParams inp_params;
    int sysKeyModif = glutGetModifiers();
    inp::Inputer::MapGLInputToApp(NULL, &button, &state, &sysKeyModif, &x, &y, inp_params);
    App._inputer->processInput(inp_params);
    return;
}

void Application::_MotionFunc(int x, int y)
{  
    inp::InputParams inp_params = App._inputer->getCurParams();
    inp::Inputer::MapGLInputToApp(NULL, NULL, NULL, NULL, &x, &y, inp_params);
    App._inputer->processInput(inp_params);

    return;
}

void Application::_ReshapeFunc(int w, int h)
{
    App._processor->getCameraMng().setCameraWindow(w, h);
    return;
}

bool Application::_InitSystem(int argc, char * argv[])
{
    glutInit(&argc, (char**)&argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(_wSize, _hSize);
    _window = glutCreateWindow ("ChainApp");

    glClearColor (0.0, 0.0, 0.0, 0.0);

    _renderer = new rnd::Renderer(_wSize, _hSize);
    _timer    = new tmr::Timer;
    _inputer  = new inp::Inputer;

    _processor = new Processor;
    _renderer->addHdl(_processor);
    _timer->addHdl(_processor);
    _inputer->addHdl(_processor);

    _processor->getCameraMng().setCamera(_renderer->getCamera());

    glutDisplayFunc      (_DisplayFunc);
    glutIdleFunc         (_IdleFunc);
    glutReshapeFunc      (_ReshapeFunc);
    glutKeyboardFunc     (_KeyboardFunc);
    glutMouseFunc        (_MouseFunc);
    glutMotionFunc       (_MotionFunc);
    //   glutPassiveMotionFunc(_MotionFunc);    

    return true;
}

void Application::_TermSystem(void)
{
    delete _processor;
    _processor = NULL;
    delete _inputer;
    _inputer = NULL;
    delete _timer;
    _timer = NULL;
    delete _renderer;
    _renderer = NULL;

    glutDestroyWindow(_window);    

    return;
}


int Application::MainLoop()
{
    glutMainLoop();

    return 0;
}
