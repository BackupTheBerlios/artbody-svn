//////////////////////////////////////////////////////////////////////////
// renderer.cpp 
//////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <gl/glut.h>

#include "ab_vect4.h"
#include "ab_matr4.h"
#include "ab_adapter.h"
#include "render.h"
#include "camera.h"

using namespace rnd;
using namespace ab;

Renderer::Renderer(int xSize, int ySize)
{
   /* Use depth buffering for hidden surface elimination. */
   glEnable(GL_DEPTH_TEST);
   
   Vect4 camTrans(0.f, 0.f, 5.f);
   Matr4 c2wMatr(Vect4::UnitX, Vect4::UnitY, Vect4::UnitZNeg, camTrans);   
   CameraParams camParams;
   camParams.xView     = xSize;
   camParams.yView     = ySize;
   camParams.xFow      = 60.f;
   camParams.aspect    = (ab::Real) xSize / ySize;
   camParams.nearPlane = 0.1f;
   camParams.farPlane  = 100.f;
   _pCamera = new Camera(camParams, c2wMatr);
   _pCamera->setNaturalParams(ab::Vect4(5.f, 0.f, 0.f), ab::Vect4::Zero, ab::Vect4::UnitY);

   return;
}

Renderer::~Renderer()
{
   ;
}

bool Renderer::addHdl(RenderHdl * pHdl)
{
   if (sys::HdlCollection<RenderHdl>::addHdl(pHdl)) {
      pHdl->_renderer = this;
      return true;
   }
   return false;
}

bool Renderer::removeHdl(RenderHdl * pHdl)
{
   if (sys::HdlCollection<RenderHdl>::removeHdl(pHdl)) {
      pHdl->_renderer = NULL;
      return true;
   }
   return false;
}

void Renderer::processRender(void)
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   _pCamera->_applyCamera();

   unsigned int i;
   for (i = 0; i < _hdlArray.size(); i++) {
      _hdlArray[i]->onRender();
   }

   glutSwapBuffers();
   glFlush();

   return;
}

//
// render methods
//

void Renderer::renderLine(const Vect4& from, const Vect4& to, DWord color) const
{
   glBegin(GL_LINES);
   glColor4ub(GetColorComp(color, ab::COLOR_R), 
              GetColorComp(color, ab::COLOR_G), 
              GetColorComp(color, ab::COLOR_B), 
              GetColorComp(color, ab::COLOR_A));
   glVertex3d(from[0], from[1], from[2]);
   glVertex3d(to[0], to[1], to[2]);
   glEnd();
   return;
}

void Renderer::renderLine2D(const Vect4& from, const Vect4& to, DWord color) const
{
    glBegin(GL_LINES);
    glColor4ub(GetColorComp(color, ab::COLOR_R), 
        GetColorComp(color, ab::COLOR_G), 
        GetColorComp(color, ab::COLOR_B), 
        GetColorComp(color, ab::COLOR_A));
    glVertex2d(from[0], from[1]);
    glVertex2d(to[0], to[1]);
    glEnd();
    return;
}


void  Renderer::renderAxis(const ab::Matr4& localMatr, ab::Real axisLen) const
{
   Vect4 pos, xTo, yTo, zTo;

   pos = localMatr.getTranslate();
   xTo = localMatr.getAxisX() * axisLen + pos;   
   yTo = localMatr.getAxisY() * axisLen + pos;   
   zTo = localMatr.getAxisZ() * axisLen + pos;

   glBegin(GL_LINES);
   glColor4ub(0xFF, 0, 0, 0xFF);
   glVertex3f(pos.x, pos.y, pos.z);
   glVertex3f(xTo.x, xTo.y, xTo.z);
   glColor4ub(0, 0xFF, 0, 0xFF);
   glVertex3f(pos.x, pos.y, pos.z);
   glVertex3f(yTo.x, yTo.y, yTo.z);
   glColor4ub(0, 0, 0xFF, 0xFF);
   glVertex3f(pos.x, pos.y, pos.z);
   glVertex3f(zTo.x, zTo.y, zTo.z);
   glEnd();

   return;
}

void Renderer::renderSphereWire(const ab::Vect4& center, ab::Real radius, DWord color, int slices, int stacks)
{
   glPushMatrix();  
   glLoadIdentity();
   glColor4ub(GetColorComp(color, ab::COLOR_R), 
              GetColorComp(color, ab::COLOR_G), 
              GetColorComp(color, ab::COLOR_B), 
              GetColorComp(color, ab::COLOR_A));
   glTranslatef(center.x, center.y, center.z);
   glutWireSphere(radius, slices, stacks);
   glPopMatrix();
 
   return;
}

void Renderer::renderText(const ab::Vect4& pos, const char * str, DWord color)
{
    if (str == NULL) {
        return;
    }

    glPushMatrix();
    glLoadIdentity();
        
    glColor4ub(GetColorComp(color, ab::COLOR_R), 
                GetColorComp(color, ab::COLOR_G), 
                GetColorComp(color, ab::COLOR_B), 
                GetColorComp(color, ab::COLOR_A));

    glTranslated(pos.x, pos.y, pos.z);
    glRasterPos2d(0, 0);
//    glRasterPos3d(20.0, 20.0, 2.0);
//    glRasterPos3d(pos.x, pos.y, pos.z);
    int i, count = strlen(str);    
    for (i = 0; i < count; i++) {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
    }

    glPopMatrix();

    return;
}