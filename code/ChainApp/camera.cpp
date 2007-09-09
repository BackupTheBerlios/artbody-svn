//////////////////////////////////////////////////////////////////////////
// camera.cpp 
//////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <gl/glut.h>

#include "camera.h"

using namespace rnd;
using namespace ab;

Camera::Camera(const CameraParams& params, const ab::Matr4& c2wMatr)
{
   _params = params;
   setMatrC2W(c2wMatr);
}

void Camera::getParams(CameraParams& params) const
{
   params = _params;
   return;
}

const Matr4& Camera::getMatrC2W(void) const
{
   return _c2wMatr;
}

bool Camera::setViewport(Real xView, Real yView)
{
   _params.xView = xView;
   _params.yView = yView;
   return true;
}

bool Camera::setFow(Real xFow, Real aspect)
{
   _params.xFow   = xFow;
   _params.aspect = aspect;

   return true;
}

bool Camera::setNearFarPlanes(Real nearPlane, Real farPlane)
{
   _params.nearPlane = nearPlane;
   _params.farPlane  = farPlane;
   return true;
}

bool Camera::setMatrC2W(const Matr4& c2wMatr)
{
   _c2wMatr = c2wMatr;
   return true;
}

bool Camera::setParams(const CameraParams& params)
{
   _params = params;   
   return true;
}

bool Camera::setNaturalParams(const Vect4& pos, const Vect4& pointOfInterest, const Vect4& up)
{
   Vect4 xAxis, yAxis, zAxis;   

   yAxis = up;
   yAxis.normalize();
   zAxis = pointOfInterest - pos;
   zAxis.normalize();
   xAxis = yAxis.cross(zAxis);
   
   _c2wMatr = Matr4(xAxis, yAxis, zAxis, pos);
   return true;
}

void Camera::_applyCamera(void) const
{
   // Setup viewport
   glViewport(0, 0, (GLsizei) _params.xView, (GLsizei) _params.yView);

   // Setup frustum
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   Real yFow = _params.xFow / _params.aspect;
   gluPerspective(yFow, _params.aspect, _params.nearPlane, _params.farPlane);
   
   // setup camera position/direction      
//   glLoadIdentity();   
   Vect4 pos, forw, up;
   pos  = _c2wMatr.getTranslate();
   forw = pos + _c2wMatr.getAxisZ();
   up = _c2wMatr.getAxisY();
   gluLookAt(pos.x, pos.y, pos.z, 
             forw.x, forw.y, forw.z, 
             up.x, up.y, up.z);
   
   glMatrixMode (GL_MODELVIEW);
   return;
}

