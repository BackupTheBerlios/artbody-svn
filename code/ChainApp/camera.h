#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "ab_types.h"
#include "ab_vect4.h"
#include "ab_matr4.h"

namespace rnd {

class Renderer;

struct CameraParams {
   ab::Real xView;
   ab::Real yView;
   ab::Real xFow;
   ab::Real aspect;
   ab::Real nearPlane;
   ab::Real farPlane;
};

class Camera {
friend class Renderer;
public:
   explicit Camera(const CameraParams& params, const ab::Matr4& c2wMatr);

   void             getParams (CameraParams& params) const;
   const ab::Matr4& getMatrC2W(void) const;
      
   bool setViewport     (ab::Real xView, ab::Real yView);
   bool setFow          (ab::Real xFow, ab::Real aspect);
   bool setNearFarPlanes(ab::Real nearPlane, ab::Real farPlane);
   bool setMatrC2W      (const ab::Matr4& c2wMatr);
   bool setParams       (const CameraParams& params);
   bool setNaturalParams(const ab::Vect4& pos, const ab::Vect4& pointOfInterest, const ab::Vect4& up);
private:
   void _applyCamera() const;

   CameraParams _params;   
   ab::Matr4    _c2wMatr;
};

} // namespace rnd

#endif //_CAMERA_H_
