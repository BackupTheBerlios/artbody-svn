#ifndef _RENDER_H_
#define _RENDER_H_

#include "hdl_collect.h"
#include "ab_types.h"
#include "ab_adapter.h"

// forvard
namespace ab {
   class Vect4;
   class Matr4;
}

namespace rnd {

class Renderer;
class RenderHdl;
class Camera;

class RenderHdl {
friend class Renderer;
private:
   virtual void onRender() = 0;
protected:
   Renderer* _renderer;
};


class Renderer : public sys::HdlCollection<RenderHdl> {
public:
   explicit Renderer(int xSize, int ySize);
   virtual ~Renderer();

   void  processRender(void);

   Camera& getCamera(void) { return *_pCamera; }

   bool  addHdl   (RenderHdl* pHdl);
   bool  removeHdl(RenderHdl* pHdl);

   void  renderLine      (const ab::Vect4& from, const ab::Vect4& to, ab::DWord color = 0xFFFFFFFF) const;
   void  renderLine2D    (const ab::Vect4& from, const ab::Vect4& to, ab::DWord color = 0xFFFFFFFF) const;
   void  renderAxis      (const ab::Matr4& localMatr, ab::Real axisLen) const;
   void  renderSphereWire(const ab::Vect4& center, ab::Real radius, ab::DWord color = 0xFFFFFFFF, int slices = 10, int stacks = 10);

   void  renderText      (const ab::Vect4& pos, const char * str, ab::DWord color = 0xFFFFFFFF);

protected:
   Camera*    _pCamera;
};


} // namespace rnd

#endif //_RENDER_H_
