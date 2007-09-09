#ifndef _RENDER_OBJ_H_
#define _RENDER_OBJ_H_

#include "ab_types.h"


// forvard
namespace ab {
    class Matr4;
}

namespace ab {
class  Link;
}

namespace rnd {

class RenderObject {
public:
   explicit RenderObject() {;}
   virtual ~RenderObject() {;}
   
   virtual const ab::Matr4& getLocalMatr() const = 0;

   virtual void  render(Renderer& renderer) = 0;
protected:
};

// specializations
struct LinkRenderParams{   
   ab::Vect4 vert1;
   ab::Vect4 vert2;
};

class LinkRenderer : public RenderObject {
public:
   explicit LinkRenderer(const LinkRenderParams& linkData, const ab::Link * owner_);

   virtual const ab::Matr4& getLocalMatr() const; 
   virtual void  render(Renderer& renderer);

   void          setColor(ab::DWord color);
protected:
   const ab::Link*  _owner;
   LinkRenderParams _data;
   ab::DWord        _color;
};

} // namespace rnd

#endif //_RENDER_OBJ_H_
