#ifndef _HDL_COLLECT_H_
#define _HDL_COLLECT_H_

#include <vector>

namespace sys {

template <class T>
class HdlCollection {
public:
   explicit HdlCollection(void);
   virtual ~HdlCollection();

   virtual bool  addHdl   (T* pHdl);
   virtual bool  removeHdl(T* pHdl);
protected:   
   int _findHdl(T* pHdl);
   std::vector<T*> _hdlArray;
};

template <class T>
HdlCollection<T>::HdlCollection()
{
   ;
}

template <class T>
HdlCollection<T>::~HdlCollection()
{
   ;
}


template <class T>
bool HdlCollection<T>::addHdl(T* pHdl)
{
   if (_findHdl(pHdl) < 0) {
      _hdlArray.push_back(pHdl);
   }

   return true;
}

template <class T>
bool HdlCollection<T>::removeHdl(T* pHdl)
{
   unsigned int hdlIdx = _findHdl(pHdl);
   if (hdlIdx >= 0) {
      _hdlArray.erase(_hdlArray.begin() + hdlIdx);
      return true;
   }
   return false;
}

template <class T>
int HdlCollection<T>::_findHdl(T* pHdl)
{
   unsigned int i = 0;
   for (i = 0; i < _hdlArray.size(); i++) {
      if (pHdl == _hdlArray[i]) {
         return i;
      }
   }
   return -1;
}

} // namespace sys

#endif //_HDL_COLLECT_H_
