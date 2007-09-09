///////////////////////////////////////////////////////////////////////////////
//
//  Copyright 1998 Mitsubishi Electric Information Technology Center
//  America (MEITCA).  All Rights Reserved.
//
//  Permission to use, copy, modify and distribute this software and
//  its documentation for educational, research and non-profit
//  purposes, without fee, and without a written agreement is hereby
//  granted, provided that the above copyright notice and the
//  following three paragraphs appear in all copies.
//
//  Permission to incorporate this software into commercial products
//  may be obtained from MERL - A Mitsubishi Electric Research Lab, 201
//  Broadway, Cambridge, MA 02139.
//
//  IN NO EVENT SHALL MEITCA BE LIABLE TO ANY PARTY FOR DIRECT,
//  INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
//  LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
//  DOCUMENTATION, EVEN IF MEITCA HAS BEEN ADVISED OF THE POSSIBILITY
//  OF SUCH DAMAGES.
//
//  MEITCA SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON
//  AN "AS IS" BASIS, AND MEITCA HAS NO OBLIGATIONS TO PROVIDE
//  MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
//  Author:
//    Brian Mirtich
//    mirtich@merl.com
//    617.621.7573
//    www.merl.com/people/mirtich
//
//  Library and API:
//    James Kuffner, Jr.
//    kuffner@stanford.edu
//    650.725.8812
//    http://robotics.stanford.edu/~kuffner
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ODESOLVER_H
#define ODESOLVER_H

#include <math.h>

#include "RealType.h"

namespace mrt 
{

    // the simulation state derivative function
    // you should define one that takes exactly the arguments defined
    // below and and pass its pointer to the integrator function.

    typedef const void * OdePointer;
    typedef void (*deriv_func)(Real t,             // time           
        const Real y[],     // state vector
        Real ydot[],        // derivative vector
        OdePointer cd);     // client data

    typedef void (*copy_func)(
            Real y[],     // state vector
            Real ydot[],  // derivative vector
            OdePointer cd // client data
        );


  //////////////////////////////////////////////////////////////////////
  // OdeSolver : class for solving Ordinary Differential Equations
  //////////////////////////////////////////////////////////////////////

  class OdeSolver
  {
  public:
    // Type
    enum TYPE
    {
      EULER = 0,
      RK4
    };

    // constructors / destructors
    OdeSolver();
    ~OdeSolver();

    // 
    void setType(TYPE type) { m_type = type; }

    // Common integration interface
    void integrate(Real* y0, Real* yend, int len,
                   Real t0, Real t1, deriv_func dydt, copy_func user_copy_func, 
                   OdePointer clientData);

  private:
    // state integration methods
    void Euler(Real y0[], Real yend[], int len, Real t0, Real t1,
               deriv_func dydt, OdePointer clientData);

    void RungeKutta4(Real y0[], Real yend[], int len, Real t0, Real t1,
                     deriv_func dydt, copy_func user_copy_func, OdePointer clientData);

    // methods
    void _ReAllocate(int newSize);

    // support for Runge-Kutta 4
    void _CalcK1(Real y0[], Real h, int len);
    void _CalcK2(Real y0[], Real h, int len);
    void _CalcK3(Real y0[], Real h, int len);
    void _CalcK4(Real y0[], Real h, int len);

    //data
    int    _size;  //the size of the state variable array allocated

    Real* _ydot;  //the derivatives
    Real* _yint;
    Real* _y_prev; // intermediate

    Real* _k1;    //the iterations of Runge-Kutta
    Real* _k2;
    Real* _k3;
    Real* _k4;

    TYPE m_type;
  };

} // namespace mrt

#endif  // ODESOLVER_H


