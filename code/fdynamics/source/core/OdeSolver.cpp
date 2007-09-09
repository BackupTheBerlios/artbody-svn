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

// Standard
#include <stdio.h>
#include <assert.h>
// this system
#include "OdeSolver.h"

using namespace mrt;

// default size for state array
const int DEFAULT_SIZE = 200;


// useful fractions
const Real ONE_HALF  = (1.0/2.0);
const Real ONE_THIRD = (1.0/3.0);
const Real ONE_SIXTH = (1.0/6.0);


//////////////////////////////////////////////////////////////////////////
// Constructor
//////////////////////////////////////////////////////////////////////////
OdeSolver::OdeSolver() :
  m_type(RK4)
{
  _size = 0;

  _ydot = NULL;
  _yint = NULL;

  _k1 = NULL;
  _k2 = NULL;
  _k3 = NULL;
  _k4 = NULL;

  _ReAllocate(DEFAULT_SIZE);
}


//////////////////////////////////////////////////////////////////////////
// Destructor
//////////////////////////////////////////////////////////////////////////
OdeSolver::~OdeSolver()
{
  if (_ydot) delete [] _ydot;
  if (_yint) delete [] _yint;
  if (_k1)   delete [] _k1;
  if (_k2)   delete [] _k2;
  if (_k3)   delete [] _k3;
  if (_k4)   delete [] _k4;
}

void OdeSolver::integrate(Real* y0, Real* yend, int len,
                          Real t0, Real t1, deriv_func dydt, copy_func user_copy_func, 
                          OdePointer clientData)
{
  switch(m_type)
  {
    case EULER:
    {
      Euler(y0, yend, len, t0, t1, dydt, clientData);
      break;
    }
    default:
    {
      assert(RK4 == m_type);
      RungeKutta4(y0, yend, len, t0, t1, dydt, user_copy_func, clientData);
      break;
    }
  }
}


//////////////////////////////////////////////////////////////////////////
// Euler : perform one step of Euler's method for solving ODEs
//////////////////////////////////////////////////////////////////////////
void OdeSolver::Euler(Real y0[], Real yend[], int len,
          Real t0, Real t1,
          deriv_func dydt, OdePointer clientData)

{
  // reallocate if necessary
  if (len > _size)
    _ReAllocate(len);

  Real h = t1 - t0;

  dydt(t0, y0, _ydot, clientData);

  for (int i = 0; i < len; i++) {
    yend[i] = y0[i] + h * _ydot[i];
  }
}


//////////////////////////////////////////////////////////////////////////
// RungeKutta : perform one step of the Fourth-order Runge-Kutta 
//   method for solving ODEs.
//////////////////////////////////////////////////////////////////////////
void OdeSolver::RungeKutta4(Real y0[], Real yend[], int len,
          Real t0, Real t1,
          deriv_func dydt, copy_func user_copy_func, OdePointer clientData)
{
  // reallocate if necessary
  if (len > _size)
    _ReAllocate(len);

  Real h = t1 - t0;
  Real h_2 = h * 0.5;

  // first iteration
  dydt(t0, y0, _ydot, clientData);
  _CalcK1(y0, h, len);
  user_copy_func(_yint, _ydot, clientData);
  _CalcK1(y0, h, len);

  // second iteration
  dydt(t0 + h_2, _yint, _ydot, clientData);
  _CalcK2(y0, h, len);
  user_copy_func(_yint, _ydot, clientData);
  _CalcK2(y0, h, len);

  // third iteration
  dydt(t0 + h_2, _yint, _ydot, clientData);
  _CalcK3(y0, h, len);
  user_copy_func(_yint, _ydot, clientData);
  _CalcK3(y0, h, len);  

  // fourth iteration
  dydt(t0 + h, _yint, _ydot, clientData);
  _CalcK4(y0, h, len);
  user_copy_func(_yint, _ydot, clientData);
  _CalcK4(y0, h, len);

  int i;
  for (i = 0; i < len; i++) {
    yend[i] = y0[i] + (_k1[i] * ONE_SIXTH) + (_k2[i] * ONE_THIRD)
                    + (_k3[i] * ONE_THIRD) + (_k4[i] * ONE_SIXTH);
  }
}


////////////////
//
// PRIVATE METHODS



//////////////////////////////////////////////////////////////////////////
// _ReAllocate : reallocate storage for intermediate values
//////////////////////////////////////////////////////////////////////////
void OdeSolver::_ReAllocate(int newSize)
{
  if (_ydot) delete [] _ydot;
  if (_yint) delete [] _yint;
  if (_k1)   delete [] _k1;
  if (_k2)   delete [] _k2;
  if (_k3)   delete [] _k3;
  if (_k4)   delete [] _k4;

  _size = newSize;

  _ydot = new Real[_size];
  _yint = new Real[_size];
  _k1 = new Real[_size];
  _k2 = new Real[_size];
  _k3 = new Real[_size];
  _k4 = new Real[_size];
}

//////////////////////////////////////////////////////////////////////////
// _CalcK1() - calc 'K1' coeff for Runge-Kutta4 method
//////////////////////////////////////////////////////////////////////////
void OdeSolver::_CalcK1(Real y0[], Real h, int len)
{
    int i;
    for (i = 0; i < len; i++) {    // first iteration
        _k1[i] = h*_ydot[i];
    }

    for (i = 0; i < len; i++) {
        _yint[i] = y0[i] + _k1[i] * ONE_HALF;
    }
    return;
}   

void OdeSolver::_CalcK2(Real y0[], Real h, int len)
{
    int i;
    for (i = 0; i < len; i++) {    // second iteration
        _k2[i] = h*_ydot[i];
    }

    for (i = 0; i < len; i++) {
        _yint[i] = y0[i] + _k2[i] * ONE_HALF;
    }
    
    return;
}

void OdeSolver::_CalcK3(Real y0[], Real h, int len)
{
    int i;
    for (i = 0; i < len; i++) {    // third iteration
        _k3[i] = h*_ydot[i];
    }

    for (i = 0; i < len; i++) {
        _yint[i] = y0[i] + _k3[i];
    }
    return;
}

void OdeSolver::_CalcK4(Real y0[], Real h, int len)
{
    int i;
    for (i = 0; i < len; i++) {    // fourth iteration
        _k4[i] = h*_ydot[i];
    }    
    return;
}

