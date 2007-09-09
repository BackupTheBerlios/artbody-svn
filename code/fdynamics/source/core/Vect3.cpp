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

#include "Vect3.h"

using namespace mrt;

///////////////////////////////////////////////////////////////////////////////
//  constants
///////////////////////////////////////////////////////////////////////////////


#define STR_LENGTH 1000  // for i/o operations

const Vect3 Vect3::ZERO(0, 0, 0);
const Vect3 Vect3::I   (1, 0, 0);
const Vect3 Vect3::J   (0, 1, 0);
const Vect3 Vect3::K   (0, 0, 1);
const Vect3 Vect3::I_  (-1,  0,  0);
const Vect3 Vect3::J_  ( 0, -1,  0);
const Vect3 Vect3::K_  ( 0,  0, -1);


///////////////////////////////////////////////////////////////////////////////
//
//  class Vect3
//
///////////////////////////////////////////////////////////////////////////////


//ostream& Vect3::print(ostream &os) const
//{
//  int oldFlags = os.setf(ios::showpos);
//  os << '(' << x << ' ' << y << ' ' << z << ')';
//  os.flags(oldFlags);
//  return os;
//}
//istream& Vect3::read(istream &is)
//{
//  char tok[STR_LENGTH];
//  char *code;
//  is >> tok;
//  if (tok[0] == '+' || tok[0] == '-') code = tok+1;
//  else code = tok;
//  if (*code >= 'i' && *code <= 'k' && *(code+1) == '\0') {
//    switch (*code) {
//    case 'i': *this = Vect3::I; break;
//    case 'j': *this = Vect3::J; break;
//    case 'k': *this = Vect3::K; break;
//    default: break;
//    }
//    if (tok[0] == '-') negate();
//    return is;
//  }
//  x = (Real) atof(tok);
//  return (is >> y >> z);
//}

