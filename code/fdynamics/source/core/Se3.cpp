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

#include <math.h>

#include "Quat.h"
#include "Se3.h"

//#ifndef DEPEND_IGNORE
//#include <strstream.h>
//#endif //DEPEND_IGNORE

using namespace mrt;

///////////////////////////////////////////////////////////////////////////////
//  constants
///////////////////////////////////////////////////////////////////////////////

static const double DEG_TO_RAD = (3.14158 / 180.0);

#define STR_LENGTH 1000  // for i/o operations

const Se3  Se3::ID    (Se3::getIDSe3());
//const Se3  Se3::ID    (Quat(1.0, 0.0, 0.0, 0.0), Vect3::ZERO);


///////////////////////////////////////////////////////////////////////////////
//
//  class Se3
//
///////////////////////////////////////////////////////////////////////////////
void Se3::set(const Se3 &another) 
{ 
    q.set(another.q); 
    d = another.d; 
}

void Se3::set(const Quat &q_, const Vect3 &d_)
{
    q.set(q_); 
    d = d_;
}


//istream& Se3::read(istream &is)
//{
//  char c;
//  int i;
//  Real x;
//  Vect3 vect;
//  Quat quat;
//  Se3 op;
//  char buffer[STR_LENGTH];
//  char tok[STR_LENGTH];
//  
//  *this = Se3::ID;
//  is >> ws;
//  if (is.peek() == '{') is.get(c);
//  else {
//    cerr << "Se3::read : didn't find '{' \a" << endl;
//    return is;
//  }
//    
//  // read until closing '}'
//  i = 0;
//  while (1) {
//    is.get(c);
//    if (c == '}') {
//      buffer[i++] = 0;
//      break;
//    }
//    buffer[i++] = c;
//    if (i == STR_LENGTH || is.eof()) {
//      cerr << "Se3::read : didn't find '}' or specification too long\a" << endl;
//      return is;
//    }
//  }
//  istrstream iss(buffer);
//  
//  while (!((iss >> tok).fail())) {
//    if (!strcmp(tok, "trans")) {
//      iss >> vect;
//      op.set(Quat::ID, vect);
//    }
//    else if (!strcmp(tok, "rot")) {
//      iss >> tok >> vect;
//      x = atof(tok) * DEG_TO_RAD;
//      quat.set(x, vect);
//      op.set(quat, Vect3::ZERO);
//    }
//    else {
//      cerr << "Se3::read : unknown token " << tok << " \a" << endl;
//      break;
//    }
//    postmult(op);
//    q.normalize();  // just to be sure.
//  }
//  return is;
//}

