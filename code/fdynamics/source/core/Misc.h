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


#ifndef MISC_H
#define MISC_H

//#ifndef DEPEND_IGNORE
#include <stdlib.h>
//#include <iostream.h>
#include <vector>
//#endif

#include <math.h>

namespace mrt {

///////////////////////////////////////////////////////////////////////////////
// container stuff
///////////////////////////////////////////////////////////////////////////////


#define FOR_EACH(container, iterator) \
  for(iterator = (container).begin(); iterator != (container).end(); ++iterator)


///////////////////////////////////////////////////////////////////////////////
// crash and burn
///////////////////////////////////////////////////////////////////////////////

inline void crash(char *msg, 
		  char *msg2 = NULL,
		  char *msg3 = NULL,
		  char *msg4 = NULL,
		  char *msg5 = NULL,
		  char *msg6 = NULL,
		  char *msg7 = NULL,
		  char *msg8 = NULL)
{
//  cout << "\nfatal error:\n";
//  cout << msg << msg2 << msg3 << msg4 << msg5 << msg6 << msg7 << msg8 << endl;
  exit(1);
}

} // namespace mrt


#endif  // #ifndef MISC_H
