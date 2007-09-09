#ifndef _AB_ANTHROPOMETRIC_H_
#define _AB_ANTHROPOMETRIC_H_

// standard
#include <stdio.h>
// this system
#include "ab_types.h"

namespace ab
{

// Specifiers used in OpenInfo() and SegmentInfo() functions.
// Specifiers for SegmentInfo() must be in exactly the same order, as they goes in information file.
// It's unnecessary to define left leg and right hand in file
enum 
{
	// OpenInfo() specifiers
	MALE = 1,
	FEMALE,
	CHI,
	GER,	
	// SegmetInfo() specifiers
	HEAD = 0,
	UT,							//upper torso
	MT,							//middle torso
	LT,							//lower torso
	LTHIGH,	
	LSHANK,
	LFOOT,
	RUA,						//upper arm
	RFA,						//forearm
	RHAND,
	// Numerical values for bellow segments are equal to previously defined (LTHIGH .. RHAND)
	RTHIGH,						
	RSHANK,
	RFOOT,
	LUA,
	LFA,
	LHAND,	

	TOTAL_SEGMENTS_NUM,			//Total number of segments
	UNI_SEGMENTS_NUM = RTHIGH	//Left copys of arm and leg
};


// TYPE DEFINITIONS


// This type saves information from file for one element
typedef struct ANT_INFO {
	float mass;			//mass of an element
	float length;		//its length
	float ix, iy, iz;	//its moment of inertia by x,y,z
	float rcm;			//relative length of CM from the top of element
} ANT_INFO;

typedef struct ANT_DESK {
	FILE *seg;
	FILE *cm;
	FILE *model;
	int id;
} ANT_DESK;

// END OF TYPE DEFINITIONS







// COMMON INTERFACE FUNCTIONS


/*
   ANT_DESK OpenInfo()
   Opens file for reading, corresponding previously defined paths, gender and nation specifiers (use MALE\FEMALE, CHI\GER)
   on error: returns { NULL, NULL, NULL, 0} structure

   Ex: ANT fp = OpenInfo(MALE, CHI);
*/
ANT_DESK OpenInfo(int gender, int nation);


/*
   void CloseInfo()
   Close all information files
*/
void CloseInfo(ANT_DESK *info);

/*
  ANT_INFO SegmentInfo()
  Returns information after calculations.
  mas is person weight in kg,
  heigh is person height in m.
  on error: returns zero mass ( .mass = 0 )

  Ex: ANT_INFO footInf = SegmentInfo(75, 175, Foot, fp);
*/

ANT_INFO SegmentInfo(const float mas, const float heigh, int segment, ANT_DESK info);


/*
  int FullSegmentInfo()
  Returns information about segment and it's bounds.
  Coordinates is relative. (0, 0, 0) is eqal to CM of segment. 
  Positive direction af axis:
		x - goes towards
		y - goes right
		z - goes top
  segment is a specifier of segment
  mas is person weight in kg,
  heigh is person height in m.
  pMas is array of bounds coordinates (will be calculated after function call, memory must be allocated before function call)
  pNum is total number of segment bounds element (will be calculated after function call)
  on error: returns zero

  Ex:	int pNum;
		float pMas[10];
		ANT_INFO antrop;
		int FullHeadInf = SegmentInfo(info, HEAD, 75, 1.75, pMas, &pNum, &antrop);
*/
int FullSegmentInfo(ANT_DESK info, int segment, float mas, float heigh, float *pMas, int *pNum, ANT_INFO *antrop);


// END OF INTERFACE FUNCTIONS







// INLINE FUNCTIONS USED DUARING CALCULATIONS 


// Calculate value using coefficients of regression formula
inline float RegressionResult(float mas, float heigh, float b0, float b1, float b2)
{
	return b0 + b1 * mas + b2 * heigh;
} // End of RegressionResult


// Shifts nStrings lines down through file
inline void ShiftDown(FILE *file, const int nStrings, int iter)
{
	for (iter = 0; iter < nStrings; iter++)
		while (fgetc(file) != '\n');

} // End of ShiftDown

// Reads b0, b1, b2 coefficients from one string in CSV format:
// string;float;float;float
inline int ReadInfo(FILE *file, float *b0, float *b1, float *b2)
{
	while (fgetc(file) != ';');
	return fscanf(file,"%f;%f;%f", b0, b1, b2);
} // End of ReadInfo


// Returns ANT_INFO, with zero mass.
inline ANT_INFO *ReturnReadError(ANT_INFO *a)
{
	a->mass = 0;
	return a;
} // End of ReturnReadError


// Resets all elements of ANT_INFO to zero.
inline void ResetANT(ANT_INFO *a)
{
	a->mass = 0;
	a->length = 0;
	a->ix = 0;
	a->iy = 0;
	a->iz = 0;
	a->rcm = 0;
}
// END OF INLINE FUNCTIONS USED DUARING CALCULATIONS 

} // end of namespace ab

// SOME TESTING RESULTS


// Tests info (75kg, 175cm):
//
//	             |MALE CHI | MALE GER | FEMALE CHI | FEMALE GER
//	------------------------------------------------------------
//	Total mass   |4.8      | 74.64    | 74.35      | 75.01
//  ------------------------------------------------------------
//	Total heigh  |178      | 177      | 180        | 180
//	------------------------------------------------------------

// END OF SOME TESTING RESULTS
#endif // _AB_ANTHROPOMETRIC_H_
