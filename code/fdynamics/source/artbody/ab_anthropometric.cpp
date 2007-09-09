// standard
#include <math.h>
// this system
#include "ab_anthropometric.h"

// Relative or full path to information files
#define ANTROPOMETRIC_FILES_MALE_CHI    "ChiMal.csv"     //"D:\\Projects\\pe\\fdynamics\\source\\artbody\\data\\ChiMal.csv"
#define ANTROPOMETRIC_FILES_MALE_GER    "GerMal.csv" //    "D:\\Projects\\pe\\fdynamics\\source\\artbody\\data\\GerMal.csv"
#define ANTROPOMETRIC_FILES_FEMALE_CHI  "ChiFem.csv" //    "D:\\Projects\\pe\\fdynamics\\source\\artbody\\data\\ChiFem.csv"
#define ANTROPOMETRIC_FILES_FEMALE_GER  "GerFem.csv" //    "D:\\Projects\\pe\\fdynamics\\source\\artbody\\data\\GerFem.csv"
#define ANTROPOMETRIC_FILES_CM_POSITION "CMPos.csv"  //    "D:\\Projects\\pe\\fdynamics\\source\\artbody\\data\\CMPos.csv"
#define ANTROPOMETRIC_FILES_MODEL       "modelChiMal.csv"//"D:\\Projects\\pe\\fdynamics\\source\\artbody\\data\\modelChiMal.csv"

namespace ab 
{

ANT_DESK OpenInfo(int gender, int nation)
{
	ANT_DESK t;
	
	switch (gender * nation)
	{
		case MALE * CHI:
			t.seg = fopen(ANTROPOMETRIC_FILES_MALE_CHI, "r");
			break;
		case MALE * GER:
			t.seg = fopen(ANTROPOMETRIC_FILES_MALE_GER, "r");
			break;
		case FEMALE * CHI:
			t.seg = fopen(ANTROPOMETRIC_FILES_FEMALE_CHI, "r");			
			break;
		case FEMALE * GER:
			t.seg = fopen(ANTROPOMETRIC_FILES_FEMALE_GER, "r");			
			break;
	}
	if (t.seg != NULL)
		t.cm = fopen(ANTROPOMETRIC_FILES_CM_POSITION, "r");

	if (t.cm != NULL && t.seg != NULL)
		t.model = fopen(ANTROPOMETRIC_FILES_MODEL, "r");

	if (t.cm == NULL || t.seg == NULL || t.model == NULL)
	{
		if (t.cm != NULL) 
			fclose(t.cm);
		if (t.seg != NULL)
			fclose(t.seg);
		if (t.model != NULL)
			fclose(t.model);
		t.id = 0;
		t.cm = t.seg = t.model = NULL;
	}	else {
		t.id = gender * nation;
	}
	return t;
} // End of OpenInfo()

void CloseInfo(ANT_DESK *info)
{
	if (info->cm != NULL) 
			fclose(info->cm);
	if (info->seg != NULL)
			fclose(info->seg);
	if (info->model != NULL)
			fclose(info->model);
	return;
} // End of CloseInfo()

ANT_INFO SegmentInfo(const float mas, const float hgh, int segment, ANT_DESK info)
{
	ANT_INFO a;
	int i = 0;
	float b0, b1, b2, height = hgh * 100;

	if (info.seg == NULL || info.cm == NULL || segment < 0 || segment >= TOTAL_SEGMENTS_NUM || mas < 0 || height < 0)
		*ReturnReadError(&a);
	
	if (segment >= UNI_SEGMENTS_NUM)
		segment -= TOTAL_SEGMENTS_NUM - UNI_SEGMENTS_NUM;

	rewind(info.seg);
	rewind(info.cm);

	ShiftDown(info.seg, 1, i);

	// Check of the bounds
	fscanf(info.seg, ";%f;%f", &b0, &b1);
	if (mas > b0 || mas < b1)
		return *ReturnReadError(&a);

	fscanf(info.seg, ";%f;%f\n", &b0, &b1);
	if (height > b0 || height < b1)
		return *ReturnReadError(&a);

	ShiftDown(info.seg, segment, i);
	// Mass calculation
	if (ReadInfo(info.seg, &b0, &b1, &b2) != 0)
		a.mass = RegressionResult(mas, height, b0, b1, b2);
	else 
		return *ReturnReadError(&a);

	ShiftDown(info.seg, UNI_SEGMENTS_NUM + 1, i);
	// Length calculation
	if (ReadInfo(info.seg, &b0, &b1, &b2) != 0)
		a.length = RegressionResult(mas, height, b0, b1, b2) / 100;
	else 
		return *ReturnReadError(&a);

	ShiftDown(info.seg, UNI_SEGMENTS_NUM + 1, i);
	// Ix calculation
	if (ReadInfo(info.seg, &b0, &b1, &b2) != 0)
		a.ix = RegressionResult(mas, height, b0, b1, b2) / (100 * 100);
	else 
		return *ReturnReadError(&a);

	ShiftDown(info.seg, UNI_SEGMENTS_NUM + 1, i);
	// Iy calculation
	if (ReadInfo(info.seg, &b0, &b1, &b2) != 0)
		a.iy = RegressionResult(mas, height, b0, b1, b2) / (100 * 100);
	else 
		return *ReturnReadError(&a);

	ShiftDown(info.seg, UNI_SEGMENTS_NUM + 1, i);
	// Iz calculation
	if (ReadInfo(info.seg, &b0, &b1, &b2) != 0)
		a.iz = RegressionResult(mas, height, b0, b1, b2) / (100 * 100);
	else 
		return *ReturnReadError(&a);
	
	// relative position reading
	ShiftDown(info.cm, segment + 2, i);
	while (fgetc(info.cm) != ';');
	if (fscanf(info.cm,"%f;%f;%f", &b0, &b1, &b2) != 3)
		return *ReturnReadError(&a);

	switch (info.id)
	{
		case MALE * CHI:
			a.rcm = a.length * b0 / 100;
			break;
		case FEMALE * CHI:
			a.rcm = a.length * b1 / 100;
			break;
		case MALE * GER:
			a.rcm = a.length * b2 / 100;
			break;
		case FEMALE * GER:
			if (fscanf(info.cm, ";%f", &b0) == 0)
				return *ReturnReadError(&a);
			a.rcm = a.length * b0 / 100;
			break;
	}
	
	return a;
} // End of SegmentInfo()

int FullSegmentInfo(ANT_DESK info, int segment, float mas, float heigh, float *pMas, int *pNum, ANT_INFO *antrop)
{
	int i = 0;
	float persent, norm;
	*antrop = SegmentInfo(mas, heigh, segment, info);
	if (fabs(antrop->mass) < 0.000001)
		return 0;

	if (segment >= UNI_SEGMENTS_NUM)
		segment -= TOTAL_SEGMENTS_NUM - UNI_SEGMENTS_NUM;

	rewind(info.model);

	ShiftDown(info.model, segment, i);
	while (fgetc(info.model) != ';');
	if (!fscanf(info.model, "%d", pNum))
		return 0;
	for (i = 0; i < *pNum; i++)
	{
		if (fscanf(info.model,";%f;%f;%f;%f", pMas + 3*i, pMas + 3*i + 1, pMas + 3*i + 2, &persent) != 4)
			return 0;
		norm = sqrt(pMas[3*i]*pMas[3*i] + pMas[3*i + 1]*pMas[3*i + 1] + pMas[3*i + 2]*pMas[3*i + 2]);
		norm = (persent * antrop->length) / (norm * 100);
//        norm = (antrop->length) / (norm * 100);
		pMas[3*i] *= norm;
		pMas[3*i + 1] *= norm;
		pMas[3*i + 2] *= norm;
	}
	return 1;
}

} // end of namespace ab