//convert lat/long from DMS to decimal
#include <math.h>
#include <stdio.h>

#define DMS_MAX_COORDS 100

struct DMS
{
	float degrees;
	float minutes;
	float seconds;
};

FILE *dmsFile;
struct DMS dmsCoords[DMS_MAX_COORDS];
int dmsNumCoords;
float dmsDecimalCoords[DMS_MAX_COORDS];

void dmsLoad(char *filename);
void dmsConvert();
