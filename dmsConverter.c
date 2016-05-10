#include "dmsConverter.h"

void dmsLoad(char *filename) {
	dmsFile = fopen(filename, "r");

	int count = 0;
	while (fscanf(dmsFile, "%f %f' %f\"", &dmsCoords[count].degrees, &dmsCoords[count].minutes, &dmsCoords[count].seconds) != EOF) {
		if (count == DMS_MAX_COORDS - 1) {
			break;
		}
		count++;
	}
	dmsNumCoords = count;
}

void dmsConvert() {
    for(int i = 0; i < dmsNumCoords; i++){
        printf("%f %f' %f\" = ", dmsCoords[i].degrees, dmsCoords[i].minutes, dmsCoords[i].seconds);
		dmsDecimalCoords[i] = abs(dmsCoords[i].degrees) + (dmsCoords[i].minutes/60) + (dmsCoords[i].seconds/3600);
		if (dmsCoords[i].degrees < 0) dmsDecimalCoords[i] *= -1;
		printf("%f\n", dmsDecimalCoords[i]);
    }
}
