//
// Created by vassilis on 09.03.16.
//

#ifndef SELECTIVE_SOLDERING_PICKNPLACEFILEPARSER_H
#define SELECTIVE_SOLDERING_PICKNPLACEFILEPARSER_H

#endif //SELECTIVE_SOLDERING_PICKNPLACEFILEPARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "csvparser/csvparser.h"
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <regex.h>

#define DESIGNATOR_COLUMN 0
#define X_COLUMN 6
#define Y_COLUMN 7

int getCoordinatesfromFile(char* filename, int* componentSum, int* fiducialSum, float coordinates[][2], float fiducials[][2]);
int regExMatcher(char* designator);


#ifdef __cplusplus
}
#endif

