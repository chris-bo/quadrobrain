/*
 * Compass.h
 *
 *  Created on: Apr 28, 2015
 *      Author: bohni
 */

#ifndef COMPASS_H_
#define COMPASS_H_

#include <sensor/ComplementaryFilter.h>
#include <utility/MAfilterF.h>

class Compass: public ComplementaryFilter {
public:
    Compass(Status* statusPtr, uint8_t defaultPrio, float* magn1, float* magn2,
            float* ang1, float* ang2, float* rotation, float* output,
            float* filter_coefficient);
    virtual ~Compass();
    void update();

private:
    float* angle1;
    float* angle2;

    /* setup for filters*/
    float tmpMag1;
    float tmpMag2;

    MAfilterF* mag1Filter;
    MAfilterF* mag2Filter;
};

#endif /* COMPASS_H_ */
