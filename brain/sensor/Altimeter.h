/*
 * Altimeter.h
 *
 *  Created on: Nov 17, 2017
 *      Author: chris
 */

#ifndef ALTIMETER_H_
#define ALTIMETER_H_

#include <core/Task.h>

class Altimeter: public Task {
public:
	Altimeter(Status* _status, int8_t _defaultPrio);
	virtual ~Altimeter();

    void initialize();
    void update();
    void reset();


private:
    float height_start;
};

#endif /* ALTIMETER_H_ */
