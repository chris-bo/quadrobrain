/*
 * PIDController.h
 *
 *  Created on: 22.03.2015
 *      Author: Jonas
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include "Task.h"
#include "config.h"

class PIDController: public Task {
public:
    PIDController(Status* statusPtr, uint8_t defaultPrio, float _sampleTime,
                float* _input, float* _derivedInput, float* _reference,
                float* _output, float _limit, bool _useDerivedInput);
    virtual ~PIDController();
    void update();
    void initialize(float* _p, float* _i, float* _d);
    void kill();
private:
    Status* status;
    float* p; 				// P-Anteil
    float* i; 				// I-Anteil
    float* d;				// D-Anteil
    float* input;			// Messgroesse (Ist-Wert)
    float* derivedInput;// Ableitung der Messgroesse (falls vorhanden, ansonsten null setzen)
    float* reference;		// Soll-Wert
    float* output;			// Ausgabewert des Reglers
    float limit;			// Reglerlimit
    float sampleTime;		// Abtastzeit
    float sum;				// Summe der Abweichung (fuer I-Anteil)
    float oldValue;			// Alter Ist-Wert (fuer Berechnung der Ableitung)
    bool useDerivedInput;// true: Ableitung wird nicht berechnet, sondern direkt aus *derivedInput verwendet
};

#endif /* PIDCONTROLLER_H_ */
