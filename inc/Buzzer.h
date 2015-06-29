/*
 * Buzzer.h
 *
 *  Created on: 26.04.2015
 *      Author: Jonas
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#include <Task.h>

#define BUZZER_B5	987.767f
#define BUZZER_Ap5	932.328f
#define BUZZER_A5	880.000f
#define BUZZER_Gp5	830.609f
#define BUZZER_G5	783.991f
#define BUZZER_Fp5	739.989f
#define BUZZER_F5	698.456f
#define BUZZER_E5	659.255f
#define BUZZER_Dp5	622.254f
#define BUZZER_D5	587.330f
#define BUZZER_Cp5	554.365f
#define BUZZER_C5	523.251f
#define BUZZER_B4	493.883f
#define BUZZER_Ap4	466.164f
#define BUZZER_A4	440.000f
#define BUZZER_Gp4	415.305f
#define BUZZER_G4	391.995f
#define BUZZER_Fp4	369.994f
#define BUZZER_F4	349.228f
#define BUZZER_E4	329.628f
#define BUZZER_Dp4	311.127f
#define BUZZER_D4	293.665f
#define BUZZER_Cp4	277.183f
#define BUZZER_C4	261.626f
#define BUZZER_B3	246.942f
#define BUZZER_Ap3	233.082f
#define BUZZER_A3	220.000f
#define BUZZER_Gp3	207.652f
#define BUZZER_G3	195.998f
#define BUZZER_Fp3	184.997f
#define BUZZER_F3	174.614f
#define BUZZER_E3	164.814f
#define BUZZER_Dp3	155.563f
#define BUZZER_D3	146.832f
#define BUZZER_Cp3	138.591f
#define BUZZER_C3	130.813f
#define BUZZER_B2	123.471f
#define BUZZER_Ap2	116.541f
#define BUZZER_A2	110.000f
#define BUZZER_Gp2	103.826f
#define BUZZER_G2	97.9989f
#define BUZZER_Fp2	92.4986f
#define BUZZER_F2	87.3071f
#define BUZZER_E2	82.4069f
#define BUZZER_Dp2	77.7817f
#define BUZZER_D2	73.4162f
#define BUZZER_Cp2	69.2957f
#define BUZZER_C2	65.4064f

class Buzzer: public Task {
public:
	Buzzer(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim);
	void update();
	void playToneOnBuzzer1(float frequency, uint16_t length);
	void playToneOnBuzzer2(float frequency, uint16_t length);
	virtual ~Buzzer();

private:
	TIM_HandleTypeDef* Buzzer_htim;
	uint16_t calculateReloadValue(float frequency);
	uint16_t toneLengthBuzzer1;
	uint16_t toneLengthBuzzer2;
	uint16_t elapsedLengthBuzzer1;
	uint16_t elapsedLengthBuzzer2;
};

#endif /* BUZZER_H_ */
