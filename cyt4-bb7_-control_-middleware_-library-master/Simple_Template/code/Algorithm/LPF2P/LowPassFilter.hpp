/*
 * LowPassFilter.hpp
 *
 *  Created on: 2023年3月14日
 *      Author: Zunrong Guo
 */

#ifndef _LOWPASSFILTER_HPP_
#define _LOWPASSFILTER_HPP_

#pragma once

// #include "main.h"
#include <cmath>
#include "arm_math.h"
#include <float.h>

#define M_PI_F	3.14159265f

template<typename T>
class LowPassFilter2p
{
public:

	LowPassFilter2p(float sample_freq, float cutoff_freq)
	{
		// set initial parameters
		initParam();
		set_cutoff_frequency(sample_freq, cutoff_freq);
	}

	// Change filter parameters
	void set_cutoff_frequency(float sample_freq, float cutoff_freq)
	{
		// reset delay elements on filter change
		_delay_element_1 = 0;
		_delay_element_2 = 0;

		_cutoff_freq = cutoff_freq;
		_sample_freq = sample_freq;

		const float fr = _sample_freq / _cutoff_freq;
		const float ohm = arm_sin_f32(M_PI_F / fr) / arm_cos_f32(M_PI_F / fr);
		const float c = 1.f + 2.f * arm_cos_f32(M_PI_F / 4.f) * ohm + ohm * ohm;

		_b0 = ohm * ohm / c;
		_b1 = 2.f * _b0;
		_b2 = _b0;

		_a1 = 2.f * (ohm * ohm - 1.f) / c;
		_a2 = (1.f - 2.f * arm_cos_f32(M_PI_F / 4.f) * ohm + ohm * ohm) / c;
	}

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	inline T apply(const T sample)
	{
		// Direct Form II implementation
		T delay_element_0 = sample - _delay_element_1 *_a1 - _delay_element_2 * _a2;

		const T output = delay_element_0 *_b0 + _delay_element_1 *_b1 + _delay_element_2 * _b2;

		_delay_element_2 = _delay_element_1;
		_delay_element_1 = delay_element_0;

		return output;
	}

	// Reset the filter state to this value
	T reset(const T &sample)
	{
		if (fabsf(1 + _a1 + _a2) > FLT_EPSILON) {
			_delay_element_1 = _delay_element_2 = sample / (1 + _a1 + _a2);
		}
		else {
			_delay_element_1 = _delay_element_2 = sample;
		}

		return apply(sample);
	}

protected:
	void initParam()
	{
		_delay_element_1 = 0; // buffered sample -1
		_delay_element_2 = 0; // buffered sample -2
	
		// All the coefficients are normalized by a0, so a0 becomes 1 here
		_a1 = 0.0f;
		_a2 = 0.0f;
		_b0 = 1.0f;
		_b1 = 0.0f;
		_b2 = 0.0f;
		_cutoff_freq = 0.0f;
		_sample_freq = 0.0f;
	}
	T _delay_element_1; // buffered sample -1
	T _delay_element_2; // buffered sample -2

	// All the coefficients are normalized by a0, so a0 becomes 1 here
	float _a1;
	float _a2;

	float _b0;
	float _b1;
	float _b2;

	float _cutoff_freq;
	float _sample_freq;
};



#endif
