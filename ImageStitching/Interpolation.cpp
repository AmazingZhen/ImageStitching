#include "stdafx.h"
#include "Interpolation.h"

double sinxx(double value) {
	if (value < 0)
		value = -value;

	if (value < 1.0) {
		double temp = value * value;
		return 0.5 * temp * value - temp + 2.0 / 3.0;
	}
	else if (value < 2.0) {
		value = 2.0 - value;
		value *= value * value;
		return value / 6.0;
	}
	else {
		return 0.0;
	}
}