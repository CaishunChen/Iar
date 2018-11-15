#ifndef __LIBRARY_FILTER_H
#define __LIBRARY_FILTER_H

#include "math.h"
//#include "filter.h"

typedef struct
{
	 float           fc;
	 float           a1;
	 float           a2;
	 float           b0; 
	 float           b1;
	 float           b2;
	 float           y_1;        
	 float           y_2;       
}IIR_coeff_Typedef;

void  cal_iir_coeff(IIR_coeff_Typedef *coeff,float fs, float fc);
float get_iir_output(IIR_coeff_Typedef* coeff,float sample);

#endif
