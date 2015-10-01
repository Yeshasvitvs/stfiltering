/*
 * spatialFilter.hpp
 *
 *  Created on: April 16, 2015
 *      Author: yeshasvi tvs
 */


#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <malloc.h>
#include <memory.h>
#include <utility>
#include <tuple>

#if defined MAX_RES
#else
#define MAX_RES 128
#endif

#if defined MAX_SIZE
#else
#define MAX_SIZE 31
#endif

#define _USE_MATH_DEFINES

#define MAX_THETA 2*M_PI //Maximum theta value is 360 degrees
using namespace std;


#ifndef STFILTERS_H
#define STFILTERS_H

class stFilters{

    private :

    public:

        //Data members
        //Spatial filter parameters
        int theta;
        int center_x, center_y;
        double f_spatial;
        double var_spatial;

        //Temporal filter parameters
        double f_temporal;
        double var_temporal;


        //Member functions

        //Constructor definition
        stFilters(double frequency_spatial = 0.08, double variance_spatial = 6.5, double frequency_temporal = 0.8, double variance_temporal = 5){

            std::cout<<"Spatial-Temporal Filter parameters initialization..."<<std::endl;//Debug Code

            //Spatial filter parameters
            f_spatial = frequency_spatial;
            var_spatial = variance_spatial;


            //Temporal filter parameters
            f_temporal = frequency_temporal;
            var_temporal = variance_temporal;

 	    std::cout<<"Parameters : "<< f_spatial << " " << var_spatial << " "<< f_temporal << " "<<var_temporal <<std::endl;//Debug Code	 


        }
        virtual ~stFilters();//Destructor

        std::pair<double,double> filtering(int&,int&,double&,double&); //X,Y,theta and time





};



#endif // STFILTERS_H
