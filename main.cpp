//============================================================================
// Name        : eventGabor.cpp
// Author      : yeshasvi tvs
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#if defined MAX_RES
#else
#define MAX_RES 128
#endif


#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <tuple>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/conf/options.h>
#include <iCub/emorph/all.h>

#include <opencv/cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "eventHistoryBuffer.hpp"
#include "stFilters.h"
#include "icubInterface.hpp"


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace emorph;
using namespace std;
using namespace cv;

//Global variables

//Event History buffer instance
eventHistoryBuffer event_history;
int kernel_size = 31;

//Number of directions
int number_directions = 8;
double theta_step = 0.25;
int theta_index;
std::vector<double> theta_vector;
std::vector<double>::iterator theta_vector_it;

emorph::vtsHelper unwrap;
double event_time;

//Global variables
double final_convolution;
double even_conv,odd_conv;
std::vector<double> convolution; //Vector used for flow computation
std::vector<double>::iterator convolution_it;
void motionHypothesis(int&,int&);

//Instance of stFitlering class with filter parameters as arguments
stFilters st_filters(0.0625, 6.5, 0.0625*20, 5); //Parameters, f_s, sigma_s, f_t, sigma_


int main(int argc, char *argv[])
{

   //Resource finder
   yarp::os::ResourceFinder rf;
   rf.configure(argc,argv);

   //Set up yarp network
   yarp::os::Network yarp;

   //Variables
   yarp::os::BufferedPort<emorph::vBottle> inputPort;
   yarp::os::BufferedPort<emorph::vBottle> outputPort;


   emorph::vBottle *tempEvents; //Temporary events


   //Using Resource finder for port names
   string inputPortName = rf.find("inputPort").asString();
   string outputPortName = rf.find("outputPort").asString();
   string sourcePortName = rf.find("sourcePort").asString();

   //Setting default port names
   if(inputPortName == ""){ inputPortName = "/inputEvents"; }
   if(outputPortName == ""){ outputPortName = "/outputEvents"; }
   //Old Grabber
   if(sourcePortName == ""){ sourcePortName = "/aexGrabber/vBottle:o"; }
   //New Grabber
   //if(sourcePortName == ""){ sourcePortName = "/zynqGrabber/vBottle:o"; }


   //Port connections
   bool ok = inputPort.open(inputPortName.c_str()); //Opening input port
   ok = ok && outputPort.open(outputPortName.c_str()); //Opening output port


   //checking ports
   if (!ok) {
    fprintf(stderr, "Failed to create ports.\n");
    fprintf(stderr, "Maybe you need to start a nameserver (run 'yarpserver')\n");
    return 1;
   }

   if(yarp.connect(sourcePortName.c_str(), inputPortName.c_str(), "tcp")){

    std::cout << "source port to input port connection successful... " <<std::endl;

   }
   else{std::cout << "source port to input port connection failed! " <<std::endl;}

   //Setting the theta vector based on the number of directios
   double value = 0;
   for(int p=1; p<=number_directions ;p++){ //Loop runs for number of directions

       //value = value + theta_step; //Line here in case of testing with single direction i.e theta
       theta_vector.push_back( value * M_PI);
       value = value + theta_step; //Line here in case all directions and with fixed theta step

   }



   while(true){ //TODO wait till the input port receives data and run


    tempEvents = inputPort.read(true); //Read the incoming events into a temporary vBottle
    //std::cout << "received bottle" << std::endl;
    //continue;

    emorph::vQueue q; //Event queue
   	emorph::vQueue::iterator q_it;//Event queue iterator

    tempEvents->getAll(q); // 1 ms timeframe
    //std::cout << "Processing " << q.size()<< " events" <<std::endl;


 	for (q_it = q.begin(); q_it != q.end(); q_it++) //Processing each event
            {

                emorph::AddressEvent *aep = (*q_it)->getAs<emorph::AddressEvent>();
                if(!aep) continue; //If AER is not received continue
                int channel = aep->getChannel(); //Processing events of only one channel - LEFT EYE

                //Processing each event of channel 0
                if(channel==0){

                    //NOTE : Sensor X and Y are reversed
                    int pos_x = aep->getY();
                    int pos_y = aep->getX();
                    int current_event_polarity = aep->getPolarity();
                    if(current_event_polarity == 0){current_event_polarity = -1;} //Changing polarity to negative

                    //std::cout<< "Event at "<< "X : "<< pos_x <<" Y : "<< pos_y << " Polarity : "<<current_event_polarity<<std::endl; //Debug code

                    //Time stamp of the recent event
                    event_time = unwrap (aep->getStamp());

                    //converting to seconds
                    event_time = event_time / event_history.time_scale;

                    //Updating the event history buffer
                    event_history.updateList(*aep);

                    //Setting the filter spatial center values
                    st_filters.center_x = pos_x;
                    st_filters.center_y = pos_y;

                    //std::cout << "Number of Directions : " << theta_vector.size()<<std::endl;//Debug Code
                    theta_vector_it = theta_vector.begin();
                    theta_index = 0;

                    for(; theta_vector_it != theta_vector.end() ; ){ //Iterating for each direction

                        //std::cout<<" "<< pos_x <<" "<< pos_y <<" "<< event_time; //Debug code
                        double theta = *theta_vector_it;

                        //Resetting filter convolution value for every event processing
                        final_convolution = 0;
                        even_conv = 0;
                        odd_conv = 0;

                        //Spatial and Temporal neighborhood processing
                        for(int j=1 ; j<= kernel_size ; j++){
                            for(int i=1 ; i <= kernel_size ; i++){

                                //Pixels to be processed in the spatial neighbourhood
                                int pixel_x = pos_x + i-16;
                                int pixel_y = pos_y + j-16;
                                double temporal_difference;

                                if(pixel_x >= 0 && pixel_y>= 0 && pixel_x < MAX_RES && pixel_y < MAX_RES){  //Checking for borders


                                    event_history.timeStampsList_it = event_history.timeStampList[pixel_x][pixel_y].rbegin(); //Going from latest event pushed in the back

                                    //NOTE : List size is always limited to the buffer size, took care of this in the event buffer code
                                    if(!event_history.timeStampList[pixel_x][pixel_y].empty()){ //If the list is empty skip computing

                                        for( int list_length = 1 ; list_length <= event_history.timeStampList[pixel_x][pixel_y].size() ; ){

                                             //std::cout << "Event Time : "<< event_time << std::endl; //Debug Code
                                             temporal_difference = event_time - *event_history.timeStampsList_it; //The first value is always zero

                                             //Here the neighborhood processing goes over 0 - 1 time scale


					                                   //Call the spatio-temporal filtering function
                                             std::pair<double,double> conv_value = st_filters.filtering(pixel_x, pixel_y, theta, temporal_difference);

                                             even_conv = even_conv +  conv_value.first;
                                             odd_conv = odd_conv +   conv_value.second;

                                             ++event_history.timeStampsList_it; //Moving up the list
                                             ++list_length;



                                        }//End of temporal iteration loop

                                    }
                                    else {

                                        //std::cout << "Skipping empty list..." << std::endl; //Debug Code
                                        continue;
                                    }
                                }
                                else{

                                    //std::cout<< "Pixels Out of Bordes...."<<std::endl; //Debug Code
                                    continue;
                                }

                            }
                        }//End of spatial iteration loop


                  			final_convolution = (even_conv * even_conv) + (odd_conv * odd_conv);

                        //Pushing convolution/Energy response values into a vector to store values for all directions
	                      convolution.push_back(final_convolution);

                        ++theta_vector_it;
                        ++theta_index;

                    } //End of spatio-temporal filtering

                    //std::cout << "Event processing done..." << std::endl;//Debug Code
                    //std::cout << "Size of convolution vector : " << convolution.size()<<std::endl;//Debug code
                    		    for(convolution_it=convolution.begin(); convolution_it!=convolution.end();){

			                           std::cout << *convolution_it << " ";
			                          convolution_it++;

		                              }
                            std::cout<<std::endl;

                    //Finding the direction that has maximum value among all directions
                     auto max_value = std::max_element(convolution.begin(),convolution.end());
                     //std::cout << "Index of Maximum Value : "<< std::distance(std::begin(convolution), max_value) << std::endl;//Debug Code
                     //if(std::distance(std::begin(convolution), max_value)==0){

                         //std::cout << std::distance(std::begin(convolution), max_value) << " " << *max_value << std::endl;

                     //}

                    //motionHypothesis(pos_x,pos_y); // Calling motion hypothesis routine for computing optical flow

		               //Clearing the Convolution Vector
                    convolution.clear();

                } //End of channel if, one event processing done



        }//End of event queue loop



   } //end of while loop


   std::cout<<"End of while loop"<<std::endl; //Debug code

    return 0;
}


void motionHypothesis(int &x,int &y){


    //std::cout << "Motion Estimation..."<<std::endl;//Debug Code


            //Velocity components set to zero
            double Ux = 0;
            double Uy = 0;

            theta_vector_it = theta_vector.begin();
            convolution_it = convolution.begin();
            theta_index = 0;

            for(; theta_vector_it != theta_vector.end() - 1 ; ){//Summing up motion estimation along all directons

                //std::cout << "Theta value : " << *theta_vector_it<< std::endl; //Debug Code

                Ux = Ux + ( *convolution_it  *  cos( *theta_vector_it ) )  ;
                Uy = Uy - ( *convolution_it  *  sin( *theta_vector_it ) ) ;

                //std::cout << "Velocity Components Ux : " << Ux << " Uy : " << Uy << std::endl; //Debug Code

                ++convolution_it;
                ++theta_vector_it;
                ++theta_index;

            }//End of theta loop

            std::cout <<" "<<x<<" "<<y <<" " << Ux << " " << Uy << std::endl;//Debug
            //convolution.clear();


	   }

  
