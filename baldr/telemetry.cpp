#include "baldr.h"

long unsigned int telemetry_cnt=0;
// Add local (to telemetry) variables here

// Initialise variables assocated with the RTC.
void initialise_telemetry(){
    // Add initialisation code here
}

// The main RTC function
void telemetry(){
    // Temporary variabiles that don't need initialisation
    // can go here.
    initialise_telemetry();
    while(servo_mode != SERVO_STOP){
        // Add RTC code here
        telemetry_cnt++;
    }
}
