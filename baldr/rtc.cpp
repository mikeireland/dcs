#include "baldr.h"

long unsigned int rtc_cnt=0;
unsigned int nerrors=0;
// Add local (to the RTC) variables here

// Initialise variables assocated with the RTC.
void initialise_rtc(){
    // Add initialisation code here
}

// The main RTC function
void rtc(){
    // Temporary variabiles that don't need initialisation
    // can go here.
    initialise_rtc();
    rtc_cnt = subarray.md->cnt0;
    while(servo_mode != SERVO_STOP){
        if (subarray.md->cnt0 != rtc_cnt) {
            if (subarray.md->cnt0 > rtc_cnt+1) {
                std::cout << "Missed frame" << subarray.md->cnt0 << rtc_cnt << std::endl;
                nerrors++;
            }
        // Add RTC code here
        rtc_cnt++;
        }
        // !!! Something better than sleep would be good
        usleep(10);
    }
}
