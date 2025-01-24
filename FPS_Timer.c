#include <xscutimer.h>
#include <limits.h>
#include "xtime_l.h"
#include <stdio.h>

float averagefps = 0.0;
XTime lasttime = 0;
XTime lasttime_lim = 0;
float frameTime_10ms = 0.0001;
float frameTime_1s = 1;

//Call frame_timer after the call to render_frame().
//After at least 3 frames, get_fps() will contain a valid FPS value.
void frame_timer() {
	if(lasttime == 0) {
		XTime_GetTime(&lasttime);
	} else {
		XTime thistime;

		XTime_GetTime(&thistime);

		XTime frametime = thistime - lasttime;
		lasttime = thistime;
		float timeinsecs = 1.0 * frametime / COUNTS_PER_SECOND;
		float fps = 1.0 / timeinsecs;

		if(averagefps == 0.0) {
			averagefps = fps;
		} else {
			averagefps = (averagefps + fps) / 2.0;
		}
	}
}

void reset_fps(){
	averagefps = 0;
}

float get_fps() {
	return averagefps;
}

void frame_limiter(int limit){
	//printf("boo   ");
	if(lasttime_lim == 0) {
		XTime_GetTime(&lasttime_lim);
	} else {
		XTime thistime;
		float frameTime_lim = 10;

		XTime_GetTime(&thistime);

		XTime frametime = thistime - lasttime_lim;
		//lasttime_lim = thistime;
		float timeinsecs = 1.0 * frametime / COUNTS_PER_SECOND;
		//printf("I got here ");
		switch (limit){
		case 1:
			frameTime_lim = 1;
			break;
		case 10:
			frameTime_lim = 0.1;
			break;
		case 20:
			frameTime_lim = 0.05;
			break;
		case 40:
			frameTime_lim = 0.025;
			break;
		default:
			frameTime_lim = 0.0166;
		}
		//printf("And here ");
		while (timeinsecs < frameTime_lim){
			XTime_GetTime(&thistime);
			frametime = thistime - lasttime_lim;
			timeinsecs = 1.0 * frametime / COUNTS_PER_SECOND;
		}
		lasttime_lim = thistime;
		//printf("But not here\n\r");
	}
}
































