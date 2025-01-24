#include <stdio.h>
#include "xparameters.h"
#include "platform.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xil_types.h"
#include "main.h"

#include "xparameters.h"
#include "Z7_HDMI/display_ctrl.h"
#include <math.h>
#include "xuartps_hw.h"
//#include <xscutimer.h>
//#include <limits.h>
#include "xtime_l.h"
//#include "xtoplevel.h"
//#include "xcollisions_main.h"

// Frame size (based on 1440x900 resolution, 32 bits per pixel)
#define MAX_FRAME (1440*900)
#define FRAME_STRIDE (1440*4)

#define PI 3.1416

#define MAX_WIDTH 1440
#define MAX_HEIGHT 900
#define MIN_WIDTH 1
#define MIN_HEIGHT 1
#define MAX_ANGLE 359
#define MIN_ANGLE 0

#define PRECISION 1
#define FRAME_LIMIT 60
#define SCENARIO 7
#define DIFFICULTY 3

DisplayCtrl dispCtrl;
u32 frameBuf[DISPLAY_NUM_FRAMES][MAX_FRAME] __attribute__((aligned(0x20)));
void *pFrames[DISPLAY_NUM_FRAMES];

static struct package_one msg_one;
static struct package_two msg_two;
static struct package_four partsRecv[6];
static int numParts, partTwo_recv, numParts_sent, numParts_recv;

u16 swap_u16(u16 value) {
	return (value << 8) | (value >> 8);
}

u32 swap_u32( u32 val ) {
	val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
	return (val << 16) | (val >> 16);
}

int print_out_bound(s16 x, s16 y, s8 part){
	int i = 0;
	if (x > 1440 || x < 0){
		printf("%d: x%d\n\r", part, x);
		i = 1;
	}
	if (y > 900 || y < 0){
		printf("%d: y%d\n\r", part, y);
		i = 1;
	}
	return i;
}

void udp_get_handler(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
	if (p){
		if (!(partTwo_recv)){
			memcpy(&msg_two, p->payload, p->len);
			pbuf_free(p);
			if (msg_two.header == 2 && msg_two.difficulty == msg_one.difficulty){
				partTwo_recv = 1;
				numParts = msg_two.number_Parts;
				msg_two.scenario = swap_u32(msg_two.scenario);
			}
			/*
			printf("msg2: ");
			printf("|Header %u", msg_two.header);
			printf("|Difficulty %u ", msg_two.difficulty);
			printf("|Scenario %lu ", msg_two.scenario);
			printf("|NumParts %u\n\r", msg_two.number_Parts);
			 */
		} else {
			struct package_four msg_four;
			memcpy(&msg_four, p->payload, p->len);
			msg_four.num_Agents = swap_u16(msg_four.num_Agents);
			pbuf_free(p);
			numParts_recv++;
			partsRecv[msg_four.requested_part] = msg_four;
			msg_four.scenario = swap_u32(msg_four.scenario);

			/*
			printf("\nmessge4\n\r");
			printf("header %u\n\r", msg_four.header);
			printf("difficulty %u\n\r", msg_four.difficulty);
			printf("scenario %lu\n\r", msg_four.scenario);
			printf("request_part %u\n\r", msg_four.requested_part);
			printf("numParts %u\n\n\r", msg_four.num_Agents);

			for (int i = 130; i <150; i++){
				printf("RBT: %u", i);
				printf("| Type %u", msg_four.robots[i].type);
				printf("| x_pos: %u", msg_four.robots[i].x_pos);
				printf("| y_pos %u", msg_four.robots[i].y_pos);
				printf("| heading %u\n\r", msg_four.robots[i].heading);
			}
			 */
		}
	}
}

int get_fps_limit(){
	int temp;
	printf("Please input your desired frame time:\n\r");
	printf("1 FPS, 10 FPS, 20 FPS, 60 FPS, Frame by Frame(0), No Limit (5)\n\r");
	printf("Input Here:");
	scanf("%d", &temp);
	printf("\n\r");
	return temp;
}

s16 check_x(s16 x){
	if (x < 0){
		return (x + MAX_WIDTH);
	} else if (x >= MAX_WIDTH){
		return (x - MAX_WIDTH);
	} else {
		return x;
	}
}

s16 check_y(s16 y){
	if (y < 0){
		return (y + MAX_HEIGHT);
	} else if (y >= MAX_HEIGHT){
		return (y - MAX_HEIGHT);
	} else {
		return y;
	}
}



s16 check_theta(s16 theta){
	while (theta >= 360 * PRECISION || theta < 0){
		if (theta < MIN_ANGLE * PRECISION){
			theta += 360 * PRECISION;
		} else if (theta > MAX_ANGLE * PRECISION){
			theta -= 360 * PRECISION;
		}
	}
	return theta;
}


int get_bearing(s16 x_diff, s16 y_diff){
	float theta = (atan2(y_diff, x_diff) * (180/PI) * PRECISION) + (90 * PRECISION);
	if (theta < MIN_ANGLE){
		theta += 360 + theta;
	}
	return theta;
}


int main(){
	static robot rbt_Ar[900];
	static rbt_closest closest[900];
	//hls_col_data col_ram[450];
	//hls_dist_data dist_ram[450];
	//s32 dist_ram[3150];

	static int numRobots = 0;
	static float fast_sin[360 * PRECISION], fast_cos[360 * PRECISION];
	XTime stage_start, stage_end;
	float ether_time, hdmi_time;
	int frame_limit = FRAME_LIMIT;
	int loop_exit = 0;

	//XToplevel hls_dist;
	//XCollisions_main hls_col;

	//XToplevel_Initialize(&hls_dist, XPAR_TOPLEVEL_0_DEVICE_ID);
	//XCollisions_main_Initialize(&hls_col, XPAR_COLLISIONS_MAIN_0_DEVICE_ID);


	XTime_GetTime(&stage_start);
	printf("\n\r");

	unsigned char mac_ethernet_address[] = {0x00, 0x11, 0x22, 0x33, 0x00, 0x23};
	init_platform(mac_ethernet_address, NULL, NULL);
	//numRobots = ethernet_main(&rbt_Ar, numRobots);

	struct udp_pcb *recv_pcb = udp_new();
	if (!recv_pcb) {
		printf("Error creating PCB\n");
	}

	udp_bind(recv_pcb, IP_ADDR_ANY, 51050);
	udp_recv(recv_pcb, udp_get_handler, NULL);



	msg_one.header = 1;
	u8 difficulty;
	u32 scenario;
	printf("1 = easy, 2 = medium, 3 = hard\n\rInput Requested Scenario:");
	scanf("%u", &difficulty);
	printf("\n\rInput seed value:");
	scanf("%u", &scenario);
	msg_one.difficulty = difficulty;
	msg_one.scenario = scenario;

	//msg_one.difficulty = DIFFICULTY;
	//msg_one.scenario = SCENARIO;
	msg_one.scenario = swap_u32(msg_one.scenario);
	printf("\n\r");

	printf("Header %u | ", msg_one.header);
	printf("Difficulty %u | ", msg_one.difficulty);
	printf("Scenario %lu\n\r", swap_u32(msg_one.scenario));

	struct udp_pcb *send_pcb = udp_new();

	struct pbuf * reply = pbuf_alloc(PBUF_TRANSPORT, sizeof(msg_one), PBUF_REF);
	reply->payload = &msg_one;
	reply->len = sizeof(msg_one);

	ip_addr_t ip;
	IP4_ADDR(&ip, 192, 168, 10, 1);
	udp_sendto(send_pcb, reply, &ip, 51050);
	pbuf_free(reply);
	partTwo_recv = 0;
	numParts_sent = 0;
	numParts_recv = 0;
	numParts = 6;

	//printf("here");
	while (1){
		handle_ethernet();
		if (partTwo_recv && (numParts_sent == numParts_recv) && !(numParts_sent == numParts)){
			//printf("numParts = %d\n\r", numParts);
			//printf("numParts_recv = %d\n\r", numParts_recv);

			struct package_three msg_three;
			msg_three.header = 3;
			msg_three.difficulty = msg_one.difficulty;
			msg_three.scenario = msg_one.scenario;
			msg_three.requested_Part = numParts_sent;

			struct pbuf * reply = pbuf_alloc(PBUF_TRANSPORT, sizeof(msg_three), PBUF_REF);
			reply->payload = &msg_three;
			reply->len = sizeof(msg_three);
			udp_sendto(send_pcb, reply, &ip, 51050);
			pbuf_free(reply);

			numParts_sent++;
			printf("message %d sent\n\r", numParts_sent);
		}
		if (numParts_recv == numParts){
			//printf("escape\n\r");
			break;
		}
	}
	udp_remove(send_pcb);
	//sleep(5);
	for (uint i = 0; i < numParts; i++){
		printf("I'm inner looping\n\r");
		for(uint j = 0; j < partsRecv[i].num_Agents; j++){

			rbt_Ar[j + numRobots].type = partsRecv[i].robots[j].type;
			rbt_Ar[j + numRobots].cur_x = (swap_u16(partsRecv[i].robots[j].x_pos)) * PRECISION;
			if (rbt_Ar[j + numRobots].cur_x > MAX_WIDTH * PRECISION){
				rbt_Ar[j + numRobots].cur_x = MAX_WIDTH * PRECISION;
			} else if (rbt_Ar[j+numRobots].cur_x < MIN_WIDTH * PRECISION){
				rbt_Ar[j + numRobots].cur_x = MIN_WIDTH * PRECISION;
			}


			rbt_Ar[j + numRobots].cur_y = (swap_u16(partsRecv[i].robots[j].y_pos)) * PRECISION;
			if (rbt_Ar[j + numRobots].cur_y > MAX_HEIGHT * PRECISION){
				rbt_Ar[j + numRobots].cur_y = MAX_HEIGHT * PRECISION;
			} else if (rbt_Ar[j+numRobots].cur_y < MIN_HEIGHT * PRECISION){
				rbt_Ar[j + numRobots].cur_y = MIN_HEIGHT * PRECISION;
			}



			rbt_Ar[j + numRobots].heading = (swap_u16(partsRecv[i].robots[j].heading)) * PRECISION;
			if (rbt_Ar[j + numRobots].heading > MAX_ANGLE * PRECISION){
				rbt_Ar[j + numRobots].heading = MAX_ANGLE * PRECISION;
			} else if (rbt_Ar[j+numRobots].heading < MIN_ANGLE * PRECISION){
				rbt_Ar[j + numRobots].heading = MIN_ANGLE * PRECISION;
			}


			switch (rbt_Ar[j+numRobots].type){
			case 0:
				rbt_Ar[j+numRobots].movement = 0 * PRECISION;
				break;
			case 1:
				rbt_Ar[j+numRobots].movement = 5 * PRECISION;
				break;
			case 2:
				rbt_Ar[j+numRobots].movement = 5 * PRECISION;
				break;
			case 3:
				rbt_Ar[j+numRobots].movement = 5 * PRECISION;
				break;
			case 4:
				rbt_Ar[j+numRobots].movement = 3 * PRECISION;
				break;
			case 5:
				rbt_Ar[j+numRobots].movement = 2 * PRECISION;
				break;
			case 6:
				rbt_Ar[j+numRobots].movement = 4 * PRECISION;
				break;
			}
			//printf("%d ", j);
		}
		numRobots += partsRecv[i].num_Agents;
		//sleep(10);
	}
	loop_exit = 1;
	/*
	for (int i = 0; i <150; i++){
		printf("RBT: %u", i);
		printf("| Type %u", rbt_Ar[i].type);
		printf("| x_pos: %u", rbt_Ar[i].cur_x);
		printf("| y_pos %u", rbt_Ar[i].cur_y);
		printf("| heading %u\n\r", rbt_Ar[i].heading);
		//sleep(1);

	}*/
	//sleep(3);
	while (loop_exit < 1){
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	XTime_GetTime(&stage_end);
	ether_time = 1.0 * (stage_end - stage_start) / COUNTS_PER_SECOND;
	printf("Ethernet: %f\n\r", ether_time);
	XTime_GetTime(&stage_start);


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	//Pre calculate cos and sin
	for (int i = 0; i < (MAX_ANGLE * PRECISION) + 1; i ++){
		float j = i / PRECISION;
		fast_sin[i] = sin(j * (PI / 180));
		fast_cos[i] = cos(j * (PI / 180));
	}


	//HDMI Setup
	int i;
	for (i = 0; i < DISPLAY_NUM_FRAMES; i++)
		pFrames[i] = frameBuf[i];
	DisplayInitialize(&dispCtrl, XPAR_AXIVDMA_0_DEVICE_ID, XPAR_VTC_0_DEVICE_ID, XPAR_HDMI_AXI_DYNCLK_0_BASEADDR, pFrames, FRAME_STRIDE);
	DisplayChangeFrame(&dispCtrl, 0);
	DisplaySetMode(&dispCtrl, &VMODE_1440x900);
	DisplayStart(&dispCtrl);

	u32 stride = dispCtrl.stride / 4;
	//u32 width = dispCtrl.vMode.width;
	//u32 height = dispCtrl.vMode.height;

	u32 *frame;
	u32 buff = dispCtrl.curFrame;
	//end of HDMI setuo


	XTime_GetTime(&stage_end);
	hdmi_time = 1.0 * (stage_end - stage_start) / COUNTS_PER_SECOND;
	printf("HDMI Time: %f\n\r", hdmi_time);
	printf("Total Setup: %f\n\r", (float) (ether_time + hdmi_time));
	printf("NumRobots: %d\n\r", numRobots);



















	for (int i = numRobots; i < 900; i++){
		rbt_Ar[i].type = 6;
		rbt_Ar[i].cur_x = 2881;
		rbt_Ar[i].cur_y = 1801;
		rbt_Ar[i].heading = 0;
		rbt_Ar[i].movement = 0;
		rbt_Ar[i].new_x = 2801;
		rbt_Ar[i].new_y = 1801;
		rbt_Ar[i].collided = 903;
	}




	/*
	int hls_runs = 1;
	if (numRobots > 450){
		hls_runs = 2;
	}*/





	//sleep(5);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	s32 max_dist = 2886500; //(pow((MAX_WIDTH * PRECISION) + 1, 2) + pow((MAX_HEIGHT * PRECISION) + 1, 2));
	s32 r2_dist = 10000; //pow(100 * PRECISION, 2);
	s32 r1_dist = 2500; //pow(50 * PRECISION, 2);
	s16 col_dist = 256; //pow(16 * PRECISION, 2);
	//printf("Max: %d, %d, %ld, %d\n\r", max_dist, r2_dist, r1_dist, col_dist);
	//sleep(10);

	float distance_calc, heading_calc, collision_calc, draw_calc, fps;
	while(1){ //Main Loop

		buff = !buff;
		frame = (u32 *)dispCtrl.framePtr[buff];
		memset(frame, 0xFF, MAX_FRAME*4);


		//calc robot moves;
		//printf("running main loop\n\r");
		XTime_GetTime(&stage_start);




		/*printf("dist_hls start\n\r");
		for (int j = 0; j < hls_runs; j++){

			for (int i = 0; i < 450; i++){
				dist_ram[i].type = rbt_Ar[i].type;
				dist_ram[i].cur_x = rbt_Ar[i].cur_x;
				dist_ram[i].cur_y = rbt_Ar[i].cur_y;
				dist_ram[i].heading = rbt_Ar[i].heading;
			}
			printf("here sir ");
			XToplevel_Set_ram(&hls_dist, dist_ram);
			printf("here sir ");

			if (numRobots < 450 && j == 0){
				XToplevel_Set_arg1(&hls_dist, numRobots);
			} else if (j == 0){
				XToplevel_Set_arg1(&hls_dist, (450));
			} else {
				XToplevel_Set_arg1(&hls_dist, (numRobots - 450));
			}

			Xil_DCacheFlush();
			printf("here sir");

			XToplevel_Start(&hls_dist);
			while (!XToplevel_IsDone(&hls_dist));

			printf(" and here sir");

			Xil_DCacheInvalidate();
			for (int i = 0; i < 450; i++){
				closest[i].dist_2 = dist_ram[i].dist_2;
				closest[i].dist_x = dist_ram[i].dist_x;
				closest[i].dist_y = dist_ram[i].dist_y;
			}
			printf("hi");
		}
		printf("dist_hls end\n\r");

		int wait = 1;
		for (int j = 0; j < hls_runs; j++){

			for (int i = 0; i < 450; i++){
				dist_ram[i*7] = rbt_Ar[i].type;
				dist_ram[i*7+1] = rbt_Ar[i].cur_x;
				dist_ram[i*7+2] = rbt_Ar[i].cur_y;
				dist_ram[i*7+3] = rbt_Ar[i].heading;
			}
			XToplevel_Set_ram(&hls_dist, (s32) dist_ram);

			s32 temp = numRobots;

			if (numRobots < 450 && j == 0){
				XToplevel_Set_arg1(&hls_dist, (s32) temp);
			} else if (j == 0){
				temp = 450;
				XToplevel_Set_arg1(&hls_dist, (s32) temp);
			} else {
				temp = numRobots - temp;
				XToplevel_Set_arg1(&hls_dist, (s32) temp);
			}

			Xil_DCacheFlush();

			XToplevel_Start(&hls_dist);
			while (!XToplevel_IsDone(&hls_dist)){
				printf("%lu\n\r", XToplevel_IsDone(&hls_dist));

			}

			printf(" and here sir");

			Xil_DCacheInvalidate();
			for (int i = 0; i < 450; i++){
				closest[i].dist_2 = dist_ram[i*7+4];
				closest[i].dist_x = dist_ram[i*7+5];
				closest[i].dist_y = dist_ram[i*7+6];
			}
			printf("hi");
		}
		printf("dist_hls end\n\r");
		 */







		//printf("Type: \n\r");
		for (int i = 0; i < numRobots; i++){
			for (int j = i+1; j < numRobots; j++){
				s16 x_diff = rbt_Ar[j].cur_x - rbt_Ar[i].cur_x;
				s16 y_diff = rbt_Ar[j].cur_y - rbt_Ar[i].cur_y;
				s32 x2_y2 = (x_diff * x_diff) + (y_diff * y_diff);

				if (x2_y2 < closest[i].dist_2 && (i !=j)){
					switch (rbt_Ar[i].type){
					case 4:
						if (abs(get_bearing(x_diff, y_diff) - rbt_Ar[i].heading) < 45){
							closest[i].dist_2 = x2_y2;
							closest[i].dist_x = x_diff;
							closest[i].dist_y = y_diff;
						}
						break;
					case 6:
						if ((abs(get_bearing(x_diff, y_diff) - rbt_Ar[i].heading) < 45) && (x2_y2 > r1_dist)){
							closest[i].dist_2 = x2_y2;
							closest[i].dist_x = x_diff;
							closest[i].dist_y = y_diff;
						} else if (x2_y2 < r1_dist){
							closest[i].dist_2 = x2_y2;
							closest[i].dist_x = x_diff;
							closest[i].dist_y = y_diff;
						}
						break;
					default:
						//closest[i].ind_2 = j;
						closest[i].dist_2 = x2_y2;
						closest[i].dist_x = x_diff;
						closest[i].dist_y = y_diff;
						//printf("I'm here");
					}
				}
				if (x2_y2 < closest[j].dist_2 && (i !=j)){
					switch (rbt_Ar[j].type){
					case 4:
						if (abs(get_bearing(x_diff, y_diff) - rbt_Ar[j].heading) < 45){
							closest[j].dist_2 = x2_y2;
							closest[j].dist_x = x_diff * (-1);
							closest[j].dist_y = y_diff * (-1);
						}
						break;
					case 6:
						if ((abs(get_bearing(x_diff, y_diff) - rbt_Ar[j].heading) < 45) && (x2_y2 > r1_dist)){
							closest[j].dist_2 = x2_y2;
							closest[j].dist_x = x_diff * (-1);
							closest[j].dist_y = y_diff * (-1);
						} else if (x2_y2 < r1_dist){
							closest[j].dist_2 = x2_y2;
							closest[j].dist_x = x_diff * (-1);
							closest[j].dist_y = y_diff * (-1);
						}
						break;
					default:
						closest[j].dist_2 = x2_y2;
						closest[j].dist_x = x_diff * (-1);
						closest[j].dist_y = y_diff * (-1);
						//printf("I'm here");
					}
				}

			}
		}
		//printf("\n\r");
		//sleep(5);

		XTime_GetTime(&stage_end);
		distance_calc = 1.0 * (stage_end - stage_start) / COUNTS_PER_SECOND;
		XTime_GetTime(&stage_start);






		for (int i = 0; i < numRobots; i++){ //pick direction then calculate move

			if (rbt_Ar[i].type == 5){
				rbt_Ar[i].movement = 2 * PRECISION;
			}
			rbt_Ar[i].collided = 902;

			if (rbt_Ar[i].type > 3){

				//change headings if needed
				if ((rbt_Ar[i].type == 4) && (closest[i].dist_2 < r2_dist)){
					int bearing = get_bearing(closest[i].dist_x, closest[i].dist_y);
					//printf("4: %d, %d | \n\r", bearing, rbt_Ar[i].heading);
					if (abs(bearing - rbt_Ar[i].heading) < (45 * PRECISION)){
						rbt_Ar[i].heading = check_theta(bearing);
					}

				} else if ((rbt_Ar[i].type == 5) && (closest[i].dist_2 < r1_dist)){
					int bearing = get_bearing(closest[i].dist_x, closest[i].dist_y);

					rbt_Ar[i].heading = check_theta(bearing + (180 * PRECISION));
					rbt_Ar[i].movement = 8 * PRECISION;

				} else if ((rbt_Ar[i].type == 6) && (closest[i].dist_2 < r2_dist)){
					int bearing = get_bearing(closest[i].dist_x, closest[i].dist_y);
					if (closest[i].dist_2 < r1_dist){
						rbt_Ar[i].heading = check_theta(bearing + (180 * PRECISION));

					} else if (abs(rbt_Ar[i].heading - bearing) < 45 * PRECISION){
						//rbt_Ar[i].heading = bearing;
						rbt_Ar[i].heading = check_theta(bearing);
					}
				}
			}

			// calc new positions
			rbt_Ar[i].new_x = rbt_Ar[i].cur_x + (fast_sin[rbt_Ar[i].heading] * rbt_Ar[i].movement);
			if (rbt_Ar[i].new_x > (MAX_WIDTH * PRECISION)){
				rbt_Ar[i].new_x -= (MAX_WIDTH * PRECISION);
				//printf("%d ", i);
			} else if (rbt_Ar[i].new_x < (MIN_WIDTH * PRECISION)){
				rbt_Ar[i].new_x += (MAX_WIDTH * PRECISION);
			}

			rbt_Ar[i].new_y = rbt_Ar[i].cur_y + (-fast_cos[rbt_Ar[i].heading] * rbt_Ar[i].movement);
			if (rbt_Ar[i].new_y > (MAX_HEIGHT * PRECISION)){
				rbt_Ar[i].new_y -= (MAX_HEIGHT * PRECISION);
				//printf("%d ", i);
			} else if (rbt_Ar[i].new_y < (MIN_HEIGHT * PRECISION)){
				rbt_Ar[i].new_y += (MAX_HEIGHT * PRECISION);
				//printf("%d ", i);
			}

			//reset distance
			closest[i].dist_2 = max_dist;
		}

		XTime_GetTime(&stage_end);
		heading_calc = 1.0 * (stage_end - stage_start) / COUNTS_PER_SECOND;
		XTime_GetTime(&stage_start);











		/*

		printf("col_hls start1\n\r");
		for (int j = 0; j < hls_runs; j++){



			printf("col_hls start2\n\r");
			for (int i = 0; i < 450; i++){
				col_ram[i].new_x = rbt_Ar[i].new_x;
				col_ram[i].new_y = rbt_Ar[i].new_y;
			}

			printf("col_hls start3\n\r");
			XCollisions_main_Set_ram(&hls_col, col_ram);

			printf("col_hls start4\n\r");
			if (numRobots < 450){
				XCollisions_main_Set_arg1(&hls_col, numRobots);
			} else if (j == 0){
				XCollisions_main_Set_arg1(&hls_col, 450);
			} else {
				XCollisions_main_Set_arg1(&hls_col, (numRobots - 450));
			}
			XCollisions_main_Set_arg1(&hls_col, numRobots);
			Xil_DCacheFlush();
			printf("col_hls start5\n\r");

			XCollisions_main_Start(&hls_col);
			while (!XCollisions_main_IsDone(&hls_col)){
				//printf("ahha");
			}

			Xil_DCacheInvalidate();
			for (int i = 0; i < 450; i++){
				rbt_Ar[i].collided = col_ram[i].collided;
			}
		}
		printf("col_hls end\n\r");*/






		for (int i = 0; i < numRobots; i++){ //check for collisions
			for (int j = i+1; j < numRobots; j++){
				s16 x_diff = rbt_Ar[j].new_x - rbt_Ar[i].new_x;
				s16 y_diff = rbt_Ar[j].new_y - rbt_Ar[i].new_y;
				s32 x2_y2 = (x_diff * x_diff) + (y_diff * y_diff);

				if (x2_y2 <= col_dist && (i != j)){
					rbt_Ar[i].collided = j;
					rbt_Ar[j].collided = i;
				}
			}
		}


		for (int i = 0; i < numRobots; i++){
			if ((rbt_Ar[i].collided < 902) && (rbt_Ar[i].type > 1)){
				//printf("col: %d, type %d |\n\r",rbt_Ar[i].collided, rbt_Ar[i].type);
				if (rbt_Ar[i].type == 2){
					rbt_Ar[i].heading = (rand() % (360 * PRECISION));

				} else if (rbt_Ar[i].type == 3){
					int j = rbt_Ar[i].collided;
					int bearing = get_bearing((rbt_Ar[j].cur_x - rbt_Ar[i].cur_x), (rbt_Ar[j].cur_y - rbt_Ar[i].cur_y));
					rbt_Ar[i].heading = check_theta(bearing + (180 * PRECISION));
				}
			} else {
				//printf("no col: %d, type %d |\n\r",rbt_Ar[i].collided, rbt_Ar[i].type);
				rbt_Ar[i].cur_x = rbt_Ar[i].new_x;
				rbt_Ar[i].cur_y = rbt_Ar[i].new_y;
			}
		}

		//printf("starting to draw \n\r");
		XTime_GetTime(&stage_end);
		collision_calc = 1.0 * (stage_end - stage_start) / COUNTS_PER_SECOND;
		XTime_GetTime(&stage_start);








		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



		//printf("drawing \n\r");
		//draw
		for(int i = 0; i < numRobots; i++){
			s16 cur_x = (rbt_Ar[i].cur_x / PRECISION) - 1;
			s16 cur_y = (rbt_Ar[i].cur_y / PRECISION) - 1;
			s16 heading = rbt_Ar[i]. heading;

			if (!(rbt_Ar[i].type == 0)){
				for (int j = 0; j < 10; j++){
					s16 x = check_x(cur_x + (j * fast_sin[heading]));
					s16 y = check_y(cur_y - (j * fast_cos[heading]));
					print_out_bound(x,y,1);
					frame[y*stride + x] = 0;
				}
			}

			switch (rbt_Ar[i].type){
			case 4:
				for (int j = 0; j < 100; j += 4){
					s16 x = check_x(cur_x + (j * fast_sin[check_theta(heading - (45 * PRECISION))]));
					s16 y = check_y(cur_y - (j * fast_cos[check_theta(heading - (45 * PRECISION))]));
					print_out_bound(x,y,2);
					frame[y*stride + x] = ( 0xFF << BIT_DISPLAY_RED) | (0x00 << BIT_DISPLAY_GREEN) | (0x00 << BIT_DISPLAY_BLUE); //blue r2

					x = check_x(cur_x + (j * fast_sin[check_theta(heading + (45 * PRECISION))]));
					y = check_y(cur_y - (j * fast_cos[check_theta(heading + (45 * PRECISION))]));
					print_out_bound(x,y,3);
					frame[y*stride + x] = ( 0xFF << BIT_DISPLAY_RED) | (0x00 << BIT_DISPLAY_GREEN) | (0x00 << BIT_DISPLAY_BLUE); //blue r2
				}
				break;
			case 6:
				for (int j = 0; j < 100; j += 4){
					s16 x = check_x(cur_x + (j * fast_sin[check_theta(heading - (45 * PRECISION))]));
					s16 y = check_y(cur_y - (j * fast_cos[check_theta(heading - (45 * PRECISION))]));
					print_out_bound(x,y,4);
					frame[y*stride + x] = ( 0xFF << BIT_DISPLAY_RED) | (0x00 << BIT_DISPLAY_GREEN) | (0x00 << BIT_DISPLAY_BLUE); //blue r2

					x = check_x(cur_x + (j * fast_sin[check_theta(heading + (45 * PRECISION))]));
					y = check_y(cur_y - (j * fast_cos[check_theta(heading + (45 * PRECISION))]));
					print_out_bound(x,y,5);
					frame[y*stride + x] = ( 0xFF << BIT_DISPLAY_RED) | (0x00 << BIT_DISPLAY_GREEN) | (0x00 << BIT_DISPLAY_BLUE); //blue r2
				}
				break;
			}

			for (int j = 0; j < 360 * PRECISION; j += 6 * PRECISION){
				s16 x = check_x(cur_x + (8 * fast_sin[j]));
				s16 y = check_y(cur_y - (8 * fast_cos[j]));
				/*
				if (print_out_bound(x,y,6)){
					printf("%d, ID:%d, x%d, y%d | ", j, i, cur_x, cur_y);
				}*/
				switch (rbt_Ar[i].type){
				case 0:
					frame[y*stride + x] = 0;
					//printf("me");
					break;

				case 1:
					frame[y*stride + x] = ( 0xFF << BIT_DISPLAY_RED) | (0xD7 << BIT_DISPLAY_GREEN) | (0x00 << BIT_DISPLAY_BLUE);
					//printf("me 2");
					break;

				case 2:
					frame[y*stride + x] = ( 0xFF << BIT_DISPLAY_RED) | (0x8C << BIT_DISPLAY_GREEN) | (0x00 << BIT_DISPLAY_BLUE);
					//printf("case 2");
					break;

				case 3:
					frame[y*stride + x] = ( 0xFF << BIT_DISPLAY_RED) | (0x00 << BIT_DISPLAY_GREEN) | (0x80 << BIT_DISPLAY_BLUE);
					//printf("case 3");
					break;

				case 4:
					frame[y*stride + x] = ( 0x00 << BIT_DISPLAY_RED) | (0xFF << BIT_DISPLAY_GREEN) | (0x0F << BIT_DISPLAY_BLUE);
					if (abs(j - heading) < 45 * PRECISION || abs(j - heading > 315 * PRECISION)) {
						for (int z = -2 * PRECISION; z < 3 * PRECISION; z += 2 * PRECISION){
							x = check_x(cur_x + (100 * fast_sin[check_theta(j+z)]));
							y = check_y(cur_y - (100 * fast_cos[check_theta(j+z)]));
							print_out_bound(x,y,7);
						}
						frame[y*stride + x] = ( 0xFF << BIT_DISPLAY_RED) | (0x00 << BIT_DISPLAY_GREEN) | (0x00 << BIT_DISPLAY_BLUE); //blue r2
					}
					//printf("case 4");
					break;

				case 5:
					frame[y*stride + x] = ( 0xFF << BIT_DISPLAY_RED) | (0x00 << BIT_DISPLAY_GREEN) | (0x00 << BIT_DISPLAY_BLUE);
					x = check_x(cur_x + (50 * fast_sin[j]));
					y = check_y(cur_y - (50 * fast_cos[j]));
					print_out_bound(x,y,8);
					frame[y*stride + x] = ( 0x00 << BIT_DISPLAY_RED) | (0x00 << BIT_DISPLAY_GREEN) | (0xFF << BIT_DISPLAY_BLUE);//blue r1
					//printf("case 5");
					break;

				case 6:
					frame[y*stride + x] = ( 0x0F << BIT_DISPLAY_RED) | (0x8F << BIT_DISPLAY_GREEN) | (0x8F << BIT_DISPLAY_BLUE);
					if (abs(j - heading) < 45 * PRECISION || abs(j - heading > 315 * PRECISION)){
						for (int z = -2 * PRECISION; z < 3 * PRECISION; z += 2 * PRECISION){
							x = check_x(cur_x + (100 * fast_sin[check_theta(j+z)]));
							y = check_y(cur_y - (100 * fast_cos[check_theta(j+z)]));
							print_out_bound(x,y,9);
						}
						frame[y*stride + x] = ( 0xFF << BIT_DISPLAY_RED) | (0x00 << BIT_DISPLAY_GREEN) | (0x00 << BIT_DISPLAY_BLUE);//red r2
					}
					x = check_x(cur_x + (50 * fast_sin[j]));
					y = check_y(cur_y - (50 * fast_cos[j]));
					print_out_bound(x,y,10);
					frame[y*stride + x] = ( 0x00 << BIT_DISPLAY_RED) | (0x00 << BIT_DISPLAY_GREEN) | (0xFF << BIT_DISPLAY_BLUE);//blue r1
					//printf("case 6");
					break;
				}
			}
		}
		//printf("printing fps\n\r");

		XTime_GetTime(&stage_end);
		draw_calc = 1.0 * (stage_end - stage_start) / COUNTS_PER_SECOND;


		Xil_DCacheFlush();
		DisplayChangeFrame(&dispCtrl, buff);
		//DisplayWaitForSync(&dispCtrl);
		frame_timer();
		fps = get_fps();

		//printf("FPS: %f\r", get_fps());
		printf("D:%f|H:%f|C:%f|D:%f|F:%f\r", distance_calc, heading_calc, collision_calc, draw_calc, fps);

		if (XUartPs_IsReceiveData(STDIN_BASEADDRESS)){
			frame_limit = get_fps_limit();
			reset_fps();
		}
		if (!(frame_limit == 5)){
			DisplayWaitForSync(&dispCtrl);
		}

		int waiting = 1;
		switch (frame_limit){
		case 0:
			while (waiting){
				if(XUartPs_IsReceiveData(STDIN_BASEADDRESS)) { //If the user has pressed a key
					char byte = XUartPs_RecvByte(STDIN_BASEADDRESS); //Read it in
					if (byte == 'e'){
						frame_limit = get_fps_limit();
						reset_fps();
						waiting = 0;
					} else {
						waiting = 0;
					}
				}
			}
			break;
		case 1:
			frame_limiter(frame_limit);
			break;
		case 10:
			frame_limiter(frame_limit);
			break;
		case 20:
			frame_limiter(frame_limit);
			break;
		case 40:
			frame_limiter(frame_limit);
			break;
		}
	}

	return 0;
}

/*
if (x2_y2 < closest[j].dist_2){
					switch (rbt_Ar[j].type){
					case 4:
						if (abs(get_bearing(x_diff, y_diff) - rbt_Ar[j].heading) < 45){
							closest[j].dist_2 = x2_y2;
							closest[j].dist_x = x_diff * (-1);
							closest[j].dist_y = y_diff * (-1);
						}
						break;
					case 6:
						if ((abs(get_bearing(x_diff, y_diff) - rbt_Ar[j].heading) < 45) && (x2_y2 > r1_dist)){
							closest[j].dist_2 = x2_y2;
							closest[j].dist_x = x_diff * (-1);
							closest[j].dist_y = y_diff * (-1);
						} else if (x2_y2 < r1_dist){
							closest[j].dist_2 = x2_y2;
							closest[j].dist_x = x_diff * (-1);
							closest[j].dist_y = y_diff * (-1);
						}
						break;
					default:
						//closest[i].ind_2 = j;
						closest[j].dist_2 = x2_y2;
						closest[j].dist_x = x_diff * (-1);
						closest[j].dist_y = y_diff * (-1);
						//printf("I'm here");
					}
				}
				if (closest[j].dist_2 == max_dist){
					closest[j].dist_2 = x2_y2;
					closest[j].dist_x = x_diff * (-1);
					closest[j].dist_y = y_diff * (-1);
					//printf("I'm here too");
				}*/

