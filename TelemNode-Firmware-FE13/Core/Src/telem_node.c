#include "telem_node.h"
#include "adc.h"
#include "pwm.h"
#include "can_manager.h"
#include "main.h"
#include "wheel_speed.h"
#include "math.h"

#define VOLTAGE_DIVIDER_RATIO (12.0 / (12.0 + 6.04))
#define PSI_PER_KPA 0.145038
#define HI8(x) ((x>>8)&0xFF)
#define LO8(x) (x&0xFF);
#define BUZZ_TIME_MS 1500

#define PUMP_THRESH 400
#define FAN_THRESH_1 400
#define FAN_THRESH_2 500
#define FAN_THRESH_3 600
#define HYSTERESIS 30
//might change this number later
#define NUM_SAMPLES_IN_AVGERAGE 5

TelemNodeLocation_t Location = FL;

// HANDLE TYPE DEFS from main
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

extern CAN_DATA_t can_data;
//extern allows this file to use this var
extern uint32_t ADC_RES_BUFFER[9];

// PRIVATE GLOBALS
ADC_Input_t adc_inlet_temp;
ADC_Input_t adc_outlet_temp;
ADC_Input_t adc_inlet_pres;
ADC_Input_t adc_outlet_pres;
ADC_Input_t adc_air_in_temp;
ADC_Input_t adc_air_out_temp;
ADC_Input_t adc_toe_strain_gauge;
ADC_Input_t adc_a_arm_strain_gauge;
ADC_Input_t adc_extra_3;
ADC_Input_t adc_extra_4;

PWM_Output_t pwm_fan;
PWM_Output_t pwm_pump;
WheelSpeed_t wheel_rr;
WheelSpeed_t wheel_rl;


//keep track of number of samples taken so far
uint8_t num_samples;


//vars used to calculate avg
uint64_t strain_gauge_control_avg =0;
uint64_t shock_angle_avg =0;
uint64_t strain_gauge_uf_avg=0;
uint64_t strain_gauge_ub_avg=0;
uint64_t strain_gauge_lf_avg=0;
uint64_t strain_gauge_lb_avg=0;
uint64_t ir_brake_temp_avg=0;
uint64_t k_brake_temp_avg=0;
uint64_t strain_gauge_push_avg=0;

//vars to store the current avg
//will we be using this current avg anywhere else outside of HAL_ADC_ConvCpltCallback
uint64_t strain_gauge_control_curr_avg = 0;
uint64_t shock_angle_curr_avg =0;
uint64_t strain_gauge_uf_curr_avg=0;
uint64_t strain_gauge_ub_curr_avg=0;
uint64_t strain_gauge_lf_curr_avg=0;
uint64_t strain_gauge_lb_curr_avg=0;
uint64_t ir_brake_temp_curr_avg=0;
uint64_t k_brake_temp_curr_avg=0;
uint64_t strain_gauge_push_curr_avg=0;




// PRIVATE FUNCTION PROTOTYPES
uint16_t get_pres(uint16_t adc_val);
int16_t get_temp(uint16_t adc_val);
int16_t get_air_temp(uint16_t adc_val);
//added this
//uint16_t get_strain_gauge_control(uint16_t adc_val);
//uint16_t get_shock_angle(uint16_t adc_val);
//uint16_t get_straing_gauge_uf(uint16_t adc_val);
//uint16_t get_straing_gauge_ub(uint16_t adc_val);
//uint16_t get_straing_gauge_lf(uint16_t adc_val);
//uint16_t get_straing_gauge_lb(uint16_t adc_val);
//uint16_t get_ir_brake_temp(uint16_t adc_val);
//uint16_t get_k_brake_temp(uint16_t adc_val);
//uint16_t get_strain_gauge_push(uint16_t adc_val);

void set_fan_speed(uint8_t speed);
void set_pump_speed(uint8_t speed);
void update_pwm(int16_t inlet_temp);
void buzzerer();

void TelemNode_Init(){
	CAN_Init();

	ADC_Input_Init(&adc_inlet_pres, &hadc1, ADC_CHANNEL_4, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);
	ADC_Input_Init(&adc_outlet_pres, &hadc1, ADC_CHANNEL_5, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);
	ADC_Input_Init(&adc_inlet_temp, &hadc1, ADC_CHANNEL_0, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);
	ADC_Input_Init(&adc_outlet_temp, &hadc1, ADC_CHANNEL_2, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);
	ADC_Input_Init(&adc_air_in_temp, &hadc1, ADC_CHANNEL_1, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);
	ADC_Input_Init(&adc_air_out_temp, &hadc1, ADC_CHANNEL_3, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);
	ADC_Input_Init(&adc_toe_strain_gauge, &hadc1, ADC_CHANNEL_6, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);
	ADC_Input_Init(&adc_a_arm_strain_gauge, &hadc1, ADC_CHANNEL_7, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);
	ADC_Input_Init(&adc_extra_3, &hadc1, ADC_CHANNEL_8, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);
	ADC_Input_Init(&adc_extra_4, &hadc1, ADC_CHANNEL_9, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);

	PWM_Init(&pwm_fan, &htim1, TIM_CHANNEL_1);
	PWM_Init(&pwm_pump, &htim1, TIM_CHANNEL_2);

	WheelSpeed_Init(&wheel_rr);
	WheelSpeed_Init(&wheel_rl);

	set_pump_speed(255);
	set_fan_speed(128);
}

//added this
//TODO moving converted data into where we need them to be
	//send the sensor data to the right place
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//creating array, b/c it will used for CANsend
	static uint8_t tx_data[8];

	if( num_samples == 0){
	//reading the first set of samples & assigning to the correct variable
	strain_gauge_control_avg = ADC_RES_BUFFER[0];
	shock_angle_avg = ADC_RES_BUFFER[1];
	strain_gauge_uf_avg = ADC_RES_BUFFER[2];
	strain_gauge_ub_avg = ADC_RES_BUFFER[3];
	strain_gauge_lf_avg = ADC_RES_BUFFER[4];
	strain_gauge_lb_avg = ADC_RES_BUFFER[5];
	ir_brake_temp_avg = ADC_RES_BUFFER[6];
	k_brake_temp_avg = ADC_RES_BUFFER[7];
	strain_gauge_push_avg = ADC_RES_BUFFER[8];

	++num_samples;

	}
	else if (num_samples < NUM_SAMPLES_IN_AVGERAGE){
		//calculating average for each sensor
		//equation for new average = old average * (n-1)/n + new value/n
		//++num_samples must be done first, because it will cause funky numbers if done otherwise
		++num_samples;
		strain_gauge_control_avg = strain_gauge_control_avg * (num_samples-1)/num_samples +  ADC_RES_BUFFER[0]/num_samples;
		shock_angle_avg = shock_angle_avg * (num_samples -1)/num_samples + ADC_RES_BUFFER[1]/num_samples;
		strain_gauge_uf_avg = strain_gauge_uf_avg * (num_samples - 1) / num_samples + ADC_RES_BUFFER[2] / num_samples;
		strain_gauge_ub_avg = strain_gauge_ub_avg * (num_samples - 1) / num_samples + ADC_RES_BUFFER[3] / num_samples;
		strain_gauge_lf_avg = strain_gauge_lf_avg * (num_samples - 1) / num_samples + ADC_RES_BUFFER[4] / num_samples;
		strain_gauge_lb_avg = strain_gauge_lb_avg * (num_samples - 1) / num_samples + ADC_RES_BUFFER[5] / num_samples;
		ir_brake_temp_avg = ir_brake_temp_avg * (num_samples - 1) / num_samples + ADC_RES_BUFFER[6] / num_samples;
		k_brake_temp_avg = k_brake_temp_avg * (num_samples - 1) / num_samples + ADC_RES_BUFFER[7] / num_samples;
		strain_gauge_push_avg = strain_gauge_push_avg * (num_samples - 1) / num_samples + ADC_RES_BUFFER[8] / num_samples;

	}
	else{

		uint16_t strain_gauge_control  = get_strain_gauge_val(strain_gauge_control_avg);
		uint16_t shock_angle = get_shock_angle(shock_angle_avg);
		uint16_t strain_gauge_uf = get_strain_gauge_val(strain_gauge_uf_avg);
		uint16_t strain_gauge_ub = get_strain_gauge_val(strain_gauge_ub_avg);
		uint16_t strain_gauge_lf = get_strain_gauge_val(strain_gauge_lf_avg);
		uint16_t strain_gauge_lb = get_strain_gauge_val(strain_gauge_lb_avg);
		uint16_t ir_brake_temp = get_ir_brake_temp(ir_brake_temp_avg);
		uint16_t k_brake_temp = get_k_brake_temp(k_brake_temp_avg);
		uint16_t strain_gauge_push = get_strain_gauge_val(strain_gauge_push_avg);



		//set the strain gauge UL data into the tx_data array
		tx_data[0]= HI8(strain_gauge_uf);
		tx_data[1]= LO8(strain_gauge_uf);
		tx_data[2]= HI8(strain_gauge_ub);
		tx_data[3]= LO8(strain_gauge_ub);
		tx_data[4]= HI8(strain_gauge_lf);
		tx_data[5]= LO8(strain_gauge_lf);
		tx_data[6]= HI8(strain_gauge_lb);
		tx_data[7]= LO8(strain_gauge_lb);
		//call CANsend here
		switch (Location) {
			case (FL):
				CAN_Send(SG_UPPERLOWER_FL, tx_data, 8);
				break;
			case (FR):
				CAN_Send(SG_UPPERLOWER_FR, tx_data, 8);
				break;
			case (RL):
				CAN_Send(SG_UPPERLOWER_RL, tx_data, 8);
				break;
			case (RR):
				CAN_Send(SG_UPPERLOWER_RR, tx_data, 8);
				break;

		}



		//then overwrite tx_data with control, push, and shock angle
		tx_data[0]= HI8(strain_gauge_control);
		tx_data[1]= LO8(strain_gauge_control);
		tx_data[2]= HI8(strain_gauge_push);
		tx_data[3]= LO8(strain_gauge_push);
		tx_data[4]= HI8(shock_angle);
		tx_data[5]= LO8(shock_angle);
		//then call CANsend here
		switch (Location) {
			case (FL):
				CAN_Send(SG_CONTROLSHOCK_FL, tx_data, 6);
				break;
			case (FR):
				CAN_Send(SG_CONTROLSHOCK_FR, tx_data, 6);
				break;
			case (RL):
				CAN_Send(SG_CONTROLSHOCK_RL, tx_data, 6);
				break;
			case (RR):
				CAN_Send(SG_CONTROLSHOCK_RR, tx_data, 6);
				break;

		}

		//Lastly overwrite with brake temp data
		tx_data[0]= HI8(ir_brake_temp);
		tx_data[1]= LO8(ir_brake_temp);
		tx_data[2]= HI8(k_brake_temp);
		tx_data[3]= LO8(k_brake_temp);
		//then call CANsend
		switch (Location) {
			case (FL):
				CAN_Send(BRAKE_TEMP_FL, tx_data, 4);
				break;
			case (FR):
				CAN_Send(BRAKE_TEMP_FR, tx_data, 4);
				break;
			case (RL):
				CAN_Send(BRAKE_TEMP_RL, tx_data, 4);
				break;
			case (RR):
				CAN_Send(BRAKE_TEMP_RR, tx_data, 4);
				break;

		}


		//save most recent average to current average, might use it somewhere else?
		strain_gauge_control_curr_avg = strain_gauge_control_avg;
		shock_angle_curr_avg = shock_angle_avg;
		strain_gauge_uf_curr_avg= strain_gauge_uf_avg;
		strain_gauge_ub_curr_avg= strain_gauge_ub_avg;
		strain_gauge_lf_curr_avg= strain_gauge_lf_avg;
		strain_gauge_lb_curr_avg= strain_gauge_lb_avg;
		ir_brake_temp_curr_avg= ir_brake_temp_avg;
		k_brake_temp_curr_avg= k_brake_temp_avg;
		strain_gauge_push_curr_avg= strain_gauge_push_avg;

		//reset the average, so that next set of calculations won't have any errors
		strain_gauge_control_avg = 0;
		shock_angle_avg = 0;
		strain_gauge_uf_avg = 0;
		strain_gauge_ub_avg = 0;
		strain_gauge_lf_avg = 0;
		strain_gauge_lb_avg = 0;
		ir_brake_temp_avg= 0;
		k_brake_temp_avg = 0;
		strain_gauge_push_avg = 0;

	}
}

void TelemNode_Update()
{
	static uint8_t loop_count = 0;
	static uint8_t tx_data[8];

//	buzzerer();
//	HAL_Delay(LOOP_PERIOD_MS);

	loop_count++;
	if(loop_count <= SLOW_DIVIDER) return;
	loop_count = 0;

	// NOTE: can send temps and pressures at reduced frequency if CAN traffic too high

	ADC_Measure(&adc_inlet_temp, 1000);
	ADC_Measure(&adc_outlet_temp, 1000);
	ADC_Measure(&adc_air_in_temp, 1000);
	ADC_Measure(&adc_air_out_temp, 1000);

	int16_t inlet_temp = get_temp(adc_inlet_temp.value);
	int16_t outlet_temp = get_temp(adc_outlet_temp.value);
	int16_t temp_air_in = get_air_temp(adc_air_in_temp.value);
	int16_t temp_air_out = get_air_temp(adc_air_out_temp.value);

	tx_data[0] = HI8(inlet_temp);
	tx_data[1] = LO8(inlet_temp);
	tx_data[2] = HI8(outlet_temp);
	tx_data[3] = LO8(outlet_temp);
	tx_data[4] = HI8(temp_air_in);
	tx_data[5] = LO8(temp_air_in);
	tx_data[6] = HI8(temp_air_out);
	tx_data[7] = LO8(temp_air_out);
	CAN_Send(COOLING_LOOP_TEMPS, tx_data, 8);

	ADC_Measure(&adc_inlet_pres, 1000);
	ADC_Measure(&adc_outlet_pres, 1000);

	uint16_t inlet_pres = get_pres(adc_inlet_pres.value);
	uint16_t outlet_pres = get_pres(adc_outlet_pres.value);

	tx_data[0] = HI8(inlet_pres);
	tx_data[1] = LO8(inlet_pres);
	tx_data[2] = HI8(outlet_pres);
	tx_data[3] = LO8(outlet_pres);
	CAN_Send(COOLING_LOOP_PRESSURES, tx_data, 4);

	uint32_t wheel_speed_rr = WheelSpeed_GetCPS(&wheel_rr);
	uint32_t wheel_speed_rl = WheelSpeed_GetCPS(&wheel_rl);
	tx_data[0] = HI8(wheel_speed_rr);
	tx_data[1] = LO8(wheel_speed_rr);
	tx_data[2] = HI8(wheel_speed_rl);
	tx_data[3] = LO8(wheel_speed_rl);
	CAN_Send(WHEEL_SPEED_REAR, tx_data, 4); // only sending the lower 16 bytes of each speed

	// TODO: finish when strain gauges are installed
//	ADC_Measure(&adc_toe_strain_gauge, 1000);
//	tx_data[0] = HI8(adc_toe_strain_gauge.value);
//	tx_data[1] = LO8(adc_toe_strain_gauge.value);
//	ADC_Measure(&adc_a_arm_strain_gauge, 1000);
//	tx_data[2] = HI8(adc_a_arm_strain_gauge.value);
//	tx_data[3] = LO8(adc_a_arm_strain_gauge.value);
//	ADC_Measure(&adc_a_arm_strain_gauge, 1000);
//	tx_data[4] = HI8(adc_extra_3.value);
//	tx_data[5] = LO8(adc_extra_3.value);
//	ADC_Measure(&adc_a_arm_strain_gauge, 1000);
//	tx_data[6] = HI8(adc_extra_4.value);
//	tx_data[7] = LO8(adc_extra_4.value);
//	CAN_Send(STRAIN_GAUGE_REAR, tx_data, 8);

	//update_pwm(inlet_temp);
}

void update_pwm(int16_t inlet_temp)
{
	//set_pump_speed(40);

	// allow manual fan and pump speed via CAN
//	if (can_data.PWM_requested) {
//		set_pump_speed(can_data.pumpPWM);
//		set_fan_speed(can_data.fanPWM);
//		return;
//	}

	// threshold variables to add hysteresis
	static int fan_t1 = FAN_THRESH_1 + HYSTERESIS;
	static int fan_t2 = FAN_THRESH_2 + HYSTERESIS;
	static int fan_t3 = FAN_THRESH_2 + HYSTERESIS;
	static int pump_t = PUMP_THRESH + HYSTERESIS;

	//TODO: update these values to consider ambient air temp, vehicle speed, etc?
//	if(can_data.inverter_enable || (can_data.mc_temp_max > pump_t) || (can_data.motor_temp > pump_t)){
//		set_pump_speed(255);
//		pump_t = PUMP_THRESH;
//	} else {
//		set_pump_speed(0);
//		pump_t = PUMP_THRESH + HYSTERESIS;
//	}

	if(inlet_temp > fan_t3){
		set_fan_speed(255);
		fan_t1 = FAN_THRESH_1;
		fan_t2 = FAN_THRESH_2;
		fan_t3 = FAN_THRESH_3;
	} else if(inlet_temp > fan_t2){
		set_fan_speed(180);
		fan_t1 = FAN_THRESH_1;
		fan_t2 = FAN_THRESH_2;
		fan_t3 = FAN_THRESH_3 + HYSTERESIS;
	} else if(inlet_temp > fan_t1){
		fan_t1 = FAN_THRESH_1;
		fan_t2 = FAN_THRESH_2 + HYSTERESIS;
		fan_t3 = FAN_THRESH_3 + HYSTERESIS;
		set_fan_speed(100);
	} else {
		fan_t1 = FAN_THRESH_1 + HYSTERESIS;
		fan_t2 = FAN_THRESH_2 + HYSTERESIS;
		fan_t3 = FAN_THRESH_3 + HYSTERESIS;
		set_fan_speed(0);
	}
}

uint16_t get_pres(uint16_t adc_val)
{
	// GE2098 sensor
	// equation from datasheet
	float v = (float)adc_val * (3.3/4095.0) / VOLTAGE_DIVIDER_RATIO;
	float pres = (((v / 5.0) + 0.011453) / 0.0045726) * PSI_PER_KPA;
	return (uint16_t)(pres*100);
}

int16_t get_temp(uint16_t adc_val)
{
	// need to recalibrate these sensors with new GE2098(Already calibrated)
	float temp = (99.2596*exp((-3.22926) * adc_val / 4095) - 21.4981-3.17)/1.01085;
	return (int16_t) (temp*10);
}

int16_t get_air_temp(uint16_t adc_val)
{
	float temp = 83.35412 - 0.03634221 * adc_val +0.0000034466 * adc_val * adc_val;
	return (int16_t) (temp *10);
}

uint16_t get_strain_gauge_val(uint64_t adc_val){
	return (uint16_t)(adc_val * 0.001268);
}
uint16_t get_shock_angle(uint64_t adc_val){
	return (uint16_t)(adc_val); // TODO FIX ME
}
uint16_t get_ir_brake_temp(uint64_t adc_val){
	return (uint16_t)adc_val; // TODO FIX ME YAAA
}
uint16_t get_k_brake_temp(uint64_t adc_val){
	return (uint16_t)adc_val; // TODO FIX
}






void set_pump_speed(uint8_t speed)
{
	PWM_SetDutyCycle(&pwm_pump, speed);
}

void set_fan_speed(uint8_t speed)
{
	PWM_SetDutyCycle(&pwm_fan, speed);
}

//void buzzerer()
//{
//	static VCU_STATE_t last_vcu_state = LV;
//	static uint32_t buzz_start = 0;
//	uint32_t tick = HAL_GetTick();
//
//	if(last_vcu_state == HV_ENABLED && can_data.vcu_state == DRIVE)
//	{
//		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1); // turn on buzzer
//		buzz_start = tick;
//	}
//	else if(can_data.vcu_state != DRIVE || (tick - buzz_start) > BUZZ_TIME_MS)
//	{
//		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0); // turn off buzzer
//	}
//	else
//	{
//		// ice cream?
//	}
//
//	last_vcu_state = can_data.vcu_state;
//}

// TODO dont use interrupts, see FE11-12 Dashboard code (use TIM)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
		case GPIO_PIN_4:
			wheel_rr.count++;
			break;
		case GPIO_PIN_6:
			wheel_rl.count++;
			break;
	}
}



