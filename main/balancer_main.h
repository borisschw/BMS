#ifndef MAIN_BALANCER_MAIN_H_
#define MAIN_BALANCER_MAIN_H_

#define DOCKED_TO_BASE 0

typedef enum {
	ez_config = 0,
	full_ini = 1,
	short_ini=2
}INIT_METHOD;


void configure_uart(uart_config_t* uart_config);

typedef enum
{
	e_CELL_MAXIMUM_VOLTAGE,
	e_CELL_LOW_VOLTAGE,
    e_CELLS_NOT_BALANCED,
    e_CELLS_BALANCED
} eCellVoltageStatus;

//Different state of ATM machine
typedef enum
{
	e_DOCK_STATE,
    e_MEASURE_CELL_STATE,
    e_CHARGE_ENABLE_STATE,
    e_BALANCING_STATE,
	e_STOP_BALANCING_STATE
} eSystemState;



////Different type events
//typedef enum
//{
//    Docking_Indication_Event,
//    Cell_Voltage_MAX_Event,
//    Cell_Not_balanced_Event,
//    Current_Ok_Event,
//    Balancing_Completed_Event,
//	Undocking_Event
//} eSystemEvent;
//

static void balancer_task(void *arg);
static void fule_gauge_task(void *arg);
static void docking_task(void *arg);

int docking_state_handler(void);

int measure_cells_handler(void);

int charge_enable_handler(void);

int balancing_handler(void);

void set_charge_en_switch(int val);

int stop_balancing(void);

eCellVoltageStatus check_voltages(float *cell_voltage);


#endif /* MAIN_BALANCER_MAIN_H_ */
