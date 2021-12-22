/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "bq79606.h"
#include "i2c_interface.h"
#include "max1726x.h"
#include "balancer_main.h"


xSemaphoreHandle xBinSema;
#define BOARD_ID 0

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

/* --------------------- Battery Settings ------------------------------  */
#define NUM_OF_CELLS 6

// If a Cell reach this voltage, no charging be allowed - just balancing
#define MAX_VOLTAGE_ALLOWED_FOR_CELL 4.3

// This is a minimum voltage to indicate that something wrong with Cell (or not connected)
#define MIN_VOLTAGE_ALLOWED_FOR_CELL 2.5

// Allowed voltage difference between cells
#define CELL_VOLTAGE_ALLOWED_DELTA 0.15

// The lowest cell value to indicate that Battery is charged
//#define CHARGED_CELL_VOLTAGE 4.1
#define CHARGED_CELL_VOLTAGE 3.89



int UART_RX_RDY = 0;
static eSystemState state;

TaskHandle_t Task_Handle = NULL;

#define BUF_SIZE (1024)

int init_fule_gauge(INIT_METHOD init)
{
	int ret = 0;

	if(maxim_max1726x_check_por())
	{
		ret = 0;
		printf("Power on reset\n");
	}
	else
	{
		ret = -1;
		printf("No power on reset\n");
	}

	printf("dev name = 0x%x \n", maxim_max1726x_get_register(MAX1726X_DEVNAME_REG));

	maxim_max1726x_wait_dnr();

	switch(init)
	{
	case ez_config:
		maxim_max1726x_initialize_ez_config();
		break;
	case full_ini:
		maxim_max1726x_initialize_full_ini();
		break;
	case short_ini:
		maxim_max1726x_initialize_short_ini();
		break;
	}

	if (maxim_max1726x_clear_por())
		printf("Readback error!! \n");
	else
		printf("Readback Success! \n");

	return ret;
}

void set_charge_en_switch(int val)
{
	/* Configure docking_ind pin */
	gpio_reset_pin(CHARGE_EN_GPIO);
	/* Set the GPIO as a input */
	gpio_set_direction(CHARGE_EN_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CHARGE_EN_GPIO,val);
}


static void docking_task(void *arg)
{
	int gpio_value = 0;
	uint8_t led_state = 0;
	/* Configure LED pin*/
	gpio_reset_pin(LED_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);


//	/* Configure docking_ind pin */
//    gpio_reset_pin(DOCKING_GPIO);
//	/* Set the GPIO as a input */
//	gpio_set_direction(DOCKING_GPIO, GPIO_MODE_INPUT);

	while(1)
	{
		gpio_set_level(LED_GPIO, led_state);
		led_state = ~led_state;
//		gpio_value = gpio_get_level(DOCKING_GPIO);
//		printf("----> Docking state = %d\n", gpio_value);
//		if (gpio_value != DOCKED_TO_BASE)
//		{
//			printf("Docked to base! \n");
//			/*Nofify tasks*/
//			//xTaskNotify( balancer_task, 0, eNoAction);
//			//xTaskNotifyGive(Task_Handle);
//		}
		vTaskDelay(1000 / portTICK_PERIOD_MS); /*100 ms*/
	}
}


void configure_uart(uart_config_t* uart_config)
{
	int intr_alloc_flags = 0;
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
	uart_config->baud_rate = ECHO_UART_BAUD_RATE;
	uart_config->data_bits = UART_DATA_8_BITS;
	uart_config->parity = UART_PARITY_DISABLE;
	uart_config->stop_bits = UART_STOP_BITS_1;
	uart_config->flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	uart_config->source_clk = UART_SCLK_APB;

	ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, uart_config));
	ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
}


static void fule_gauge_task(void *arg)
{
	i2c_master_init();

	init_fule_gauge(ez_config);

	while(1)
	{
		printf("Vcell voltage = %f\n",maxim_max1726x_get_register(MAX1726X_VCELL_REG)/2.5 );
		printf("Dev name = 0x%x\n",maxim_max1726x_get_register(MAX1726X_DEVNAME_REG) );
		printf("Tesmp = 0x%x\n",maxim_max1726x_get_register(MAX1726X_TEMP_REG) );
		printf("state-of-charge percentage = %d \n",maxim_max1726x_get_register(MAX1726X_REPSOC_REG));
		printf("remaining capacity in mAh = %d \n",maxim_max1726x_get_register(MAX1726X_REPCAP_REG));

		printf("\n\n");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}


int docking_state_handler(void)
{

	printf("docking_state_handler\n");
	int gpio_value  = ~DOCKED_TO_BASE; // Init to Undock position

	gpio_value = gpio_get_level(DOCKING_GPIO);
	printf("----> Docking state = %d\n", gpio_value);
	if (gpio_value != DOCKED_TO_BASE) // For normal operation, change the != to ==
	{
		printf("Docked to base! \n");
		state = e_MEASURE_CELL_STATE;
	}
	else
	{
		state = e_DOCK_STATE;
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
    return 0;
}

eCellVoltageStatus check_voltages(float *cell_voltage)
{
	int low_voltage_flag = 0;
	float maximum_voltage = 0;
	float minimum_voltage = MAX_VOLTAGE_ALLOWED_FOR_CELL * 2;
	float voltage_delta;

	low_voltage_flag = 0;

	for (int i = 0; i< NUM_OF_CELLS; i++)
	{
		if (cell_voltage[i] > maximum_voltage)
		{
			maximum_voltage = cell_voltage[i];
		}
		if (cell_voltage[i] < minimum_voltage)
		{
			minimum_voltage = cell_voltage[i];
		}
		if (cell_voltage[i] <= MIN_VOLTAGE_ALLOWED_FOR_CELL)
		{
			low_voltage_flag = 1;
		}
	}

	voltage_delta = maximum_voltage - minimum_voltage;

	printf("\nmaximum_voltage = %f\n", maximum_voltage);
	printf("minimum_voltage = %f\n", minimum_voltage);
	printf("voltage Delta = %f\n", voltage_delta);

	if (low_voltage_flag)
		return e_CELL_LOW_VOLTAGE;

	if (maximum_voltage >= MAX_VOLTAGE_ALLOWED_FOR_CELL)
	{
		return e_CELL_MAXIMUM_VOLTAGE;
	}
	else
	{
		if ((voltage_delta < CELL_VOLTAGE_ALLOWED_DELTA) && minimum_voltage > CHARGED_CELL_VOLTAGE)
		{
			return e_CELLS_BALANCED;
		}
		else
		{
			return e_CELLS_NOT_BALANCED;
		}
	}
}

int measure_cells_handler(void)
{
	BYTE response_frame[(MAXBYTES+FRAME_OVERHEAD_SIZE)*TOTALBOARDS];
	int i;
	eCellVoltageStatus voltage_status = e_CELLS_NOT_BALANCED;
	float cell_voltage[NUM_OF_CELLS] = {0};
	printf("measure_cells_handler\n");

	set_charge_en_switch(0);

	cell_adc_measurment_start();

	/*Waiting for first ADC conversion to complete*/
	delayus(1000);
	memset(response_frame, 0, sizeof(response_frame));

	ReadReg(BOARD_ID, VCELL1H, response_frame, NUM_OF_CELLS*NUM_OF_BYTES_IN_REG, 0, FRMWRT_SGL_R);

	for(i=0; i < NUM_OF_CELLS*NUM_OF_BYTES_IN_REG; i+=NUM_OF_BYTES_IN_REG)
	{
		//Each board responds with 32 data bytes + 6 header bytes
		//so need to find the start of each board by doing that * currentBoard

		//convert the two individual bytes of each cell into a single 16 bit data item (by bit shifting)
		unsigned short rawData = (response_frame[i+FRAME_DATA_OFFSET] << 8) | response_frame[i+FRAME_DATA_OFFSET+1];

		//Do the two's complement of the resultant 16 bit data item, and multiply by 190.73uV to get an actual voltage
		cell_voltage[i/2] = Complement(rawData,0.00019073);

		//print the voltages - it is i/2 because cells start from 1 up to 6
		//and there are 2 bytes per cell (i value is twice the cell number),
		//and it's +1 because cell names start with "Cell1"
		printf("Cell %d = %f\t", (i/2)+1, cell_voltage[i/2]);
	}

	voltage_status = check_voltages(cell_voltage);
	switch (voltage_status)
	{
		case e_CELL_MAXIMUM_VOLTAGE:
			state = e_BALANCING_STATE;
			break;
		case e_CELLS_NOT_BALANCED:
			state = e_CHARGE_ENABLE_STATE;
			break;
		case e_CELLS_BALANCED:
			state = e_MEASURE_CELL_STATE;
			break;
		case e_CELL_LOW_VOLTAGE:
			state = e_MEASURE_CELL_STATE;
	}

	if (gpio_get_level(DOCKING_GPIO) == DOCKED_TO_BASE) // For Normal operation change = to !=
		state = e_DOCK_STATE;

	return 0;
}

int charge_enable_handler(void)
{
	printf("charge_enable_handler\n");

	if (gpio_get_level(DOCKING_GPIO) != DOCKED_TO_BASE) // For Normal operation change != to ==
	{
		set_charge_en_switch(1);
		/*Check current*/
		//TBD
		state = e_BALANCING_STATE;
	}
	else
		state = e_DOCK_STATE;

	return 0;
}

int balancing_handler(void)
{
	printf("balancing_handler\n");
	int dock_gpio_val = ~DOCKED_TO_BASE;
	int cb_run = 0;
	BYTE dev_status_frame[1 + FRAME_OVERHEAD_SIZE];

	/*Enable Cell Balancing*/
	start_cell_balancing();

	/*Read status and wait until done*/
	do
	{
		dock_gpio_val = gpio_get_level(DOCKING_GPIO);
		ReadReg(0, DEV_STAT, dev_status_frame , 1 , 0, FRMWRT_SGL_R);
		printf("DEV_STAT = 0x%x\n", dev_status_frame[FRAME_DATA_OFFSET]);
		cb_run = (dev_status_frame[FRAME_DATA_OFFSET] >> 4) & 0x01;
		printf("cb_run = 0x%x\n", cb_run);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}while(cb_run && (dock_gpio_val != DOCKED_TO_BASE) );// For Normal operation change != to ==

	if (dock_gpio_val != DOCKED_TO_BASE) // For Normal operation change != to ==
	{
		state = e_MEASURE_CELL_STATE;
	}
	else
	{
		state = e_STOP_BALANCING_STATE;
	}
	return 0;
}

int stop_balancing(void)
{
	printf("stop balancing handler\n");
	set_charge_en_switch(0);
	stop_cell_balancing();
	state = e_DOCK_STATE;
	return 0;
}


static void balancer_task(void *arg)
{
	uart_config_t uart_config;
   // int i = 0;
//    BYTE response_frame[(MAXBYTES+6)*TOTALBOARDS];

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    xSemaphoreTake(xBinSema,portMAX_DELAY);
    configure_uart(&uart_config);
	balancer_wake();
	init_balancer();
	/*Set continuous ADC conversions, and set minimum conversion interval*/
	WriteReg(0, CELL_ADC_CONF2, 0x08, 1, FRMWRT_SGL_NR);
	configure_bal_policy();

	xSemaphoreGive(xBinSema);

	/* Configure docking_ind pin */
	gpio_reset_pin(DOCKING_GPIO);
	/* Set the GPIO as a input */
	gpio_set_direction(DOCKING_GPIO, GPIO_MODE_INPUT);

	state = e_DOCK_STATE;

	while (1) {
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    	printf("\n###  state ### = %d\n", state );
    	switch (state)
    	{
    	case e_DOCK_STATE:
    		/*wait for notification from the dock task*/
    		docking_state_handler();
    		break;
    	case e_MEASURE_CELL_STATE:
    		measure_cells_handler();
    		break;
    	case e_CHARGE_ENABLE_STATE:
    		charge_enable_handler();
    		break;
    	case e_BALANCING_STATE:
    		balancing_handler();
    		break;
    	case e_STOP_BALANCING_STATE:
    		stop_balancing();
    		break;
    	default:
    		state = e_DOCK_STATE;
    		break;

    	}
    }
}

void app_main(void)
{
	vSemaphoreCreateBinary(xBinSema);
	xTaskCreate(docking_task, "docking_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(balancer_task, "balancer_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    //xTaskCreate(fule_gauge_task, "fule_gauge_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
