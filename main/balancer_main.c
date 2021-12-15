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
/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

int UART_RX_RDY = 0;



//static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)


static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    int intr_alloc_flags = 0;
    int i = 0;

    uint8_t data[2];
    uint8_t led_state = 0;
    BYTE response_frame[(MAXBYTES+6)*TOTALBOARDS];

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    /* Configure wakeup pin */
    gpio_reset_pin(WAKEUP_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction(WAKEUP_GPIO, GPIO_MODE_OUTPUT);

	/* Configure LED pin*/
	gpio_reset_pin(LED_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);


	balancer_wake();
	memset(response_frame, 0, sizeof(response_frame));
	i2c_master_init();

	while(1)
	{
		gpio_set_level(LED_GPIO, led_state);
		led_state = ~led_state;
		if(maxim_max1726x_check_por())
			printf("Power on reset\n");
		else
			printf("No power on reset\n");

		printf("dev name = 0x%x \n", maxim_max1726x_get_devname());

		maxim_max1726x_wait_dnr();

		maxim_max1726x_initialize_ez_config();
		//maxim_max1726x_initialize_full_ini();
		if (maxim_max1726x_clear_por())
			printf("Readback error!! \n");
		else
			printf("Readback Success! \n");


		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	InitDevices();

	WriteReg(0, CELL_ADC_CONF2, 0x08, 1, FRMWRT_SGL_NR);    //set continuous ADC conversions, and set minimum conversion interval

	set_balance_duty_cycle(SEC, BAL_DUTY_10, FLTCONTINUE, ODDS_THEN_EVENS);
	set_cell_balancing_time(1, SEC, 3);
	set_cell_balancing_time(2, SEC, 3);
	set_cell_balancing_time(3, SEC, 3);
	set_cell_balancing_time(4, SEC, 3);
	set_cell_balancing_time(5, SEC, 3);
	set_cell_balancing_time(6, SEC, 3);

	cell_adc_measurment_start();
	enable_cell_balancing();
	//pause_cell_balancing();
	// resume_cell_balancing();

	delayus(3*TOTALBOARDS+901);                             //3us of re-clocking delay per board + 901us waiting for first ADC conversion to complete

    while (1) {
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    	gpio_set_level(LED_GPIO, led_state);

    	led_state = ~led_state;
		//*******************
		//READ CELL VOLTAGES
		//*******************
		//reset variables
		memset(response_frame, 0, sizeof(response_frame));

		//tcflush(fd, TCIOFLUSH);
		ReadReg(0, VCELL1H, response_frame, 6*2, 0, FRMWRT_SGL_R);

		/*
		 * ***********************************************
		 * NOTE: SOME COMPUTERS HAVE ISSUES TRANSMITTING
		 * A LARGE AMOUNT OF DATA VIA PRINTF STATEMENTS.
		 * THE FOLLOWING PRINTOUT OF THE RESPONSE DATA
		 * IS NOT GUARANTEED TO WORK ON ALL SYSTEMS.
		 * ***********************************************
		*/

		//PARSE, FORMAT, AND PRINT THE DATA
		printf("\n"); //start with a newline to add some extra spacing between loops

		//response frame actually starts with top of stack, so currentBoard is actually inverted from what it should be
		printf("BOARD %d:\t",TOTALBOARDS);

		//go through each byte in the current board (12 bytes = 6 cells * 2 bytes each)
		for(i=0; i<12; i+=2)
		{
			//each board responds with 32 data bytes + 6 header bytes
			//so need to find the start of each board by doing that * currentBoard

			//convert the two individual bytes of each cell into a single 16 bit data item (by bit shifting)
			unsigned short rawData = (response_frame[i+4] << 8) | response_frame[i+5];

			//do the two's complement of the resultant 16 bit data item, and multiply by 190.73uV to get an actual voltage
			float cellVoltage = Complement(rawData,0.00019073);

			//print the voltages - it is i/2 because cells start from 1 up to 6
			//and there are 2 bytes per cell (i value is twice the cell number),
			//and it's +1 because cell names start with "Cell1"
			printf("Cell %d = %f\t", (i/2)+1, cellVoltage);
		}
		printf("\n"); //newline per board



    }
}

void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
