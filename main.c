/* ------------------------------------------
 * Copyright (c) 2017, Synopsys, Inc. All rights reserved.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1) Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.

 * 3) Neither the name of the Synopsys, Inc., nor the names of its contributors may
 * be used to endorse or promote products derived from this software without
 * specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
--------------------------------------------- */

#include "embARC.h"
#include "embARC_debug.h"

#include "rn4020.h"

#define RN4020_UART_ID DFSS_UART_3_ID
#define RN4020_WAKE_SW_GPIO DFSS_GPIO_4B0_ID
#define RN4020_WAKE_SW_PIN 2
#define RN4020_WAKE_HW_GPIO DFSS_GPIO_4B0_ID
#define RN4020_WAKE_HW_PIN 1
#define RN4020_CMD_GPIO DFSS_GPIO_4B0_ID
#define RN4020_CMD_PIN 3


RN4020_DEFINE(rn4020_ble, RN4020_UART_ID, RN4020_WAKE_SW_GPIO, RN4020_WAKE_SW_PIN
	, RN4020_WAKE_HW_GPIO, RN4020_WAKE_HW_PIN, RN4020_CMD_GPIO, RN4020_CMD_PIN);

uint8_t test_private_services_uuid[RN4020_PRIVATE_UUID_LENGTH_BYTES] =
				{0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

uint8_t test_private_characteristic_0_uuid[RN4020_PRIVATE_UUID_LENGTH_BYTES] =
				{0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

uint8_t test_private_characteristic_1_uuid[RN4020_PRIVATE_UUID_LENGTH_BYTES] =
				{0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};


#define GPIO4B2_0_OFFSET	0
#define GPIO4B2_1_OFFSET	1

DEV_GPIO_PTR gpio_4b2;
DEV_PWM_TIMER_PTR pwm_timer_test;
void init();

int main(void) {
	int speed=40;
	pwm_timer_test = pwm_timer_get_dev(DW_PWM_TIMER_0_ID);
	init();
	char Commend,preCom;
	// pre_rx_buffer
	//gpio_4b2->gpio_write(0 << GPIO4B2_0_OFFSET, 1 << GPIO4B2_0_OFFSET);
	io_arduino_config(ARDUINO_PIN_5, ARDUINO_PWM, IO_PINMUX_ENABLE);//pwm timer ch1
	io_arduino_config(ARDUINO_PIN_6, ARDUINO_PWM, IO_PINMUX_ENABLE);//pwm timer ch2
	io_arduino_config(ARDUINO_PIN_9, ARDUINO_PWM, IO_PINMUX_ENABLE);//pwm timer ch3
	io_arduino_config(ARDUINO_PIN_10, ARDUINO_PWM, IO_PINMUX_ENABLE);//pwm timer ch4
	pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左前
	pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左後
	pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右後
	pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右前
	while (1) {
		
		if(rn4020_ble->need_advertise){
			rn4020_ble->need_advertise = 0;
			rn4020_advertise(rn4020_ble);
		}

		char data = 0;
		uint8_t rd_cnt;

		do {
			rd_cnt = ez_sio_read(rn4020_ble->sio_uart, &data, 1);

			if (rd_cnt != 0 && data != 0) {
				rn4020_ble->rx_buffer[rn4020_ble->rx_cnt++] = data;
			}
		} while (data != '\n' && rn4020_ble->rx_cnt < RN4020_RX_BUFFER_SIZE);

		if (rn4020_ble->rx_buffer[rn4020_ble->rx_cnt - 2] == '\r') {
			/* remove \r\n */
			rn4020_ble->rx_buffer[rn4020_ble->rx_cnt - 2] = 0;
		} else 
			rn4020_ble->rx_buffer[rn4020_ble->rx_cnt - 1] = 0;

		rn4020_ble->rx_cnt = 0;

		Commend=rn4020_ble->rx_buffer[9];

		switch(Commend){
			case '4':
				if(preCom==1){
					pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左後
					pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右後				
					pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 80);//左前
					pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 80);//右前
				}else if(preCom==2){
					pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左前
					pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右後
					pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 80);//左後				
					pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 80);//右前
				}else if(preCom=3){
					pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右前
					pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左後
					pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 80);//右後
					pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 80);//左前
				}else{
					pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左前
					pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左後
					pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右後
					pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右前
				}
				break;
			case '5':
				if(preCom==1){
					pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左後
					pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右後				
					pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//左前
					pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//右前
				}else if(preCom==2){
					pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左前
					pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右後
					pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//左後				
					pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//右前
				}else if(preCom=3){
					pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右前
					pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左後
					pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//右後
					pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//左前
				}else{
					pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左前
					pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左後
					pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右後
					pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右前
				}
				break;
			case '0':
				pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左前
				pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左後
				pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右後
				pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右前
				//printf("Stop\n");
				break;
			case '1':
				pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左後
				pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右後				
				pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//左前
				pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//右前
				preCom=1;
				//printf("Front\n");
				break;
			case '2':
				pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左前
				pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右後
				pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//左後				
				pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//右前
				preCom=2;
				//printf("Left\n");
				break;
			case '3':
				pwm_timer_test->pwm_timer_write(4, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//右前
				pwm_timer_test->pwm_timer_write(2, DEV_PWM_TIMER_MODE_PWM, 100000, 0);//左後
				pwm_timer_test->pwm_timer_write(3, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//右後
				pwm_timer_test->pwm_timer_write(1, DEV_PWM_TIMER_MODE_PWM, 100000, 40);//左前
				preCom=3;
				//printf("Rightp\n");
				break;
			break;
		}
		//printf("%c\n", rn4020_ble->rx_buffer[9]);



		
	}
	

}
void init(){
	// gpio_4b2 = gpio_get_dev(DFSS_GPIO_4B2_ID); // get GPIO_4B2 handler
	// gpio_4b2->gpio_open(1 << GPIO4B2_0_OFFSET ); // open
	// gpio_4b2->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT, (void *)(1 << GPIO4B2_0_OFFSET )); 
	pwm_timer_test->pwm_timer_open();



	EMBARC_PRINTF("rn4020 test application\r\n");
	rn4020_setup(rn4020_ble);
	rn4020_reset_to_factory(rn4020_ble);
	rn4020_set_dev_name(rn4020_ble, "embARC");
	rn4020_set_services(rn4020_ble, RN4020_SERVICE_USER_DEFINED);
	rn4020_clear_private(rn4020_ble);
	//rn4020_add_prv_service(rn4020_ble, test_private_services_uuid);
	//rn4020_add_prv_char(rn4020_ble, test_private_characteristic_0_uuid,RN4020_PRIVATE_CHAR_PROP_NOTIFY | RN4020_PRIVATE_CHAR_PROP_READ,5, RN4020_PRIVATE_CHAR_SEC_NONE);
	rn4020_add_prv_char(rn4020_ble, test_private_characteristic_1_uuid,RN4020_PRIVATE_CHAR_PROP_WRITE | RN4020_PRIVATE_CHAR_PROP_READ,5, RN4020_PRIVATE_CHAR_SEC_NONE);
	rn4020_set_features(rn4020_ble,RN4020_FEATURE_SERVER_ONLY );//0x00000800 Auto-enter MLDP Mode    RN4020_FEATURE_SERVER_ONLY
	rn4020_reset(rn4020_ble);
	rn4020_advertise(rn4020_ble);
	rn4020_refresh_handle_uuid_table(rn4020_ble);
}