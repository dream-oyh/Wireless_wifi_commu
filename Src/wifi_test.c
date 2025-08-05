#include "wifi_test.h"
#include <stdint.h>
#include <stdbool.h>

bool isWifiConnected(void){
	int sum = 0;
	uint8_t primaryRelayRequest[8] = {};
	uint8_t primaryRelayResponse[8] = {};
	HAL_StatusTypeDef isPrimaryRelayResponse = HAL_TIMEOUT;
	do {
		usart2_print(primaryRelayRequest, 8);
		HAL_Delay(1000);
		isPrimaryRelayResponse = usart2_receive(primaryRelayResponse, 8, 20);
		sum ++;
		if (sum>WIFI_CONNECT_TIMEOUT){
			Error_Handler();
			return false;
			break;
		}
	}while(isPrimaryRelayResponse != HAL_OK);
	if (primaryRelayResponse[0] == 0xAA){  // 判断回复数据是否正确
		return true;
	}else {
		return false;
	}

}

