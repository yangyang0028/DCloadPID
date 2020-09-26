#include "main.h"

#define LEN_i 50
uint16_t lv_I[LEN_i];
uint16_t I_i=0;


uint16_t pingjunzhi(){
	uint32_t ans=0;
	for(int i=0;i<LEN_i;i++){
		ans+=lv_I[i];
	}
	return (uint16_t)(ans/LEN_i);
}