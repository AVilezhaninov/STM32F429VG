/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

/******************************************************************************/
/* Private definitions ********************************************************/
/******************************************************************************/
#define UNIQUE_ID_ADDRESS   0x1FFF7A10u
#define UNIQUE_ID_SIZE      12u

/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main() {
  uint8_t *unique_id_address = (uint8_t *)UNIQUE_ID_ADDRESS;
  uint8_t unique_id[UNIQUE_ID_SIZE] = {0u};

  for (uint8_t i = 0u; i < UNIQUE_ID_SIZE; ++i) {
    unique_id[i] = *unique_id_address;
    ++unique_id_address;
  }

  while (1) {
    ;
  }
}
