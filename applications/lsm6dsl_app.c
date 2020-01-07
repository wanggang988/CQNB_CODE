#include "sensor_st_lsm6dsl.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#define LSM6DSL_ADDR (0xD4 >> 1)
#define LSM6DSL_IR_PIN    GET_PIN(D, 11)	//PD11:LSM6DSLµÄÖÐ¶Ï¹Ü½Å¡£
int lsm6dsl_port(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.dev_name = "i2c1";
    cfg.intf.user_data = (void *)LSM6DSL_ADDR;    //LSM6DSL_ADDR_DEFAULT
	cfg.irq_pin.pin = RT_PIN_NONE;
    rt_hw_lsm6dsl_init("lsm6dsl", &cfg);
    return 0;
}
//INIT_APP_EXPORT(lsm6dsl_port);