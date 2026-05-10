#include <stdio.h>
#include <stdlib.h>
#include <ctime>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

int main()
{
    LOG_INF("Reiki SDK Shell");
    k_thread_suspend(k_current_get());
    return 0;
}
