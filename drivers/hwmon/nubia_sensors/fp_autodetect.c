#include "fp_autodetect.h"

static char fingerprint_gpio_status[CHAR_LENGTH] = FINGERPRINT_GPIO_STATUS_UNKNOW;

static int fingerprint_gpio_status_get(char *param)
{
    memcpy(fingerprint_gpio_status, param, strlen(param));
    return 0;
}
early_param("fp_target", fingerprint_gpio_status_get);

int fingerprint_device_autodetect(char *target_fingerprint_name)
{
    char pcb_version[CHAR_LENGTH];
    char pcb_config[CHAR_LENGTH];
    char right_fingerprint_name[CHAR_LENGTH] = FINGERPRINT_DEVICE_NAME_UNKNOW;

    ztemt_get_hw_pcb_version(pcb_version);
    ztemt_get_config_standard(pcb_config);

    if ((strcmp(pcb_version, "MB_A")>0)) {
        switch(fingerprint_gpio_status[0]) {
            case FINGERPRINT_GPIO_STATUS_GOODIX: {
                sprintf(right_fingerprint_name, FINGERPRINT_DEVICE_NAME_GOODIX);
                break;
            }

            case FINGERPRINT_GPIO_STATUS_EGIS: {
                sprintf(right_fingerprint_name, FINGERPRINT_DEVICE_NAME_EGIS);
                break;
            }

            case FINGERPRINT_GPIO_STATUS_FPC: {
                sprintf(right_fingerprint_name, FINGERPRINT_DEVICE_NAME_FPC);
                break;
            }

            default: {
                break;
            }
        }
    } else {
        sprintf(right_fingerprint_name, "%s", pcb_config);
    }

    FPAUTODETECT_LOG_INFO("the target_fingerprint_name fingerprint device is %s\n",target_fingerprint_name);
    FPAUTODETECT_LOG_INFO("the %s is the %s fingerprint device\n",target_fingerprint_name,
                           (strcmp(right_fingerprint_name, target_fingerprint_name)==0)? "right" : "wrong");

    return strcmp(right_fingerprint_name, target_fingerprint_name)==0 ? true : false;
}
EXPORT_SYMBOL_GPL(fingerprint_device_autodetect);
