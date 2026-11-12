/**
  * @file       usb_task.c
  * @brief      USB CDC binary protocol task for the sentry lower controller.
  */

#include "usb_task.h"

#include "cmsis_os.h"
#include "robot_comm.h"
#include "usb_device.h"
#include "stm32f4xx_hal_pcd.h"
#include "usbd_core.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

#define USB_LINK_RECOVERY_MS    1000U
#define USB_RECONNECT_LOW_MS     20U
#define USB_RECOVERY_RETRY_MS   1000U

static uint8_t usb_device_link_present(void)
{
    /*
     * A configured or suspended USB device is still attached to the host.
     * Missing upper-computer frames only means the logical control link is
     * idle; it must not cause the USB peripheral to be reinitialized.
     */
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED ||
            hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED) ? 1U : 0U;
}

static void usb_device_recover(void)
{
    (void)HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    (void)HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS);
    osDelay(USB_RECONNECT_LOW_MS);

    /* USBD_DeInit stops the class and resets the PCD before full re-registration. */
    (void)USBD_DeInit(&hUsbDeviceFS);
    MX_USB_DEVICE_Init();
}

void usb_task(void const *argument)
{
    uint32_t last_recovery_tick;
    uint32_t link_lost_tick;

    (void)argument;

    robot_comm_init();
    MX_USB_DEVICE_Init();
    last_recovery_tick = HAL_GetTick() - USB_RECOVERY_RETRY_MS;
    link_lost_tick = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (usb_device_link_present())
        {
            link_lost_tick = now;
        }
        else if ((uint32_t)(now - link_lost_tick) >= USB_LINK_RECOVERY_MS &&
                 (uint32_t)(now - last_recovery_tick) >= USB_RECOVERY_RETRY_MS)
        {
            last_recovery_tick = now;
            usb_device_recover();
            link_lost_tick = HAL_GetTick();
        }

        robot_comm_update();
        osDelay(1);
    }
}
