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

/* Kept global so ST-Link can inspect them without stopping the USB task. */
volatile uint32_t usb_debug_loop_count;
volatile uint32_t usb_debug_recovery_count;
volatile uint32_t usb_debug_suspend_trigger_count;
volatile uint32_t usb_debug_recovery_stage;
volatile uint32_t usb_debug_dev_state;
volatile uint32_t usb_debug_dsts;
volatile uint32_t usb_debug_stack_free_words;

#define USB_LINK_RECOVERY_MS     1000U
#define USB_SUSPEND_RECOVERY_MS  2000U
#define USB_RECONNECT_LOW_MS       20U
#define USB_RECOVERY_RETRY_MS    1000U
#define USB_OTG_FS_DEVICE \
    ((USB_OTG_DeviceTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))

static uint8_t usb_device_link_present(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
}

static uint8_t usb_device_is_suspended(void)
{
    return ((hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED) ||
            ((USB_OTG_FS_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) != 0U)) ? 1U : 0U;
}

static void usb_device_recover(void)
{
    usb_debug_recovery_stage = 1U;
    (void)HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    (void)HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS);
    osDelay(USB_RECONNECT_LOW_MS);

    /* USBD_DeInit stops the class and resets the PCD before full re-registration. */
    usb_debug_recovery_stage = 2U;
    (void)USBD_DeInit(&hUsbDeviceFS);
    usb_debug_recovery_stage = 3U;
    MX_USB_DEVICE_Init();
    usb_debug_recovery_stage = 4U;
}

void usb_task(void const *argument)
{
    uint32_t last_recovery_tick;
    uint32_t link_lost_tick;
    uint32_t suspend_started_tick;
    uint8_t suspend_active;

    (void)argument;

    robot_comm_init();
    MX_USB_DEVICE_Init();
    last_recovery_tick = HAL_GetTick() - USB_RECOVERY_RETRY_MS;
    link_lost_tick = HAL_GetTick();
    suspend_started_tick = 0U;
    suspend_active = 0U;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        uint8_t recover_requested = 0U;

        usb_debug_loop_count++;
        usb_debug_dev_state = hUsbDeviceFS.dev_state;
        usb_debug_dsts = USB_OTG_FS_DEVICE->DSTS;
        usb_debug_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(NULL);

        if (usb_device_link_present())
        {
            link_lost_tick = now;
            suspend_active = 0U;
        }
        else if (usb_device_is_suspended())
        {
            if (!suspend_active)
            {
                suspend_active = 1U;
                suspend_started_tick = now;
            }
            else if ((uint32_t)(now - suspend_started_tick) >=
                     USB_SUSPEND_RECOVERY_MS)
            {
                recover_requested = 1U;
            }
        }
        else
        {
            suspend_active = 0U;
            if ((uint32_t)(now - link_lost_tick) >= USB_LINK_RECOVERY_MS)
            {
                recover_requested = 1U;
            }
        }

        if (recover_requested &&
            (uint32_t)(now - last_recovery_tick) >= USB_RECOVERY_RETRY_MS)
        {
            last_recovery_tick = now;
            usb_debug_recovery_count++;
            if (suspend_active)
            {
                usb_debug_suspend_trigger_count++;
            }
            usb_device_recover();
            link_lost_tick = HAL_GetTick();
            suspend_active = 0U;
        }

        robot_comm_update();
        osDelay(1);
    }
}
