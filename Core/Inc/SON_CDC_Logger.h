/*
 * SON_I2C_MPU6050.h
 *
 *  Created on: Jan 7, 2025
 *      Author: Thai-Son Nguyen
 */

/**
 * @brief Log utility for STM32 built on top of STM32 CDC (Communication Device Class - Virtual Port Com)
 *
 * Thư viện này được build dựa trên chế độ CDC (Communication Device Class - Virtual Port Com) của STM32
 *
 * Dùng để gửi message qua cổng COM máy tính.
 *
 * Hoạt động giống như Printf hay ESP_LOG bên ESP-IDF.
 *
 * ### Điều kiện tiên quyết:
 * Phải setup STM32 ở chế độ USB -> USB Device -> CDC (Communication Device Class - Virtual Port Com) trước.
 *
 * Xem hướng dẫn setup tại `https://tapit.vn/usb-cdc-stm32f103/`.
 *
 * ### Các hàm
 *
 * Xem các hàm và chức năng của các hàm ở phần **INTERFACE** .
 *
 * Không cần quan tâm phần **IMPLEMENTATION** .
 *
 * */

#ifndef _SON_CDC_Logger
#define _SON_CDC_Logger

#ifdef __cplusplus
 extern "C" {
#endif




// ***************** INTERFACE *******************



// ************** INCLUDE ***************

#include "usbd_cdc_if.h"

// ************** DEFINE ****************

/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE {
    while (CDC_Transmit_FS((uint8_t *)&ch, 1) == USBD_BUSY);
    return ch;
}


/* USER CODE END 4 */

// **************** IMPLEMENTATION *****************




#ifdef __cplusplus
}
#endif

#endif /*  _SON_CDC_Logger */
