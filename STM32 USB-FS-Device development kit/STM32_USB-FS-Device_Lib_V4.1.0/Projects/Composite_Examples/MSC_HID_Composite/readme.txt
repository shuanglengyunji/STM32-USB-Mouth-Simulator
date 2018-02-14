/**
  ******************** (C) COPYRIGHT 2017 STMicroelectronics *******************
  * @file    readme.txt
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Description of the Composite Example Demo.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


Example description
===================
This Demo provides a description of how to use the USB-FS-Device on the STM32F10xxx, 
STM32F37xxx, STM32F30xxx and STM32L15xxx devices.
The examples provide a composite device
A composite device is one that has multiple interfaces controlled independently of each other.
Using composite device, multiple functions are combined into a single device.
In this Example the independent interfaces are : Mass Storage (MSC) and Custom HID

8MHZ Cristal should be mounted on boards to recognize USB device.

LED1,LED2 and LED3 are reserved for Custom HID demo
LED4 reserved for MSC, it is ON when MicroSD is Ready for write and read operations.

When even one of the interface class of the device is changed, it should be handled
differently by Windows. However, it doesn't recognize the modification. To avoid conflict on
Windows, we suggest to assign another VID/PID to the device (idProduct = 0x5750) or
delete device instance from device manager.

More details about this Demo implementation is given in the User manual 
"UM0424 STM32F10xxx USB development kit", available for download from the ST
microcontrollers website: www.st.com/stm32


Directory contents
==================
 + \inc: contains the Demo firmware header files
 + \EWARM: contains preconfigured projects for EWARM toolchain
 + \MDK-ARM: contains preconfigured projects for MDK-ARM toolchain
 + \SW4STM32: contains preconfigured projects for STM32 System Workbench for toolchain          
 + \src: contains the Demo firmware source files


Hardware environment
====================
This example runs these STMicroelectronics evaluation boards and can be easily tailored to any other hardware: 

- STM3210B-EVAL
- STM3210E-EVAL
- STM32L152-EVAL
- STM32F373C_EVAL
- STM32F303C_EVAL
- STM32F303E_EVAL
- STM32L152D-EVAL

- STM3210B-EVAL Set-up 
    - Jumper JP1 (USB disconnect) should be connected in position 2-3.

- STM3210E-EVAL Set-up 
    - Jumper JP14 (USB disconnect) should be connected in position 2-3. 

- STM32L152-EVAL Set-up 
    - Jumpers JP18 and JP19 should be connected. 
    - Jumper JP17 should be connected in position 5-6 (PB12 connector)
    - Note that Joystick Right and Left buttons are used for this Demo.
	 
- STM32L152D-EVAL Set-up 
	- LCD Glass should be mounted On IO position for Potentiometer usage.
	- JP13 should be connected to ADC input. 
  
How to use it
=============

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - In the workspace toolbar select the project configuration:
        - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
        - STM3210E-EVAL: to configure the project for STM32 High-density devices
        - STM3210E-EVAL_XL: to configure the project for STM32 XL-density devices
        - STM32L152-EVAL: to configure the project for STM32 Medium-Density Low-Power devices
        - STM32L152D-EVAL: to configure the project for STM32 High-Density Low-Power devices
        - STM32373C-EVAL:  to configure the project for STM32F37xxx devices
        - STM32303C-EVAL:  to configure the project for STM32F303xx devices.
        - STM32303E-EVAL:  to configure the project for STM32F303xE devices
 - Run the application
 
************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE******
