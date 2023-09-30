# f401cc-audio 2.1 Blackpill Class-D audio amplifier
[![YoutubeVideo](https://img.youtube.com/vi/Q_hg8kxJaRE/0.jpg)](https://www.youtube.com/watch?v=Q_hg8kxJaRE)

BlackPill STM32f401cc 2.1 USB Audio Class Delta-Sigma Digital Audio Amplifier

A fully digital USB Audio class power audio amplifier.
48khz Sample rate 10 OSR (480khz PWM) 2-order delta sigma modulation.
This amplifier overclocks your STM32f401 to 120MHz. 
I think it sounds cleaner now that most China 30USD 'hifi'  alternatives.
Maybe you need to reset the device to work.

## Flash:
- Download f401-audio.bin then flash it to your device.
- Plug it to your PC and a USB STM32f401cc "Virtual COM" sound card  will appear.
- Connect the PWM outputs to your halfbridges.
- 
The complementary outputs are:
- PA8/PB13 Left Channel 
- PA9/PB14 Right Channel 
- PA10/PB15 Subwoofer PWM



### DIAGRAM:
USB-INPUT -> SIGMA-DELTA-> PWM-> HALF/FULL BRIDGE-> LC-FILTER->SPEAKERS





## How to use:
You will need at least 3 fast half bridges.

- I use 2 dual half bridge IXYS IXDD604PI.  
- 1 Half bridge left channel
- 1 half bridge right channel
- 2 half bridge Subwoofer  (full bridge)

Maybe you can use it with irs2011 with a pair of transistors to get much more power.



## Build:
- Load f401-audio.ioc project to STM32CubeMX and generate all source files.
- Copy funciones.S and Makefile to your project folder
- Copy usbd_audio_if.c to your USB_DEVICE/App/ folder
- Copy usbd_audio.h to Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Inc/
- Compile and get fun.

The blackpill clock is a bit slower than 48khz so the buffer gets full from time to time.
Then I increase the PWM frequency about 0.5% to avoid overflow.
This introduces a 0.5% off tune distortion time from time.
It is barely audible unless you have a good pitch.

### TODO:

Utilize two Timers to avoid crosstalk between the channel (tracks are too close)
I need a start/unplug to avoid noises.
Sometimes produces noises, reset it.


UDIAZR
