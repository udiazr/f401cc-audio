# f401cc-audio
BlackPill STM32f401cc USB Audio Class Delta-Sigma Digital Audio Amplifier

An fully digital USB Audio class power audio amplifier.
48khz Sample rate 14 OSR (672khz PWM)
2-order delta sigma modulation.

Flash:
Download f401cc.bin then flash it to your device.

Plug it to your PC and a USB STM32f401cc "Virtual COM" sound card  will appear.

Connect the PWM outputs to your halfbridges.



PA8 Left Channel PWM 
PB14 Right Channel PWM (inverted)
PA10 Subwoofer PWM
PB15 Subwoofer PWM (inverted)

+-------------------STM32F401CC--------------------+
| +-----------+   +---------------+    +---------+ |  +-----------------+   +-----------+  +----------+
| |-USB INPUT |---+  Sigma-Delta  |----|   PWM   |--->| HALF/FULL BRIDGE|---| LC FILTER |--| SPEAKERS |
| +-----------+   +---------------+    +---------+ |  +-----------------+   +-----------+  +----------+
+--------------------------------------------------+




How to use:
You will need at least 3 fast half bridges.

I use 2 dual half bridge IXYS IXDD604PI.  
1 Half bridge left channel
1 half bridge right channel
2 half bridge Subwoofer  (full bridge)

Maybe you can use it with irs2011 with a pair of transistors to get much more power.



Build:
Create a USB Audio Class device with STM32CubeMX
Copy usbd_audio_if.c to your USB_DEVICE/App/ folder
Funciones.S have the routines


The blackpill clock is a bit slower than 48khz so the buffer gets full from time to time.
Then I increase the PWM frequency about 1% to avoid overflow.
This introduces a 1% off tune distortion time from time.
It is barely audible unless you have a good pitch.

TODO:
I need a start routine to avoid noises at start.

