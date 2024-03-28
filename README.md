# armtimer

Raspberry Pi 3 Model B ARM Timer FIQ module (Linux RaspberryPi 4.19.71-rt24-v7+ SMP PREEMPT RT)

In order to get the FIQ exception to work with the ARM Timer (broadcom spec pg. 196 "Timer (ARM side)"), do the following:
## Warning: this will require us to build a custom kernel driver (module) for a custom compiled real-time Linux kernel, so no stdio.h, printf, not even an mmap() will be used throughout the source code. We are using linux source files and headers for low-level memory access in order to inject the FIQ service routine that we write in ARM assembly language into the low-addresses (0x00000000) of Physical memory (RAM). Linux GUI interface will most likely freeze at sufficiently high FIQ interrupt frequency but the FIQ will be doing its job on the bare-metal peripherals and GPIO.

1) Download the rpi-4.19y-rt kernel source files (git clone)
   
2) In the arch/arm/kernel/fiq.c change the following line:
<pre>
...
void set_fiq_handler(void *start, unsigned int length)
{
         void *base = vectors_page;
         unsigned offset = FIQ_OFFSET;
 
         memcpy(base + offset, start, length);
         if (!cache_is_vipt_nonaliasing())
                 flush_icache_range((unsigned long)base + offset, offset +
                                    length);
         <b>flush_icache_range(0xffff0000 + offset, 0xffff0000 + offset + length);</b>
 }
...
</pre>
to
```
flush_icache_range(0x00000000 + offset, 0x00000000 + offset + length);
```
And this should write the FIQ routine into the correct place in the Vector table when you install the linux module using the `sudo insmod armtimer.ko` command.

3) In the arch/arm/Kconfig add the following line:

<pre>
...
config FIQ
         bool
         <b>default y</b>
...
</pre>

4) Run the 'make bcm2709_defconfig' command or other default config if you are doing this on a Raspberry Pi board other than Raspberry Pi 3 Model B Rev 1.2.
   
5) Make sure that in the linux directory the newly created .config file contains the following line:
   
```
...
CONFIG_FIQ=y
...
```

6) Compile the 4.19 real-time Linux kernel by running the following commands:

```
KERNEL=kernel7
make zImage modules dtbs
sudo make modules_install
sudo cp arch/arm/boot/dts/*.dtb /boot/
sudo cp arch/arm/boot/dts/overlays/*.dtb* /boot/overlays/
sudo cp arch/arm/boot/zImage /boot/$KERNEL.img
sudo reboot
```

7) Once the board has restarted, verify that your new kernel version is 4.19 rt by running the `uname -a` command.
   
8) In the directory that contains armtimer source files (timer-sp804.c, rpi-pg-unaligned.h, fiqhandler.S, Makefile) run the following commands:
   
```
make
sudo insmod armtimer.ko
```

If there is no errors reported, type `lsmod` command to see the newly installed module, then type `dmesg` command to see printk() debug messages printed during the module initialization. This must be done after every board restart for the module to be working. To disable the module run the command:

```
sudo rmmod armtimer.ko
```

9) Append the following two settings to the end of the /boot/cmdline.txt
    
```
dwc_otg.fiq_fsm_enable=0 dwc_otg.fiq_enable=0
```

in order to prevent the USB driver (the way it is coded) from claiming the FIQ early during start-up (Only one interrupt source can be wired to the FIQ, it is the highest priority interrupt).

#Reboot again for these changes to take place

10) Overclocking is useful to speed up the FIQ handling between high-frequency timer firings. For more predictable delays of the FIQ service routine, overclock the board by adding the following two settings in the /boot/config.txt:
    
```
force_turbo=1
core_freq=472
```

This makes Raspberry Pi 3 Model B board arm (CPU) clock run at 1200MHz and core clock (the one used by ARM Timer) run at 472MHz. You can confirm the clock frequencies that your CPU runs at by running the commands `vcgencmd measure_clock arm` and `vcgencmd measure_clock core`. Core clock is the VideoCore IV proprietary co-processor clock, which also drives peripherals like the ARM Timer. Also, force_turbo option, when enabled, disables the ARM CPU clock frequency throttling based on CPU chip temperature. Avoid running all 4 cores at 100% with this setting (you can check by running the `htop` command) as it will very quickly melt the CPU chip. You can check the current CPU chip temperature of the Raspberry Pi 3 Model B by running the command `vcgencmd measure_temp`.

