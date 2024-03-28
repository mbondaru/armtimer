ksrc = "/home/pi/Desktop/linux"
obj-m = armtimer.o
armtimer-objs = fiqhandler.o timer-sp804.o

all:
	make -C $(ksrc) M=$(PWD) modules #EXTRA_CFLAGS='-v'

modules_install:
	make -C $(ksrc) M=$(PWD) modules_install

clean:
	make -C $(ksrc) M=$(PWD) clean
