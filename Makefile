obj-m+=rtc-rx8731.o

KERN_DIR=/lib/modules/`uname -r`/build/

all:
	make -C $(KERN_DIR) M=`pwd` modules
clean:
	make -C $(KERN_DIR) M=`pwd` clean
