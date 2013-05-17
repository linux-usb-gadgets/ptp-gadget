CPPFLAGS	:= -Wall -I$(KERNEL_SRC)/include

ptp:		ptp.o
	$(CROSS_COMPILE)gcc -lpthread -o $@ $^

ptp.o:		ptp.c
	$(CROSS_COMPILE)gcc $(CPPFLAGS) -c -o $@ $<

all:		ptp

clean:
	rm -f ptp ptp.o

install:	ptp
	install -m 0755 -t $(DESTDIR)/usr/local/bin/ ptp

.PHONY:		all clean install
