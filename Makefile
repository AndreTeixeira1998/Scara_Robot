CC = avr-gcc
CFLAGS = -Os -DF_CPU=16000000UL -mmcu=atmega328p
OBJDIR = build
SRCDIR = src
INCLUDEDIR = include
vpath %.c $(SRCDIR)
vpath %.h $(INCLUDEDIR)
BINNAME = main
OBJS = $(addprefix $(OBJDIR)/, uart.o)

default: main clean

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	@mkdir -p $@

main: $(OBJS) $(OBJDIR)/main.o -lm
	$(CC) $(CFLAGS) $(OBJS) $(OBJDIR)/main.o -o $(OBJDIR)/$(BINNAME) -lm
	avr-objcopy -O ihex -R .eeprom $(OBJDIR)/$(BINNAME) $(BINNAME).hex 

upload:
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyACM0 -b 115200 -U flash:w:$(BINNAME).hex

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) -I $(INCLUDEDIR) -c -o $@ $< -lm

clean:
	-rm -rf $(OBJDIR)

.force:
