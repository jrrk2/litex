include ../include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

OBJECTS = spieth.o

all: liblitespieth.a

liblitespieth.a: $(OBJECTS)
	$(AR) crs liblitespieth.a $(OBJECTS)

# pull in dependency info for *existing* .o files
-include $(OBJECTS:.o=.d)

%.o: $(SOC_DIRECTORY)/software/liblitespieth/%.c
	$(compile)

.PHONY: all clean

clean:
	$(RM) $(OBJECTS) liblitespieth.a .*~ *~
