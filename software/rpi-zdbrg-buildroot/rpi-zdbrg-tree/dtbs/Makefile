OVERLAYS = uart-ctsrts.dtbo

DTC ?= dtc

%.dtbo: %.dts
	$(DTC) -@ -H epapr -O dtb -o $@ -Wno-unit_address_vs_reg $^

.phony: all
all: $(OVERLAYS)

.phony: clean
clean:
	rm -f *.dtbo

