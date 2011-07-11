APPS :=  ${sort ${dir ${wildcard Apps/*/}}}

update:
	@set -e; for d in $(APPS); do $(MAKE) -C $$d update; done

all:
	@set -e; for d in $(APPS); do $(MAKE) -C $$d all; done
	
rebuild:
	@set -e; for d in $(APPS); do $(MAKE) -C $$d rebuild; done
	
gdbproxy:
	@echo "Starting msp430-gdbproxy"
	@echo ""
	msp430-gdbproxy --port=3333 msp430 TIUSB
	
reset:
	@echo "Resetting the MSP430..."
	@echo ""
	msp430-jtag -r -lTIUSB
	
clearinfo:
	msp430-jtag -l TIUSB --erase=0x1000-0x1100
		
alldocs:
	set -e; for d in $(APPS); do cd $$d && doxygen && cd ../..; done

		