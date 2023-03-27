#
#  Makefile for the Linux Controller Area Network SPI drivers.
#
PWD	:= $(shell pwd)

default:
	@echo Making generic AP art module build. This build target is used for 3rd party AP processor. 
	make ARCH=$(LINUX_KARCH) CROSS_COMPILE=$(TARGET_CROSS) -C $(LINUX_KARCH) M=$(PWD)/modules modules

obj-$(CONFIG_CAN_MCP25XXFD)	+= mcp25xxfd.o
mcp25xxfd-objs                  := mcp25xxfd_base.o
mcp25xxfd-objs                  += mcp25xxfd_can.o
mcp25xxfd-objs                  += mcp25xxfd_can_debugfs.o
mcp25xxfd-objs                  += mcp25xxfd_can_fifo.o
mcp25xxfd-objs                  += mcp25xxfd_can_int.o
mcp25xxfd-objs                  += mcp25xxfd_can_rx.o
mcp25xxfd-objs                  += mcp25xxfd_can_tx.o
mcp25xxfd-objs                  += mcp25xxfd_clock.o
mcp25xxfd-objs                  += mcp25xxfd_cmd.o
mcp25xxfd-objs                  += mcp25xxfd_crc.o
mcp25xxfd-objs                  += mcp25xxfd_debugfs.o
mcp25xxfd-objs                  += mcp25xxfd_ecc.o
mcp25xxfd-objs                  += mcp25xxfd_gpio.o
mcp25xxfd-objs                  += mcp25xxfd_int.o

clean:
	rm -rf *.o *.ko .*.cmd *mod.c .tmp_versions