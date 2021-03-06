IFACEPATH := interface
IFACEDEPS := $(IFACEPATH)/numachip-defines.h $(IFACEPATH)/numachip-autodefs.h
UCODEDEPS := $(IFACEPATH)/numachip-mseq.h
CFLAGS    := -I$(IFACEPATH)
COPT      := -g -Wall -Wextra -Wno-unused-parameter -Wshadow -fno-inline -O3

syslinux_version := 4.07
syslinux_dir     := syslinux-$(syslinux_version)

mjson_version    := 1.7.0
mjson_dir        := json-$(mjson_version)

COM32DEPS := $(syslinux_dir)/com32/libutil/libutil_com.a $(syslinux_dir)/com32/lib/libcom32.a

.PHONY: all
all: dnc-bootloader.c32 test-routing gen-ucode

.PRECIOUS: %.bz2 %.tar.gz

.PHONY: upload
upload: dnc-bootloader.c32
	rsync -z dnc-bootloader.c32 build:/net/numastore/tftpboot/dnc-bootloader-$(USER).c32

.PHONY: publish
publish: dnc-bootloader.c32
	rsync -z dnc-bootloader.c32 resources@resources:resources/bootloader/validating/dnc-bootloader-$(shell git describe).c32

.PHONY: check
check:
	cppcheck -q --enable=all --inconclusive *.h *.c

.PHONY: clean
clean:
	rm -f *~ *.o *.c32 *.elf .*.o.d *.orig *.aml *.amx *.dsl test-masternode test-slavenode test-routing test-aml test-vector gen-ucode dnc-version.h

.PHONY: realclean
realclean: clean
	rm -rf $(mjson_dir) mjson-$(mjson_version).tar.gz
	rm -rf $(syslinux_dir) syslinux-$(syslinux_version).tar.bz2

syslinux-%:
	wget -O $@.tar.bz2 http://www.kernel.org/pub/linux/utils/boot/syslinux/4.xx/$@.tar.bz2 || rm -f $@.tar.bz2

json-%:
	wget -O $@.tar.gz http://sourceforge.net/projects/mjson/files/latest/download?source=files || rm -f $@.tar.gz

$(syslinux_dir)/com32/samples/Makefile: syslinux-$(syslinux_version)
	tar -jxf $<.tar.bz2
	touch -c $(syslinux_dir)/com32/samples/Makefile
	(cd $(syslinux_dir) && make all-local)

# Needed for syslinux 4
$(syslinux_dir)/com32/tools/relocs: $(syslinux_dir)/com32/samples/Makefile
	(cd $(syslinux_dir)/com32/tools && make all)

$(syslinux_dir)/com32/libutil/libutil_com.a: $(syslinux_dir)/com32/samples/Makefile $(syslinux_dir)/com32/tools/relocs
	(cd $(syslinux_dir)/com32/libutil && make all)

$(syslinux_dir)/com32/lib/libcom32.a: $(syslinux_dir)/com32/samples/Makefile $(syslinux_dir)/com32/tools/relocs
	(cd $(syslinux_dir)/com32/lib && make all)

$(mjson_dir)/src/json.h \
$(mjson_dir)/src/json.c: json-$(mjson_version)
	echo $@
	tar -zxf $<.tar.gz
	touch -c $(mjson_dir)/src/json.h
	perl -npi -e 's/#include <memory.h>/#include <string.h>/' $(mjson_dir)/src/json.c
	perl -npi -e 's/SIZE_MAX/10485760/' $(mjson_dir)/src/json.h

%.o: %.c $(syslinux_dir)/com32/samples/Makefile
	(rm -f $@ && cd $(syslinux_dir)/com32/samples && make CC="g++ -fno-rtti -fpermissive -Wall -Wextra -ftrapv" $(CURDIR)/$@ NOGPL=1)

%.o: %.S $(syslinux_dir)/com32/samples/Makefile
	(rm -f $@ && cd $(syslinux_dir)/com32/samples && make $(CURDIR)/$@ NOGPL=1)

%.elf: %.o
	(rm -f $@ && \
	cd $(syslinux_dir)/com32/samples && \
	cmd=$$(make -s -n $(CURDIR)/$@ NOGPL=1) && \
	cmd="$$cmd $(patsubst %, $(CURDIR)/%, $(wordlist 2, $(words $^), $^))" && \
	echo $$cmd && \
	$$cmd)

%.c32: %.elf $(syslinux_dir)/com32/samples/Makefile
	(cd $(syslinux_dir)/com32/samples && make $(CURDIR)/$@ NOGPL=1)

$(mjson_dir)/src/json.o: $(mjson_dir)/src/json.c

dnc-version.h: dnc-access.h dnc-commonlib.h dnc-devices.h dnc-mmio.h dnc-maps.h dnc-route.h dnc-acpi.h dnc-config.h dnc-fabric.h dnc-monitor.h dnc-escrow.h dnc-trace.h dnc-bootloader.h dnc-defs.h dnc-masterlib.h dnc-regs.h ddr_spd.h dnc-access.c dnc-commonlib.c dnc-fabric.c dnc-monitor.c dnc-escrow.c dnc-trace.c test-slavenode.c dnc-acpi.c dnc-config.c dnc-masterlib.c dnc-route.c gen-ucode.c test-masternode.c dnc-bootloader.c dnc-dimmtest.c dnc-devices.c dnc-mmio.c dnc-test-access.c test-routing.c Makefile
	@echo \#define VER \"`git describe --always --dirty=+`\" >dnc-version.h

dnc-bootloader.elf: dnc-bootloader.o dnc-commonlib.o dnc-dimmtest.o dnc-devices.o dnc-monitor.o dnc-escrow.o dnc-trace.o dnc-masterlib.o dnc-mmio.o dnc-maps.o \
	dnc-fabric.o dnc-access.o dnc-route.o dnc-acpi.o dnc-aml.o dnc-config.o dnc-e820-handler.o ddr_spd.o \
	$(mjson_dir)/src/json.o $(COM32DEPS)

dnc-bootloader.o: dnc-bootloader.c dnc-bootloader.h $(IFACEDEPS) dnc-regs.h \
	dnc-fabric.h dnc-access.h dnc-route.h dnc-acpi.h dnc-aml.h dnc-config.h dnc-version.h \
	dnc-commonlib.h dnc-devices.h dnc-monitor.h dnc-escrow.h dnc-trace.h dnc-masterlib.h dnc-mmio.h dnc-maps.h

dnc-e820-handler.o: dnc-defs.h

dnc-commonlib.o: dnc-commonlib.c dnc-commonlib.h dnc-bootloader.h dnc-devices.h dnc-monitor.h dnc-escrow.h dnc-trace.h dnc-access.h $(IFACEPATH)/regconfig_200_cl4_bl4_genericrdimm.h $(IFACEPATH)/mctr_define_register_C.h ddr_spd.h

ddr_spd.o: ddr_spd.c dnc-commonlib.h ddr_spd.h

dnc-dimmtest.o: dnc-dimmtest.c dnc-commonlib.h dnc-access.h $(IFACEPATH)/mctr_define_register_C.h

dnc-config.o: dnc-config.c dnc-config.h $(mjson_dir)/src/json.h

dnc-masterlib.o: dnc-masterlib.c $(UCODEDEPS) dnc-commonlib.h dnc-devices.h dnc-monitor.h dnc-escrow.h dnc-trace.h dnc-masterlib.h dnc-mmio.h dnc-maps.h dnc-access.h

dnc-mmio.o: dnc-mmio.c dnc-mmio.h

dnc-maps.o: dnc-maps.c dnc-maps.h

dnc-trace.o: dnc-trace.c dnc-trace.h

dnc-escrow.o: dnc-escrow.c dnc-escrow.h

dnc-fabric.o: dnc-fabric.c dnc-fabric.h

dnc-access.o: dnc-access.c dnc-access.h

dnc-route.o: dnc-route.c dnc-route.h

dnc-acpi.o: dnc-acpi.c dnc-acpi.h

dnc-aml.o: dnc-aml.c dnc-aml.h

test-masternode: test-masternode.o dnc-test-commonlib.o dnc-test-masterlib.o dnc-test-mmio.o \
	dnc-test-fabric.o dnc-test-access.o dnc-test-route.o dnc-test-config.o \
	test-json.o
	$(CXX) $(COPT) $^ -o $@

test-slavenode: test-slavenode.o dnc-test-commonlib.o dnc-test-fabric.o \
	dnc-test-access.o dnc-test-route.o dnc-test-config.o test-json.o
	$(CXX) $(COPT) $^ -o $@

test-routing: test-routing.o dnc-test-route.o dnc-test-access.o
	$(CXX) $(COPT) $^ -o $@

test-aml: test-aml.o dnc-aml.c
	$(CXX) $(COPT) $^ -o $@

test-aml.o: test-aml.c dnc-aml.c dnc-aml.h
	$(CXX) $(COPT) -c $< -o $@

test-vector: test-vector.o
	$(CXX) $(COPT) $^ -o $@

test-vector.o: test-vector.c dnc-types.h
	$(CXX) $(COPT) -c $< -o $@

gen-ucode: gen-ucode.c
	$(CXX) $(COPT) $^ -o $@

test-masternode.o: test-masternode.c $(IFACEDEPS) dnc-commonlib.h dnc-devices.h dnc-monitor.h dnc-escrow.h dnc-trace.h \
	dnc-masterlib.h dnc-mmio.h dnc-maps.h dnc-fabric.h dnc-regs.h dnc-access.h \
	dnc-route.h dnc-config.h
	$(CXX) $(COPT) -c $< -o $@

test-slavenode.o: test-slavenode.c $(IFACEDEPS) dnc-commonlib.h dnc-devices.h dnc-monitor.h dnc-escrow.h dnc-trace.h \
	dnc-fabric.h dnc-regs.h dnc-access.h \
	dnc-route.h dnc-config.h
	$(CXX) $(COPT) -c $< -o $@

test-routing.o: test-routing.c $(IFACEDEPS) dnc-commonlib.h dnc-devices.h dnc-monitor.h dnc-escrow.h dnc-trace.h dnc-route.h dnc-route.c \
	dnc-regs.h dnc-access.h dnc-fabric.h
	$(CXX) $(COPT) -c $< -o $@

test-json.o: $(mjson_dir)/src/json.c
	$(CXX) $(COPT) -c $< -o $@

dnc-test-commonlib.o: dnc-commonlib.c dnc-commonlib.h dnc-devices.h dnc-monitor.h dnc-escrow.h dnc-trace.h $(IFACEPATH)/regconfig_200_cl4_bl4_genericrdimm.h
	$(CXX) $(COPT) -c $< -o $@

dnc-test-masterlib.o: dnc-masterlib.c $(UCODEDEPS) dnc-commonlib.h dnc-devices.h dnc-monitor.h dnc-escrow.h dnc-trace.h dnc-masterlib.h dnc-mmio.h dnc-maps.h dnc-access.h
	$(CXX) $(COPT) -c $< -o $@

dnc-test-mmio.o: dnc-mmio.c $(UCODEDEPS) dnc-mmio.h
	$(CXX) $(COPT) -c $< -o $@

dnc-test-fabric.o: dnc-fabric.c dnc-fabric.h
	$(CXX) $(COPT) -c $< -o $@

dnc-test-access.o: dnc-test-access.c dnc-access.h
	$(CXX) $(COPT) -c $< -o $@

dnc-test-route.o: dnc-route.c dnc-access.h dnc-route.h
	$(CXX) $(COPT) -c $< -o $@

dnc-test-config.o: dnc-config.c dnc-config.h
	$(CXX) $(COPT) -c $< -o $@

messages: messages.c
	$(CXX) $(COPT) $< -o $@

run-tests: test-aml
	valgrind -q --track-origins=yes ./test-aml

$(IFACEDEPS):
	cd $(IFACEPATH) && make $(notdir $@)
