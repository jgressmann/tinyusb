#!/bin/bash

set -e
set -x

cwd=$PWD

trap "cd $cwd" EXIT

MAKE_ARGS="-j V=1"
readme_file=README.md
script_dir=$(readlink -f $(dirname $0))
projects_dir=$script_dir/..
target_dir_name=firmware
target_dir=$script_dir/$target_dir_name
pre_built_dir=$script_dir/pre-built/$target_dir_name
commit=$(git rev-parse HEAD)

# clean target directory
rm -rf "$target_dir"

# Save current commit
mkdir -p $target_dir/supercan
echo $commit >$target_dir/supercan/COMMIT

###################
# pre-built stuff #
###################
cp -r $pre_built_dir $target_dir/..

VID=0x1d50
#PID_RT=0x5035
PID_DFU=0x5036

############
# D5035-01 #
############
BOOTLOADER_NAME="D5035-01 SuperCAN DFU"
hw_revs=3
export BOARD=d5035_01

# make output dirs for hw revs
for i in $hw_revs; do
	mkdir -p $target_dir/supercan/$BOARD/0$i
done



# SuperDFU
project=superdfu
project_dir=$projects_dir/$project
cd $project_dir


for i in $hw_revs; do
	rm -rf _build
	make $MAKE_ARGS BOARD=$BOARD BOOTLOADER=1 VID=$VID PID=$PID_DFU PRODUCT_NAME="$BOOTLOADER_NAME" INTERFACE_NAME="$BOOTLOADER_NAME" HWREV=$i
	cp _build/$BOARD/${project}.hex $target_dir/supercan/$BOARD/0$i/
	cp _build/$BOARD/${project}.bin $target_dir/supercan/$BOARD/0$i/
	rm -rf _build
	make $MAKE_ARGS BOARD=$BOARD BOOTLOADER=1 VID=$VID PID=$PID_DFU PRODUCT_NAME="$BOOTLOADER_NAME" INTERFACE_NAME="$BOOTLOADER_NAME" HWREV=$i APP=1 dfu
	cp _build/$BOARD/${project}.dfu $target_dir/supercan/$BOARD/0$i/

	# generate J-Link flash script
	cat <<EOF >$target_dir/supercan/$BOARD/0$i/superdfu.jlink
r
loadfile superdfu.hex
r
go
exit
EOF

	# generate README
	cat <<EOF >>$target_dir/supercan/$BOARD/0$i/$readme_file
# SuperCAN Device Firmware

## Content
### Device Bootloader (SuperDFU)

- superdfu.bin: binary, flash with debug probe
- superdfu.hex: hex, flash with debug probe
- superdfu.dfu: update with dfu-util
- superdfu.jlink: J-Link flash script

EOF

done

# SuperCAN
project=supercan
project_dir=$projects_dir/$project
cd $project_dir


for i in $hw_revs; do
	rm -rf _build
	make $MAKE_ARGS HWREV=$i
	cp _build/$BOARD/${project}.hex $target_dir/supercan/$BOARD/0$i/supercan-standalone.hex
	cp _build/$BOARD/${project}.bin $target_dir/supercan/$BOARD/0$i/supercan-standalone.bin
	rm -rf _build
	make $MAKE_ARGS HWREV=$i APP=1 dfu
	cp _build/$BOARD/${project}.superdfu.hex $target_dir/supercan/$BOARD/0$i/supercan-app.hex
	cp _build/$BOARD/${project}.superdfu.bin $target_dir/supercan/$BOARD/0$i/supercan-app.bin
	cp _build/$BOARD/${project}.dfu $target_dir/supercan/$BOARD/0$i/supercan.dfu

	# generate J-Link flash script (standalone)
	cat <<EOF >$target_dir/supercan/$BOARD/0$i/supercan-standalone.jlink
r
loadfile supercan-standalone.hex
r
go
exit
EOF

	# generate J-Link flash script (requires bootloader)
	cat >$target_dir/supercan/$BOARD/0$i/supercan-app.jlink <<EOF
r
loadfile supercan-app.hex
r
go
exit
EOF


	cat <<EOF >>$target_dir/supercan/$BOARD/0$i/$readme_file
### CAN Application (SuperCAN)

- supercan-standalone.bin: binary, no bootloader required, flash with debug probe
- supercan-standalone.hex: hex, no bootloader required, flash with debug probe
- supercan-standalone.jlink: J-Link flash script
- supercan-app.bin: binary, requires bootloader, flash with debug probe to 0x2000
- supercan-app.hex: hex, requires bootloader, flash with debug probe to 0x2000
- supercan-app.jlink: J-Link flash script
- supercan.dfu: requires bootloader, update with dfu-util


## Installation / Update

If you are using the bootloader, update it *first*.
After the bootloader update, you need to re-install the CAN application.

### Device Bootloader

#### Flash with J-Link

\`\`\`bash
JLinkExe -device ATSAME51J18 -if swd -JTAGConf -1,-1 -speed auto -CommandFile superdfu.jlink
\`\`\`

#### Update through DFU

*NOTE: Please do adhere to the update matrix, otherwise you risk bricking your device.*

##### Upgrade Matrix

| from | to |
|:----|:----|
| 0.5.x or before| 0.6.0 |
| 0.6.0 | 6.0.1 or newer |

##### Steps to determine the current bootloader version on the device

If the bootloader is not already running, descend into the bootloader by issuing a DFU detach sequence.

\`\`\`bash
sudo dfu-util -d 1d50:5035,:5036 -e
\`\`\`

The bootloader version is encoded in the USB descriptor's \`bcdDevice\` field. For example

\`\`\`
New USB device found, idVendor=1d50, idProduct=5036, bcdDevice= 6.01
\`\`\`

would mean the device has bootloader version 0.6.1 installed.

##### Bootloader Update

*NOTE: again, please make sure you pick the right file.*

\`\`\`bash
sudo dfu-util -d 1d50:5035,:5036 -R -D superdfu.dfu
\`\`\`



### CAN Application (no Bootloader Required)

#### Flash with J-Link

\`\`\`bash
JLinkExe -device ATSAME51J18 -if swd -JTAGConf -1,-1 -speed auto -CommandFile supercan-standalone.jlink
\`\`\`

### CAN Application (Bootloader Required)

#### Flash with J-Link

\`\`\`bash
JLinkExe -device ATSAME51J18 -if swd -JTAGConf -1,-1 -speed auto -CommandFile supercan-app.jlink
\`\`\`

#### Update with dfu-util

\`\`\`bash
sudo dfu-util -d 1d50:5035,:5036 -R -D supercan.dfu
\`\`\`



EOF

done
unset hw_revs


############
# D5035-05 #
############
hw_revs=1
export BOARD=d5035_05

# make output dirs for hw revs
for i in $hw_revs; do
	mkdir -p $target_dir/supercan/$BOARD/0$i
done


# SuperCAN
project=supercan
project_dir=$projects_dir/$project
cd $project_dir


for i in $hw_revs; do
	rm -rf _build
	make $MAKE_ARGS HWREV=$i APP=1 _build/$BOARD/${project}.hex
	make $MAKE_ARGS HWREV=$i APP=1 _build/$BOARD/${project}.bin
	cp _build/$BOARD/${project}.hex $target_dir/supercan/$BOARD/0$i/supercan-app.hex
	cp _build/$BOARD/${project}.bin $target_dir/supercan/$BOARD/0$i/supercan-app.bin

	# generate J-Link flash script
	cat >$target_dir/supercan/$BOARD/0$i/supercan-app.jlink <<EOF
r
loadfile supercan-app.hex
r
go
exit
EOF


	cat <<EOF >>$target_dir/supercan/$BOARD/0$i/$readme_file
# SuperCAN Device Firmware

## Content

- supercan-app.bin: binary, flash with debug probe to 0x08000000 or through ST DFU
- supercan-app.hex: hex, flash with debug probe to 0x08000000
- supercan-app.jlink: J-Link flash script

## Installation / Update

### Flash with J-Link

\`\`\`bash
JLinkExe -device STM32G0B1CE -if swd -JTAGConf -1,-1 -speed auto -CommandFile supercan-app.jlink
\`\`\`

### Update with dfu-util

NOTE: On Windows you may need to install the device USB driver, e.g. by installing [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html).

\`\`\`bash
sudo dfu-util -e -R -a 0 --dfuse-address 0x08000000 -D supercan-app.bin
\`\`\`

You may need to re-plug the device after flashing to get back to the CAN application.



EOF

done
unset hw_revs



########################
# SAM E54 Xplained Pro #
########################

export BOARD=same54xplainedpro

mkdir -p $target_dir/supercan/$BOARD

rm -rf _build
make $MAKE_ARGS

cp _build/$BOARD/${project}.hex $target_dir/supercan/$BOARD/
cp _build/$BOARD/${project}.bin $target_dir/supercan/$BOARD/


# generate J-Link flash script (standalone)
cat <<EOF >$target_dir/supercan/$BOARD/$readme_file
# SuperCAN Device Firmware

## Content
- supercan.bin: binary, flash with (on-board) debug probe
- supercan.hex: hex, flash with (on-board) debug probe

## Installation

### EDBG

\`\`\`
edbg --verbose -t same54 -pv -f supercan.bin
\`\`\`

EOF


##########################
# Other Boards (bin/hex) #
##########################


boards="stm32h7a3nucleo stm32f303disco teensy_40 d5035_03"
for board in $boards; do
	export BOARD=$board

	mkdir -p $target_dir/supercan/$BOARD

	rm -rf _build
	make $MAKE_ARGS

	cp _build/$BOARD/${project}.hex $target_dir/supercan/$BOARD/
	cp _build/$BOARD/${project}.bin $target_dir/supercan/$BOARD/
done

######################
# Other Boards (uf2) #
######################

boards="feather_m4_can_express"
for board in $boards; do
	export BOARD=$board

	mkdir -p $target_dir/supercan/$BOARD

	rm -rf _build
	make $MAKE_ARGS uf2

	cp _build/$BOARD/${project}.uf2 $target_dir/supercan/$BOARD/
done



##########################
# SAME5X boards with DFU #
##########################


boards=("longan_canbed_m4")
names=("CANBED M4")
for i in "${!boards[@]}"; do
	export BOARD=${boards[$i]}
	export BOOTLOADER_NAME="${names[$i]}"

	mkdir -p $target_dir/supercan/$BOARD

	# SuperDFU
	project=superdfu
	project_dir=$projects_dir/$project
	cd $project_dir


	rm -rf _build
	make $MAKE_ARGS BOARD=$BOARD BOOTLOADER=1 VID=$VID PID=$PID_DFU PRODUCT_NAME="$BOOTLOADER_NAME" INTERFACE_NAME="$BOOTLOADER_NAME"
	cp _build/$BOARD/${project}.hex $target_dir/supercan/$BOARD/
	cp _build/$BOARD/${project}.bin $target_dir/supercan/$BOARD/
	rm -rf _build
	make $MAKE_ARGS BOARD=$BOARD BOOTLOADER=1 VID=$VID PID=$PID_DFU PRODUCT_NAME="$BOOTLOADER_NAME" INTERFACE_NAME="$BOOTLOADER_NAME" APP=1 dfu
	cp _build/$BOARD/${project}.dfu $target_dir/supercan/$BOARD/

	# generate J-Link flash script
	cat <<EOF >$target_dir/supercan/$BOARD/superdfu.jlink
r
loadfile superdfu.hex
r
go
exit
EOF

	# generate README
	cat <<EOF >>$target_dir/supercan/$BOARD/$readme_file
# SuperCAN Device Firmware

## Content
### Device Bootloader (SuperDFU)

- superdfu.bin: binary, flash with debug probe
- superdfu.hex: hex, flash with debug probe
- superdfu.dfu: update with dfu-util
- superdfu.jlink: J-Link flash script

EOF



	# SuperCAN
	project=supercan
	project_dir=$projects_dir/$project
	cd $project_dir

	make $MAKE_ARGS
	cp _build/$BOARD/${project}.hex $target_dir/supercan/$BOARD/supercan-standalone.hex
	cp _build/$BOARD/${project}.bin $target_dir/supercan/$BOARD/supercan-standalone.bin
	rm -rf _build
	make $MAKE_ARGS APP=1 dfu
	cp _build/$BOARD/${project}.superdfu.hex $target_dir/supercan/$BOARD/supercan-app.hex
	cp _build/$BOARD/${project}.superdfu.bin $target_dir/supercan/$BOARD/supercan-app.bin
	cp _build/$BOARD/${project}.dfu $target_dir/supercan/$BOARD/supercan.dfu

	# generate J-Link flash script (standalone)
	cat <<EOF >$target_dir/supercan/$BOARD/supercan-standalone.jlink
r
loadfile supercan-standalone.hex
r
go
exit
EOF

	# generate J-Link flash script (requires bootloader)
	cat >$target_dir/supercan/$BOARD/supercan-app.jlink <<EOF
r
loadfile supercan-app.hex
r
go
exit
EOF


	cat <<EOF >>$target_dir/supercan/$BOARD/$readme_file
### CAN Application (SuperCAN)

- supercan-standalone.bin: binary, no bootloader required, flash with debug probe
- supercan-standalone.hex: hex, no bootloader required, flash with debug probe
- supercan-standalone.jlink: J-Link flash script
- supercan-app.bin: binary, requires bootloader, flash with debug probe to 0x2000
- supercan-app.hex: hex, requires bootloader, flash with debug probe to 0x2000
- supercan-app.jlink: J-Link flash script
- supercan.dfu: requires bootloader, update with dfu-util


## Installation / Update

If you are using the bootloader, update it *first*.
After the bootloader update, you need to re-install the CAN application.

### Device Bootloader

#### Flash with J-Link

\`\`\`bash
JLinkExe -device ATSAME51J18 -if swd -JTAGConf -1,-1 -speed auto -CommandFile superdfu.jlink
\`\`\`

#### Update through DFU

*NOTE: Please do adhere to the update matrix, otherwise you risk bricking your device.*

##### Upgrade Matrix

| from | to |
|:----|:----|
| 0.5.x or before| 0.6.0 |
| 0.6.0 | 6.0.1 or newer |

##### Steps to determine the current bootloader version on the device

If the bootloader is not already running, descend into the bootloader by issuing a DFU detach sequence.

\`\`\`bash
sudo dfu-util -d 1d50:5035,:5036 -e
\`\`\`

The bootloader version is encoded in the USB descriptor's \`bcdDevice\` field. For example

\`\`\`
New USB device found, idVendor=1d50, idProduct=5036, bcdDevice= 6.01
\`\`\`

would mean the device has bootloader version 0.6.1 installed.

##### Bootloader Update

*NOTE: again, please make sure you pick the right file.*

\`\`\`bash
sudo dfu-util -d 1d50:5035,:5036 -R -D superdfu.dfu
\`\`\`



### CAN Application (no Bootloader Required)

#### Flash with J-Link

\`\`\`bash
JLinkExe -device ATSAME51J18 -if swd -JTAGConf -1,-1 -speed auto -CommandFile supercan-standalone.jlink
\`\`\`

### CAN Application (Bootloader Required)

#### Flash with J-Link

\`\`\`bash
JLinkExe -device ATSAME51J18 -if swd -JTAGConf -1,-1 -speed auto -CommandFile supercan-app.jlink
\`\`\`

#### Update with dfu-util

\`\`\`bash
sudo dfu-util -d 1d50:5035,:5036 -R -D supercan.dfu
\`\`\`
EOF

done



# archive
cd $target_dir && (tar c supercan | pixz -9 >supercan-firmware.tar.xz)

echo A-OK

