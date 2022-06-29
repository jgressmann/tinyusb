#!/bin/sh

set -e
# set -x

cwd=$PWD

trap "cd $cwd" EXIT



MAKE_ARGS="-j V=1"
VID=0x1d50
PID_RT=0x5037
PID_DFU=0x5038

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
mkdir -p $target_dir/sllin
echo $commit >$target_dir/sllin/COMMIT

###################
# SuperDFU Boards #
###################

superdfu_build()
{
	local board_id=$1
	local board_name=$2
	local hw_rev=$3

	shift
	shift
	shift


	local hw_rev_02=$(printf "%02u" $hw_rev)
	local bootloader_name="$board_name slLIN DFU"
	export BOARD=$board_id

	# make output dirs for hw revs
	mkdir -p $target_dir/sllin/$BOARD/$hw_rev_02



	# SuperDFU
	local project=atsame51_dfu
	local project_dir=$projects_dir/$project
	cd $project_dir


	rm -rf _build
	make $MAKE_ARGS BOARD=$BOARD BOOTLOADER=1 VID=$VID PID=$PID_DFU PRODUCT_NAME="$bootloader_name" INTERFACE_NAME="$bootloader_name" HWREV=$hw_rev
	cp _build/$BOARD/${project}.hex $target_dir/sllin/$BOARD/$hw_rev_02/superdfu.hex
	cp _build/$BOARD/${project}.bin $target_dir/sllin/$BOARD/$hw_rev_02/superdfu.bin
	rm -rf _build
	make $MAKE_ARGS BOARD=$BOARD BOOTLOADER=1 VID=$VID PID=$PID_DFU PRODUCT_NAME="$bootloader_name" INTERFACE_NAME="$bootloader_name" HWREV=$hw_rev APP=1 dfu
	cp _build/$BOARD/${project}.dfu $target_dir/sllin/$BOARD/$hw_rev_02/superdfu.dfu

	# generate J-Link flash script
	cat <<EOF >$target_dir/sllin/$BOARD/$hw_rev_02/superdfu.jlink
r
loadfile superdfu.hex
r
go
exit
EOF

	# generate README
	cat <<EOF >>$target_dir/sllin/$BOARD/$hw_rev_02/$readme_file
# slLIN Device Firmware

## Content
### Device Bootloader (SuperDFU)

- superdfu.bin: binary, flash with debug probe
- superdfu.hex: hex, flash with debug probe
- superdfu.dfu: update with dfu-util
- superdfu.jlink: J-Link flash script

EOF



	# sllin
	project=sllin
	project_dir=$projects_dir/$project
	cd $project_dir


	rm -rf _build
	make $MAKE_ARGS HWREV=$hw_rev
	cp _build/$BOARD/${project}.hex $target_dir/sllin/$BOARD/$hw_rev_02/sllin-standalone.hex
	cp _build/$BOARD/${project}.bin $target_dir/sllin/$BOARD/$hw_rev_02/sllin-standalone.bin
	rm -rf _build
	make $MAKE_ARGS HWREV=$hw_rev APP=1 dfu
	cp _build/$BOARD/${project}.superdfu.hex $target_dir/sllin/$BOARD/$hw_rev_02/sllin-app.hex
	cp _build/$BOARD/${project}.superdfu.bin $target_dir/sllin/$BOARD/$hw_rev_02/sllin-app.bin
	cp _build/$BOARD/${project}.dfu $target_dir/sllin/$BOARD/$hw_rev_02/sllin.dfu


	# generate J-Link flash script (standalone)
	cat <<EOF >$target_dir/sllin/$BOARD/$hw_rev_02/sllin-standalone.jlink
r
loadfile sllin-standalone.hex
r
go
exit
EOF

	# generate J-Link flash script (requires bootloader)
	cat >$target_dir/sllin/$BOARD/$hw_rev_02/sllin-app.jlink <<EOF
r
loadfile sllin-app.hex
r
go
exit
EOF

cat <<EOF >>$target_dir/sllin/$BOARD/$hw_rev_02/$readme_file
### LIN Application (slLIN)

- sllin-standalone.bin: binary, no bootloader required, flash with debug probe
- sllin-standalone.hex: hex, no bootloader required, flash with debug probe
- sllin-standalone.jlink: J-Link flash script
- sllin-app.bin: binary, requires bootloader, flash with debug probe to 0x4000
- sllin-app.hex: hex, requires bootloader, flash with debug probe to 0x4000
- sllin-app.jlink: J-Link flash script
- sllin.dfu: requires bootloader, update with dfu-util


## Installation / Update

If you are using the bootloader, update it *first*.
After the bootloader update, you need to re-install the LIN application.

### Device Bootloader

#### Flash with J-Link

\`\`\`bash
JLinkExe -device ATSAME51J18 -if swd -JTAGConf -1,-1 -speed auto -CommandFile superdfu.jlink
\`\`\`

#### Update through DFU


\`\`\`bash
sudo dfu-util -d 1d50:5037,:5038 -R -D superdfu.dfu
\`\`\`


### LIN Application (no Bootloader Required)

#### Flash with J-Link

\`\`\`bash
JLinkExe -device ATSAME51J18 -if swd -JTAGConf -1,-1 -speed auto -CommandFile sllin-standalone.jlink
\`\`\`

### LIN Application (Bootloader Required)

#### Flash with J-Link

\`\`\`bash
JLinkExe -device ATSAME51J18 -if swd -JTAGConf -1,-1 -speed auto -CommandFile sllin-app.jlink
\`\`\`

#### Update with dfu-util

\`\`\`bash
sudo dfu-util -d 1d50:0x5037,:5038 -R -D sllin.dfu
\`\`\`



EOF

}

d5035_50_hw_revs="1"
for rev in $d5035_50_hw_revs; do
	superdfu_build d5035_50 "D5035-50" $rev
done

d5035_51_hw_revs="1"
for rev in $d5035_51_hw_revs; do
	superdfu_build d5035_51 "D5035-51" $rev
done




######################
# Other Boards (uf2) #
######################

project=sllin
project_dir=$projects_dir/$project
cd $project_dir

boards="trinket_m0"
for board in $boards; do
	export BOARD=$board

	mkdir -p $target_dir/sllin/$BOARD

	rm -rf _build
	make $MAKE_ARGS uf2
	cp _build/$BOARD/${project}.uf2 $target_dir/sllin/$BOARD/
done


# archive
cd $target_dir && (tar c sllin | pixz -9 >sllin-firmware.tar.xz)

echo A-OK