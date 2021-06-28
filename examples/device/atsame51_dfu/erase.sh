#!/bin/bash

#set -e
#set -x

script_dir=$(dirname $0)


jlink_options="-device ATSAME51J19 -if swd -JTAGConf -1,-1 -speed auto"


tmp_dir=
date_str=$(date +"%F_%H%M%S")

cleanup()
{
	if [ ! -z "$tmp_dir" ] && [ -d "$tmp_dir" ]; then
		echo INFO: Removing $tmp_dir
		rm -rf "$tmp_dir"
	fi
}

error_cleanup()
{
	local last=$_
	local rc=$?
	if [ 0 -ne $rc ]; then
		echo ERROR: command $last failed with $rc
	fi

	cleanup
}

trap error_cleanup EXIT

tmp_dir=$(mktemp -d)

# generate erase file (erase command fails but does end up erasing the flash memory)
cat >"$tmp_dir/erase.jlink" << EOF
ExitOnError 1
r
ExitOnError 0
erase
exit
EOF


errors=0

# erase target
echo INFO: Erase target flash | tee -a "$meta_log_path"
JLinkExe $jlink_options -CommandFile "$tmp_dir/erase.jlink" 2>&1 | tee -a "$log_dir/erase.log"
exit_code=${PIPESTATUS[0]}

if [ $exit_code -ne 0 ]; then
	echo "ERROR: failed to erase device (exit code $exit_code)" | tee -a "$meta_log_path"
	errors=$((errors+1))
fi


#######################
# archive results
#######################
echo

exit $errors

