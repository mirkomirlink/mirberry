#!/bin/sh

for i in `cat /tmp/commit_ids`
do
	echo
	echo COMMIT $i
	echo
	cherry-pick.sh ../linux-2.6-imx-freescale/ $i
	if [ $? -ne 0 ]
	then
		git am --abort
		git reset --hard ORIG_HEAD
		echo Commit $i fails
	else
		echo Commit $i succeeds
	fi
done
