#! /bin/bash

help()
{
	echo "Usage: ./`basename $0` image_image sdcard_dev_name" 
	exit 0
}

IMAGE=$1
DISK=$2
RFS_PART=2

if [ "x$1" == "x"  -o "x$2" == "x" ]; then
    help
    exit -1
fi

echo "`basename $0` $1 $2"

DISK_REMOVABLE=`lsblk ${DISK} | grep disk | awk '{ print $3 }'`
if [ "x$DISK_REMOVABLE" != "x1" ]; then
	echo "You are not writing image to a removable disk, please check it."
	exit -1
fi

sudo dd if=${IMAGE} of=${DISK} bs=1M status=progress

sudo sync

echo "Resize the rootfs partition"

sudo parted -s ${DISK} "resizepart ${RFS_PART} -1" quit
sudo e2fsck -f ${DISK}${RFS_PART}
sudo resize2fs ${DISK}${RFS_PART}

sudo sync

echo "Done..."
