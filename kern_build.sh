#! /bin/bash

# chrooted gentoo on DesireHD.

ARCH=${ARCH:=arm}
CROSS_COMPILE=${CROSS_COMPILE:=}

cpuinfo=$(grep -i processor /proc/cpuinfo | wc -l)
git_repo=$(git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e 's/*\s//g')
obj_dir="$(readlink -f ..)/build/${git_repo}/obj"
finished="$(readlink -f ..)/build/${git_repo}/finished"
anykernel_dir="$(readlink -f ..)/build/AnyKernel"
def_config=lordmodaufs_defconfig

die() {
    echo -e "\033[1;30m>\033[0;31m>\033[1;31m> ERROR:\033[0m ${@}"
    exit 1
}

einfo() {
    echo -ne "\033[1;30m>\033[0;36m>\033[1;36m> \033[0m${@}\n"
}

ewarn() {
    echo -ne "\033[1;30m>\033[0;33m>\033[1;33m> \033[0m${@}\n"
}

config_kernel ()
{
	einfo "Configure kernel"
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorize -floop-interchange -floop-strip-mine -floop-block' \
		O=${obj_dir} $def_config
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorize -floop-interchange -floop-strip-mine -floop-block' \
		O=${obj_dir} menuconfig
	die "Interrupt compile"
}

compile_kernel () {
	einfo "Compile kernel"
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		KERNEL_DIR=$KERNEL_DIR \
		EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorize -floop-interchange -floop-strip-mine -floop-block' \
		O=${obj_dir} zImage 2>&1 || die "Compile kernel error"
	einfo "Compile modules"
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		KERNEL_DIR=$KERNEL_DIR \
		EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorizeine -floop-interchange -floop-strip-mine -floop-block' \
		O=${obj_dir} modules 2>&1 || die "Compile module error"
}

setup_anykernel () {
	test -d $anykernel_dir || 
		git clone git://github.com/ac1965/AnyKernel.git $anykernel_dir 2>&1 > /dev/null
	(
		cd $anykernel_dir
		git pull 2>&1 >/dev/null
		git archive --format zip --output ../u.zip master 2>&1 > /dev/null
	)
	test -d $finished && rm -fr $finished
	unzip ../build/u.zip -d $finished 2>&1 > /dev/null
	rm -f ../build/u.zip
}

install_kernel () {
	einfo  "Install modules"
	rm -fr ../build/install
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		KERNEL_DIR=$KERNEL_DIR \
		EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorize -floop-interchange -floop-strip-mine -floop-block' \
		O=${obj_dir} modules_install INSTALL_MOD_PATH=../install 2>&1 >/dev/null

	einfo "Setup Anykernel"
	setup_anykernel
	dt=$(date +%Y%m%d%H%M)
	zipedf="update-${dt}.zip"

	einfo "Install packages [$zipedf]"
	find ${obj_dir}/arch/${ARCH}/boot -name zImage -exec install -D {} ${finished}/kernel/zImage \;
	for f in $(find ../build/${git_repo}/install -name "*.ko")
	do
		echo " - $(basename $f)"
		cp $f ${finished}/system/lib/modules/
	done
	(
		cd ${finished}
		zip -r ../${zipedf} .
	)
}

test -d ${obj_dir} || install -d ${obj_dir}
test -f ${obj_dir}/.config || config_kernel

if [ -d .git ]; then
	KERNEL_DIR=$(readlink -f $(dirname .))
	einfo "Git Repository: $git_repo"

	einfo "Clean-up old modules"
	test -d ${finished}/system/lib/modules && rm -f ${finished}/system/lib/modules/*
	test -d ${finished}/kernel/zImage && rm -f ${finished}/kernel/zImage
	find ${obj_dir} -name "*.ko" | xargs rm -f

	compile_kernel
	install_kernel
	
else
	ewarn "not in git repository"
fi
