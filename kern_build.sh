#! /bin/bash

# chrooted gentoo on DesireHD.

cpuinfo=$(grep -i processor /proc/cpuinfo | wc -l)
git_repo=$(git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e 's/*\s//g')
obj_dir="$(readlink -f ..)/build/${git_repo}/obj"
finished="$(readlink -f ..)/build/${git_repo}/finished"
anykernel_dir="$(readlink -f ..)/build/AnyKernel"
def_config=lordmodaufs_defconfig

ARCH=${ARCH:=arm}
CROSS_COMPILE=${CROSS_COMPILE:=}
EXTRA_AFLAGS="-mfpu=vfpv3 -ftree-vectorize -floop-interchange -floop-strip-mine -floop-block"
LOG="$(readlink -f ..)/build/${git_repo}/$(basename $0 .sh).log"

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

dexec () {
    CMD="$@"
    echo "Exec:$CMD" >> $LOG
    eval $CMD >> $LOG 2>&1 || die "Die:$CMD"
}

config_kernel ()
{
	einfo "Configure kernel"
	dexec make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		EXTRA_AFLAGS=\'$EXTRA_AFLAGS\' \
		O=${obj_dir} $def_config
	dexec make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		EXTRA_AFLAGS=\'$EXTRA_AFLAGS\' \
		O=${obj_dir} menuconfig
	die "Interrupt compile"
}

compile_kernel () {
	einfo "Compile kernel"
    einfo " * you can read log: tail -f $LOG"
	dexec make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		KERNEL_DIR=$KERNEL_DIR \
		EXTRA_AFLAGS=\'$EXTRA_AFLAGS\' \
		O=${obj_dir} zImage || die "Compile kernel error"
	einfo "Compile modules"
	dexec make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		KERNEL_DIR=$KERNEL_DIR \
		EXTRA_AFLAGS=\'$EXTRA_AFLAGS\' \
		O=${obj_dir} modules || die "Compile module error"
}

setup_anykernel () {
	test -d $anykernel_dir || 
	dexec git clone git://github.com/ac1965/AnyKernel.git $anykernel_dir
	(
		cd $anykernel_dir
		dexec git pull
		dexec git archive --format zip --output ../u.zip master
	)
	test -d $finished && dexec rm -fr $finished
	dexec unzip ../build/u.zip -d $finished
	dexec rm -f ../build/u.zip
}

install_kernel () {
	einfo  "Install modules"
	dexec test -d ../build/${git_repo}/install && rm -fr ../build/${git_repo}/install
	dexec make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		KERNEL_DIR=$KERNEL_DIR \
		EXTRA_AFLAGS=\'$EXTRA_AFLAGS\' \
		O=${obj_dir} modules_install INSTALL_MOD_PATH=../install

	einfo "Setup Anykernel"
	setup_anykernel
	dt=$(date +%Y%m%d%H%M)
	zipedf="update-${dt}.zip"

	einfo "Install packages [$zipedf]"
	find ${obj_dir}/arch/${ARCH} -name zImage -exec install -D {} ${finished}/kernel/zImage \;
	for f in $(find ../build/${git_repo}/install -name "*.ko")
	do
		echo " - $(basename $f)"
		dexec cp $f ${finished}/system/lib/modules/
	done
	(
		cd ${finished}
		dexec zip -r ../${zipedf} .
	)
}

# -- start script

test -d ${obj_dir} || install -d ${obj_dir}
test -f ${obj_dir}/.config || config_kernel
test -f ${LOG} && rm -f ${LOG}

if [ -d .git ]; then
	KERNEL_DIR=$(readlink -f $(dirname .))
	einfo "Git Repository: $git_repo"

	einfo "Clean-up old modules"
	test -d ${finished}/system/lib/modules && dexec rm -f ${finished}/system/lib/modules/*
	test -d ${finished}/kernel/zImage && dexec rm -f ${finished}/kernel/zImage
	dexec find ${obj_dir} -name "*.ko" | xargs rm -f

	compile_kernel
	install_kernel
	
else
	ewarn "not in git repository"
fi
