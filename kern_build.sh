#! /bin/bash

# chrooted gentoo on DesireHD.
# if toolchain use, type `CROSS_COMPILE="...." ./kern_build.sh'

myname="$(basename $0 .sh)"
def_config=lordmodaufs_defconfig

build_dir="$(readlink -f ../build)"
cpuinfo=$(grep -i processor /proc/cpuinfo | wc -l)
git_repo=$(git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e 's/*\s//g')
obj_dir="${build_dir}/${git_repo}/obj"
finished="${build_dir}/${git_repo}/finished"
anykernel_dir="${build_dir}/AnyKernel"

ARCH=${ARCH:=arm}
CROSS_COMPILE=${CROSS_COMPILE:=}
EXTRA_AFLAGS="-mfpu=vfpv3 -ftree-vectorize -floop-interchange -floop-strip-mine -floop-block"
LOG="${build_dir}/${git_repo}/${myname}.log"

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
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		O=${obj_dir} $def_config
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		O=${obj_dir} menuconfig
	die "Interrupt compile"
}

compile_kernel () {
	einfo "Compile kernel"
    	einfo " * you can read log: tail -f $LOG"
	ewarn " * make oldconfig"
	make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		O=${obj_dir} oldconfig
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
	test -d ../build/${git_repo}/install && rm -fr ../build/${git_repo}/install
	dexec make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		KERNEL_DIR=$KERNEL_DIR \
		EXTRA_AFLAGS=\'$EXTRA_AFLAGS\' \
		O=${obj_dir} modules_install INSTALL_MOD_PATH=../install

	einfo "Setup Anykernel"
	setup_anykernel
	dt=$(date +%Y%m%d%H%M)
	zipedf="update_${git_repo}_${dt}.zip"

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

clean () {
	einfo "Clean-up old modules"
	test -d ${finished}/system/lib/modules && dexec rm -f ${finished}/system/lib/modules/*
	test -d ${finished}/kernel/zImage && dexec rm -f ${finished}/kernel/zImage
	dexec make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		KERNEL_DIR=$KERNEL_DIR \
		O=${obj_dir} clean
}

usage () {
	echo "usage: $(basename $0) [all | compile | install |clean]"
	exit 0
}

# -- start script

test "$#" = 0 && usage
test -f ${LOG} && rm -f ${LOG}

if [ -d .git ]; then
	einfo "Git Repository: $git_repo"
	KERNEL_DIR=$(readlink -f $(dirname .))
	test -d ${obj_dir} || install -d ${obj_dir}
	test -f ${obj_dir}/.config || config_kernel

	case "$1" in
	all)
		clean && compile_kernel && install_kernel;;
	compile)
		compile_kernel;;
	install)
		install_kernel;;
	clean)
		clean;;
	*)
		echo "invalid argument: $1"
		usage;;
	esac
else
	ewarn "not in git repository"
fi
