#! /bin/bash

# chrooted gentoo on DesireHD.

ARCH=arm
CROSS_COMPILE=

cpuinfo=$(grep -i processor /proc/cpuinfo | wc -l)
git_ver=$(git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e 's/*\s//g')
obj_dir="../build/${git_ver}/obj"
finished="../build/${git_ver}/finished"
def_config=lordmoduebfs_defconfig

die() {
    echo -e "\033[1;30m>\033[0;31m>\033[1;31m> ERROR:\033[0m ${@}" && exit 1
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
		EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorize' \
		O=${obj_dir} $def_config
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorize' \
		O=${obj_dir} menuconfig
	die "Interrupt compile"
}

compile_kernel () {
	einfo "Compile kernel"
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorize' \
		O=${obj_dir} zImage 2>&1 | tee ${obj_dir}/kern.log || die "Compile kernel error"
	einfo "Compile modules"
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorizeine' \
		O=${obj_dir} modules 2>&1 | tee ${obj_dir}/kern.log || die "Compile module error"
}

install_kernel {
	einfo  "Install modules"
	rm -fr ../build/install
	make -j${cpuinfo} ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE \
		EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorize' \
		O=${obj_dir} modules_install INSTALL_MOD_PATH=../install 2>&1 >/dev/null

	einfo "Install packages"
	test -d ${finished}/system/lib/modules || install -d ${finished}/system/lib/modules
	test -d ${finished}/kernel || install -d ${finished}/kernel
	cp ${obj_dir}/arch/arm/boot/zImage ${finished}/kernel/
	for f in $(find ../build/${git_ver}/install -name "*.ko")
	do
		echo " $(basename $f)"
		cp $f ${finished}/system/lib/modules/
	done
}

test -d ${obj_dir} || install -d ${obj_dir}
test -f ${obj_dir}/.config || config_kernel

if [ -d .git ]; then
	einfo "Pulling git repository: $git_ver"
	git pull 2>&1 >/dev/null || die "Does not pulling from repository"

	einfo "Clean-up old modules"
	test -d ${finished}/system/lib/modules && rm -f ${finished}/system/lib/modules/*
	test -d ${finished}/kernel/zImage && rm -f ${finished}/kernel/zImage
	find ${obj_build} -name "*.ko" | xargs rm -f

	compile_kernel
	install_kernel
	
else
	ewarn "not in git repository"
fi
