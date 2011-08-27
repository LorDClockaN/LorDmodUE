# Script to build TalonMSM 7x30 kernel source
# Created by eXistZ


VERSION="v1.01"
KERNEL_SRC="/usr/src/htc-kernel-msm7x30"

# Linaro Android 4.5 (GCC 4.5.4) toolchain - http://www.linaro.org
export CROSS_COMPILE="/opt/toolchains/android-toolchain-eabi-4.5/bin/arm-eabi-"

export ARCH=arm
export LOCALVERSION="-TalonACE_7x30-$VERSION"

START=$(date +%s)

make talon_msm7230_defconfig

if [ -e ./releasetools/system/lib/modules ]; then
 rm -rf ./releasetools/system/lib/modules
fi

mkdir ./releasetools/system/lib/modules

export INSTALL_MOD_PATH=./mod_inst
make modules -j`grep 'processor' /proc/cpuinfo | wc -l`
make modules_install

for i in `find mod_inst -name "*.ko"`; do
 cp $i ./releasetools/system/lib/modules/
done

rm -rf ./mod_inst

make -j`grep 'processor' /proc/cpuinfo | wc -l`
cp $KERNEL_SRC/arch/arm/boot/zImage $KERNEL_SRC/releasetools/kernel/
cd $KERNEL_SRC/releasetools
rm -f *.zip
zip -r TalonACE_7x30-$VERSION.zip *
rm $KERNEL_SRC/releasetools/kernel/zImage

cd $KERNEL_SRC

END=$(date +%s)
ELAPSED=$((END - START))
E_MIN=$((ELAPSED / 60))
E_SEC=$((ELAPSED - E_MIN * 60))
printf "Elapsed: "
[ $E_MIN != 0 ] && printf "%d min(s) " $E_MIN
printf "%d sec(s)\n" $E_SEC
echo "Finished."
