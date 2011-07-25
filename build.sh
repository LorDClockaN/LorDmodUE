# Script taken from Imoseyon and modified by LorD CLockaN #

CPU_JOB_NUM=4
TOOLCHAIN=/home/lord/android/system/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/
TOOLCHAIN_PREFIX=arm-eabi-


make -j$CPU_JOB_NUM ARCH=arm CROSS_COMPILE=$TOOLCHAIN/$TOOLCHAIN_PREFIX;


cp arch/arm/boot/zImage ../finished/;
cp drivers/net/wireless/bcm4329/bcm4329.ko ../finished/;
cp arch/arm/mach-msm/qdsp5v2_1x/qc_pcm_in.ko ../finished/;

echo "COMPILING FINISHED!!!";
read;
make clean

