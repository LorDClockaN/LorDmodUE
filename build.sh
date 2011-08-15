# Script taken from Imoseyon and modified by LorD CLockaN #

CPU_JOB_NUM=4
#TOOLCHAIN=/home/lord/android/system/prebuilt/linux-x86/toolchain/arm-2011/bin/
TOOLCHAIN=/home/lord/android/system/prebuilt/linux-x86/toolchain/arm-eabi-4.5.4/bin/
TOOLCHAIN_PREFIX=arm-eabi-


make -j8 ARCH=arm CROSS_COMPILE=$CCOMPILER EXTRA_AFLAGS='-mfpu=vfpv3 -ftree-vectorize -floop-interchange -floop-strip-mine -floop-block';



cp arch/arm/boot/zImage ../finished/;
cp drivers/net/wireless/bcm4329/bcm4329.ko ../finished/;
#cp drivers/staging/zram/zram.ko ../finished/;
#cp lib/lzo/lzo_compress.ko ../finished/;
cp arch/arm/mach-msm/qdsp5v2_1x/qc_pcm_in.ko ../finished/;

echo "COMPILING FINISHED!!!";
read;
#PATH=$PATH:~/bin:/home/lord/android/system/prebuilt/linux-x86/toolchain/arm-2011/bin/:/home/$USER/lord/android/system/host/linux-x86/bin/
#export CCOMPILER=/home/lord/android/system/prebuilt/linux-x86/toolchain/arm-2011/bin/arm-none-eabi-
PATH=$PATH:~/bin:/home/lord/android/system/prebuilt/linux-x86/toolchain/arm-eabi-4.5.4/bin/:/home/$USER/lord/android/system/host/linux-x86/bin/
export CCOMPILER=/home/lord/android/system/prebuilt/linux-x86/toolchain/arm-eabi-4.5.4/bin/arm-eabi-
alias make='make -j4 ARCH=arm CROSS_COMPILE=$CCOMPILER'
make clean

