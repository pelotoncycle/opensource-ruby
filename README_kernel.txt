BUILD ENVIRONMENT:

	1. Build Host OS: Ubuntu 14.04 64-bit

	2. Toolchain: Prebuilt toolchain of Android

	3. Installing required packages (Ubuntu 14.04 64-bit):
		$ sudo apt-get install libc6:i386 libncurses5:i386 libstdc++6:i386 lzop xz-utils



HOW TO BUILD KERNEL:

	1. Extract Kernel source
		$ tar -xJvf kernel.tar.xz

	2. Download the prebuilt toolchain of android:
		$ git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9

	3. Build commands:
		$ cd <path-to-kernel-source>/kernel/mediatek/mt8173/3.10
		$ export ARCH=arm64
		$ export CROSS_COMPILE=<absolute-path-to-toolchain>/bin/aarch64-linux-android-
		$ rm -rf out
		$ mkdir -p out
		$ make O=out ruby_defconfig
		$ make O=out dtbs
		$ make O=out

	4. Output file:
		Image: <path-to-kernel-source>/kernel/mediatek/mt8173/3.10/out/arch/arm64/boot/Image
		dtb:   <path-to-kernel-source>/kernel/mediatek/mt8173/3.10/out/arch/arm64/boot/dts/ruby.dtb




HOW TO UPDATE KERNEL AND RECOVERY IMAGE:

	1. In settings->developer options of Android, "OEM unlocking" needs to be checked/enabled.

	2. Plug-out USB cable and power off device

	3. Press VOLUME DOWN + POWER button to enter fastboot mode.

	4. Plug-in USB cable

	5. Unlock booloader
		5.1. Unlock boolotader with following command
			$ fastboot oem unlock

		5.2. Press VOLUME UP button to confirm unlocking

	6. Open a terminal and change to the folder that contains this README text file.

	7. The commands to upate kernel and recovery:
		$ ./mkbootimg --kernel <path-to-Image> --ramdisk ramdisk.img --second <path-to-dtb> --output boot.img
		$ ./mkbootimg --kernel <path-to-Image> --ramdisk ramdisk-recovery.img --second <path-to-dtb> --output recovery.img
		$ ./fastboot flash boot boot.img
		$ ./fastboot flash recovery recovery.img
		$ ./fastboot reboot
