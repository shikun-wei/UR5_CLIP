# How to build a *PREEMPT_RT realtime Linux kernel* ?

> To build this new kernel, you will need at least 30Gb free disk space.

## Step 1 : Install kernel building tools and find out your current kernel version

First of all, we need to install several tools, which will help us patching and building a new realtime Linux kernel :

`sudo apt build-dep linux`
`sudo apt install libncurses-dev flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf fakeroot dwarves zstd`

If these commands fail with the error : `E: You must put some 'source' URIs in your sources.list`, you must open the *Software & Updates* app, and check **Source Code** in the *Ubuntu Software* tab.

You will then need to get your current kernel version with :

`uname -r`

In our case, this command returns *5.4.0-72-generic*. The following commands will assume this kernel version, and must be updated according to your own kernel specifications !

## Step 2 : Download the latest stable RT_PREEMPT patch for your kernel version, and the corresponding kernel source files

Check http://cdn.kernel.org/pub/linux/kernel/projects/rt to get the latest supported kernel version, and make sure it is higher than your current kernel version. At the time of writing this, the latest supported kernel version is 5.9, so no worries for our 5.4 kernel.

Click on your current kernel version number, and check for the corresponding latest realtime kernel patch version. In our case, it is *patch-5.4.115-rt57.patch.gz*.

Before switching to download, create a dedicated directory in your home directory with :

`mkdir ~/kernel && cd ~/kernel`

Then, download and unpack the patch using:

`wget http://cdn.kernel.org/pub/linux/kernel/projects/rt/5.4/patch-5.4.115-rt57.patch.gz`
`gunzip patch-5.4.115-rt57.patch.gz`

Let's now look for the kernel itself. Search for the downloaded real time kernel patch version in https://mirrors.edge.kernel.org/pub/linux/kernel/. Once found, download and unpack the corresponding kernel with :

`wget https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.4.115.tar.gz` 
`tar -xzf linux-5.4.115.tar.gz`

## Step 3 : Apply the RT_PREEMPT patch on the new kernel, import the current Ubuntu installation configuration

Switch into the linux subdirectory with :

`cd linux-5.4.115/`

And patch the kernel with the real time patch :

`patch -p1 < ../patch-5.4.115-rt57.patch`

>**Intel RealSense users**
>
>Intel RealSense cameras need some patches to be applied on the kernel to work properly, and the standard DKMS install may fail due to the PREEMPT-RT realtime patch. 
>
>To avoid such failures, first clone the latest release of `librealsense` into your `home` directory :
>
>`cd ~ && git clone https://github.com/IntelRealSense/librealsense.git`
>
>Get the necessary patches : 
>
>`cp librealsense/scripts/realsense-camera-formats_ubuntu-$(lsb_release -cs)-Ubuntu-hwe-5.4.patch ~/kernel/`
>`cp librealsense/scripts/realsense-metadata-ubuntu-$(lsb_release -cs)-Ubuntu-hwe-5.4.patch ~/kernel/`
>`cp librealsense/scripts/realsense-hid-ubuntu-$(lsb_release -cs)-Ubuntu-hwe-5.4.patch ~/kernel/`
>`cp librealsense/scripts/realsense-powerlinefrequency-control-fix.patch ~/kernel/`
>
>And finally apply the patches as for the PREEMPT-RT realtime one :
>
>`cd ~/kernel/linux-5.4.115/`
>`patch -p1 < ../realsense-camera-formats_ubuntu-$(lsb_release -cs)-Ubuntu-hwe-5.4.patch`
>`patch -p1 < ../realsense-metadata-ubuntu-$(lsb_release -cs)-Ubuntu-hwe-5.4.patch`
>`patch -p1 < ../realsense-hid-ubuntu-$(lsb_release -cs)-Ubuntu-hwe-5.4.patch`
>`patch -p1 < ../realsense-powerlinefrequency-control-fix.patch` 
>
>You can now skip the kernel patch step in the [installation instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) !

We do not want to override our current Ubuntu installation configuration, so we import it using :

`cp /boot/config-5.4.0-72-generic .config`

To enable your current Ubuntu installation configuration, simply use :

`yes '' | make oldconfig`

## Step 4 : Enable and configure RT_PREEMPT

In order to configure the realtime parameters of the future linux kernel, use :

`make menuconfig`

And set the following parameters :

- Enable CONFIG_PREEMPT_RT
	- General Setup
  		- Preemption Model (Fully Preemptible Kernel (Real-Time))
      	`(X) Fully Preemptible Kernel (Real-Time)`

- Enable CONFIG_HIGH_RES_TIMERS
	- General setup
  		- Timers subsystem
   			`[*] High Resolution Timer Support`

- Enable CONFIG_NO_HZ_FULL
	- General setup
		- Timers subsystem
 			- Timer tick handling (Full dynticks system (tickless))
    		`(X) Full dynticks system (tickless)`

- Set CONFIG_HZ_1000
	- Processor type and features
      - Timer frequency (1000 HZ)
     		`(X) 1000 HZ`

- Set CPU_FREQ_DEFAULT_GOV_PERFORMANCE
	- Power management and ACPI options
      - CPU Frequency scaling
        - CPU Frequency scaling (CPU_FREQ)
          - Default CPUFreq governor
     				`(X) performance`

Once all set up, save and exit menuconfig.
  
## Step 5 : Build the new realtime linux kernel !

> Building a new kernel may take some time - Usually 10 to 30 minutes on a modern CPU.

To build the kernel, run :

`make -j $(nproc) deb-pkg`
  
If the build fails with an error linked to the target `debian/canonical-certs.pem`, modify the previously imported `.config` file with :
  
`CONFIG_SYSTEM_TRUSTED_KEYS=""` instead of `CONFIG_SYSTEM_TRUSTED_KEYS="debian/canonical-certs.pem"`.

If it exists with the error `cannot represent change to vmlinux-gdb.py`, simply delete the symlink `vmlinux-gdb.py` located in the folder.

Then, install all kernel debian packages using :

`sudo dpkg -i ../*.deb`

And finally, reboot your system with :

`sudo reboot`
  
If you are using a Windows/Ubuntu dual boot, rebooting with the newly built kernel will surely fail. To solve this issue, you must disable *Secure Boot* in your BIOS.

Normally, the new realtime linux kernel should now be installed, and `uname -a` should return something like :

`Linux your_username your_new_kernel_verison #1 SMP PREEMPT_RT`
 
## Step 6 : Allow a user to set real-time permission for its processes
  
Once the realtime linux kernel is installed and running, add a group named *realtime* and add the user controlling the robot to this group :
  
`sudo addgroup realtime
&& sudo usermod -a -G realtime $(whoami)`
  
Then, add the following limits to the *realtime* group in `/etc/security/limits.conf` :
  
`@realtime soft rtprio 99`
`@realtime soft priority 99`
`@realtime soft memlock 102400`
`@realtime hard rtprio 99`
`@realtime hard priority 99`
`@realtime hard memlock 102400`

 
To apply these limits, log out and in again.
 
## *[Optional]* Step 7 : Delete the old kernel version

To delete the old Linux kernel version, simply run : 
`sudo apt --purge autoremove`

Note that you can display all installed kernel versions (and related packages) by using :
`dpkg --list | grep -i -E --color 'linux-image|linux-kernel|linux-headers|linux-modules|linux-modules-extra|linux-hwe|linux-tools|linux-modules-nvidia' | grep '^ii'`

## *[Nvidia GPU users only]* Step 8 : Fix your now broken Nvidia driver 

Unfortunately, Nvidia drivers do not support Linux real-time kernels, and to keep using your GPU normally, a few additional steps are required.

First of all, try to run the `nvidia-smi` command to check whether the newly installed Linux real-time kernel did actually mess up your Nvidia driver. Usually, this command will fail with the following error message : 
`NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver. Make sure that the latest NVIDIA driver is installed and running.`

To fix this issue, first switch to your Nvidia driver source directory :
`cd "$(dpkg -L nvidia-kernel-source-470 | grep -m 1 "nvidia-drm" | xargs dirname)"`

In our case, the 470 version of the driver was installed, but you will have to change this command, and the following ones according to your own driver version !

Then, re-build the Nvidia driver with the appropriate options : 
`sudo env NV_VERBOSE=1 \`
	`make -j8 NV_EXCLUDE_BUILD_MODULES='' \`
	`KERNEL_UNAME=$(uname -r) \`
	`IGNORE_XEN_PRESENCE=1 \`
	`IGNORE_CC_MISMATCH=1 \`
  `IGNORE_PREEMPT_RT_PRESENCE=1 \`
	`SYSSRC=/lib/modules/$(uname -r)/build \`
	`LD=/usr/bin/ld.bfd \`
	`modules`
  
Once the build is completed, you will have to transfer the linux kernel module files into your _lib_ folder, and call depmode to update the corresponding dependencies : 
`sudo mv *.ko /lib/modules/$(uname -r)/updates/dkms/ && depmod -a`

If the folder `updates/dkms/` does not exists, you simply have to create it.

Finally, reboot the system for the changes to make effect, and run the `nvidia-smi` command to verify that the Nvidia driver works properly.