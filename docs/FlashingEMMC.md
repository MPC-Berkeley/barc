# Flashing the odroid

We provide an image file of the odroid which you can flash onto your odroid using the steps described below. By downloading and flashing this image file onto the odroid's eMMC chip, you will have an exact copy of the entire operating system, including installed software, packages, and tools (e.g. ROS, openCV, Julia, etc)  necessary to run experiments. A compressed 16GB image file is available [here](https://drive.google.com/a/berkeley.edu/file/d/1lZKGgRujMVKwhIR_gWwntpjj5qGZN3HW/view?usp=drive_web). The image file is large, so it is recommended to download it with a stable connection. 

Before flashing the eMMC, you will need to a labtop, a microSD-to-eMMC adaptor and a microSD card reader (if your laptop doesn't have an SD port). The main steps for flashing are as follows 

1. Remove eMMC from odroid (refer to odroid diagram bottom view, below)
2. Connect eMMC to microSD-to-eMMC adapter (refer to adapter diagram below)
3. Attach eMMC adapter (with eMMC) to laptop (using the microSD-to-USB adapter if necessary)
4. Flash the image
    * [Win32DiskImager](https://sourceforge.net/projects/win32diskimager/) (Windows)
    * [Disk Utility](http://osxdaily.com/2012/01/04/format-an-external-hard-drive-or-usb-flash-drive-for-mac-os-x/) (Mac)
    * dd command (Mac/Linux, instructions [here](https://help.ubuntu.com/community/DriveImaging) )
5. Re-attached eMMC the odroid

Before turning on the odroid, attach real-time clock battery to the corresponding port on the odroid (at mid-top edge, see image below). 
Next, use a HDMI-to-DVI cable to connect your odroid to a monitor. 
Use the outlet adapter to power the odroid via the 5V4A DC Input port (mid-left). 
If everything goes smoothly, you should a desktop background similar to the one below.


### Odroid Diagram (top)
![alt tag](https://www.crazypi.com/image/cache/data/Odroid/XU4/xu4_3-autoxauto.jpg)

### Odroid Diagram (bottom)
<img src="http://dn.odroid.com/homebackup/201412042047141455.jpg"/>

### Connecting eMMC (red) to eMMC-to-microSD adapter (blue), with microSD-to-USB adapter (black)
<img src="http://static.generation-robots.com/5914-thickbox_default/8-gb-odroid-xu3xu4-emmc-module.jpg" width="200" height="200" />

### Default Desktop background
<img src="http://www.cnx-software.com/wp-content/uploads/2014/12/ODROID-XU3_Lite_Ubuntu_Desktop.png"/>




# Expanding/Resizing the eMMC

After the image is flashed, the usable space of the eMMC will be about the size of the image file (~14GB), so we want to expand the size of the eMMC to its full capacity (~58GB).
 
you will need to a labtop with Linux or Unbuntu, Gparted (Linux partition manager), a microSD-to-eMMC adaptor, and a microSD card reader (if your laptop doesn't have an SD port). The main steps are as follows:

1. Install Gparted by typing the command below in terminal if it has not been installed. Gparted is not installed by default.

   $ sudo apt-get install gparted 

2. Remove eMMC from ordroid, connect eMMC to microSD-to-eMMC adapter, and attach eMMC adapter (with eMMC) to laptop. (See first three steps in the previous section of "Flasing the odroid").
3. Select a USB drive to read by going to Devices(top left taps)> USB> "your USB name". (See the image of selecting USB below). 
4. Open Gparted and be sure to select the correct drive to resize at the top right corner (the one with 58GB). (See image of selecting a drive in Gparted below).
5. Select the correct partition with a file system of "ext4" and a size of about 14.55GB and click the orange arrow to resize. (See image below).
6. Move the bar all the way to the right so that the new size matches the maximum size and click the resize button. And click the apply button to apply operations to the device. (Refer to images below).

### Selecting USB 
<img src="https://github.com/MPC-Berkeley/barc/blob/master/docs/imgs/selectUSBDrive.png" alt="Drawing" style="width: 400px;"/>

### Selecting the correct drive in Gparted
<img src="https://raw.githubusercontent.com/MPC-Berkeley/barc/master/docs/imgs/selectDriveInGparted.png" alt="Drawing" style="width: 400px;"/>

### Clicking resize arrow
<img src="https://github.com/BARCproject/barc/raw/master/docs/imgs/resizeArrow.PNG" alt="Drawing" style="width: 400px;"/>

### Moving the bar to resize the partition
<img src="https://raw.githubusercontent.com/MPC-Berkeley/barc/master/docs/imgs/barBeforeResizing.png" alt="Drawing" style="width: 400px;"/>


<img src="https://raw.githubusercontent.com/MPC-Berkeley/barc/master/docs/imgs/barAfterResizing.png" alt="Drawing" style="width: 400px;"/>


<img src="https://raw.githubusercontent.com/MPC-Berkeley/barc/master/docs/imgs/resizeConfirmation.png" alt="Drawing" style="width: 400px;"/>


