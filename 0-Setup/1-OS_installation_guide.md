# OS Installation Guide 
The Operating Systems (OS) that support ROS2 are Linux and Windows. Although Windows is supported, certain features are broken. Hence, it is advised that ROS 2 run on Linux machine.

Linux has different versions or distributions (distros), the most popular being Ubuntu. Ubuntu in-turn has different distros, like Ubuntu 18.04, Ubuntu 20.4, Ubuntu 22.04. Analogous to Windows 8, Windows 10, Windows 11. 

Just like Ubuntu, ROS 2 has its own ditros too- ROS 2 Foxy, ROS 2 Galactic, ROS 2 Humble. [source](https://docs.ros.org/en/humble/Releases.html)

For the entirety of this course, we will be using **Ubuntu 22.04** and **ROS 2 Humble**.

### Options to install Ubuntu 22.04:
#### Option 1: Use Ubuntu through a Virtual Machine (VM) (the easiest option):
  * VMWare (Better Performance VM) 
    1. Follow the instructions at the bottom of the page to install VMWare and Ubuntu 22.04 image.
  * VirtualBox (Alternative VM - Available on lab computers)
    1. Follow the instructions at the bottom of the page to install VirtualBox and Ubuntu 22.04 image.
    
#### Option 2: Install Ubuntu Natively (harder to setup, but will give best performance):
  * Dual boot- Windows and Ubuntu: [source](https://help.ubuntu.com/community/WindowsDualBoot)
  * Upgrade existing Ubuntu version [source](https://ubuntu.com/tutorials/upgrading-ubuntu-desktop#1-before-you-start)
#### Option 3: Using Ubuntu 22.04 via docker container, for macOS users or any OS users (Advanced):
  * Install docker:
    * Install docker for macOS. [link](https://docs.docker.com/desktop/install/mac-install/)
    * Install docker for ubuntu. [link](https://docs.docker.com/engine/install/ubuntu/)

### Install VMWare
VMWare Workstation Pro (personal use) is the best free Virtual Machine available:
1. Read 'Using a Virtual Machine'
1. Begin downloading the ME597 Ubuntu 22.04 image here: [Link](https://purdue0-my.sharepoint.com/:f:/g/personal/bergman9_purdue_edu/Em6RvmPqnJVJkFtfomJFiWMBApOg5o9bXQoma4h_oVpdkQ?e=Ul0Y2K)
1. Go to VMWare (broadcom) website [Link](https://support.broadcom.com/group/ecx/downloads)
1. Create an account and enter your information.
1. In the "My Downloads" tab, click "VMware Workstation Pro", then "VMware Workstation Pro xx.x for Personal Use (Windows)" and select a release version.
1. There will be a blue cloud download icon on the right side of the page. Click this to first verify your information for personal use and then you may use it to download the software.
1. Go through the VMWare Workstation Pro installation Wizard.
1. Import the .ova file: `File` --> `Open` --> Find .ova
1. Select the location to store your virtual machine.

### Install VirtualBox
1. Read 'Using a Virtual Machine'
1. Begin downloading the ME597 Ubuntu 22.04 image here: [Link](https://purdue0-my.sharepoint.com/:f:/g/personal/bergman9_purdue_edu/Em6RvmPqnJVJkFtfomJFiWMBApOg5o9bXQoma4h_oVpdkQ?e=Ul0Y2K)
1. (Pre-installed in POTR176 and ME2038) Download VirtualBox from the following [link](https://www.virtualbox.org/wiki/Downloads) and install it.
1. Click `File` --> `Import Appliance...` --> Find .ova
1. Change `Machine Base Folder` (C:\temp for POTR176 and ME2038)
1. Click `Finish` and click the VM to begin importing it
1. Change VM settings. The optimal settings will vary by machine.
      1. `System` --> Set your Base Memory, Processors
      1. `Display` --> Disable 3D acceleration, Graphics Controller: VMSVGA  

### Using a Virtual Machine
1. If you use a virtual machine, ESPECIALLY if you are using the POTR176 and ME2038 lab computers, you must back up your work every single session. Virtual machines can potentially corrupt, such as if you do not shut them down properly. Shared lab computers are prone to data loss. You are responsible for backing up your work frequently.

1. You must have ~31GB of free space minimum on your computer for the VM image (.ova) (~13GB) and the VM virtual disk (.vmdk)(17GB). Once you finish these steps, you may delete the .ova. As you continue to work on the VM, the .vmdk file will grow in size.

1. To close out of the virtual machine, make sure to shutdown properly. For VirtualBox, this is "Send Shutdown Signal". For VMware, this is "Shut Down Guest".

1. POTR176 and ME2038 users: 
    1. Use the C:\temp folder to download your .ova and save your virtual machine to. This is a shared local directory, so it does not have a storage limitation, but it can be deleted by any user.
    1. Since your virtual disk will be saved locally on a computer, if you use a different computer, you will not be able to access that same virtual machine instance.
    1. From the above two points, we highly recommend turning your ros2 workspaces into github repos to quickly and efficiently back up your data. There are many tutorials online and vscode has a useful tab for source control that may be easier to use than the command line interface.