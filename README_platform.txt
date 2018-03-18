1. Install repo tool:
	$ mkdir ~/bin -p
	$ sudo apt-get install curl
	$ curl https://dl-ssl.google.com/dl/googlesource/git-repo/repo > ~/bin/repo
	$ chmod a+x ~/bin/repo
	$ export PATH=~/bin:$PATH



2. Installing required packages:
	$ sudo apt-get install xz-utils



3. Get Android Filesystem Sources:
	Open a terminal and move into the folder that you want to download Android source code.
	$ repo init -u https://android.googlesource.com/platform/manifest -b android-5.1.0_r1
	$ repo sync



4. There are 4 modules in "external" folder, copy the modules to original Android source code.
	If the same module exists in Android source code, you should replace it.



5. There is 1 modules in "frameworks" folder, copy the module to original Android source code.
	If the same module exists in Android source code, you should replace it.

