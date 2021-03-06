# FastMarkerLocalizer
Code to my PhD thesis "Design and evaluation of a camera-based indoor positioning system for forklift trucks"

German title is "Entwicklung und Evaluierung einer kamerabasierten Lokalisierungsmethode für Flurförderzeuge".

## Requirements

* Raspberry Pi running Debian 8 (jessie) with gcc/g++ toolchain. Newer versions might work as well, but could require some adaption to the build process.
* raspicam version 1.3: https://sourceforge.net/projects/raspicam/files/ (newer versions should work as well, but you will need to adapt the patch, which I only provide for version 1.3)
* OpenCV library version 2.4.x: https://opencv.org/releases.html
* ArUco library sources version 1.2.5 or 1.3: https://sourceforge.net/projects/aruco/files/OldVersions/

## Notes

* This program was a side-project of mine during my work as a scientific assistant at TUM (chair fml, see https://www.fml.mw.tu-muenchen.de), which turned out to become my PhD topic. It is therefore only a technology demonstration and never received the attention required for industrial-grade software.
* The good news : this code is published as open source software. It is subject to the 3-Clause BSD license (see LICENSE file for details).
* Now here is the catch
  * Beware this is highly experimental software and was only intended for scientific investigation.
  * I have no time to improve the code/build process or provide assistance. You are alone with the code - take it or leave it.
  * I do not provide compiled binaries for various reasons, and I never will.

## How to build

* You can build the binaries directly on the Raspberry pi, since setting up a cross-compile toolchain with all the depending libraries is going to be much more work than directly compiling on the pi (which takes only a few hours and is only required once). You can deploy the binary to other pi's afterwards.
* Since there is currently no auto-build process for this software, you will need c/c++ developer skills in order to successfully build this software on any other pi than mine. There is a list of packages that were installed on the pi I used to build this software, if you want to create a similar build environment. Be prepared to fix several compiler- and linker-related issues, since your system likely differs from mine.
* Once you have compiled the program, you can take a look at the *.sh files used to execute the program.

### Here is what you have to do:
1. Login to the Raspberry Pi as user pi
2. Copy the sources to the target machine on a Raspberry Pi 1 (type A or B) running debian 8 (jessie)
3. Extract the sources of raspicam to /home/pi/fml/raspicam-0.1.3, apply the patch fml_raspicam-0.1.3.patch and compile raspicam
4. The folder FastMarkerLocalizer has to be located in /home/pi/fml/FastMarkerLocalizer
5. Follow the instructions in src/aruco/README
6. If you are compiling for another architecture than the Raspberry Pi 1, you need to adapt the CFLAGS in all the makefiles appropriately.
7. Go to subfolder /home/pi/fml/FastMarkerLocalizer/Release and run make
8. Wait until build process is finished.
9. If it fails, fix the errors given by the compiler :-).

## How to configure
1. check the configuration files in config folder
2. calibration information goes to the calibration file (e.g. calibration_pi1.xml) See src/configure/calibrationparameters.cpp
3. camera information goes to camera file (e.g. camera_pi1.xml). See src/configure/cameraparameters.cpp
4. marker information goes to marker_ceiling_ground_truth.xml. See src/configure/markerparameters.cpp
5. marker detection and localization configuration is done in the tracker file (e.g. tracker.xml). See trackerparameters.cpp
6. there is one config, which will be supplied to the program and tell it which of the above files to use (e.g. config_pi1.xml). See src/configure/config.cpp

## How to run
1. go to folder /home/pi/fml/FastMarkerLocalizer
2. check run_rpi1.sh on how to run the program.
3. there are some other example configs and scripts available in the FastMarkerLocalizer folder

## FAQ
* Q: Why isn't there a better build process?
* A: This software was written for my PhD thesis and the class names and API were in constant flux during development. This would have required continuous maintenance of the build process, which would have led to a lot of extra work. Plus having a nice build process doesn't score you bonus points in science, so you make it quick (and dirty if necessary) :-)

---
* Q: Why do you not supply a binary?
* A: The code is supplied to help others comprehend the algorithms I adapted/invented in my PhD thesis. I created binaries for experiment purposes, but since they are compiled for a very specific environment and only operate on a Raspberry Pi model B (which is rare nowadays) you have to compile it yourself. There might also be legal issues I can dodge by not supplying any executable binary (the standard providing source code is freedom of speech bla bla).

---
* Q: Why is the code so ugly?
* A: The code is basically the result of a sequence of trial-and-error attempts at improving state of the art localization approaches. Ideally you would know exactly what and how to implement beforehand (and also what not to implement), but truth is, if you are still investigating what actually works and implement ideas while they are fresh, then you always have to make compromises between clear structure and implementation time. Since a priori you do not even know if your next idea is going to work or not, you favor quick-and-dirty code over a lot more wasted time. Especially if you are not gonna suffer from the bad quality yourself :-). Another reason is, that runtime-critical parts of the code were optimized to achieve faster execution by replacing inefficient initial code with optimized faster code - this almost never looks good.

---
* Q: I do not understand why/how a specific section of the code works.
* A: Read my PhD thesis: "Entwicklung und Evaluierung einer kamerabasierten Lokalisierungsmethode für Flurförderzeuge". It is available as a PDF (only in German) from the TUM library (https://mediatum.ub.tum.de/?id=1395267). If that doesn't help, analyze what happens during code execution (when running with the test data set)

---
* Q: You promised to publish the test data set of your evaluation (see your thesis p.126)
* A: And I did. You can find it at https://doi.org/10.5281/zenodo.1323818 .

---
* Q: I want to improve the localization algorithms. What can I do?
* A: See chapter 8 of my PhD thesis for some pointers (see link above). You should specifically look into better camera calibration, because that is probably the easiest remaining error source to avoid.

---
* Q: Are you going to fix bugs/issues?
* A: No. If you wish to provide a better version of this code, I strongly recommend to fork the project and take it to another repository. As long as you abide by the license terms, you are welcome to do so.

---
* Q: May I use the sources in a research project?
* A: If you abide by the license terms, you may. See LICENSE file for details.

---
* Q: May I use the sources in a commercial project?
* A: If you abide by the license terms, you may. See LICENSE file for details.

---
* Q: I do not like the license terms for reason "X". What can I do?
* A: Then you must not use the sources. Sorry, this is as free as it is going to get. Take it or leave it.

---
* Q: I want to make a product out of it. Will you help me?
* A: I work in a completely different area now and have other more interesting things (at least to me) to work on in my free time - I will help neither for free nor if you offer money. You have to find somebody else to do that work.
