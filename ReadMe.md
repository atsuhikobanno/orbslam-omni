Easy build for orbslam-omni on Windows

1.
 Make a directory called build in orbslam-windows/Thirdparty/DBoW2
 
 Run CMake GUI and set source code to orbslam-windows/Thirdparty/DBoW2 
 and where to build the binaries to orbslam-windows/Thirdparty/DBoW2/build

 Press Configure and choose Visual Studio 14 2015 Win64 or Visual Studio 12 2013 Win64
 OpenCV_DIR is to C:/opencv3.2.0/build (in my case)
 Press Generate
 	
 Open the resulting project in the build directory in Visual Studio
 Change build type to Release (in white box up top, should initially say Debug)
 Right click on DBoW2 project -> Properties -> General: change Target Extension to .lib 
 and Configuration Type to Static Library (.lib)	
 Go to C/C++ Tab -> Code Generation and change Runtime Library to Multi-threaded (/MT)
 Build ALL_BUILD. You should get lots of warnings but no errors

2.
 Make a directory called build in orbslam-windows/Thirdparty/g2o
 
 Run CMake GUI and set source code to orbslam-windows/Thirdparty/g2o 
 and where to build the binaries to orbslam-windows/Thirdparty/g2o/build
	
 Press Configure and choose Visual Studio 14 2015 Win64 or Visual Studio 12 2013 Win64
 Press Generate
	
 Open the resulting project in the build directory in Visual Studio
 Change build type to Release (in white box up top, should initially say Debug)
 Right click on g2o project -> Properties -> General: change Target Extension to .lib 
 and Configuration Type to Static Library (.lib)
 Go to C/C++ Tab -> Code Generation and change Runtime Library to Multi-threaded (/MT)
 Go to C/C++ -> Preprocessor and press the dropdown arrow in the Preprocessor Definitions, 
 then add a new line with WINDOWS on it (no underscore), then press OK, then Apply
 Build ALL_BUILD.

3.
 Make a directory called build in orbslam-windows/Thirdparty/Pangolin

 Run CMake GUI and set source code to orbslam-windows/Thirdparty/Pangolin 
 and where to build the binaries to orbslam-windows/Thirdparty/Pangolin/build
 
 Press Configure and choose Visual Studio 14 2015 Win64 or Visual Studio 12 2013 Win64. You'll have a lot of RED and a lot of things that say DIR-NOTFOUND but as long as the window at the bottom says Configuring Done you're fine
 Press Generate

 Open the resulting project in the build directory in Visual Studio
 Change build type to Release (in white box up top, should initially say Debug)
 Build ALL_BUILD. You'll have an error by project testlog that says 
 "cannot open input file 'pthread.lib'" but that doesn't matter cause we don't use testlog. 
 Everything else should build fine, i.e., you should have 
 ========== Build: 18 succeeded, 1 failed, 0 up-to-date, 0 skipped ==========

4.
 Make a directory called build in orbslam-windows

 Run CMake GUI and set source code to orbslam-windows 
 and where to build the binaries to orbslam-windows/build
	
 Press Configure and choose Visual Studio 14 2015 Win64 or Visual Studio 12 2013 Win64
 Press Generate
	
 Open the resulting project in the build directory in Visual Studio
 Change build type to Release (in white box up top, should initially say Debug)
 Right click on ORB_SLAM2 project -> Properties -> General: change Target Extension to .lib 
 and Configuration Type to Static Library (.lib)
 Go to C/C++ Tab -> Code Generation and change Runtime Library to Multi-threaded (/MT)

 Right click on the ORB_SLAM2 project (NOT ALL_BUILD) and click Build
 If you're lucky, that will take few minutes then successfully build!


To build mono_omni, do the following:
?Right click on that project and go to Properties -> C/C++ -> Code Generation, and change Runtime Library to Multi-threaded (/MT). Then press apply
?Right click on it and press build

To Run (at \orbslam_omni-windows)
./Examples/Monocular/Release/mono_omni.exe ./Vocabulary/ORBvoc.txt ./Examples/Monocular/AL_LAB.yaml 1000

For your input images, please modify LoadImages() function in mono_omni.cc.


