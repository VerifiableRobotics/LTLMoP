This installation has been tested with ubuntu 10.04 LTS at Oct. 2010

 - After install ubuntu, update with "Update Manager" in System>>Administration

 - After restart, open "Synaptic Package Manager" in System>>Administration

 - Using the manager, install 

	PackageName		Version

	python-scipy		0.7.0-2
	python-wxglade		0.6.3-0.1ubuntu3
	stage			    2.0.3-2ubuntu2
	python-dev		    2.6.5-Oubuntu1
	swig			    1.3.40-2ubuntu1
	robot-player-dev	2.0.4-3.3ubuntu5
	libjpeg62-dev		6b-15ubuntu1
	libgtk2.0-dev		2.20.1-Oubuntu2
	freeglut3		    2.6.0-Oubuntu2

 - The package manager will automatically mark the necessary dependencies
   some of the packages above might already be installed. if so, just ignore it.


================ Install Player and Stage ================================

 - download the player (2.0.5) and stage (2.0.3) from 
    http://sourceforge.net/projects/playerstage/files/

 - unzip player and cd into the folder

 - in the console, enter the following lines of code

	fichero=`mktemp`
	echo "#include <cstdlib>" >> $fichero
	cat server/drivers/mixed/erratic/erratic.cc >> $fichero
	mv $fichero server/drivers/mixed/erratic/erratic.cc

	fichero=`mktemp`
	echo "#include <cstring>" >> $fichero
	cat client_libs/libplayerc++/actarrayproxy.cc >> $fichero
	mv $fichero client_libs/libplayerc++/actarrayproxy.cc

	fichero=`mktemp`
	echo "#include <cstring>" >> $fichero
	cat examples/libplayerc++/clientgraphics.cc >> $fichero
	mv $fichero examples/libplayerc++/clientgraphics.cc 

	fichero=`mktemp`
	echo "#include <cstring>" >> $fichero
	cat client_libs/libplayerc++/test/test.cc >> $fichero
	mv $fichero client_libs/libplayerc++/test/test.cc

 - add the following line at the end of file /etc/bash.bashrc
	export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python/$PYTHON_VERSION/site-package

 - restart ubuntu	

 - then enter
	./configure --disable-garminnmea

 - after configuring, check if the output shows something like:
    Support for plugin drivers will be included.
 
    Python bindings to libplayerc will be built

 - enter make
    if any error shows during building, try to reconfigure player and disable the driver that cause the error
    the list of driver can be obtained by enter ./configure -help

 - after sucessfully build, enter 
    sudo make install

 - the player should be installed

 - unzip stage package and cd in to the folder and enter
    ./configure
    make
    sudo make install

  - the player and stage can be tested by
    player <stageInstallationFileFolder>/worlds/simple.cfg


=========================== Install Polygon Package ==============================

 - Download the Polygon package from 
    http://polygon.origo.ethz.ch/download
 - Sugguested version is 2.0.3

 - After downloading, unzip it and install it by entering
    sudo python setup.py install

 - The installation should be fairly smooth

 - After installation, delete the unzipped folder (otherwise the package will try to run in that folder)

 - The Polygon package can be tested in python, the instruction can be found here
    http://download.origo.ethz.ch/polygon/2101/Polygon.pdf



	


