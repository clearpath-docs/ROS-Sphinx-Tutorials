Getting Started with Ubuntu
=============================

To learn how to use ROS, we must first learn the basics of Ubuntu! Ubuntu is a Linux based open source operating system. The bulk of development will be done using the terminal. You can open up a terminal window by clicking the terminal icon or by typing "terminal" in the start menu.

.. image:: UbuntuScreen.png

To use the terminal, simply type in a command in the window and push "enter". When working in a terminal, the directory you are currently located in is called the "working directory" and shown at the start of the terminal line.  Here are some of the most used commands

=======================		===============================================================
Command 					Description
=======================		===============================================================
ls 							Lists files and folders
cd <folder>					Change the working directory to <folder>
pwd							Prints the current working directory
cp <src><dest>				Copies <src> to <dest>
sudo						Executes the command as the root user
mkdir<directory>			Creates a directory in your working directory named <directory>
gedit or nano <file>		Opens a text editor to edit <file>
=======================		===============================================================

Ubuntu programs are installed from repositories. This allows for very easy installation of programs and applications through the apt system. To install a package, simply type

.. parsed-literal::

    sudo apt-get install

ROS packages will be named ros-. For example, to install the Clearpath Robotics Indigo Husky packages you would use:

.. parsed-literal::

	sudo apt-get install ros-indigo-husky-desktop

A very convenient feature of Ubuntu is tab auto completion! While typing a command, press the tab key to complete the rest of the command. If there are multiple commands that could be used to finish your line, double tap the tab key to list the possible options.

You should now have the tools you need to install ROS on your machine! Instructions can be found on the `ROS Wiki <http://wiki.ros.org>`_.
