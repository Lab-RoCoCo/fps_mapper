# README #

Fast Pose Solver mapping suite
A thin package for mapping with multiple depth sensors

### What is this repository for? ###

### How do I get set up? ###

The system works on ubuntu 14.04, ros indigo (but should work also on other versions)

1) install the ubuntu packages

sudo apt-get install \
     libeigen3-dev \
     libflann-dev \
     libsuitesparse-metis-dev \
     freeglut3-dev \
     libqglviewer-qt4-dev 
     

2) checkout, install and build g2o. Locally.

cd <YOUR_SRC_FOLDER_WHERE_YOU_DOWNLOAD_EXTERNAL_LIBS>
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cmake ../
make -j <as much as you have>

g2o is now compiled, we need to set up the environment so that cmake finds our local installation.
To this end add this to your ~/.bashrc
export G2O_ROOT=<YOUR_SRC_FOLDER_WHERE_YOU_DOWNLOAD_EXTERNAL_LIBS>/g2o

source ~/.bashrc

3) install and build fps_mapper

put the folder of fps_mapper in your catkin workspace
run catkin_make from the root of the workspace

That should do.


### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact