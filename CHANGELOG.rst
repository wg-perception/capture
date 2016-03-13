^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package object_recognition_capture
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Use "mono8" for mask and swap brg to rgb
* Fix encoding when saving bag file
* use catkin macro to install Python scripts
* cleanup tests
* clean build dependencies
* Contributors: JimmyDaSilva, Vincent Rabaud

0.3.0 (2014-07-27)
------------------
* make sure doc works with the latest ecto
* no need for stack.xml
* drop Fuerte support
* update website
* Added argument `-p` for preview.
  This is frequently used argument so it's good to have a short form.
* Corrected errors in parsing arguments `--input` and `--matches`.
* Contributors: Denis Štogl, Vincent Rabaud

0.2.22 (2013-05-29)
-------------------
* switch to a C++ implementation of FeatureFinder, comply to K_image/K_depth
* use the proper K_image/K_depth
* make sur ewe use the right K
* add a visualization for the mask
* remove useless line
* Merge branch 'master' of github.com:wg-perception/capture
* update the docs for clarity
* update email
* add an error message
* fixes `#8 <https://github.com/wg-perception/capture/issues/8>`_
  The docs were indeed out of date :) Thx !
* Contributors: Vincent Rabaud

0.2.21 (2013-03-12)
-------------------
* update docs
* update docs acoording to `#6 <https://github.com/wg-perception/capture/issues/6>`_
* no ouput when no plane is found
* fix coordinates
* fix potential out of bound
* deal with the case where T is given with NaN's
* improve docs a bit
* add ecto_ros dependency
* Contributors: Vincent Rabaud

0.2.20 (2013-02-28)
-------------------
* update the mask definition in the doc
* cleanup
* fix a bad parameter input
* add a pdf version of the pattern
* Contributors: Vincent Rabaud

0.2.19 (2013-02-25)
-------------------
* remove warning from `#4 <https://github.com/wg-perception/capture/issues/4>`_
* comply to the tighter BlackBox API
* useless folder
* fix more config
* add reference to ork_capture
* make documentation build on its own
* fix the tests after the file change
* Contributors: Vincent Rabaud

0.2.18 (2013-02-06)
-------------------
* rename the scripts to not have .py
* improve docs
* share code for pose filtering and drawing
* make the pose stick to the plane even more
* K is not needed
* use the new plane clusterer
* cleanups
* track planes for robustness
* clean the capture and make it scale independent by tracking planes
* use the option group
* Merge pull request `#3 <https://github.com/wg-perception/capture/issues/3>`_ from mikeferguson/master
  Update docs to reflect package being renamed to object_recognition_capture. Fix some broken scripts.
* fix other scripts broken by taking 3d clouds out of openni source
* fix typo from blackboxing
* package is now named capture, update docs
* allow camera parameter changes from the command line
* CMake cleanups
* Contributors: Michael Ferguson, Vincent Rabaud

0.2.17 (2013-01-13)
-------------------
* use the proper catkin variable
* Contributors: Vincent Rabaud

0.2.16 (2013-01-04)
-------------------
* port more to the new BlackBox
* comply to the new BlackBox API
* remove the old scheduler options
* remove a warning
* fix the catkin buildtool_depend
* Contributors: Vincent Rabaud

0.2.15 (2012-11-18)
-------------------
* make the setup.py work under Fuerte
* Contributors: Vincent Rabaud

0.2.14 (2012-11-03)
-------------------
* use catkin_pkg
* Contributors: Vincent Rabaud

0.2.13 (2012-11-01)
-------------------
* remove the copyright tag
* use the new ecto_catkin interface
* get the information from the package.xml
* comply to the new API
* also install the manifest.xml
* remove electric support
* Contributors: Vincent Rabaud

0.2.12 (2012-10-10)
-------------------
* remove the old load_manifest
* move the odometry to an ecto_opencv sample
* comply to the new API
* comply to the new catkin API
* no need for roscompat anymore
* fix the Groovy install
* Contributors: Vincent Rabaud

0.2.11 (2012-09-08)
-------------------
* have code work with Electric/Fuerte/Groovy
* use the new ectomodule API
* remove redundant maintainer
* changed doc index heading
* move odometry to ecto_opencv
* use the code from the ecto_opencv RGDB module
* Contributors: David Gossow, Vincent Rabaud

0.2.10 (2012-06-07)
-------------------
* add bogus content
* fix install issues
* Contributors: Vincent Rabaud

0.2.9 (2012-06-06)
------------------
* better install of files
* add a bogus manifest.xml to be able to rosrun
* cleanups
* switch to the new catkin
* Contributors: Vincent Rabaud

0.2.8 (2012-05-18)
------------------
* better docs
* do not copy roscompat
* fix bad imports
* Contributors: Vincent Rabaud

0.2.7 (2012-05-10 16:11)
------------------------
* fix a bad install
* Contributors: Vincent Rabaud

0.2.6 (2012-05-10 14:49)
------------------------
* fix a compilation problem on the farm
* no need for Eigen in capture
* clean the tests
* add a few more dependencies
* Contributors: Vincent Rabaud

0.2.5 (2012-05-09)
------------------
* fix typo
* Merge branch 'master' of github.com:wg-perception/capture
* fix after renaming
* Merge branch 'master' of github.com:wg-perception/capture
* fix bad paths
* fix after renaming
* better name for egg
* fix a bad import
* Contributors: Maria Dimashova, Vincent Rabaud, mdim

0.2.4 (2012-05-01)
------------------
* remake it catkin only
* warp the first image for a fly through
* warp the color image
* fix Python with the new stack name
* be more agnostic to the package name
* add Maria's warping
* better looking docs
* more renaming
* rename the stack
* improve indentation
* improved docs
* remove useless import
* add basic odometry
* add test that was in object_recognition_core before
* remove useless ROS dependencies
* better .gitignore
* fix the table
* prettier docs
* move the docs and some files over from object_recognition_core
* update the docs
* use the new toggle directive
* comply to the new ecto_ros
* be more ROS independent
* clean the CMake file and make it return right away if catkin is not found
* fix the docs
* update the docs
* remove useless folder
* link properly to or_core
* Contributors: Vincent Rabaud

0.2.3 (2012-04-10)
------------------
* rename by prepending or
* fix typo
* add one more Python folder
* let cakin handle the Python and make sure the tests pass
* no need for loadpybindings anymore
* fix the auto formatting
* comply to the new ImageSaver API
* fix the bad install
* fix some install path issues
* make sure the tests pass
* catkinize the project and make sure the tests pass
* reenable the tests
* build libraries in a cleaner hierarchy
* do not use catkin sphinx anymore
* rename object_recognition to object_recognition_core
* use proper catkin_sphinx
* use catkin for the docs too
* make sure it works with catkin and fuerte
* fix the docs to be more up-to-date
* use the new --help macros
* add docs about how to change the resolution of the Kinect
* make sure to install the capture scripts
* use the new location of LshMatcher
* use highres... will break action pro...
* Adding docs.
* Merge branch 'master' of git://github.com/wg-perception/capture
* pep8 conform.
* Merge branch 'master' of git://github.com/wg-perception/capture
* move some tests from object_recognition
* Adding sphinx conf.
* Testing.
* Ignores.
* Working on capture, checkpoint.
* Working on a bit of sample.
* Adding some orb tests.
* Reorg.
* A readme.
* Migration.
* Bringing capture into its own thing.
* Contributors: Ethan, Ethan Rublee, Vincent Rabaud
