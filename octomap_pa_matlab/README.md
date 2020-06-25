# octomap_pa_matlab package

This package only contains the matlab wrapper (class) for the
octomap_pa package.

## Matlab

Usage instructions based on
  https://de.mathworks.com/matlabcentral/answers/283695 \
"Updating existing custom message types with rosgenmsg"

### 1. Download package
Download this matlab package, which is part of the ProAut OctoMap repository.

~~~~~
    e.g.
    $ mkdir -p ~/catkin_ws/src
    $ cd       ~/catkin_ws/src
    $ git clone https://github.com/TUC-ProAut/ros_octomap
    $ cd ros_octomap
    $ pwd

~~~~~

In the remainder of the instructions, it is assumed that this path is
"REPO_PATH/".

### 2. Run rosgenmsg
Within matlab: run rosgenmsg on the folder containing the custom
message definitions.

~~~~~
    >> rosgenmsg('REPO_PATH/octomap_pa_matlab/msgs/')
~~~~~

In order to keep this instruction simple "MSGS_PATH" refers to
"REPO_PATH/octomap_pa_matlab/msgs/".

### 3. Edit the javaclasspath.txt
Follow the instructions to edit the javaclasspath.txt file.

In addition to the four JAR file paths, you also need to tell
Matlab to use these JAR files instead of the builtin ones. Add
the "before" token in front of the four JAR file paths:

~~~~~
    <before>
    MSGS_PATH/matlab_gen/jar/octomap_pa_msgs-1.3.3.jar
~~~~~

### 4. Restart Matlab
The caption says it all.

### 5. Regenerate files
Delete the previously generated Matlab files and run
rosgenmsg again. Now, it should pick up the new definitions:

~~~~~
    >> rmdir('MSGS_PATH/matlab_gen/', 's')
    >> rosgenmsg('MSGS_PATH')
~~~~~

### 6. done
You should now be able to use these new message definitions.
