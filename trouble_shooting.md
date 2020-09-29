# RSS Practical 2020 - Trouble shooting
Setting up a new environment can be painful sometimes. In fact, it is easier to set up with a clean environment - a system without Python or conda. If you decide to migrate to Ubuntu 18.04, there is a step by step guide video on Learn. It is garuanteed to work if you do the same steps as shown in the video.

- [RSS Practical 2020 - Trouble shooting](#rss-practical-2020---trouble-shooting)
  - [General advice](#general-advice)
  - [Possible situations](#possible-situations)
    - [1. Command not found: conda](#1-command-not-found-conda)
    - [2. Failed while installing Pybullet](#2-failed-while-installing-pybullet)
  - [Dependency list](#dependency-list)

## General advice 
1. Make sure your system is up-to-date before each installation procedure
   * To upgrade Ubuntu: **sudo apt update && sudo apt upgrade**
   * To upgrade conda: **conda upgrade conda**
   * To upgrade pip3: **python3 -m pip install --upgrade pip**
2. Read carefully the important messages
   * Don't change the default directories if you are not sure
   * Read **slowly and carefully** through the error message
3. Try yourself to install a particular package for you system 
   * There may be some packages that are not avaliable for your system or there exist some conflictions for installing a package. In this case, the fast way is to identify the problem before you try other methods or turorials.
4. Especially for this practical, you should stick with Pybullet 2.8.7
5. If you are using ROS, then start a new terminal and deactive conda as they conflict with each other

</br>

## Possible situations
Here are the solutions to some situations that I have learnt from the past, you could try the ones that are similar to your situation.

### 1. Command not found: conda
Your terminal needs the directory of conda in order to run it. Conda provides a set-up wizard *init* to initialise the PATH variable (append directory to conda). You should run this by enter *yes* on the last step of conda installation. Or, you could navigate to the executable file conda and run **./conda init** from there. The default installation directory for current user only is "/home/YOUR_USERNAME/anaconda3/bin". So the complete bash commands are:
```
$ cd /home/YOUR_USERNAME/anaconda3/bin 
$ cd /home/YOUR_USERNAME/miniconda3/bin # if you install Miniconda
$ ./conda init
```
The directory of global installation is "/opt/anaconda3/bin", so the commands should be:
```
$ cd /opt/anaconda3/bin
$ cd /opt/miniconda3/bin # if you install Miniconda
$ ./conda init
```
Alternatively, you could [export](https://www.tutorialspoint.com/unix_commands/export.htm) the PATH appended with the path to conda:
```
$ export PATH="PATH_TO_CONDA:$PATH" 
$ conda init
```
</br>

### 2. Failed while installing Pybullet
If, for some reasons, conda or pip has failed while installing Pybullet, you could try to install it explicitly with pip3. 

You should already have pip3 installed, but just in case you don't, here is how to install it: (if you don't even have python3.6 installed, please go back to the README.md for more info)
```
$ wget https://bootstrap.pypa.io/get-pip.py -P /tmp 
$ python3 /tmp/get-pip.py
```
Update pip3:
```
$ pip3 --version
$ python3 -m pip install --upgrade pip
$ pip3 --version
```
You should see the pip version has increased after upgrade.

Now, let's try to install Pybullet 2.8.7:
```
$ conda activate RSS_Practical # skip this if you don't use conda
$ python3 -m pip install pybullet==2.8.7
```
Validate your installation by:
```
$ python3 -m pip freeze | grep pybullet

or

$ python3 -c "import pybullet"
and you will see something like "pybullet build time: ...... "
```

</br>

## Dependency list
| Package    |   Version   |
|------------|-------------|
| Pybullet   | 2\.8\.7     |
| Scipy      | >= 1\.5\.2  |
| Numpy      | >= 1\.19\.1 |
| Matplotlib | >= 3\.3\.1  |
| PyYAML     | >= 5\.3\.1  |

