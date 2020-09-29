# RSS Practical 2020

![Git](https://github.com/favicon.ico)

```
$ git clone https://github.com/john-zcliu/RSS_Practical_2020.git
$ git clone git@github.com:john-zcliu/RSS_Practical_2020.git # if you have logged in
$ cd ./RSS_Practical_2020
```

<p align="center">
    <img height="300" src="/Nextage_robot.png">
</p>

## Setting up the environment:
NOTE: This tutorial is only guaranteed to work with Ubuntu >=18.04
From here, you should be under ./RSS_Practical_2020 directory.

### Option 1: Setup with conda
Make sure you have either miniconda or anaconda installed. If not, you can follow the instructions on the [website](https://docs.conda.io/projects/conda/en/latest/user-guide/install/) to get conda. To make sure your conda is up-to-date, run:
```
$ conda update conda
```

Conda automatically installs everything you need and create an isolated envirnoment. Run following code in your terminal to create a conda environment called RSS_Practical. 
```
$ conda env create --file environment.yml
$ # If you wish to name it differently, change <YOURENVNAME> with following command: 
$ conda env create --name <YOURENVNAME> --file environment.yml
```

When it is all finished, run following command to start your conda environment and you are good to go!
```
$ conda activate RSS_Practical
```

### Option 2: Setup up with pip
If you are not a fan of conda, you can get everything working with pip, too! But, you will have to manually install some packages yourself which is not so hard.
First, let’s install python 3.6 and pip3. Run following commands:
```
$ sudo apt update && sudo apt upgrade
$ sudo apt install python3.6
$ wget https://bootstrap.pypa.io/get-pip.py -P /tmp 
$ python3 /tmp/get-pip.py
```
To verify your python version is 3.6.x and pip version is 20.x, run:
```
$ python3 --version
$ python3 -m pip --version
```

Now, run this command to install packages:
```
$ python3 -m pip install -r requirement.txt
```

Congratulations, you are now ready to start!

## Hello world Pybullet!
To test the installation, you can run some hello-world programs:
```
(base)$ conda activate RSS_Practical
(RSS_Practical)$ python3 hello_world.py
(RSS_Practical)$ python3 hello_Nextage.py
```


### Dependency list
| Package    |   Version   |
|------------|-------------|
| Pybullet   | 2\.8\.7     |
| Scipy      | >= 1\.5\.2  |
| Numpy      | >= 1\.19\.1 |
| Matplotlib | >= 3\.3\.1  |
| PyYAML     | >= 5\.3\.1  |


## Trouble shooting
Please see [Trouble shooting](https://github.com/john-zcliu/RSS_Practical_2020/blob/master/trouble_shooting.md)