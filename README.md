# Group13

Set Up:

Install Webots https://github.com/cyberbotics/webots

```
cd "your_project_path"\libraries
make clean
make

cp youbot_control\youbot_control.dll ..\controllers\youbot\

cd "your_project_path"\controllers\youbot\
make clean
make
```

This is a Windows set up. To make it work on other OS you need to change makefiles
