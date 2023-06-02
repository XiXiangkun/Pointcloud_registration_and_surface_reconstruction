## Pointcloud_registration_and_surface_reconstruction
### 基于VS2015，openCV2.4.13.6，PCL1.8.1实现的点云配准和曲面重建一些算法的实现效果示例

### 2023.06.02更新
很抱歉，我最近才注意到这些issues。
当时源代码文件中的一些文件超过了允许上传文件的最大限制，我觉得无所谓，所以只上传了部分代码文件和可执行文件。
我已经没有这个项目的环境了，甚至编码环境也不再是windows操作系统了。
我是有想过尝试重新搭建一段环境运行项目，但是我回过头看了下源码工程，emmmmmm，一言难尽。
放到现在来看，我觉得这些代码内容实在是惨不忍睹，而且没有维护的必要，所以放弃了重新构建的念头。

### 如果你真的需要
项目大概的思路就是运行pcl库对点云数据进行的处理，点云配准+曲面重建的过程基本都是基于现有接口实现的，我个人应该没有进行大的内容修整。
参考书目是**《点云库pcl从入门到精通》**

我曾经的笔记里有几个要注意的点，大家可以参考：
* dll配置过但是文件丢失，可以去vs2015\PCL\PCL 1.8.1\bin目录中找到并复制到C:\Windows\System32
* 点云格式使用的是PointXYZ

#### 文件内容
ConsoleApplication1.cpp是算法使用的代码，main函数注释为测试用例
.pcd文件为使用的点云数据文件
bun_zipper和bun_zipper_t等主要为小兔子模型
dy.py是窗体应用的软件设计，使用tkinter实现
dy.exe基于ConsoleApplication1.exe完成软件功能

---
### Note：
Sorry for nelgecting those issues for like 3 years.
This project is really awful and has no need for mainteinance.
