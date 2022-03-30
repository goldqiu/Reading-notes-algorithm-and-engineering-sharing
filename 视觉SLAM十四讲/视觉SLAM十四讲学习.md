# 视觉SLAM十四讲学习

## 第一讲：预备知识

### SLAM介绍

SLAM 是 Simultaneous Localization and Mapping 的缩写，中文译作“同时定位与地图构建”。它是指搭载特定传感器的主体，在没有环境先验信息的情况下，于运动过程中建立环境的模型，同时估计自己的运动。如果这里的传感器主要为相机，那就称为“视觉 SLAM”。

SLAM的目的是解决“定位”与“地图构建”这两个问题。也就是说，一边要估计传感器自身的位置，一边要建立周围环境的模型。当用相机作为传感器时，要做的就是根据一张张连续运动的图像（它们形成一段视频），从中推断相机的运动，以及周围环境的情况。

这本书将完整的SLAM系统分成几个模块：视觉里程计、后端优化、建图以及回环检测。

与SLAM相关的书籍主要有《概率机器人》（Probabilistic robotics）、《计算机视觉中的多视图几何》（Multiple View Geometry in Computer Vision）、《机器人学中的状态估计》（State Estimation for Robotics: A Matrix-Lie-Group Approach）等。

这是作者在github提供的书中源码。

https://github.com/gaoxiang12/slambook

https://github.com/gaoxiang12/slambook2.git   （第二版）

### 全书内容

分两个部分：

第一部分为数学基础篇，包括：

• 第1讲：前言，基本信息

• 第2讲：SLAM 系统概述，编程环境的搭建过程。

• 第3讲：三维空间运动，旋转矩阵、四元数、欧拉角，Eigen库。

• 第4讲：李群和李代数，Sophus 。

• 第5讲：针孔相机模型、图像、OpenCV 

• 第6讲：非线性优化，状态估计理论基础、最小二乘问题、梯度下降方法，Ceres 和 g2o 

第二部分为 SLAM 技术篇，包括：

• 第7讲：特征点法的视觉里程计，特征点的提取与匹配、对极几何约束的计算、PnP 和 ICP 等。

• 第8讲：直接法的视觉里程计，光流。

• 第9讲：后端优化，Bundle Adjustment，Ceres和g2o。

• 第10讲：后端优化中的位姿图。

• 第11讲：回环检测，词袋方法，dbow3

• 第12讲：地图构建，极线搜索与块匹配，点云地图和八叉树地图的构建。

• 第13讲：搭建一个双目视觉里程计框架

• 第14讲：介绍开源 SLAM 项目以及未来的发展方向。

### 掌握基础：

高等数学、线性代数、概率论。

C++ 语言基础

Linux 基础

### 习题：

高斯分布的一维形式？高维形式？

https://blog.csdn.net/weixin_34129696/article/details/93156160

C++11 标准？C++11新特性？

http://c.biancheng.net/cplus/11/

Linux 的目录结构是什么样的？

https://www.runoob.com/linux/linux-system-contents.html



## 第二讲：初识SLAM

“定位”和“建图”，可以看成感知的“内外之分”。

### 传感器分类：

传感器分为两类: 一类传感器是携带于机器人本体上的，例如机器人的轮式编码器、相机、激光等等。另一类是安装于环境中的，例如导轨、二维码标志等等。安装于环境中的传感设备，通常能够直接测量到机器人的位置信息，简单有效地解决定位问题。然而，由于它们必须在环境中设置，在一定程度上限制了机器人的使用范围。比方说，有些地方没有GPS信号，有些地方无法铺设导轨。

这类传感器约束了外部环境。只有在这些约束满足时，基于它们的定位方案才能工作。虽然这类传感器简单可靠，但它们无法提供一个普遍的，通用的解决方案。相对的，那些携带于机器人本体上的传感器，比如激光传感器、相机、轮式编码器、惯性测量单元（Inertial Measurement）等等，它们测到的通常都是一些间接的物理量而不是直接的位置数据。例如，轮式编码器会测到轮子转动的角度、IMU测量运动的角速度和加速度，相机和激光则读取外部环境的某种观测数据。我们只能通过一些间接的手段，从这些数据推算自己的位置。明显的好处是，它没有对环境提出任何要求，使得这种定位方案可适用于未知环境。

按照相机的工作方式，我们把相机分为单目（Monocular）、双目（Stereo）和深度相机（RGB-D）三个大类。直观看来，单目相机只有一个摄像头，双目有两个，而 RGB-D原理较复杂，除了能够采集到彩色图片之外，还能读出每个像素离相机的距离。此外，SLAM 中还有全景相机、Event 相机等。

#### 单目相机：

只使用一个摄像头进行SLAM的做法称为单目SLAM（Monocular SLAM）。

照片，本质上是拍照时的场景（Scene），在相机的成像平面上留下的一个投影。它以二维的形式反映了三维的世界。显然，这个过程丢掉了场景的一个维度：也就是所谓的深度（或距离）。在单目相机中，我们无法通过单个图片来计算场景中物体离我们的距离（远近），这个距离是SLAM中非常关键的信息。

在单张图像里，你无法确定一个物体的真实大小。它可能是一个很大但很远的物体，也可能是一个很近但很小的物体。由于近大远小的原因，它们可能在图像中变成同样大小的样子。

由于单目相机只是三维空间的二维投影，所以，如果我们想恢复三维结构，必须移动相机的视角。在单目SLAM中也是同样的原理。我们必须移动相机之后，才能估计它的运动（Motion），同时估计场景中物体的远近和大小。

那么，怎么估计这些运动和结构呢？从生活经验中我们知道，如果相机往右移动，那么图像里的东西就会往左边移动——这就给我们推测运动带来了信息。另一方面，我们还知道近处的物体移动快，远处的物体则运动缓慢。于是，当相机移动时，这些物体在图像上的运动，形成了视差。通过视差，我们就能定量地判断哪些物体离得远，哪些物体离的近。

然而，即使我们知道了物体远近，它们仍然只是一个相对的值。如果把相机的运动和场景大小同时放大两倍，单目所看到的像是一样的。同样的，把这个大小乘以任意倍数，我们都将看到一样的景象。这说明了单目SLAM 估计的轨迹和地图，将与真实的轨迹、地图，相差一个因子，也就是所谓的尺度（Scale）。由于单目 SLAM 无法仅凭图像确定这个真实尺度，所以又称为尺度不确定性。 平移之后才能计算深度，以及无法确定真实尺度，这两件事情给单目SLAM 的应用造成了很大的麻烦。后面为了得到这个深度，开始使用双目和深度相机。

双目相机和深度相机的目的，在于通过某种手段测量物体离我们的距离，克服单目无法知道距离的缺点。如果知道了距离，场景的三维结构就可以通过单个图像恢复出来，也就消除了尺度不确定性。

#### 双目相机：

由两个单目相机组成，两个相机之间的距离（称为基线（Baseline））是已知的。我们通过这个基线来估计每个像素的空间位置。人类可以通过左右眼图像的差异，判断物体的远近，在计算机上也是同样的道理。计算机上的双目相机需要大量的计算才能估计每一个像素点的深度，相比于人类是非常的笨拙。双目相机测量到的深度范围与基线相关。基线距离越大，能够测量到的就越远。双目相机的距离估计是比较左右眼的图像获得的，并不依赖其他传感设备，所以它既可以应用在室内，也可应用于室外。双目或多目相机的缺点是配置与标定较为复杂，其深度量程和精度受双目的基线与分辨率限制，而且视差的计算非常消耗计算资源，需要使用 GPU 和 FPGA 设备加速后，才能实时输出整张图像的距离信息。

#### 深度相机

又称 RGB-D相机，可以通过红外结构光或 Time-of-Flight（ToF）原理，像激光传感器那样，通过主动向物体发射光并接收返回的光，测出物体离相机的距离。 不像双目那样通过软件计算来解决，而是通过物理的测量手段，可节省大量的计算量。目前常用的 RGB-D 相机包括Kinect/Kinect V2、 Realsense等。不过，现在多数 RGB-D 相机还存在测量范围窄、噪声大、视野小、易受日光干扰、无法测量透射材质等诸多问题，在SLAM方面，主要用于室内SLAM。

### 视觉SLAM流程

1. 传感器信息读取。在视觉 SLAM 中主要为相机图像信息的读取和预处理。如果在机器人中，还可能有测速码盘、惯性传感器等信息的读取和同步。 

2. 视觉里程计 (Visual Odometry, VO)。任务是估算相邻图像间相机的运动， 以及局部地图的构建。VO 又称为前端（Front End）。 

3. 后端优化（Optimization）。后端接受不同时刻视觉里程计测量的相机位姿，以及回环检测的信息，对它们进行优化，得到全局一致的轨迹和地图。由于接在VO之后， 又称为后端（Back End）。 

4. 回环检测（Loop Closing）。判断机器人是否曾经到达过先前的位置。如果检测到回环，它会把信息提供给后端进行处理。 

5. 建图（Mapping）。它根据估计的轨迹，建立与任务要求对应的地图。

![视觉SLAM十四讲-1](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-1.png)

#### 视觉里程计

视觉里程计关心相邻图像之间的相机运动，最简单的情况是两张图像之间的运动关系。

计算机是如何通过图像确定相机的运动呢？图像在计算机里只是一个数值矩阵。这个矩阵里表达着什么东西，计算机毫无概念（这也正是现在机器学习要解决的问题）。

视觉 SLAM 中，我们只能看到一个个像素，知道它们是某些空间点在相机的成像平面上投影的结果。所以，为了定量地估计相机运动，必须在了解相机与空间点的几何关系之后进行。 VO能够通过相邻帧间的图像估计相机运动，并恢复场景的空间结构。VO只计算相邻时刻的运动，而和往前的信息没有关联。假定我们已有了一个视觉里程计，估计了两张图像间的相机运动。那么，只要把相邻时刻的运动串起来，就构成了机器人的运动轨迹，从而解决了定位问题。另一方面，我们根据每个时刻的相机位置，计算出各像素对应的空间点的位置，就得到了地图。

但是，有了VO，并没有解决SLAM问题。

仅通过视觉里程计来估计轨迹，将出现累计漂移（Accumulating Drift）。这是由于视觉里程计（在最简单的情况下）只估计两个图像间运动造成的。每次估计都带有一定的误差，而由于里程计的工作方式，先前时刻的误差将会传递到下一时刻， 导致经过一段时间之后，估计的轨迹将不再准确。比方说，机器人先向左转90度，再向右 转了90度。由于误差，我们把第一个90度估计成了89度。向右转之后机器人的估计位置并没有回到原点。即使之后的估计再准确，与真实值相比，都会带上这-1度的误差。这也就是所谓的漂移（Drift）。这将导致我们无法建立一致的地图。为了解决漂移问题，我们还需要两种技术：后端优化和回环检测。回环检测负责把机器人回到原始位置检测出来，而后端优化则根据该信息，校正整个轨迹的形状。

#### 后端优化：

主要指处理SLAM过程中噪声的问题。现实中，再精确的传感器也带有一定的噪声。便宜的传感器测量误差较大，昂贵的则较小，有的传感器还会受磁场、温度的影响。

所以，除了解决如何从图像估计出相机运动之外，我们还要关心这个估计带有多大的噪声，这些噪声是如何从上一时刻传递到下一时刻的、而我们又对当前的估计有多大的信心。

后端优化要考虑的问题，就是如何从这些带有噪声的数据中，估计整个系统的状态，以及这个状态估计的不确定性有多大——这称为最大后验概率估计（Maximum-a-Posteriori，MAP）。这里的状态既包括机器人自身的轨迹，也包含地图。在SLAM框架中，前端给后端提供待优化的数据，以及这些数据的初始值。而后端负责整体的优化过程，它往往面对的只有数据，不必关心这些数据到底来自什么传感器。在视觉 SLAM 中，前端和计算机视觉研究领域更为相关，比如图像的特征提取与匹配等，后端则主要是滤波与非线性优化算法。早期的SLAM问题是一个状态估计问题，这正是后端优化要解决的东西。

SLAM 问题的本质：对运动主体自身和周围环境空间不确定性的估计。为了解决SLAM，我们需要状态估计理论，把定位和建图的不确定性表达出来，然后采用滤波器或非线性优化，去估计状态的均值和不确定性（方差）。

#### 回环检测

又称闭环检测（Loop Closure Detection），主要解决位置估计随时间漂移的问题。假设实际情况下，机器人经过一段时间运动后回到了原点，但是由于漂移，它的位置估计值却没有回到原点。

如果有某种手段，让机器人知道回到了原点这件事，或者把原点识别出来，我们再把位置估计值拉过去，就可以消除漂移了。这就是所谓的回环检测。我们认为，地图存在的主要意义，是为了让机器人知晓自己到达过的地方。为了实现回环检测，我们需要让机器人具有识别曾到达过的场景的能力。我们更希望机器人能使用携带的传感器——也就是图像本身，来完成这一任务。例如，我们可以判断图像间的相似性，来完成回环检测。如果回环检测成功，可以显著地减小累积误差。所以视觉回环检测，实质上是一种计算图像数据相似性的算法。由于图像的信息非常丰富，使得正确检测回环的难度也降低了不少。在检测到回环之后，我们会把“A 与 B 是同一个点”这样的信息告诉后端优化算法。然后，后端根据这些新的信息，把轨迹和地图调整到符合回环检测结果的样子。这样，如果我们有充分而且正确的回环检测，就可以消除累积误差，得到全局一致的轨迹和地图。

#### 建图

指构建地图的过程。地图是对环境的描述，但这个描述并不是固定的，需要视SLAM的应用而定。

对于家用扫地机器人来说，这种主要在低矮平面里运动的机器人，只需要一个二维的地图，标记哪里可以通过，哪里存在障碍物，就够它在一定范围内导航了。

而对于一个相机，它有六自由度的运动，我们至少需要一个三维的地图。有时我们不需要地图，或者地图可以由其他人提供，例如行驶的车辆往往可以得到已经绘制好的当地地图。建图并没有一个固定的形式和算法。一组空间点的集合也可以称为地图，一个漂亮的3D模型也是地图，一个标记着城市、村庄、铁路、河道的图片也是地图。地图的形式随SLAM的应用场合而定。大体上讲，它们可以分为度量地图与拓扑地图两种。

度量地图（Metric Map）强调精确地表示地图中物体的位置关系，通常我们用稀疏（Sparse）与稠密（Dense）对它们进行分类。稀疏地图进行了一定程度的抽象，并不需要表达所有的物体。例如，我们选择一部分具有代表意义的东西，称之为路标（Landmark），那么一张稀疏地图就是由路标组成的地图，而不是路标的部分就可以忽略掉。

稠密地图着重于建模所有看到的东西。对于定位来说，稀疏路标地图就足够了。而用于导航时，我们往往需要稠密的地图。稠密地图通常按照某种分辨率，由许多个小块组成。二维度量地图是许多个小格子（Grid），三维则是许多小方块（Voxel）。一般地，一个小块含有占据、空闲、未知三种状态，以表达该格内是否有物体。当我们查询某个空间位置时，地图能够给出该位置是否可以通过的信息。这样的地图可以用于各种导航算法。这种地图需要存储每一个格点的状态，耗费大量的存储空间，而且多数情况下地图的许多细节部分是无用的。另一方面，大规模度量地图有时会出现一致性问题。很小的一点转向误差，可能会导致两间屋子的墙出现重叠，使得地图失效。

拓扑地图（Topological Map）相比于度量地图的精确性，拓扑地图则更强调地图元素之间的关系。拓扑地图是一个图（Graph），由节点和边组成，只考虑节点间的连通性，例如 A，B 点是连通的，而不考虑如何从A点到达B点的过程。它放松了地图对精确位置的需要，去掉地图的细节问题，是一种更为紧凑的表达方式。拓扑地图不擅长表达具有复杂结构的地图。

如何对地图进行分割形成结点与边，又如何使用拓扑地图进行导航与路径规划，仍是有待研究的问题。

### SLAM 问题的数学表述

1. 什么是运动？要考虑从k−1时刻到k时刻，机器人的位置x是如何变化的。

机器人会携带一个测量自身运动的传感器，比如说码盘或惯性传感器。这个传感器可以测量有关运动的读数，但不一定直接是位置之差，还可能是加速度、角速度等信息。无论是什么传感器，我们都能使用一个通用的、抽象的数学模型： xk = f (xk−1,uk, wk).

这里uk是运动传感器的输入，wk 为噪声。这称为运动方程。

 

2. 什么是观测？假设机器人在k时刻，于xk处探测到了某一个路标yj，要考虑这件事情是如何用数学语言来描述的。

观测方程描述的是，当机器人在xk位置上看到某个路标点yj，产生了一个观测数据zk,j。用一个抽象的函数h来描述这个关系： zk,j = h (yj , xk, vk,j )。

vk,j是这次观测里的噪声。

 

根据机器人的真实运动和传感器的种类，存在着若干种参数化方式（Parameterization）。

假设机器人在平面中运动，那么，它的位姿由两个位置和一个转角来描述，x1, x2 是两个轴上的位置而 θ 为转角，即

![视觉SLAM十四讲-2](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-2.jpg)

输入的指令是两个时间间隔位置和转角的变化量：

![视觉SLAM十四讲-3](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-3.jpg)

此时运动方程：

![视觉SLAM十四讲-4](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-4.jpg)

并不是所有的输入指令都是位移和角度的变化量，比如“油门”或者 “控制杆”的输入就是速度或加速度量，存在着其他形式更加复杂的运动方程，我们需要进行动力学分析。关于观测方程，比方说机器人携带着一个二维激光传感器。我们知道激光传感器观测一个2D路标点时，能够测到两个量：路标点与机器人本体之间的距离 r 和夹角 ϕ。



记路标点为

![视觉SLAM十四讲-5](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-5.jpg)

位姿为

![视觉SLAM十四讲-6](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-6.jpg)

观测数据为

![视觉SLAM十四讲-7](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-7.jpg)

那么观测方程就为：

![视觉SLAM十四讲-8](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-8.jpg)

考虑视觉 SLAM 时，传感器是相机，那么观测方程就是“对路标点拍摄后，得到了图像中的像素”的过程。 针对不同的传感器，这两个方程有不同的参数化形式。

取成通用的抽象形式，SLAM过程可总结为两个基本方程：

![视觉SLAM十四讲-9](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-9.jpg)

这两个方程描述了最基本的SLAM问题：当我们知道运动测量的读数u，以及传感器的读数z 时，如何求解定位问题（估计x）和建图问题（估计y）？这时，我们把 SLAM 问题建模成了一个状态估计问题：如何通过带有噪声的测量数据，估计内部的、隐藏着的状态变量？

SLAM 状态估计问题的求解，与两个方程的具体形式，以及噪声服从哪种分布有关。

我们按照运动和观测方程是否为线性，噪声是否服从高斯分布进行分类，分为线性/非线性和高斯/非高斯系统。其中线性高斯系统（Linear Gaussian, LG 系统）是最简单的，它的无偏的最优估计可以由卡尔曼滤波器（Kalman Filter, KF）给出。

而在复杂的非线性非高斯系统 （Non-Linear Non-Gaussian，NLNG 系统）中，我们会使用以扩展卡尔曼滤波器（Extended Kalman Filter, EKF）和非线性优化两大类方法去求解它。

直至21世纪早期，以 EKF 为主的滤波器方法占据了 SLAM 中的主导地位。在工作点处把系统线性化，并以预测 ——更新两大步骤进行求解。最早的实时视觉SLAM系统即是基于EKF开发的。随后，为了克服 EKF 的缺点（例如线性化误差和噪声高斯分布假设），人们开始使用粒子滤波器（Particle Filter）等其他滤波器，乃至使用非线性优化的方法。

时至今日，主流视觉 SLAM 使用以图优化（Graph Optimization）为代表的优化技术进行状态估计。优化技术已经明显优于滤波器技术，只要计算资源允许，我们通常都偏向于使用优化方法。

机器人更多时候是一个三维空间里的机器人。三维空间的运动由三个轴构成，所以机器人的运动要由三个轴上的平移，以及绕着三个轴的旋转来描述，这一共有六个自由度。在视觉SLAM 中，对6自由度的位姿如何表达，优化。观测方程如何参数化，即空间中的路标点是如何投影到一张照片上的，这需要解释相机的成像模型。最后，怎么求解上述方程？这需要非线性优化的知识。



### 编程基础：

#### linux编程入门

新建一个cpp文件，命名为helloSLAM.cpp

代码如下：

```c++
#include <iostream> 
using namespace std;
int main( int argc, char** argv )
{ 
      cout<<"Hello SLAM!"<<endl; return 0;
}
```

用g++编译器对其编译

命令行敲：

```shell
g++ helloSLAM.cpp
```

输出a.out文件

命令行敲：

```
./a.out 
```

即可运行此程序

窗口会输出

```
Hello SLAM!
```

如果仅靠g++命令，我们需要输入大量的编译指令，整个编译过程会变得异常繁琐。所以我们使用cmake。

在同个文件夹下新建一个 CMakeLists.txt 文件。输入如下：

```xml
# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )
# 声明一个 cmake 工程 project( HelloSLAM )
# 添加一个可执行程序
# 语法：add_executable( 程序名源代码文件）
add_executable( helloSLAM helloSLAM.cpp )
```

终端命令行输入： 

```
cmake .
```

生成了MakeFile ，它是系统自动生成的编译指令文件。

同样执行可执行文件helloSLAM ，会有同样效果。

但是还有很多中间文件，我们可以建立一个build文件夹，在这个文件夹下面cmake ..

用CMAKE的好处就是在大型项目中很方便的能够对项目进行编译管理。

下面使用自己打包的库：书写一个 libHelloSLAM.cpp 文件写下：

```c++
//这是一个库文件
#include <iostream>
using namespace std; 
void printHello()
{ 
   cout<<"Hello SLAM"<<endl;
}
```

它没有main函数，这意味着这个库中没有可执行文件。我们在 CMakeLists.txt 里加一句：

```
add_library( hello libHelloSLAM.cpp )
```

重新cmake 编译后，生成一个 libhello.a 文件。        

在Linux中，库文件分成静态库和共享库（动态库）两种。静态库以.a作为后缀名，共享库以.so结尾。所有库都是一些函数打包后的集合，差别在于静态库每次被调用都会生成一个副本，而共享库则只有一个副本，更省空间。如果我们想生成共享库而不是静态库，只需用：

```
add_library( hello_shared SHARED libHelloSLAM.cpp
```

此时得到的文件是 libhello_shared.so

库文件是一个压缩包，里头带有编译好的二进制函数。不过，仅有.a或.so库文件的话，我们并不知道它里头的函数到底是什么，调用的形式又是什么样的。为了让别人（或者自己）使用这个库，我们需要提供一个头文件，说明这些库里都有些什么。因此，对于库的使用者，只要拿到了头文件和库文件，就可以调用这个库了。

下面写 libhello 的头文件libHelloSLAM.h

```c++
#ifndef LIBHELLOSLAM_H_ 
#define LIBHELLOSLAM_H_ 
void printHello();
#endif
```

写一个可执行程序useHello.cpp，调用这个简单的函数。

```c++
#include "libHelloSLAM.h" 
int main( int argc, char** argv )
{ 
  printHello(); 
  return 0; 
}
```

在CMakeLists.txt中添加一个可执行程序的生成命令，链接到刚才我们使用的库上： 

```cmake
add_executable( useHello useHello.cpp ) 
target_link_libraries( useHello hello_shared )
```

通过这两句话，useHello程序就能顺利使用hello_shared库中的代码了。这个小例子演示了如何生成并调用一个库。对于他人提供的库，我们也可用同样的方式对它们进行调用，整合到自己的程序中。

总结：

1. 首先，程序代码由头文件和源文件组成； 

2. 带有main函数的源文件编译成可执行程序，其他的编译成库文件。 

3. 如果可执行程序想调用库文件中的函数，它需要参考该库提供的头文件，以明白调用的格式。同时，要把可执行程序链接到库文件上。

#### 开发环境：

##### Clion：

Clion 相比于 Kdevelop和qtcreator来说cmake支持更加完善一些，但是它需要正版帐号。我们可以用学生邮箱（带edu）免费使用一年。

下面是官网：

https://www.jetbrains.com/idea/buy/#discounts?billing=yearly

选择For students and teachers下的 learn more

![视觉SLAM十四讲-10](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-10.jpg)

点击APPLY NOW

![视觉SLAM十四讲-11](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-11.jpg)

输入学生邮箱，验证完成后 他会发验证链接到邮箱，打开邮箱内链接，注册JBA

在注册JBA时候还需要你输入一个邮箱，这个时候就用自己的常用邮箱就可以了。

![视觉SLAM十四讲-12](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-12.jpg)

注册成功如下：

![视觉SLAM十四讲-13](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-13.jpg)

回到开始的窗口 ，激活CLion

![视觉SLAM十四讲-18](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-18.jpg)

**现在是调试cmakelists项目：**

打开cmakelists文件，导入项目

![视觉SLAM十四讲-14](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-14.jpg)

需要选择你要debug的源文件和工作路径

![视觉SLAM十四讲-15](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-15.jpg)

将编译输出可执行文件路径选好：

![视觉SLAM十四讲-16](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-16.jpg)

可以开始调试了：

![视觉SLAM十四讲-17](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-17.jpg)

##### **Kdevelop ：**

这个ide是免费的，但是不太好用，用不习惯。

##### **Qtcreator：**

这个IDE挺好用的，但是比较适合用qmake管理项目，特别适合做上位机，也可以用cmake管理项目，但是没有clion好用。在校学生建议还是用clion，有免费版。

##### **Vscode：**

这个ide也很适合开发cmake和ros。

**注：后面有空写ROS开发环境搭建的教程，使用clion或者vscode。**

### 课后作业

1. 书中文献阅读：

[1] L. Haomin, Z. Guofeng, and B. Hujun, “A survey of monocular simultaneous localization and mapping,” Journal of Computer-Aided Design and Compute Graphics, vol. 28, no. 6, pp. 855–868, 2016. in Chinese.

[2] A. Davison, I. Reid, N. Molton, and O. Stasse, “Monoslam: Real-time single camera SLAM,” IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 29, no. 6, pp. 1052–1067, 2007.

[3] R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision. Cambridge university press, 2003.

[4] R. C. Smith and P. Cheeseman, “On the representation and estimation of spatial uncertainty,” International Journal of Robotics Research, vol. 5, no. 4, pp. 56–68, 1986.

[5] S. Thrun, W. Burgard, and D. Fox, Probabilistic robotics. MIT Press, 2005.

[6] T. Barfoot, “State estimation for robotics: A matrix lie group approach,” 2016.

[7] A. Pretto, E. Menegatti, and E. Pagello, “Omnidirectional dense large-scale mapping and navigation based on meaningful triangulation,” 2011 IEEE International Conference on Robotics and Automation (ICRA 2011),pp. 3289–96, 2011.

[8] B. Rueckauer and T. Delbruck, “Evaluation of event-based algorithms for optical flow with ground-truth from inertial measurement sensor,” Frontiers in neuroscience, vol. 10, 2016.

[9] C. Cesar, L. Carlone, H. C., Y. Latif, D. Scaramuzza, J. Neira, I. D. Reid, and L. John J., “Past, present, and future of simultaneous localization and mapping: Towards the robust-perception age,” arXiv preprint arXiv:1606.05830,2016.

[10] P. Newman and K. Ho, “Slam-loop closing with visually salient features,” in proceedings of the 2005 IEEE International Conference on Robotics and Automation, pp. 635–642, IEEE, 2005.

[11] R. Smith, M. Self, and P. Cheeseman, “Estimating uncertain spatial relationships in robotics,” in Autonomous robot vehicles, pp. 167–193, Springer, 1990.

[12] P. Beeson, J. Modayil, and B. Kuipers, “Factoring the mapping problem: Mobile robot map-building in the hybrid spatial semantic hierarchy,” International Journal of Robotics Research, vol. 29, no. 4, pp. 428–459, 2010.

[13] H. Strasdat, J. M. Montiel, and A. J. Davison, “Visual slam: Why filter?,” Image and Vision Computing, vol. 30, no. 2, pp. 65–77, 2012. [14] M. Liang, H. Min, and R. Luo, “Graph-based slam: A survey,” ROBOT, vol. 35, no. 4, pp. 500–512, 2013. in Chinese.

2. 综述文章阅读

[9] C. Cesar, L. Carlone, H. C., Y. Latif, D. Scaramuzza, J. Neira, I. D. Reid, and L. John J., “Past, present, and future of simultaneous localization and mapping: Towards the robust-perception age,” arXiv preprint arXiv:1606.05830,2016.

[15] J. Fuentes-Pacheco, J. Ruiz-Ascencio, and J. M. Rendón-Mancha, “Visual simultaneous localization and mapping: a survey,” Artificial Intelligence Review, vol. 43, no. 1, pp. 55–81, 2015. 

[16] J. Boal, Á. Sánchez-Miralles, and Á. Arranz, “Topological simultaneous localization and mapping: a survey,” Robotica, vol. 32, pp. 803–821, 2014. 

[17] S. Y. Chen, “Kalman filter for robot vision: A survey,” IEEE Transactions on Industrial Electronics, vol. 59, no. 11, pp. 4409–4420, 2012. [18] Z. Chen, J. Samarabandu, and R. Rodrigo, “Recent advances in simultaneous localization and map-building using computer vision,” Advanced Robotics, vol. 21, no. 3-4, pp. 233–265, 2007.

3. g++命令有哪些参数？怎么填写参数可以更改生成的程序文件名？g++的错误信息？

https://blog.csdn.net/chengqiuming/article/details/88410794

4. 阅读《cmake实践》，了解cmake的其他语法。

5.完善hello SLAM的小程序，把它做成一个小程序库，安装到本地硬盘中。然后，新建一个工程，使用find_package找这个库并调用它。 

这块目前还不需要，后面再学。

6.寻找其他cmake教学材料，深入了解cmake，例如

https://github.com/TheErk/CMake-tutorial

7. 寻找Kdevelop的官方网站，看看它还有哪些特性。试试Kdevelop的vim编辑功能，这块看需求学吧，有好的IDE可选择。

## 第三讲：三维空间刚体运动

这章主要是讲一个刚体在三维空间中的运动是如何描述的。同时还介绍线性代数库Eigen。它提供了C++中的矩阵运算，并且它的 Geometry 模块还提供了四元数等刚体运动的描述。

### 3.1. 旋转矩阵：

三维空间由三个轴组成，所以一个空间点的位置可以由三个坐标指定。考虑刚体，它不光有位置，还有自身的姿态。相机可以看成三维空间的刚体，相机位置是指相机在空间中的哪个地方，而姿态则是指相机的朝向，合起来称位姿。

#### 3.1.1点和向量，坐标系：

**点：**点是空间中的基本元素，没有长度，没有体积。

**向量：**把两个点连接起来，就构成了向量。向量可以看成从某点指向另一点的一个箭头。不要把向量与它的坐标两个概念混淆。一个向量是空间当中的一样东西，并不需要和若干个实数相关联的。只有当我们指定这个三维空间中的某个坐标系时，才可以谈论该向量在此坐标系下的坐标，也就是找到若干个实数对应这个向量。（就是向量没有坐标系也在那，有了坐标系向量就有了坐标）

用线性代数的知识来说，三维空间中的某个点的坐标也可以用R3来描述。假设在这个线性空间内，有该空间的一组基 (e1,e2,e3)，那么，任意向量a在这组基下就有一个坐标，这里 (a1,a2,a3)T 称为a在此基下的坐标。公式如下：                                          ![视觉SLAM十四讲-19](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-19.png)          

注：1.这里基就是张成这个空间的一组线性无关的向量。

2.坐标的具体取值，一是和向量本身有关，二是和坐标系（基）的选取有关。

3.坐标系通常由3个正交的坐标轴组成，非正交的很少见。通常使用右手系，给定x和y轴时，z 轴就可以通过右手法则由x × y定义出来。左手系的第3个轴与右手系的方向相反。

**向量内积：**

公式如下：

![视觉SLAM十四讲-20](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-20.png)

注：也可以描述向量间的投影关系。<a,b>指向量间夹角。

**向量外积：**

外积的结果是一个向量，它的方向垂直于这两个向量，大小为 |a||b|sin⟨a,b⟩，是两个向量张成的四边形的有向面积。

![视觉SLAM十四讲-21](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-21.png)

对于外积运算，我们引入∧符号，把a写成一个矩阵，事实上是一个反对称矩阵（Skew-symmetric matrix），你可以将 ∧记成一个反对称符号。这样就把外积a × b写成了矩阵与向量的乘法a∧b，把它变成了线性运算。这意味着任意向量都对应着唯一的一个反对称矩阵，反之亦然。

注：1.反对称矩阵满足AT = -A 

2.向量和加减法、内外积，即使在不谈论它们的坐标时也可以计算。例如，虽然内积在有坐标时，可以用两个向量的分量乘积之和表达，但是即使不知道它们的坐标时，也可以通过长度和夹角来计算二者的内积。所以两个向量的内积结果和坐标系的选取是无关的。

#### 3.1.2坐标系间的欧氏变换：

考虑运动的机器人，常见的做法是设定一个惯性坐标系（或者叫世界坐标系），可以认为它是固定不动的。相机或机器人则是一个移动坐标系。相机视野中某个向量p，它在相机坐标系下的坐标为pc，而在世界坐标系下看，它的坐标为pw，那么，这两个坐标之间是如何转换的呢？需要先得到该点针对机器人坐标系的坐标值，再根据机器人位姿变换到世界坐标系中。

两个坐标系之间的运动由一个旋转加上一个平移组成，这种运动称为刚体运动。相机运动就是一个刚体运动。刚体运动过程中，同一个向量在各个坐标系下的长度和夹角都不会发生变化。此时，我们说手机坐标系到世界坐标之间，相差了一个欧氏变换（Euclidean Transform）。对于同一个向量 p，它在世界坐标系下的坐标 pw 和在相机坐标系下的坐标 pc 是不同的。这个变换关系由变换矩阵T来描述。

欧氏变换由旋转和平移组成。首先考虑旋转，设某个单位正交基 (e1, e2, e3) 经过一次旋转变成了 (e1', e2', e3') ，那么，对于同一个向量 a（该向量并没有随着坐标系的旋转而发生运动），它在两个坐标系下的坐标为[a1, a2, a3]T和[a1', a2', a3']T。因为向量本身没变，根据坐标的定义，有：

![视觉SLAM十四讲-22](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-22.png)

为了描述两个坐标之间的关系，我们对上述等式的左右两边同时左乘

![视觉SLAM十四讲-23](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-23.png)

那么左边的系数就变成了单位矩阵，把中间的矩阵拿出来，定义成一个矩阵R。

![视觉SLAM十四讲-24](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-24.jpg)

这个矩阵由两组基之间的内积组成，刻画了旋转前后同一个向量的坐标变换关系。只要旋转是一样的，那么这个矩阵也是一样的。可以说，矩阵R描述了旋转本身。因此称为旋转矩阵（Rotation matrix）。该矩阵各分量是两个坐标系基的内积，由于基向量的长度为1，所以实际上是各基向量的夹角之余弦。所以这个矩阵也叫方向余弦矩阵（Direction Cosine matrix）。

旋转矩阵有一些特别的性质。它是一个行列式为1的正交矩阵。反之，行列式为1的正交矩阵也是一个旋转矩阵。以把 n 维旋转矩阵的集合定义如下：

![视觉SLAM十四讲-25](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-25.png)

SO(n) 是特殊正交群（Special Orthogonal Group）的意思。这个集合由n维空间的旋转矩阵组成，特别地，SO(3) 就是指三维空间的旋转。通过旋转矩阵，我们可以直接谈论两个坐标系之间的旋转变换，而不用再从基开始谈起。

由于旋转矩阵为正交矩阵，它的逆（即转置）描述了一个相反的旋转。

a′ = R−1a = RTa.

在欧氏变换中，除了旋转之外还有平移。考虑世界坐标系中的向量a，经过一次旋转（用R描述）和一次平移t后，得到了a′，把旋转和平移合到一起，有：a′ = Ra + t.

t称为平移向量。平移部分只需把平移向量加到旋转之后的坐标上。

通过上式，我们用一个旋转矩阵R和一个平移向量t完整地描述了一个欧氏空间的坐标变换关系。

定义坐标系1、坐标系2，那么向量a在两个系下坐标为 a1, a2 ，完整的写法：a1 = R12a2 + t12.

R12 是指把坐标系2的向量变换到坐标系1中。同理，如果我们要表达从1到2的旋转矩阵时，就写成R21。关于平移t12，实际对应的是坐标系1原点指向坐标系2原点的向量，在坐标系1下取的坐标，把它记作从 1到2的向量。但是反过来的t21，即从2指向1的向量在坐标系2下的坐标，并不等于−t12，这里和两个系的旋转还有关系。从向量层面来看，它们确实是反向的关系，但这两个向量的坐标值并不是相反数。

注：1.正交矩阵即逆为自身转置的矩阵。

2.行列式为1是人为定义的，实际上只要求它的行列式为 ±1，但行列式为 −1 的称为瑕旋转，即一次旋转加一次反射。

#### 3.1.3变换矩阵与齐次坐标：

上面的变换关系不是一个线性关系。这样的形式在变换多次之后会显得很繁杂。因此，我们引入齐次坐标和变换矩阵，

![视觉SLAM十四讲-26](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-26.jpg)

这是一个数学技巧：我们在一个三维向量的末尾添加 1，将其变成了四维向量，称为齐次坐标。对于这个四维向量，我们可以把旋转和平移写在一个矩阵里面，使得整个关系变成线性关系。该式中，矩阵T称为变换矩阵（Transform Matrix）。依靠齐次坐标和变换矩阵，两次变换的叠加就可以有很好的形式：

b = T1a, c = T2b ⇒ c = T2T1a.

变换矩阵T具有比较特别的结构：左上角为旋转矩阵，右侧为平移向量，左下角为0 向量，右下角为 1。这种矩阵又称为特殊欧氏群（Special Euclidean Group）：

![视觉SLAM十四讲-27](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-27.png)

与 SO(3) 一样，求解该矩阵的逆表示一个反向的变换：

![视觉SLAM十四讲-28](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-28.jpg)

同样，我们用T12 这样的写法来表示从2到1的变换。

齐次坐标和非齐次坐标之间的转换非常容易，在C++程序中可以使用运算符重载来完成这个功能，保证在程序中看到的运算是统一的。

**总结：**坐标系之间的运动由欧氏变换描述，它由平移和旋转组成。旋转可以由旋转矩阵 SO(3) 描述，而平移直接由一个R3向量描述。最后，如果将平移和旋转放在一个矩阵中，就形成了变换矩阵 SE(3)。

### 3.2实践：Eigen

代码在 slambook2/ch3/useEigen 

链接：

https://github.com/gaoxiang12/slambook

Eigen 是一个 C++ 开源线性代数库。它提供了快速的有关矩阵的线性代数运算，还包括解方程等功能。

安装：

```shell
sudo apt−get install libeigen3−dev
```

这个库由头文件和库文件组成。Eigen 头文件的默认位置在“/usr/include/eigen3/”中。如果不确定，可以输入以下命令查找：

```shell
sudo update db 
locate eigen3
```

 官方主页：

http://eigen.tuxfamily.org/index.php?title=Main_Page

相比于其他库，Eigen 的特殊之处在于，它是一个纯用头文件搭建起来的库。这意味着你只能找到它的头文件，而没有.so 或.a 那样的二进制文件。在使用时，只需引入 Eigen 的头文件即可，不需要链接库文件。  

注意：

1. Kdevelop 可能不会提示 C++ 成员运算，这是它做得不够完善导致的。Clion 则会完整地给出提示。

2. Eigen 提供的矩阵和 MATLAB 很相似，几乎所有的数据都当作矩阵来处理。但是，为了实现更好的效率，在 Eigen 中需要指定矩阵的大小和类型。对于在编译时期就知道大小的矩阵，处理起来会比动态变化大小的矩阵更快一些。因此，像旋转矩阵、变换矩阵这样的数据，完全可在编译时期确定它们的大小和数据类型。

3. Eigen 矩阵不支持自动类型提升，这和 C++ 的内建数据类型有较大差异。在 C++ 程序中，我们可以把一个 float 数据和 double 数据相加、相乘，编译器会自动把数据类型转换为最合适的那种。而在 Eigen 中，出于性能的考虑，必须显式地对矩阵类型进行转换。而如果忘了这样做，Eigen会提示你一个“YOU MIXED DIFFERENT NUMERIC TYPES ...”的编译错误。你可以尝试找一下这条信息出现在错误提示的哪个部分。如果错误信息太长最好保存到一个文件里再找。

4. 同理，在计算过程中也需要保证矩阵维数的正确性，否则会出现“YOU MIXED MATRICES OF DIFFERENT SIZES”错误。若发现 Eigen 出错，你可以直接寻找大写的部分，推测出了什么问题。

5. 可以阅读 Eigen 官网教程：

http://eigen.tuxfamily.org/dox-devel/modules.html 

来学习更多的Eigen知识。

6. 最后一段代码中比较了求逆与求 QR 分解的运行效率，你可以看看自己机器上的时间差异，两种方法是否有明显的差异？

### 3.3旋转向量和欧拉角

#### 3.3.1. 旋转向量

矩阵表示方式至少有以下几个缺点：

1.SO(3)的旋转矩阵有9个量，但一次旋转只有3个自由度。因此这种表达方式是冗余的。同理，变换矩阵用16个量表达了6自由度的变换。那么，是否有更紧凑的表示呢？

2.旋转矩阵自身带有约束：它必须是个正交矩阵，且行列式为 1。变换矩阵也是如此。当想要估计或优化一个旋转矩阵/变换矩阵时，这些约束会使得求解变得更困难。

因此，我们希望有一种方式能够紧凑地描述旋转和平移。例如，用一个三维向量表达旋转，用六维向量表达变换。事实上，任意旋转都可以用一个旋转轴和一个旋转角来刻画。于是，我们可以使用一个向量，其方向与旋转轴一致，而长度等于旋转角。这种向量称为旋转向量（或轴角/角轴，Axis-Angle），只需一个三维向量即可描述旋转。同样，对于变换矩阵，我们使用一个旋转向量和一个平移向量即可表达一次变换。这时的变量维数正好是六维。

考虑某个用R表示的旋转。如果用旋转向量来描述，假设旋转轴为一个单位长度的向量n，角度为 θ，那么向量 θn也可以描述这个旋转。

从旋转向量到旋转矩阵的转换过程由罗德里格斯公式（Rodrigues’s Formula ）表明，转换的结果 ：

![视觉SLAM十四讲-29](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-29.jpg)

符号∧是向量到反对称矩阵的转换符。反之，我们也可以计算从一个旋转矩阵到旋转向量的转换。对于转角 θ，取两边的迹，有

![视觉SLAM十四讲-30](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-30.jpg)

得：

![视觉SLAM十四讲-31](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-31.png)

关于转轴n，由于旋转轴上的向量在旋转后不发生改变，说明： 

Rn = n 

因此，转轴n是矩阵R特征值1对应的特征向量。求解此方程，再归一化，就得到了旋转轴。

也可以从“旋转轴经过旋转之后不变”的几何角度看待这个方程。

注：求迹（trace）即是求矩阵的对角线元素之和。

#### 3.3.2欧拉角

无论是旋转矩阵、旋转向量，它们虽然能描述旋转，但对我们人类是非常不直观的。当我们看到一个旋转矩阵或旋转向量时，很难想象出这个旋转究竟是什么样的。当它们变换时，我们也不知道物体是向哪个方向在转动。

而欧拉角则提供了一种非常直观的方式来描述旋转——它使用了3个分离的转角，把一个旋转分解成 3 次绕不同轴的旋转。而人类很容易理解绕单个轴旋转的过程。

但是，由于分解方式有许多种，所以欧拉角也存在着众多不同的、易于混淆的定义方法。比如说，先绕X轴旋转，再绕Y轴，最后绕Z轴，就得到了一个XYZ轴的旋转。同理，可以定义ZYZ、ZYX等旋转方式。

如果讨论得更细一些，还需要区分每次是绕固定轴旋转的，还是绕旋转之后的轴旋转的。这种定义方式上的不确定性带来了很多实用当中的困难。在特定领域内，欧拉角通常有统一的定义方式。

欧拉角当中比较常用的一种，便是用“偏航−俯仰−横滚”（yaw-pitch-roll）3个角度来描述一个旋转。它等价于ZYX轴的旋转。假设一个刚体的前方（朝向我们的方向）为X轴，右侧为Y轴，上方为Z轴。那么，ZYX转角相当于把任意旋转分解成以下3个轴上的转角：

1. 绕物体的Z轴旋转，得到偏航角 yaw；

2. 绕旋转之后的Y轴旋转，得到俯仰角 pitch；

3. 绕旋转之后的 X 轴旋转，得到横滚角 roll。

此时，可以使用 [r,p,y]T 这样一个三维的向量描述任意旋转。

大部分领域在使用欧拉角时都有各自的坐标方向和顺序上的习惯，比如东北天和右前上。

欧拉角的一个重大缺点是会碰到万向锁问题（Gimbal Lock ）：在俯仰角为 ±90◦时，第一次旋转与第三次旋转将使用同一个轴，使得系统丢失了一个自由度（由 3 次旋转变成了 2 次旋转）。这被称为奇异性问题，只要想用3个实数来表达三维旋转时，都会不可避免地碰到。

由于这种问题，欧拉角不适于插值和迭代，往往只用于人机交互中。我们很少在SLAM程序中直接使用欧拉角表达姿态，同样不会在滤波或优化中使用欧拉角表达旋转，因为它具有奇异性。如果想验证自己的算法是否有错，转换成欧拉角能够帮你快速分辨结果是否正确。

注：1.可以看看相关视频来理解万向锁。

2.旋转向量也有奇异性，发生在转角θ超过2π而产生周期性时。

### 3.4四元数

#### 3.4.1 四元数的定义

前面讲到，旋转矩阵用9个量描述3自由度的旋转，具有冗余性；欧拉角和旋转向量是紧凑的，但具有奇异性。事实上，我们找不到不带奇异性的三维向量描述方式。类似于用两个坐标表示地球表面（如经度和纬度），必定存在奇异性（纬度为 ±90◦ 时经度无意义）。

回忆以前学习过的复数。我们用复数集C表示复平面上的向量，而复数的乘法则表示复平面上的旋转：乘上复数i相当于逆时针把一个复向量旋转 90◦。

类似地，在表达三维空间旋转时，也有一种类似于复数的代数：四元数（Quaternion）。四元数是Hamilton找到的一种扩展的复数。它既是紧凑的，也没有奇异性。缺点是四元数不够直观，其运算稍复杂些。

把四元数与复数类比可以更快地理解四元数。例如，当我们想要将复平面的向量旋转 θ 角时，可以给这个复向量乘以 eiθ。这是极坐标表示的复数，它也可以写成普通的形式，只要使用欧拉公式即可：eiθ = cosθ + isinθ.

这正是一个单位长度的复数。所以，在二维情况下，旋转可以由单位复数来描述。类似地，三维旋转则可以由单位四元数来描述。

一个四元数q拥有一个实部和三个虚部：

q = q0 + q1i + q2j + q3k,

其中i,j,k为四元数的三个虚部。这三个虚部满足以下关系式：

![视觉SLAM十四讲-32](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-32.jpg)

如果把i,j,k看成三个坐标轴，那么它们与自己的乘法和复数一样，相互之间的乘法和外积一样。有时人们也用一个标量和一个向量来表达四元数：

​    q = [s,v]T ,   s = q0 ∈ R, v = [q1,q2,q3]T ∈ R3,

s称为四元数的实部，v称为它的虚部。如果一个四元数的虚部为0，称之为实四元数。反之，若它的实部为 0，则称之为虚四元数。

可以用单位四元数表示三维空间中任意一个旋转，不过这种表达方式和复数有着微妙的不同。

在复数中，乘以i意味着旋转 90◦。这是否意味着四元数中，乘i就是绕i轴旋转90◦？那么，ij=k是否意味着，先绕i转90◦，再绕j转90◦，就等于绕k转90◦？

情况并不是这样。应该是，乘以i对应着旋转180◦，这样才能保证ij=k的性质。而i2 = −1，意味着绕i轴旋转360◦后得到一个相反的东西。这个东西要旋转两周才会和它原先的样子相等。

#### 3.4.2 四元数的运算

常见的有四则运算、数乘、求逆、共轭等。

现有两个四元数qa,qb，它们的向量表示为 [sa,va]T,[sb,vb]T，或者原始四元数表示为：

qa = sa + xai + yaj + zak, qb = sb + xbi + ybj + zbk.

那么：

1. 四元数qa,qb 的加减运算为：

qa ± qb = [sa ± sb,va ± vb]T 

2. 乘法是把qa 的每一项与qb 的每项相乘，最后相加。整理可得：

![视觉SLAM十四讲-33](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-33.jpg)

写成向量形式并利用内外积运算：

![视觉SLAM十四讲-34](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-34.jpg)

在该乘法定义下，两个实的四元数乘积仍是实的，这与复数也是一致的。然而，注意到，由于最后一项外积的存在，四元数乘法通常是不可交换的，除非va 和vb 在R3 中共线，此时外积项为零。

3. 四元数的模长定义为

![视觉SLAM十四讲-35](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-35.jpg)

两个四元数乘积的模即为模的乘积。这使得单位四元数相乘后仍是单位四元数。

![视觉SLAM十四讲-36](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-36.jpg)

4. 共轭四元数的共轭是把虚部取成相反数：

![视觉SLAM十四讲-37](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-37.jpg)

四元数共轭与其本身相乘，会得到一个实四元数，其实部为模长的平方：

![视觉SLAM十四讲-38](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-38.jpg)

5. 一个四元数的逆为

![视觉SLAM十四讲-39](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-39.png)

按此定义，四元数和自己的逆的乘积为实四元数1：

![视觉SLAM十四讲-40](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-40.png)

如果q为单位四元数，其逆和共轭就是同一个量。同时，乘积的逆和矩阵有相似的性质：

![视觉SLAM十四讲-42](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-42.png)

6. 数乘

和向量相似，四元数可以与数相乘：

​    kq = [ks,kv]T .

#### 3.4.3 用四元数表示旋转

可以用四元数表达对一个点的旋转。假设一个空间三维点p = [x,y,z] ∈ R3，以及一个由单位四元数q指定的旋转。三维点p经过旋转之后变为p′。如果使用矩阵描述，那么有p′ = Rp。而如果用四元数描述旋转，它们的关系又如何来表达呢？

首先，把三维空间点用一个虚四元数来描述：p = [0,x,y,z]T = [0,v]T.

相当于把四元数的3个虚部与空间中的3个轴相对应。那么，旋转后的点p′即可表示为这样的乘积：p′ = qpq−1. 

这里的乘法均为四元数乘法，结果也是四元数。最后把p′的虚部取出，即得旋转之后点的坐标。计算结果的实部为 0，故为纯虚四元数。

#### 3.4.4 四元数到其他旋转表示的转换

任意单位四元数描述了一个旋转，该旋转亦可用旋转矩阵或旋转向量描述。四元数乘法也可以写成一种矩阵的乘法。设q = [s,v]T，那么，定义如下的符号 + 和 ⊕ 为：

​	![视觉SLAM十四讲-43](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-43.png)

这两个符号将四元数映射成为一个 4×4 的矩阵。于是四元数乘法可以写成矩阵的形式：

![视觉SLAM十四讲-44](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-44.png)

同理：

![视觉SLAM十四讲-45](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-45.png)

考虑使用四元数对空间点进行旋转的问题。有：

![视觉SLAM十四讲-46](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-46.png)

代入两个符号对应的矩阵，得：

![视觉SLAM十四讲-47](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-47.png)

因为p′和p都是虚四元数，那么事实上该矩阵的右下角即给出了从四元数到旋转矩阵的变换关系：

![视觉SLAM十四讲-48](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-48.png)

对上式两侧求迹，得：

![视觉SLAM十四讲-49](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-49.png)

由：

![视觉SLAM十四讲-50](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-50.png)

得：

![视觉SLAM十四讲-53](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-53.png)

所以：

![视觉SLAM十四讲-51](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-51.png)

总之，四元数到旋转向量的转换公式可列写如下：

![视觉SLAM十四讲-52](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-52.png)

在实际编程中，程序库通常会为我们准备好各种形式之间的转换。无论是四元数、旋转矩阵还是轴角，它们都可以用来描述同一个旋转。我们应该在实际中选择最为方便的形式。

### 3.5相似、仿射、射影变换

除了欧氏变换之外，3D空间还存在其他几种变换方式。它们一部分和测量几何有关。欧氏变换保持了向量的长度和夹角，相当于我们把一个刚体原封不动地进行了移动或旋转，不改变它自身的样子。其他几种变换则会改变它的外形。它们都拥有类似的矩阵表示。

#### 3.5.1相似变换

相似变换比欧氏变换多了一个自由度，它允许物体进行均匀缩放，其矩阵表示为：

![视觉SLAM十四讲-54](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-54.png)

旋转部分多了一个缩放因子s，表示我们在对向量旋转之后，可以在 x,y,z 三个坐标上进行均匀缩放。由于含有缩放，相似变换不再保持图形的面积不变。可以想象一个边长为1的立方体通过相似变换后，变成边长为10的样子（但仍然是立方体）。三维相似变换的集合也叫做相似变换群，记作Sim(3)。

#### 3.5.2仿射变换

仿射变换的矩阵形式如下：

![视觉SLAM十四讲-55](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-55.png)

与欧氏变换不同的是，仿射变换只要求A是一个可逆矩阵，而不必是正交矩阵。仿射变换也叫正交投影。经过仿射变换之后，立方体就不再是方的了，但是各个面仍然是平行四边形。

#### 3.5.3 射影变换

射影变换是最一般的变换，它的矩阵形式为：

![视觉SLAM十四讲-56](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-56.png)

左上角为可逆矩阵A，右上角为平移t，左下角为缩放a。由于采用了齐次坐标，当 v≠0时，我们可以对整个矩阵除以 v 得到一个右下角为 1 的矩阵；否则得到右下角为 0 的矩阵。因此，2D 的射影变换一共有 8 个自由度，3D 则共有 15 个自由度。射影变换是变换中，形式最为一般的。从真实世界到相机照片的变换可以看成一个射影变换。可以想象一个原本方形的地板砖，在照片当中是什么样子：首先，它不再是方形的。由于近大远小的关系，它甚至不是平行四边形，而是一个不规则的四边形。

**总结：**在“不变性质”中，从上到下是有包含关系的。例如，欧氏变换除了保体积之外，也具有保平行、相交等性质。

![视觉SLAM十四讲-57](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-57.png)

从真实世界到相机照片的变换是一个射影变换。如果相机的焦距为无穷远，那么这个变换为仿射变换。

### 3.6实践：Eigen几何模块

#### 3.6.1 Eigen几何模块的数据演示

slambook2/ch3/useGeometry/useGeometry.cpp

代码中有注释。

Eigen中对各种形式的表达方式总结如下。每种类型都有单精度和双精度两种数据类型，不能由编译器自动转换。以双精度为例，把最后的d改成f，即得到单精度的数据结构。

• 旋转矩阵（3 × 3）：Eigen::Matrix3d。

• 旋转向量（3 × 1）：Eigen::AngleAxisd。

• 欧拉角（3 × 1）：Eigen::Vector3d。

• 四元数（4 × 1）：Eigen::Quaterniond。

• 欧氏变换矩阵（4 × 4）：Eigen::Isometry3d。

• 仿射变换（4 × 4）：Eigen::Affine3d。

• 射影变换（4 × 4）：Eigen::Projective3d。

程序中，演示了如何使用Eigen中的旋转矩阵、旋转向量（AngleAxis）、欧拉角和四元数。

进一步了解Eigen的几何模块可以参考

http://eigen.tuxfamily.org/dox/group__TutorialGeometry.html）。

注意，程序代码和数学表示有一些细微的差别。例如，通过运算符重载，四元数和三维向量可以直接计算乘法，但在数学上则需要先把向量转成虚四元数，再利用四元数乘法进行计算，同样也适用于变换矩阵乘三维向量的情况。总体而言，程序中的用法会比数学公式更灵活一些。

#### 3.6.2 实际的坐标变换例子

举例演示坐标变换：

设有机器人一号和机器人二号位于世界坐标系中。记世界坐标系为W，机器人的坐标系为 R1和R2。机器人一号的位姿为q1 = [0.35, 0.2, 0.3, 0.1]T, t1 = [0.3, 0.1, 0.1]T。机器人二号的位姿为q2 = [−0.5, 0.4, −0.1, 0.2]T, t2 = [−0.1, 0.5, 0.3]T。这里的q和t表达的是世界坐标系到相机坐标系的变换关系。机器人一号看到某个点在自身的坐标系下坐标为pR1 = [0.5, 0, 0.2]T，求该向量在机器人二号坐标系下的坐标。

程序为：slambook2/ch3/examples/coordinateTransform.cpp

注意：四元数使用之前需要归一化。

### 3.7可视化演示

#### 3.7.1 显示运动轨迹

轨迹文件记录了一个机器人的运动轨迹，存储于trajectory.txt，每一行用下面的格式存储：

time,tx,ty,tz,qx,qy,qz,qw, 把它画到一个窗口中。

其中time指该位姿的记录时间，t为平移，q为旋转四元数，均是以世界坐标系到机器人坐标系记录。

在画轨迹的时候，可以把“轨迹”画成一系列点组成的序列。这其实是机器人（相机）坐标系的原点在世界坐标系中的坐标。考虑机器人坐标系的原点，即OR，那么，此时的OW就是这个原点在世界坐标系下的坐标：

![视觉SLAM十四讲-60](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-60.png)

OW正是TWR 的平移部分。因此，可以从TWR中直接看到相机在何处，这是TWR更为直观的原因。在可视化程序里，轨迹文件存储了TWR而不是TRW。

在linux 中，基于OpenGL的Pangolin库，在支持OpenGL的绘图操作基础之上还提供了一些GUI的功能。

使用git的submodule功能来管理依赖的第三方库。可以进入3rdparty文件夹中直接安装所需的库，git保证了使用的版本是一致的。

程序演示了如何在Panglin中画出3D的位姿。用红、绿、蓝三种颜色画出每个位姿的三个坐标轴（计算了各坐标轴的世界坐标），然后用黑色线将轨迹连起来。

程序是：slambook2/ch3/examples/plotTrajectory.cpp

运行步骤：

https://github.com/stevenlovegrove/Pangolin

一开始从github下载最新版本的Pangolin，发现需要3.10的cmake，

然后先安装3.10版本的cmake，在[https://cmake.org/files/v3.10/](https://cmake.org/files/v3.18/)

官网下载源文件后，解压进去文件夹中

```shell
./bootstrap --prefix=/opt/cmake-install
make
make install
sudo gedit ~/.bashrc
```

\#把下面两个命令加入~/.bashrc中

```cmake
export PATH=/opt/cmake-install/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/cmake-install:$CMAKE_PREFIX_PATH
```

\#保存一下并运行

```shell
source ~/.bashrc
```

 

后面发现直接从https://github.com/gaoxiang12/slambook2.git

下载的pangolin不需要安装高版本的cmake。

需要安装一些依赖：

```shell
sudo apt-get install libglew-dev
sudo apt-get install cmake
sudo apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev
sudo apt-get install libxkbcommon-x11-dev
sudo apt-get install wayland-protocols
```

然后：

```shell
mkdir build && cd build
cmake ..
```

运行程序遇到Pangolin X11: Unable to retrieve framebuffer options的问题。

解决方法是注释掉Pangolin的源代码src/display/device/display_x11.cpp中的L123-L124，即：

```c++
//GLX_SAMPLE_BUFFERS , glx_sample_buffers,
//GLX_SAMPLES , glx_sample_buffers > 0 ? glx_samples : 0,
```

程序运行结果：

![视觉SLAM十四讲-58](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-58.png)

#### 3.7.2 显示相机的位姿

slambook2/ch3/visualizeGeometry 中，以可视化的形式演示了相机位姿的各种表达方式。用鼠标操作相机时，左侧的方框里会实时显示相机位姿对应的旋转矩阵、平移、欧拉角和四元数。效果如下：

![视觉SLAM十四讲-59](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-59.png)

### 习题

1. 验证旋转矩阵是正交矩阵。

2. 寻找罗德里格斯公式的推导过程并加以理解。

3. 验证四元数旋转某个点后，结果是一个虚四元数（实部为零），所以仍然对应到一个三维空间点。

4. 总结旋转矩阵、轴角、欧拉角、四元数的转换关系。

5. 假设有一个大的 Eigen 矩阵，把它的左上角 3 × 3 的块取出来，然后赋值为单位阵。编程实现。

6. 一般线性方程Ax = b有哪几种做法？在 Eigen 中实现。

## 第四讲：李群和李代数

### 4.1 基础

在SLAM中的旋转，除了表示之外，我们还要对它们进行估计和优化。因为在SLAM中位姿是未知的，而我们需要解决什么样的相机位姿最符合当前观测数据这样的问题。一种典型的方式是把它构建成一个优化问题，求解最优的R, t，使得误差最小化。旋转矩阵自身是带有约束的（正交且行列式为1）。它们作为优化变量时，会引入额外的约束，使优化变得困难。通过李群——李代数间的转换关系，我们希望把位姿估计变成无约束的优化问题，简化求解方式。

三维旋转矩阵构成了特殊正交群 SO(3)，而变换矩阵构成了特殊欧氏群 SE(3)。

旋转矩阵和变换矩阵对加法是不封闭的。换句话说，对于任意两个旋转矩阵 R1, R2，按照矩阵加法的定义，它们的和不再是一个旋转矩阵，对于变换矩阵亦是如此。

这两种矩阵并没有良好定义的加法，相对的，它们只有一种较好的运算：乘法。SO(3) 和SE(3)关于乘法是封闭的:   R1R2 ∈ SO(3), T1T2 ∈ SE(3). 

乘法对应着旋转或变换的复合——两个旋转矩阵相乘表示做了两次旋转。这种只有一个运算的集合叫做群。

#### 4.1.1 群

群（Group）是一种集合加上一种运算的代数结构。我们把集合记作A，运算记作 · ，

那么群可以记作 G = (A, ·)。群要求这个运算满足以下几个条件：

1. 封闭性: ∀a1, a2 ∈ A, a1 · a2 ∈ A.

2. 结合律: ∀a1, a2, a3 ∈ A, (a1 · a2) · a3 = a1 · (a2 · a3).

3. 幺元: ∃a0 ∈ A, s.t. ∀a ∈ A, a0 · a = a · a0 = a.

4. 逆: ∀a ∈ A, ∃a−1 ∈ A, s.t. a · a−1 = a0.

可以记作“封结幺逆”，谐音凤姐咬你。

旋转矩阵集合和矩阵乘法构成群，同样变换矩阵和矩阵乘法也构成群。其他常见的群包括整数的加法 (Z, +)，去掉0后的有理数的乘法（幺元为 1）(Q\0, ·) 等等。矩阵中常见的群有：

一般线性群 GL(n) 指 n × n 的可逆矩阵，它们对矩阵乘法成群。

特殊正交群 SO(n) 也就是所谓的旋转矩阵群，其中 SO(2) 和 SO(3) 最为常见。

特殊欧氏群 SE(n) 也就是前面提到的 n 维欧氏变换，如 SE(2) 和 SE(3)。

群结构保证了在群上的运算具有良好的性质，而群论则是研究群的各种结构和性质的理论，可以参考任意一本近世代数教材。

李群是指具有连续（光滑）性质的群。像整数群Z那样离散的群没有连续性质，所以不是李群。而SO(n)和SE(n)，它们在实数空间上是连续的。直观地想象一个刚体能够连续地在空间中运动，所以它们都是李群。SO(3)和SE(3)对于相机姿态估计尤其重要。

#### 4.1.2李代数

考虑任意旋转矩阵 R，满足：

![视觉SLAM十四讲-61](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-61.png)

假设R是某个相机的旋转，它会随时间连续地变化，即为时间的函数：R(t)。有：

![视觉SLAM十四讲-62](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-62.png)

等式两边对时间求导，得到：

![视觉SLAM十四讲-63](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-63.png)

整理得：

![视觉SLAM十四讲-64](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-64.png)

可以看出

![视觉SLAM十四讲-65](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-65.png)

是一个反对称矩阵。

而叉积引入了∧符号，将一个向量变成了反对称矩阵。同理，对于任意反对称矩阵，我们亦能找到一个与之对应的向量。把这个运算用符号∨表示。如下：

![视觉SLAM十四讲-66](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-66.png)

由于

![视觉SLAM十四讲-65](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-65.png)

是一个反对称矩阵，我们可以找到一个三维向量 ϕ(t) ∈ R3 与之对应：

![视觉SLAM十四讲-67](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-67.png)

等式两边右乘 R(t)，由于R为正交阵，有：

![视觉SLAM十四讲-68](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-68.png)

每对旋转矩阵求一次导数，只需左乘一个 ϕ(t)∧矩阵（反对称矩阵）即可。

考虑 t0 = 0 时刻，设此时旋转矩阵为 R(0) = I。按照导数定义，可以把R(t) 在 t = 0 附近进行一阶泰勒展开：

![视觉SLAM十四讲-69](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-69.png)

ϕ反映了R的导数性质，故称它在SO(3)原点附近的正切空间(Tangent Space)上。

同时在 t0 附近，设ϕ(t0) = ϕ0，有

![视觉SLAM十四讲-70](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-70.png)

求解关于R的微分方程，有初始值 R(0) = I，得：

![视觉SLAM十四讲-71](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-71.png)

说明在 t = 0 附近，旋转矩阵可以由 exp (ϕ0∧t)计算出来，即旋转矩阵 R 与另一个反对称矩阵 ϕ0∧t 通过指数关系发生了联系。

有两个问题：

1. 给定某时刻的R，我们就能求得一个ϕ，它描述了R在局部的导数关系。ϕ正是对应到 SO(3) 上的李代数so(3)；

2. 给定某个向量 ϕ 时，矩阵指数 exp(ϕ∧) 如何计算？反之，给定R时，有无相反的运算来计算 ϕ？这正是李群与李代数间的指数/对数映射，即李代数（ϕ）到李群（R）是指数关系，而李群（R）到李代数（ϕ）是对数关系。

#### 4.1.3李代数的定义：

每个李群都有与之对应的李代数。李代数描述了李群的局部性质。李代数由一个集合V，一个数域 F 和一个二元运算 [, ] 组成，满足以下几条性质：

1.封闭性 

2.双线性

3.自反性（指自己与自己的运算为零） 

4.雅可比等价 

其中二元运算被称为李括号。相比于群中的较为简单的二元运算，李括号表达了两个元素的差异。它不要求结合律，而要求元素和自己做李括号之后为零的性质。三维向量R3上定义的叉积×是一种李括号，因此 g=(R3, R, ×) 构成了一个李代数。

#### 4.1.4李代数so(3) 

ϕ是一种李代数。SO(3)对应的李代数是定义在R3上的向量，我们记作ϕ。每个ϕ都可以生成一个反对称矩阵：

![视觉SLAM十四讲-72](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-72.png)

在此定义下，两个向量 ϕ1, ϕ2 的李括号为： [ϕ1, ϕ2] = (Φ1Φ2 − Φ2Φ1) ∨ 

向量ϕ与反对称矩阵是一一对应的，所以就说so(3) 的元素是三维向量或者三维反对称矩阵，不加区别。

李代数是一个由三维向量组成的集合，每个向量对应到一个反对称矩阵，可以用于表达旋转矩阵的导数。 它与 SO(3) 的关系由指数映射给定：

![视觉SLAM十四讲-73](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-73.png)

#### 4.1.5李代数se(3)

与so(3)相似，se(3)位于R6空间中。

![视觉SLAM十四讲-74](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-74.png)

把每个se(3)元素记作ξ，它是一个六维向量。前三维为平移，记作ρ；后三维为旋转，记作ϕ，实质上是so(3)元素。同时拓展了∧符号的含义，在se(3)中，同样使用∧符号，将一个六维向量转换成四维矩阵，但这里不再表示反对称。

![视觉SLAM十四讲-75](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-75.png)

即仍使用∧和∨符号来指代“从向量到矩阵”和“从矩阵到向量”的关系，以保持和so(3)的一致性。可以简单地把se(3)理解成“由一个平移加上一个so(3)元素构成的向量”。 

### 4.2指数与对数映射 

#### 4.2.1 SO(3)上的指数映射

exp(ϕ∧)如何计算？它是一个矩阵的指数，在李群和李代数中，称为指数映射（Exponential Map）。

任意矩阵的指数映射可以写成一个泰勒展开，但是只有在收敛的情况下才会有结果，其结果仍是一个矩阵。

![视觉SLAM十四讲-76](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-76.png)

同样地，对so(3)中任意一元素ϕ，也定义它的指数映射：

![视觉SLAM十四讲-77](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-77.png)

由于ϕ是三维向量，我定义它的模长和它的方向，分别记作θ和a，于是有 ϕ = θa。这里a是一个长度为1的方向向量。首先，对于a∧，有以下两条性质：

![视觉SLAM十四讲-78](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-78.png)

![视觉SLAM十四讲-79](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-79.png)

这提供了处理 a∧高阶项的方法。利用这两个性质，可以得到一个式子：

![视觉SLAM十四讲-80](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-80.png)

这和罗德里格斯公式如出一辄。

这表明，so(3)实际上就是由旋转向量组成的空间，而指数映射即罗德里格斯公式。通过这，我们把so(3)中任意一个向量对应到了一个位于SO(3)中的旋转矩阵。反之，如果定义对数映射，我们也能把SO(3)中的元素对应到so(3)中。但没必要泰勒展开去计算对数映射，可以根据旋转矩阵计算对应的李代数，利用迹的性质分别求解转角和转轴（分别对应模长θ和方向a）。 

指数映射是一个满射，并不是单射。这意味着每个SO(3)中的元素，都可以找到一个so(3)元素与之对应；但是可能存在多个so(3)中的元素，对应到同一个SO(3)。即旋转矩阵可以找到一个旋转向量对应，而多个旋转向量（旋转角有周期性）可以对应同一个旋转矩阵。比如对于旋转角θ，多转360度和没有转是一样的。如果我们把旋转角度固定在±π之间，那么李群和李代数元素是一一对应的。 

SO(3)与so(3)的结论和前面讲的旋转向量与旋转矩阵很相似，而指数映射即是罗德里格斯公式。旋转矩阵的导数可以由旋转向量指定，指导着如何在旋转矩阵中进行微积分运算。

####  4.2.2 SE(3)上的指数映射 

se(3) 上的指数映射形式如下：

![视觉SLAM十四讲-81](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-81.png)

照着so(3)上的做法推导，把exp进行泰勒展开推导此式，得

![视觉SLAM十四讲-82](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-82.png)

该式与罗德里格斯有些相似。平移部分经过指数映射之后，发生了一次以J为系数矩阵的线性变换。

李群、李代数的定义与相互的转换关系，总结如图所示：

![视觉SLAM十四讲-83](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-83.png)

### 4.3 李代数求导与扰动模型

#### 4.3.1 BCH公式与近似形式

使用李代数的一大目的是进行优化，在优化过程中导数是非常必要的信息。考虑一个问题，当在SO(3)中完成两个矩阵乘法时，李代数中so(3)上发生了什么改变？反过来说，当so(3)上做两个李代数的加法时，SO(3)上是否对应着两个矩阵的乘积？如果成立，相当于：

 ![视觉SLAM十四讲-84](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-84.jpg)

如果ϕ1,ϕ2为标量，显然该式成立；但此处计算的是矩阵的指数函数，而非标量的指数。

换言之是在研究下式是否成立：

ln(exp(A)exp(B)) = A + B  (这个式子有些问题)

该式在矩阵时并不成立。两个李代数指数映射乘积的完整形式，由 Baker-Campbell-Hausdorff公式（BCH公式）出。

 ![视觉SLAM十四讲-85](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-85.jpg)

其中 [] 为李括号。

BCH 公式说明，当处理两个矩阵指数之积时，它们会产生一些由李括号组成的余项。特别地，考虑SO(3)上的李代数，当ϕ1或ϕ2为小量时，小于二次以上的项都可以被忽略掉。此时，BCH拥有线性近似表达 ：

 ![视觉SLAM十四讲-86](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-86.jpg)

以第一个近似为例。该式说明，当对一个旋转矩阵R2（李代数为ϕ2）左乘一个微小旋转矩阵R1（李代数为ϕ1）时，可以近似地看作，在原有的李代数ϕ2上加上了一项Jl(ϕ2)−1ϕ1。同理，第二个近似描述了右乘一个微小位移的情况。于是，李代数在BCH近似下，分成了左乘近似和右乘近似两种，在使用时我们须注意使用的是左乘模型还是右乘模型。 

左乘 BCH 近似雅可比 

![视觉SLAM十四讲-87](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-87.jpg)

它的逆为：

![视觉SLAM十四讲-88](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-88.jpg)

右乘雅可比仅需要对自变量取负号即可：

![视觉SLAM十四讲-89](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-89.jpg)

**BCH近似的意义：**假定对某个旋转R，对应的李代数为ϕ。我们给它左乘一个微小旋转，记作∆R，对应的李代数为∆ϕ。那么，在李群上，得到的结果就是∆R·R，而在李代数上，根据BCH近似，为

![视觉SLAM十四讲-90](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-90.jpg)

合并起来，可以写成：

 ![视觉SLAM十四讲-91](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-91.jpg)

反之，如果我们在李代数上进行加法，让一个ϕ加上 ∆ϕ，可以近似为李群上的左右雅可比的乘法：

 ![视觉SLAM十四讲-92](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-92.jpg)

这为在李代数上做微积分提供了理论基础。同样，对于SE(3)，亦有类似的BCH近似：

​    ![视觉SLAM十四讲-93](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-93.jpg)

这里的左右雅可比形式比较复杂，是一个6×6的矩阵，可以参考文献[6]。

[6] T. Barfoot, “State estimation for robotics: A matrix lie group approach,” 2016.



#### 4.3.2 SO(3)李代数上的求导

在SLAM中，要估计一个相机的位置和姿态，该位姿是由SO(3)上的旋转矩阵或SE(3) 上的变换矩阵描述的。设某个时刻机器人的位姿为T，它观察到了一个世界坐标位于p的点，产生了一个观测数据z。由坐标变换关系知：

​    z = Tp + w.  

其中w为随机噪声。由于它的存在，z 往往不可能精确地满足z=Tp的关系。所以通常会计算理想的观测与实际数据的误差：

e =z−Tp.

假设一共有N个这样的路标点和观测，于是就有N个上式。那么，对机器人的位姿估计，相当于是寻找一个最优的T，使得整体误差最小化：

 ![视觉SLAM十四讲-94](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-94.jpg)

求解此问题，需要计算目标函数J关于变换矩阵T的导数。

重点是构建与位姿有关的函数，讨论该函数关于位姿的导数，以调整当前的估计值。然而SO(3),SE(3)上并没有良好定义的加法，它们是群。如果把T当成一个普通矩阵来处理优化，那就必须对它加以约束（旋转矩阵的约束是行列式值唯一，计算复杂）。而从李代数角度来说，由于李代数由向量组成，具有良好的加法运算。

使用李代数解决求导问题的思路分为两种：

1. 用李代数表示姿态，然后根据李代数加法来对李代数求导。

2. 对李群左乘或右乘微小扰动，然后对该扰动求导，称为左扰动和右扰动模型。

第一种方式对应到李代数的求导模型，而第二种则对应到扰动模型。

#### 4.3.3 李代数求导

考虑SO(3)上的情况。假设对一个空间点p进行了旋转，得到了Rp。计算旋转之后点的坐标相对于旋转的导数，记为 ：

 ![视觉SLAM十四讲-95](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-95.jpg)

由于SO(3)没有加法，所以该导数无法按照导数的定义进行计算。设R对应的李代数为ϕ，转而计算 ：

 ![视觉SLAM十四讲-96](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-96.jpg)

按照导数的定义，推导出了旋转后的点相对于李代数的导数：

 ![视觉SLAM十四讲-97](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-97.jpg)

注：这里并不能按照矩阵微分来定义导数，只是一个记号。

#### 4.3.4 扰动模型（左乘）

另一种求导方式是对R进行一次扰动∆R，看结果相对于扰动的变化率。这个扰动可以乘在左边也可以乘在右边，最后结果会有一点微小的差异。以左扰动为例，设左扰动∆R对应的李代数为φ。对φ求导，即：

 ![视觉SLAM十四讲-98](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-98.jpg)

相比于直接对李代数求导，省去了一个雅可比矩阵的计算。这使得扰动模型更为实用，在位姿估计当中具有重要的意义。

#### 4.3.5 SE(3)上的李代数求导

书中只给出SE(3)上的扰动模型。假设某空间点p经过一次变换T（对应李代数为ξ），得到Tp 。给T左乘一个扰动∆T = exp(δξ∧)，设扰动项的李代数为 δξ = [δρ,δϕ]T，那么：

 ![视觉SLAM十四讲-99](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-99.jpg)

把最后的结果定义成一个算符 ，它把一个齐次坐标的空间点变换成一个4×6的矩阵。

（这里不太理解）

### 4.4 实践：Sophus

### 4.4.1 Sophus的基本使用方法

Sophus 库是Strasdat 维护的  。Sophus 库支持SO(3) 和 SE(3)，此外还含有二维运动 SO(2),SE(2) 以及相似变换 Sim(3) 的内容。它是直接在 Eigen 基础上开发的，不需要安装额外的依赖库。可以直接从 GitHub 上获取 Sophus，在代码目录 slambook/3rdparty 下也提供了 Sophus 源代码。

slambook/ch4/useSophus.cpp

该演示程序分为两部分。前半部分介绍 SO(3) 上的操作，后半部分则为 SE(3)。演示了如何构造 SO(3),SE(3) 对象，对它们进行指数、对数映射，以及当知道更新量后，如何对李群元素进行更新。

编译：在 CMakeLists.txt 里添加以下几行：

```cmake
# 为使用 sophus，需要使用 find_package 命令找到它
find_package( Sophus REQUIRED ) 
include_directories( ${Sophus_INCLUDE_DIRS} ) 
add_executable( useSophus useSophus.cpp )
```

如： slambook/ch4/useSophus/CMakeLists.txt

注：find_package 命令是 cmake 提供的寻找某个库的头文件与库文件的指令。如果 cmake 能够找到它，就会提供头文件和库文件所在的目录的变量。在 Sophus 这个例子中，就是 Sophus_INCLUDE_ DIRS。基于模板的 Sophus 库和 Eigen 一样，是仅含头文件而没有源文件的。

#### 4.4.2 例子：评估轨迹的误差

在实际工程中，经常需要评估一个算法的估计轨迹与真实轨迹的差异，来评价算法的精度。真实轨迹往往通过某些更高精度的系统获得，而估计轨迹则是待评价的算法计算得到。

考虑一条估计轨迹Testi,i 和真实轨迹Tgt,i，其中 i = 1,··· ,N，那么可以定义一些误差指标来描述它们之间的差别。

误差指标有很多种，常见的有绝对轨迹误差（Absolute Trajectory Error, ATE），这是每个位姿李代数的均方根误差（Root-Mean-Squared Error, RMSE）。这种误差可以刻画两条轨迹的旋转和平移误差。仅考虑平移误差 [23]，可以定义绝对平移误差（Average Translational Error）。

因为从整条轨迹上来看，旋转出现误差后，随后的轨迹在平移上也会出现误差，所以两种指标在实际当中都适用。

除此之外，也可以定义相对的误差。例如考虑 i 时刻到 i + ∆t 时刻的运动，有相对位姿误差（Relative Pose Error, RPE）；同样地，亦可只取平移部分。

利用 Sophus 库，很容易实现这部分计算。

如：slambook/ch4/example/trajectoryError.cpp

在这个例子中，程序将读取 groundtruth.txt 和 estimated.txt 两条轨迹，计算误差，然后显示到 3D 窗口中。

**程序依赖安装和运行：**

安装pangolin，直接从https://github.com/gaoxiang12/slambook2.git

下载pangolin（或者slambook1）

需要安装一些依赖：

```shell
sudo apt-get install libglew-dev
sudo apt-get install cmake
sudo apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev
sudo apt-get install libxkbcommon-x11-dev
sudo apt-get install wayland-protocols
```

然后：

```shell
mkdir build && cd build
cmake ..
```

安装sophus库，直接github下载最新的（老版本的不兼容）

https://github.com/strasdat/Sophus.git

需要安装3.3以上的eigen，用ubuntu16.04 ros自带的eigen版本太低了。

可能还需要安装fmt库，去github下载并安装：

https://github.com/fmtlib/fmt.git

运行程序遇到Pangolin X11: Unable to retrieve framebuffer options的问题。

解决方法是注释掉Pangolin的源代码src/display/device/display_x11.cpp中的L123-L124，即：

```c++
//GLX_SAMPLE_BUFFERS , glx_sample_buffers,
//GLX_SAMPLES , glx_sample_buffers > 0 ? glx_samples : 0,
```

程序运行结果：

![视觉SLAM十四讲-106](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-106.jpg)



### 4.5 *相似变换群与李代数

相似变换群 Sim(3)是在单目视觉中使用的，对应的李代数是sim(3)。双目 SLAM 或 RGB-D SLAM 没有用到。

如果在单目SLAM中使用 SE(3) 表示位姿，那么由于尺度不确定性与尺度漂移，整个 SLAM 过程中的尺度会发生变化，这在 SE(3) 中未能体现出来。因此，在单目情况下一般会显式地把尺度因子表达出来。用数学语言来说，对于位于空间的点p，在相机坐标系下要经过一个相似变换，而非欧氏变换：

![视觉SLAM十四讲-100](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-100.jpg)

在相似变换中，把尺度s表达了出来。它同时作用在p的3个坐标之上，对p进行了一次缩放。与 SO(3)、SE(3) 相似，相似变换亦对矩阵乘法构成群，称为相似变换群 Sim(3)：

![视觉SLAM十四讲-101](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-101.jpg)

同样地，Sim(3) 也有对应的李代数、指数映射、对数映射等。李代数 sim(3) 元素是一个7维向量ζ。它的前 6 维与 se(3) 相同，最后多了一项 σ。

![视觉SLAM十四讲-104](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-104.jpg)

关联Sim(3)和sim(3)的仍是指数映射和对数映射。指数映射为：

![视觉SLAM十四讲-103](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-103.jpg)

通过指数映射，能够找到李代数与李群的关系。对于李代数ζ，它与李群的对应关系为：

 ![视觉SLAM十四讲-102](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-102.jpg)

![视觉SLAM十四讲-105](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-105.jpg)

旋转部分和 SO(3) 是一致的。平移部分，在 se(3) 中需要乘一个雅可比J，而相似变换的雅可比更复杂一些。对于尺度因子，可以看到李群中的 s 即为李代数中 σ 的指数函数。

Sim(3) 的 BCH 近似与 SE(3) 是类似的。可以讨论一个点p经过相似变换Sp后，相对于S 的导数，存在微分模型和扰动模型两种方式，而扰动模型较为简单。设给予Sp左侧一个小扰动 exp(ζ∧)，并求Sp对于扰动的导数。因为Sp 是 4 维的齐次坐标，ζ 是 7 维向量，该导数应该是 4 × 7 的雅可比。

更详细的关于 Sim(3) 的资料，参见文献 [24]：H. Strasdat, Local accuracy and global consistency for efficient visual slam. PhD thesis, Citeseer, 2012.

### 4.6 小结

这讲引入了李群 SO(3) 和 SE(3)，以及它们对应的李代数 so(3) 和 se(3)。介绍了位姿在它们上面的表达和转换，然后通过 BCH 的线性近似，就可以对位姿进行扰动并求导了。这给讲解位姿的优化打下了理论基础，因为需要经常地对某一个位姿的估计值进行调整，使它对应的误差减小。

在实际应用中，可以使用 SO(3) 加上平移的方式来代替 SE(3)，从而回避一些雅可比的计算。

### 习题

1. 验证 SO(3)、SE(3) 和 Sim(3) 关于乘法成群。

2. 验证 (R3,R,×) 构成李代数。

3. 验证 so(3) 和 se(3) 满足李代数要求的性质。

4. 证明：Rp∧RT = (Rp)∧.

5. 证明：R exp(p∧) RT = exp((Rp)∧).

该式称为 SO(3) 上的伴随性质。同样地，在 SE(3) 上亦有伴随性质：

​    T exp(ξ∧)T−1 = exp((Ad(T)ξ)∧)

6. 仿照左扰动的推导，推导 SO(3) 和 SE(3) 在右扰动下的导数。
7. 搜索 cmake 的 find_package 指令是如何运作的。它有哪些可选的参数？为了让 cmake 找到某个库，需要哪些先决条件？

## 第五讲：相机与图像

目标

1.理解针孔相机的模型、内参与径向畸变参数。

2.理解一个空间点如何投影到相机成像平面。

3.掌握 OpenCV 的图像存储与表达方式。

4.学会基本的摄像头标定方法。

在以相机为主的视觉SLAM中，观测主要是指相机成像的过程。

三维世界中的一个物体反射或发出的光线，穿过相机光心后，投影在相机的成像平面上。相机的感光器件接收到光线后，产生测量值，就得到了像素，形成了我们见到的照片。

###  5.1 相机模型

相机将三维世界中的坐标点（单位为米）映射到二维图像平面（单位为像素）的过程能够用一个几何模型进行描述，称为针孔模型，它描述了一束光线通过针孔之后，在针孔背面投影成像的关系。同时，由于相机镜头上的透镜的存在，使得光线投影到成像平面的过程中会产生畸变。因此，我们使用针孔和畸变两个模型来描述整个投影过程。这两个模型能够把外部的三维点投影到相机内部成像平面，构成相机的内参数（Intrinsics）。

#### 5.1.1 针孔相机模型

初中物理的蜡烛投影实验：在一个暗箱的前方放着一支点燃的蜡烛，蜡烛的光透过暗箱上的一个小孔投影在暗箱的后方平面上，并在这个平面上形成一个倒立的蜡烛图像。小孔模型能够把三维世界中的蜡烛投影到一个二维成像平面。同理，可以用这个简单的模型来解释相机的成像过程。对这个简单的针孔模型进行几何建模。设 O − x − y − z 为相机坐标系，z 轴指向相机前方，x 向右，y 向下。O为摄像机的光心，也是针孔模型中的针孔。现实世界的空间点P，经过小孔O投影之后，落在物理成像平面 O′ − x′ − y′ 上，成像点为 P′。设 P 的坐标为 [X,Y,Z]T，P′ 为 [X′,Y′,Z′]T，设物理成像平面到小孔的距离为f（焦距）。那么，根据三角形相似关系，有：

![视觉SLAM十四讲-108](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-108.jpg)

![视觉SLAM十四讲-107](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-107.jpg)

其中负号表示成的像是倒立的。不过，实际相机得到的图像并不是倒像，可以等价地把成像平面对称地放到相机前方，和三维空间点一起放在摄像机坐标系的同一侧，如图所示。

![视觉SLAM十四讲-109](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-109.jpg)

把公式中的负号去掉，X′,Y′ 放到等式左侧，整理得：

![视觉SLAM十四讲-110](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-110.jpg)

这描述了点 P和它的像之间的空间关系。不过，在相机中最终获得的是一个个的像素，这还需要在成像平面上对像进行采样和量化。为了描述传感器将感受到的光线转换成图像像素的过程，设在物理成像平面上固定着一个像素平面 o − u − v，在像素平面有P′的像素坐标：[u,v]T。

像素坐标系通常的定义方式是：原点o′位于图像的左上角，u 轴向右与 x 轴平行，v 轴向下与 y 轴平行。像素坐标系与成像平面之间，相差了一个缩放和一个原点的平移。设像素坐标在 u 轴上缩放了 α 倍，在 v 上缩放了 β 倍。同时，原点平移了 [cx,cy]T。那么，P′ 的坐标与像素坐标[u,v]T 的关系为：

![视觉SLAM十四讲-111](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-111.jpg)

代入式

![视觉SLAM十四讲-110](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-110.jpg)

把 αf 合并成 fx，把 βf 合并成 fy，得：

![视觉SLAM十四讲-112](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-112.jpg)

其中，f 的单位为米，α,β 的单位为像素/米，所以 fx,fy 和 cx,cy 的单位为像素。写成矩阵形式：

![视觉SLAM十四讲-113](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-113.jpg)

K矩阵称为相机的内参数矩阵（Camera Intrinsics）。通常相机的内参在出厂之后是固定的，不会在使用过程中发生变化。但有时需要自己确定相机的内参，也就是所谓的标定。（单目棋盘格张正友标定法[25]Z. Zhang, “Flexible camera calibration by viewing a plane from unknown orientations,” in Computer Vision, 1999. The Proceedings of the Seventh IEEE International Conference on, vol. 1, pp. 666–673, Ieee, 1999.）

前面内参公式中的P是在相机坐标系下的坐标。由于相机在运动，所以P是相机的世界坐标（记为Pw）根据相机的当前位姿变换到相机坐标系下的结果。相机的位姿由它的旋转矩阵R和平移向量t来描述。那么有：

![视觉SLAM十四讲-114](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-114.jpg)

后一个式子隐含了一次齐次坐标到非齐次坐标的转换。它描述了P的世界坐标到像素坐标的投影关系。相机的位姿R,t称为相机的外参数（Camera Extrinsics） 。
相比于不变的内参，外参会随着相机运动发生改变，同时也是 SLAM 中待估计的目标，代表着机器人的轨迹。
式子表明，可以把一个世界坐标点先转换到相机坐标系，再除掉它最后一维（Z）的数值（即该点距离相机成像平面的深度），这相当于把最后一维进行归一化处理，得到点 P 在相机归一化平面上的投影：

![视觉SLAM十四讲-115](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-115.jpg)

归一化坐标可看成相机前方z=1处的平面上的一个点，这个 z = 1 平面也称为归一化平面。归一化坐标再左乘内参就得到了像素坐标，所以可以把像素坐标 [u,v]T 看成对归一化平面上的点进行量化测量的结果。从这个模型中可以看出，对相机坐标同时乘以任意非零常数，归一化坐标都是一样的，这说明点的深度在投影过程中被丢失了，所以单目视觉中没法得到像素点的深度值。

注：1. 相机输出的图像并不是倒像，但相机自身会翻转这张图像，所以实际得到的是正像，也就是对称的成像平面上的像。尽管从物理原理来说，小孔成像应该是倒像。

2. 在机器人或自动驾驶车辆中，外参也可以解释成相机坐标系到机器人本体坐标系之间的变换。

#### 5.1.2 畸变

为了获得好的成像效果，在相机的前方加了透镜。透镜的加入对成像过程中光线的传播会产生新的影响：一是透镜自身的形状对光线传播的影响，引起的畸变（Distortion，也叫失真）称为径向畸变。在针孔模型中，一条直线投影到像素平面上还是一条直线。可是，在实际拍摄的照片中，摄像机的透镜往往使得真实环境中的一条直线在图片中变成了曲线 。越靠近图像的边缘，这种现象越明显。由于实际加工制作的透镜往往是中心对称的，这使得不规则的畸变通常径向对称。它们主要分为两大类：桶形畸变和枕形畸变。

桶形畸变是由于图像放大率随着与光轴之间的距离增加而减小，而枕形畸变则恰好相反。在这两种畸变中，穿过图像中心和光轴有交点的直线还能保持形状不变。

二是在机械组装过程中，透镜和成像平面不可能完全平行，这也会使得光线穿过透镜投影到成像面时的位置发生变化，这引入切向畸变。


用数学形式对两者进行描述。省略具体过程，

对于相机坐标系中的一点P，能够通过 5 个畸变系数找到这个点在像素平面上的正确位置：

1. 将三维空间点投影到归一化图像平面。设它的归一化坐标为 [x,y]T。

2. 对归一化平面上的点计算径向畸变和切向畸变。

3. 将畸变后的点通过内参数矩阵投影到像素平面，得到该点在图像上的正确位置。

在实际应用中，可以灵活选择纠正模型，比如只选择 k1,p1,p2 这 3 项等。

实际的图像系统中，学者们提出了很多其他的模型，比如相机的仿射模型和透视模型等，同时也存在很多其他类型的畸变。视觉 SLAM 中一般都使用普通的摄像头，针孔模型及径向畸变和切向畸变模型已经足够。
当一个图像去畸变之后，我们就可以直接用针孔模型建立投影关系，不用考虑畸变了。

小结单目相机的成像过程：

1. 首先，世界坐标系下有一个固定的点 P，世界坐标为Pw。
2. 由于相机在运动，它的运动由 R,t 或变换矩阵T∈SE(3) 描述。P 的相机坐标为 P˜c =RPw + t。
3. 这时的 P˜c 的分量为 X,Y,Z，把它们投影到归一化平面 Z = 1 上，得到 P 的归一化坐标：Pc = [X/Z,Y /Z,1]T 。
4. 有畸变时，根据畸变参数计算Pc 发生畸变后的坐标。
5. 最后，P 的归一化坐标经过内参后，对应到它的像素坐标：Puv = KPc。
   一共有四种坐标：世界坐标、相机坐标、归一化坐标和像素坐标。

#### 5.1.3 双目相机模型

对于单目相机而言，仅根据一个像素，我们无法确定这个空间点的具体位置。这是因为，从相机光心到归一化平面连线上的所有点，都可以投影至该像素上（相当于没有了Z轴维度）。只有当P的深度确定时（比如通过双目或 RGB-D 相机），我们才能确切地知道它的空间位置。如图所示。

![视觉SLAM十四讲-116](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-116.png)

测量像素距离（或深度）的方式有很多种，比如人眼可以根据左右眼看到的景物差异（视差）来判断物体离我们的距离。双目相机的原理一样：通过同步采集左右相机的图像，计算图像间视差，来估计每一个像素的深度。

双目相机一般由左眼相机和右眼相机两个水平放置的相机组成。在左右双目相机中，我们可以把两个相机都看作针孔相机。它们是水平放置的，意味着两个相机的光圈中心都位于 *x* 轴上。两者之间的距离称为双目相机的基线（Baseline，记作 *b*），是双目相机的重要参数。

考虑一个空间点 *P*，它在左眼相机和右眼相机各成一像，记作 *PL,PR*。由于相机基线的存在，这两个成像位置是不同的。理想情况下，由于左右相机只在 *x* 轴上有位移，因此 *P* 的像也只在 *x* 轴（对应图像的u轴）上有差异。记它的左侧坐标为 *uL*，右侧坐标为 *uR*，几何关系如图所示。

![视觉SLAM十四讲-117](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-117.jpg)根据 △*PPLPR* 和 △*POLOR* 的相似关系，整理得：

![视觉SLAM十四讲-118](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-118.jpg)

双目相机的成像模型：*OL,OR* 为左右光圈中心，方框为成像平面，*f* 为焦距。*uL* 和 *uR* 为成像平面的坐标。注意，按照图中坐标定义，*uR* 应该是负数，所以图中标出的距离为 −*uR*。

其中 *d* 定义为左右图的横坐标之差，称为视差（Disparity）。根据视差，我们可以估计一个像素与相机之间的距离。视差与距离成反比：视差越大，距离越近。同时，由于视差最小为一个像素，于是双目的深度存在一个理论上的最大值，由 *fb* 确定。可以看到，当基线越长时，双目能测到的最大距离就会越远。类似人眼在看非常远的物体时（如很远的飞机），通常不能准确判断它的距离。

视差 *d* 的计算比较困难，需要确切地知道左眼图像某个像素出现在右眼图像的哪一个位置（即对应关系）。当想计算每个像素的深度时，其计算量与精度都将成为问题，而且只有在图像纹理变化丰富的地方才能计算视差。由于计算量的原因，双目深度估计仍需要使用 GPU 或FPGA 来实时计算。

#### 5.1.4 RGB-D相机模型

RGB-D 相机是主动测量每个像素的深度。目前的 RGB-D 相机按原理可分为两大类：

1. 红外结构光（Structured Light）： Kinect 1 代、Project Tango 1 代、Intel RealSense 等。

2. 通过飞行时间法（Time-of-flight，ToF）：Kinect 2 代和一些现有的 ToF 传感器等。

无论是哪种类型，RGB-D 相机都需要向探测目标发射一束光线（通常是红外光）。在结构光原理中，相机根据返回的结构光图案，计算物体与自身之间的距离。而在 ToF 原理中，相机向目标发射脉冲光，然后根据发送到返回之间的光束飞行时间，确定物体与自身之间的距离。ToF原理的相机和激光雷达十分相似，只不过激光雷达是通过逐点扫描来获取这个物体的距离，而ToF相机则可以获得整个图像的像素深度。

在测量深度之后，RGB-D 相机通常按照生产时的相机摆放位置，自己完成深度与彩色图像素之间的配对，输出一一对应的彩色图和深度图。可以在同一个图像位置，读取到色彩信息和距离信息，计算像素的 3D 相机坐标，生成点云（Point Cloud）。对 RGB-D 数据，既可以在图像层面进行处理，也可在点云层面处理。

RGB-D 相机能够实时地测量每个像素点的距离。但用红外光进行深度值测量的 RGB-D 相机，容易受到日光或其他传感器发射的红外光干扰，因此不能在室外使用。在没有调制的情况下，同时使用多个 RGB-D 相机时也会相互干扰。对于透射材质的物体，因为接收不到反射光，所以无法测量这些点的位置。

### 5.2 图像

相机把三维世界中的信息转换成了一张由像素组成的照片，存储在计算机中，作为后续处理的数据来源。在数学中，图像可以用一个矩阵来描述；而在计算机中，它们占据一段连续的磁盘或内存空间，可以用二维数组来表示。

最简单的图像——灰度图：每个像素位置 (*x,y*) 对应一个灰度值 *I*，一张宽度为 *w*、高度为 *h* 的图像，数学上可以记为一个函数：

![视觉SLAM十四讲-120](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-120.jpg)

其中 (*x,y*) 是像素的坐标。然而，计算机并不能表达实数空间，所以需要对下标和图像读数在某个范围内进行量化（类似于模拟到数字的概念）。在常见的灰度图中，用 0~255 的整数（一个 unsigned char或1 个字节）来表达图像的灰度读数。那么，一张宽度为 640 像素、高度为 480 像素分辨率的灰度图就可以表示为：

```c++
unsigned char image[480][640] //二维数组表达图像
```

在程序中，图像以二维数组形式存储。它的第一个下标是指数组的行，而第二个下标则是列。在图像中，数组的行数对应图像的高度，而列数对应图像的宽度。

当访问某一个像素时，需要指明它所处的坐标。像素坐标系原点位于图像的左上角，*X* 轴向右，*Y* 轴向下（也就是u,v* 坐标）。如果还有第三个轴—*Z* 轴，根据右手法则，*Z* 轴向前。这种定义方式是与相机坐标系一致的。图像的宽度或列数，对应着 *X* 轴；而图像的行数或高度，则对应着它的 *Y* 轴。

根据这种定义方式，访问一个位于 *x,y* 处的像素，那么在程序中应该是：

```c++
unsigned char pixel = image[y][x];  //访问图像像素
```

它对应着灰度值 *I*(*x,y*) 的读数。

![视觉SLAM十四讲-119](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-119.jpg)



在 RGB-D 相机的深度图中，记录了各个像素与相机之间的距离。这个距离通常是以毫米为单位，而 RGB-D 相机的量程通常在十几米左右，超过了 255。这时会采用 16 位整数（unsigned short）来记录深度图的信息，也就是位于 0~65535 的值。换算成米的话，最大可以表示 65 米，足够 RGB-D 相机使用。

彩色图像的表示则需要通道（channel）的概念。在计算机中，用红色、绿色和蓝色这三种颜色的组合来表达任意一种色彩。于是对于每一个像素，就要记录其 R、G、B 三个数值，每一个数值就称为一个通道。最常见的彩色图像有三个通道，每个通道都由 8 位整数表示。在这种规定下，一个像素占据 24 位空间。通道的数量、顺序都是可以自由定义的。在 OpenCV 的彩色图像中，通道的默认顺序是 B、G、R。也就是说，当得到一个 24 位的像素时，前 8 位表示蓝色数值，中间 8 位为绿色，最后 8 位为红色。如果还想表达图像的透明度，就使用 R、G、B、A 四个通道。

### 5.3 实践：计算机中的图像

#### 5.3.1 OpenCV 的基础使用方法

安装 OpenCV，网站：

http://opencv.org/

OpenCV提供了大量的开源图像算法，是计算机视觉中使用极广的图像处理算法库。

在Ubuntu下，有两种安装方式： 

1. 从源代码安装，指从OpenCV网站下载所有的OpenCV源代码，并在机器上编译安装，以便使用。好处是可以选择的版本比较丰富，而且能看到源代码，不过需要编译。还可以调整一些编译选项，匹配编程环境（例如，需不需要GPU加速等），还可以使用一些额外的功能。 源代码安装OpenCV 目前维护了两个主要版本，分为 OpenCV2.4系列和 OpenCV3系列。

2. 只安装库文件，指通过Ubuntu来安装由Ubuntu社区人员已经编译好的库文件，这样无须编译。 

源代码安装，安装依赖项：

```shell
sudo apt−get install build−essential libgtk2.0−dev libvtk5−dev libjpeg−dev libtiff4−dev libjasper−dev libopenexr−dev libtbb−dev
```

OpenCV 的依赖项很多，缺少某些编译项会影响它的部分功能，但可能不会用上。OpenCV 会在 cmake 阶段检查依赖项是否会安装，并调整自己的功能。如果电脑上有GPU并且安装了相关依赖项，OpenCV也会把GPU加速打开。

安装：

```shell
cmake ..
make -j8
sudo make install
```

安装后，OpenCV 默认存储在/usr/local 目录下

**操作 OpenCV 图像**

slambook/ch5/imageBasics/imageBasics.cpp

在该例程中操作有：图像读取、显示、像素遍历、复制、赋值等。编译该程序时，需要在CMakeLists.txt中添加 OpenCV的头文件，然后把程序链接到库文件上，还使用了C++11标准（如 nullptr 和 chrono）。

编译运行：

![视觉SLAM十四讲-121](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-121.jpg)

报错：

```shell
CMakeFiles/joinMap.dir/joinMap.cpp.o：在函数‘fmt::v7::detail::compile_parse_context<char, fmt::v7::detail::error_handler>::char_type const* fmt::v7::detail::parse_format_specs<Eigen::Transpose<Eigen::Matrix<double, 4, 1, 0, 4, 1> >, fmt::v7::detail::compile_parse_context<char, fmt::v7::detail::error_handler> >(fmt::v7::detail::compile_parse_context<char, fmt::v7::detail::error_handler>&)’中：
joinMap.cpp:(.text._ZN3fmt2v76detail18parse_format_specsIN5Eigen9TransposeINS3_6MatrixIdLi4ELi1ELi0ELi4ELi1EEEEENS1_21compile_parse_contextIcNS1_13error_handlerEEEEEPKNT0_9char_typeERSB_[_ZN3fmt2v76detail18parse_format_specsIN5Eigen9TransposeINS3_6MatrixIdLi4ELi1ELi0ELi4ELi1EEEEENS1_21compile_parse_contextIcNS1_13error_handlerEEEEEPKNT0_9char_typeERSB_]+0x247)：对‘fmt::v7::detail::error_handler::on_error(char const*)’未定义的引用
```

需要将之前安装的fmt库链接joinMap.cpp，rgbd文件夹中的cmakelists如下：

```cmake
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
find_package(Pangolin REQUIRED)
find_package(FMT REQUIRED)
add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap fmt::fmt ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
```

在图像中，鼠标点击图像中的每个点都能在左下角得到UV坐标值和RGB三通道值。

函数解析如下：

1. cv::imread：函数读取图像，并把图像和基本信息显示出来。

2. OpenCV 提供了迭代器，可以通过迭代器遍历图像的像素。cv::Mat::data 提供了指向图像数据开头的指针，可以直接通过该指针自行计算偏移量，然后得到像素的实际内存位置。

3. 复制图像中直接赋值是浅拷贝，并不会拷贝数据，而clone方法是深拷贝，会拷贝数据，这在图像存取中会经常用到。

4. 在编程过程中碰到图像的旋转、插值等操作，自行查阅函数对应的文档，以了解它们的原理与使用方式。 

注：1. cv::Mat 亦是矩阵类，除了表示图像之外，我们也可以用它来存储位姿等矩阵数据，但一般还是使用eigen，更快一些。

2. cmake默认编译的是debug模式，如果使用release模式会快很多。 

#### 5.3.2 图像去畸变

OpenCV 提供了去畸变函数 cv::Undistort()，这个例程从公式出发计算了畸变前后的图像坐标（代码中有内参数据）。

slambook/ch5/imageBasics/undistortImage.cpp

运行如下：

![视觉SLAM十四讲-122](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-122.jpg)

可以看到去畸变前后图像差别还是蛮大的。

### 5.4 实践：3D 视觉

#### 5.4.1 双目视觉

在stereo文件夹中，有左右目的图像和对应代码。其中代码计算图像对应的视差图，然后再计算各像素在相机坐标系下的坐标，它们共同构成点云。

slambook/ch5/stereoVision/stereoVision.cpp

运行如下：（比较大的图片是视差图）

![视觉SLAM十四讲-123](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-123.jpg)

例程中调用了OpenCV实现的SGBM算法（Semi-global Batch Matching）[26] H. Hirschmuller, “Stereo processing by semiglobal matching and mutual information,” IEEE Transactions on pattern analysis and machine intelligence, vol. 30, no. 2, pp. 328–341, 2008. 

计算左右图像的视差，然后通过双目相机的几何模型把它变换到相机的3D空间中。SGBM 使用了来自网络的经典参数配置，主要调整了最大和最小视差。视差数据结合相机的内参、基线，即能确定各点在三维空间中的位置。感兴趣可以阅读相关的参考文献[27, 28]。

[27] D. Scharstein and R. Szeliski, “A taxonomy and evaluation of dense two-frame stereo correspondence algorithms,” International journal of computer vision, vol. 47, no. 1-3, pp. 7–42, 2002.

[28] S. M. Seitz, B. Curless, J. Diebel, D. Scharstein, and R. Szeliski, “A comparison and evaluation of multi-view stereo reconstruction algorithms,” in null, pp. 519–528, IEEE, 2006.

#### 5.4.2 RGB-D 视觉

RGB-D相机能通过物理方法获得像素深度信息。如果已知相机的内外参，可以计算任何一个像素在世界坐标系下的位置，从而建立一张点云地图。

位于 slambook/ch5/rgbd 文件夹中有5对图像。在 color/下有 1.png 到 5.png 共 5 张 RGB 图，而在 depth/下有 5 张对应的深度图。同时，pose.txt 文件给出了5张图像的相机外参位姿。位姿记录的形式为平移向量加旋转四元数： [x, y, z, qx, qy, qz, qw], 其中 qw 是四元数的实部。

这一段程序，完成了两件事：(1). 根据内参计算一对 RGB-D图像对应的点云；(2). 根据各张图的相机位姿（也就是外参），把点云加起来，组成地图。 

slambook/ch5/rgbd/jointMap.cpp

运行程序如下：

![视觉SLAM十四讲-124](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-124.jpg)

### 习题 

1. 寻找一部相机，标定它的内参。可能会用到标定板， 或者棋盘格。 

2. 相机内参的物理意义。如果一部相机的分辨率变为原来的两倍而其他地方不变，它的内参如何变化？ 
3. 搜索特殊相机（鱼眼或全景相机）的标定方法。它们与普通的针孔模型有何不同？ 
4. 调研全局快门相机（global shutter）和卷帘快门相机（rolling shutter）的异同。它们在SLAM中有何优缺点？ 
5. RGB-D 相机是如何标定的？以 Kinect 为例，需要标定哪些参数？（参照https://github.com/code-iai/iai_kinect2）
6. 除了示例程序演示的遍历图像的方式，还能举出哪些遍历图像的方法？
7. 阅读 OpenCV 官方教程，学习它的基本用法。

习题有代码学习和工程应用的知识，后面实际开发中会很有帮助。



## 第六讲：非线性优化

经典SLAM模型的位姿可以由变换矩阵来描述，然后用李代数进行优化。观测方程由相机成像模型给出，其中内参是随相机固定的，而外参则是相机的位姿。由于噪声的存在，运动方程和观测方程的等式必定不是精确成立的。得到的数据通常是受各种未知噪声影响的。即使有高精度的相机，运动方程和观测方程也只能近似成立。所以问题是如何在有噪声的数据中进行准确的状态估计，这需要一定程度的最优化背景知识。

### 6.1 状态估计问题

#### 6.1.1 批量状态估计与最大后验估计

经典 SLAM 模型由一个运动方程和一个观测方程构成：

![视觉SLAM十四讲-125](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-125.jpg)                               

xk是相机的位姿变量，可以由Tk∈SE(3)表达。运动方程与输入的具体形式有关，在视觉SLAM中没有特殊性（和普通的机器人、车辆的情况一样）。观测方程则由针孔模型给定。假设在xk处对路标yj进行了一次观测，对应到图像上的像素位置z*k,j*，那么，观测方程可以表示成  

 ![视觉SLAM十四讲-126](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-126.png)

其中K为相机内参，*s*为像素点的距离，也是(R*k*y*j* +t*k*)的第三个分量。如果使用变换矩阵Tk描述位姿，那么路标点yj必须以齐次坐标来描述，计算完成后要转换为非齐次坐标。

在运动和观测方程中，通常假设两个噪声项wk,*v*k,j满足零均值的高斯分布：

 ![视觉SLAM十四讲-127](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-127.jpg)

其中N表示高斯分布，**0**表示零均值，Rk,Qk,j为协方差矩阵。在这些噪声的影响下，希望通过带噪声的数据z和u推断位姿x和地图y（以及它们的概率分布），这构成了一个状态估计问题。

处理这个状态估计问题的方法分成两种。

第一种：由于在SLAM过程中，这些数据是随时间逐渐过来的，所以在直观上，应该持有一个当前时刻的估计状态，然后用新的数据来更新它。这种方式称为增量（incremental）的方法，或者叫滤波器。在SLAM的早期研究，主要使用扩展卡尔曼滤波器（EKF）及其衍生方法来求解。

第二种：是把数据累加起来一并处理，这种方式称为批量（batch）的方法。例如，可以把0到*k*时刻所有的输入和观测数据都放在一起，求在这样的输入和观测下，如何估计整个0到*k*时刻的轨迹与地图？

增量方法仅关心当前时刻的状态估计xk，而对之前的状态则不多考虑；相对地，批量方法可以在更大的范围达到最优化，被认为优于传统的滤波器，成为当前视觉SLAM的主流方法。极端情况下，可以让机器人或无人机收集所有时刻的数据，再带回计算中心统一处理，这也正是SfM（Structure from Motion）的主流做法。这种极端情况不实时，不符合SLAM的运用场景。所以在SLAM中，实用的方法通常是一些折中的手段。比如，固定一些历史轨迹，仅对当前时刻附近的一些轨迹进行优化，这就是滑动窗口估计法。还有因子图增量平滑优化的方法，能够增量增加优化问题并进行动态调整，能够达到有滤波器的速度和图优化的精度。

先讨论批量方法，考虑从1到*N*的所有时刻，并假设有*M*个路标点。定义所有时刻的机器人位姿和路标点坐标为：

 ![视觉SLAM十四讲-128](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-128.jpg)

用不带下标的u表示所有时刻的输入，z表示所有时刻的观测数据。对机器人状态的估计，从概率学的观点来看，就是已知输入数据u和观测数据z的条件下，求状态x*,*y的条件概率分布：

 ![视觉SLAM十四讲-129](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-129.jpg)

特别地，当不知道控制输入，只有一张张图像时，即只考虑观测方程 带来的数据时，相当于估计*P*(x*,*y|z)的条件概率分布，此问题也称为Structure from Motion（SfM），即如何从许多图像中重建三维空间结构。

为了估计状态变量的条件分布，利用贝叶斯法则，有：

 ![视觉SLAM十四讲-130](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-130.jpg)

贝叶斯法则左侧称为后验概率，右侧的 *P*(z|x) 称为似然（Likehood），另一部分 *P*(x) 称为先验（Prior）。直接求后验分布是困难的，但是求一个状态最优估计，使得在该状态下后验概率最大化（Maximize a Posterior，MAP），则是可行的：

 ![视觉SLAM十四讲-131](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-131.jpg)

贝叶斯法则的分母部分与待估计的状态x*,*y无关，因而可以忽略。贝叶斯法则说明，求解最大后验概率等价于最大化似然和先验的乘积。进一步，如果不知道机器人位姿或路标大概在什么地方，此时就没有了先验。那么，可以求解最大似然估计（Maximize Likelihood Estimation，MLE）：

 ![视觉SLAM十四讲-132](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-132.jpg)

似然是指“在现在的位姿下，可能产生怎样的观测数据”。但是由于知道观测数据，所以最大似然估计可以理解成：“在什么样的状态下，最可能产生现在观测到的数据”。这就是最大似然估计的直观意义。

#### 6.1.2 最小二乘的引出

如何求最大似然估计呢？在高斯分布的假设下，最大似然能够有较简单的形式。回顾观测模型，对于某一次观测：

 ![视觉SLAM十四讲-133](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-133.jpg)

假设噪声项vk ∼ N (0,Qk,j)，观测数据的条件概率为：

 ![视觉SLAM十四讲-134](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-134.jpg)

它依然是一个高斯分布。考虑单次观测的最大似然估计，可以使用最小化负对数来求一个高斯分布的最大似然。

高斯分布在负对数下有较好的数学形式。考虑任意高维高斯分布x ∼ N(µ,Σ)，它的概率密度函数展开形式为：

![视觉SLAM十四讲-135](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-135.jpg)                 

对其取负对数，则变为：

![视觉SLAM十四讲-136](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-136.jpg)

因为对数函数是单调递增的，所以对原函数求最大化相当于对负对数求最小化。在最小化上式的x时，第一项与x无关，可以略去。于是，只要最小化右侧的二次型项，就得到了对状态的最大似然估计。代入SLAM的观测模型，相当于在求：

![视觉SLAM十四讲-137](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-137.jpg)

该式等价于最小化噪声项（即误差）的一个二次型。这个二次型称为马哈拉诺比斯距离（Mahalanobis distance），又叫马氏距离。它也可以看成是由(Qk,j)-1 加权之后的欧氏距离（二范数），这里(Qk,j)-1也叫做信息矩阵，即高斯分布协方差矩阵之逆。

现在考虑批量时刻的数据。通常假设各个时刻的输入和观测是相互独立的，这意味着各个输入之间是独立的，各个观测之间是独立的，并且输入和观测也是独立的。于是可以对联合分布进行因式分解：

![视觉SLAM十四讲-138](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-138.jpg)

这说明可以独立地处理各时刻的运动和观测。定义各次输入和观测数据与模型之间的误差：

 ![视觉SLAM十四讲-139](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-139.jpg)

那么，最小化所有时刻估计值与真实值之间的马氏距离，等价于求最大似然估计。负对数允许把乘积变成求和：

![视觉SLAM十四讲-140](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-140.jpg)

这样就得到了一个最小二乘问题（Least Square Problem），它的解等价于状态的最大似然估计。直观上看，由于噪声的存在，当把估计的轨迹与地图代入 SLAM 的运动、观测方程中时，它们并不会完美地成立。这时对状态的估计值进行微调，使得整体的误差下降一些。当然这个下降也有限度，它一般会到达一个极小值。这就是一个典型非线性优化的过程。

SLAM 中的最小二乘问题具有一些特定的结构：

•  整个问题的目标函数由许多个误差的（加权的）二次型组成。虽然总体的状态变量维数很高，但每个误差项都是简单的，仅与一两个状态变量有关。例如，运动误差只与x*k*−1*,*x*k* 有关，观测误差只与xk,yj有关。这种关系会让整个问题有一种稀疏的矩阵形式，计算量大大减少。

•  如果使用李代数表示增量，则该问题是无约束的最小二乘问题。但如果用旋转矩阵/变换矩阵描述位姿，则会引入旋转矩阵自身的约束，即需在问题中加入RTR = I且det(R) =1的条件。额外的约束会使优化变得更困难。这体现了李代数的优势。

• 使用了二次型度量误差。误差的分布将影响此项在整个问题中的权重。例如，某次的观测非常准确，那么协方差矩阵就会“小”，而信息矩阵就会“大”，所以这个误差项会在整个问题中占有较高的权重。

#### 6.1.3 例子：批量状态估计

考虑一个离散时间系统：

 ![视觉SLAM十四讲-141](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-141.jpg)

这可以表达一辆沿 *x* 轴前进或后退的汽车。第一个公式为运动方程，uk 为输入，wk 为噪声；第二个公式为观测方程，zk 为对汽车位置的测量。取时间 k = 1,...,3，现希望根据已有的 v,y 进行状态估计。设初始状态 *x*0 已知，来推导批量（batch）状态的最大似然估计。

首先，令批量状态变量为x = [*x*0*,x*1*,x*2*,x*3]T，令批量观测为z = [*z*1*,z*2*,z*3]T，定义u = [*u*1*,u*2*,u*3]T。最大似然估计为：

 ![视觉SLAM十四讲-142](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-142.jpg)

运动方程：

![视觉SLAM十四讲-143](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-143.jpg)

观测方程：

![视觉SLAM十四讲-144](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-144.jpg)

构建误差变量：

 ![视觉SLAM十四讲-145](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-145.jpg)

于是最小二乘的目标函数为

![视觉SLAM十四讲-146](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-146.jpg)            

这个系统是线性系统，将它写成向量形式。定义向量y = [u*,*z]T，那么可以写出矩阵H，使得：

 ![视觉SLAM十四讲-147](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-147.jpg)

那么：

![视觉SLAM十四讲-148](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-148.png)

![视觉SLAM十四讲-149](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-149.jpg)

整个问题可以写成：

![视觉SLAM十四讲-150](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-150.jpg)

这个问题有唯一的解：

![视觉SLAM十四讲-151](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-151.jpg)



### 6.2 非线性最小二乘

考虑一个最小二乘问题：

![视觉SLAM十四讲-152](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-152.jpg)                           

其中，自变量x ∈ Rn，f是任意标量非线性函数 f(x) : Rn→ R。注意这里的系数1/2是无关紧要的。如何求解这样一个优化问题：如果 f 是个数学形式上很简单的函数，那么该问题可以用解析形式来求。令目标函数的导数为零，然后求解x的最优值，就和求二元函数的极值一样：

![视觉SLAM十四讲-153](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-153.jpg)

解此方程，就得到了导数为零处的极值。可能是极大、极小或鞍点处的值，只要逐个比较函数值大小即可。如果 *f* 为简单的线性函数，那么这个问题就是简单的线性最小二乘问题，但是有些导函数形式复杂，使得该方程不容易求解。求解这个方程需要知道关于目标函数的全局性质，而通常这是不大可能的。对于不方便直接求解的最小二乘问题，可以用迭代的方式，从一个初始值出发，不断地更新当前的优化变量，使目标函数下降。具体步骤可列写如下：

1. 给定某个初始值x0。  

2. 对于第 *k* 次迭代，寻找一个增量 ∆x*k*，使得

   ![视觉SLAM十四讲-154](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-154.jpg)

   达到极小值。  

3. 若 ∆xk 足够小，则停止。 

4. 否则，令x(k+1) =  xk +  ∆xk，返回第 2 步。  

这让求解导函数为零的问题变成了一个不断寻找下降增量 ∆xk 的问题。由于可以对f进行线性化，增量的计算将简单很多。当函数下降直到增量非常小的时候，就认为算法收敛，目标函数达到了一个极小值。在这个过程中，问题在于如何找到每次迭代点的增量，而这是一个局部的问题，只需要关心 *f* 在迭代值处的局部性质而非全局性质。这类方法在最优化、机器学习等领域应用非常广泛。

下面是如何寻找这个增量 ∆xk的方法。

#### 6.2.1 一阶和二阶梯度法

考虑第 k 次迭代，假设在xk处，想要找到增量 ∆xk，最直观的方式是将目标函数在xk附近进行泰勒展开：

![视觉SLAM十四讲-155](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-155.jpg)

其中J(xk) 是 F(x)关于x的一阶导数（也叫梯度、雅可比矩阵﹝Jacobian﹞），H 则是二阶导数（海塞﹝Hessian﹞矩阵），它们都在xk 处取值。可以选择保留泰勒展开的一阶或二阶项，那么对应的求解方法则称为一阶梯度或二阶梯度法。如果保留一阶梯度，取增量为反向的梯度，即可保证函数下降：∆x∗ = −J(xk)           

这只是个方向，通常还要再指定一个步长 λ。步长可以根据一定的条件来计算，在机器学习当中也有一些经验性质的方法。这种方法被称为最速下降法。它的直观意义是：只要沿着反向梯度方向前进，在一阶（线性）的近似下，目标函数必定会下降。

以上都是在第k次迭代时进行的，并不涉及其他的迭代信息。为了简化符号，省略下标k，并认为这些对任意一次迭代都成立。

选择保留二阶梯度信息，此时增量方程为：

![视觉SLAM十四讲-156](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-156.jpg)

右侧只含 ∆x的零次、一次和二次项。求右侧等式关于 ∆x的导数并令它为零，得到：

![视觉SLAM十四讲-157](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-157.jpg)

求解这个线性方程就得到了增量。此类方法又称为牛顿法。

一阶和二阶梯度法都十分直观，只要把函数在迭代点附近进行泰勒展开，并针对更新量做最小化即可。用一个一次或二次的函数近似原函数，然后用近似函数的最小值来估计原函数的极小值。只要原目标函数局部看起来像一次或二次函数，这类算法就是成立的。不过这两种方法也存在问题：最速下降法过于贪心，容易走出锯齿路线，反而增加了迭代次数。而牛顿法则需要计算目标函数的H矩阵，这在问题规模较大时非常困难，通常倾向于避免H的计算。所以引出了下面的一些方法。

#### 6.2.2 高斯牛顿法

牛顿法是对目标函数 F(x) 进行一阶泰勒展开，而高斯牛顿法是对f(x)进行一阶泰勒展开：f (x + ∆x) ≈ f(x) + J(x)T∆x. 

注：把 J(x) 写成列向量，那么它可以和 ∆x 进行内积，得到一个标量。

这里J(x)T为f(x)关于x的导数，为n×1的列向量。目标是寻找增量∆x，使得∥f(x+∆x)∥2达到最小。为了求 ∆x，需要解一个线性的最小二乘问题：

![视觉SLAM十四讲-159](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-159.jpg)

根据极值条件，将上述目标函数对 ∆x求导，并令导数为零。可以得到如下方程组：

 ![视觉SLAM十四讲-160](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-160.jpg)

这个方程是关于变量∆x的线性方程组，称它为增量方程，也可以称为高斯牛顿方程（GaussNewton equation）或者正规方程（Normal equation）。把左边的系数定义为H，右边定义为g，那么上式变为：H∆x = g.             

对比牛顿法可见，高斯牛顿法用JJT作为牛顿法中二阶Hessian矩阵的近似，从而省略了计算H的过程。求解增量方程是整个优化问题的核心所在。高斯牛顿法的算法步骤可以写成：

1. 给定初始值x0。

2. 对于第 k 次迭代，求出当前的雅可比矩阵J(xk) 和误差f(xk)。

3. 求解增量方程：H∆x*k* = g。

4.  若 ∆xk 足够小，则停止。否则，令x(k+1) = xk+ ∆xk，返回第 2 步。

从算法步骤中，主要还是增量的求解。只要能够顺利解出增量，就能保证目标函数能够正确地下降。

为了求解增量方程，需要求解H−1，这需要H矩阵可逆，但实际数据中计算得到的JJT只有半正定性。也就是说，在使用高斯牛顿法时，可能出现JJT为奇异矩阵或者病态（ill-condition）的情况，此时增量的稳定性较差，导致算法不收敛。直观地说，原函数在这个点的局部近似不像一个二次函数。假设H非奇异也非病态，如果求出来的步长∆x太大，也会导致采用的局部近似式：f (x + ∆x) ≈ f(x) + J(x)T∆x.不够准确，这样一来无法保证它的迭代收敛。

在非线性优化领域，相当多的算法都可以归结为高斯牛顿法的变种。这些算法都借助了高斯牛顿法的思想并且通过改进修正其缺点。例如一些线搜索方法 (line search method) 加入了一个步长α，在确定了∆x后进一步找到 α使得 ∥f(x + α∆x)∥2 达到最小，而不是简单地令 α = 1。

列文伯格—马夸尔特方法在一定程度上修正了这些问题，比高斯牛顿法更为鲁棒，但收敛速度会比高斯牛顿法更慢，被称为阻尼牛顿法（Damped Newton Method）。

#### 6.2.3 列文伯格—马夸尔特方法

高斯牛顿法中采用的近似二阶泰勒展开只能在展开点附近有较好的近似效果，所以应该给∆x添加一个范围，称为信赖区域（Trust Region）。这个范围定义了在什么情况下二阶近似是有效的，这类方法也称为信赖区域方法（Trust Region Method）。在信赖区域里边，近似是有效的；出了这个区域，近似可能会出问题。

那么如何确定这个信赖区域的范围呢？一个比较好的方法是根据近似模型跟实际函数之间的差异来确定：如果差异小，说明近似效果好，扩大近似的范围；反之，如果差异大，就缩小近似的范围。我们定义一个指标 *ρ* 来刻画近似的好坏程度：

![视觉SLAM十四讲-161](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-161.jpg)

*ρ* 的分子是实际函数下降的值，分母是近似模型下降的值。如果 *ρ* 接近于 1，则近似是好的。如果 *ρ* 太小，说明实际减小的值远少于近似减小的值，则认为近似比较差，需要缩小近似范围。反之，如果 *ρ* 比较大，则说明实际下降的比预计的更大，可以放大近似范围。

于是构建一个改良版的非线性优化框架，该框架会比高斯牛顿法有更好的效果：

1. 给定初始值 x0，以及初始优化半径 µ。 

2. 对于第 k 次迭代，在高斯牛顿法的基础上加上信赖区域，求解：

   ![视觉SLAM十四讲-162](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-162.jpg)

   其中 µ 是信赖区域的半径，D 为系数矩阵。 

3. 计算 ρ。 

4. 若 ρ > 3/4，则设置 µ = 2µ。 

5. 若 ρ < 1/4，则设置 µ = 0.5µ。

6. 如果 ρ 大于某阈值，则认为近似可行。令 xk+1 = xk + ∆xk。 

7. 判断算法是否收敛。如不收敛则返回第 2 步，否则结束。

这里近似范围扩大的倍数和阈值都是经验值，可以替换成别的数值。

![视觉SLAM十四讲-162](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-162.jpg)

这个式子中，把增量限定于一个半径为 *µ* 的球中，认为只在这个球内才是有效的。带上D之后，这个球可以看成一个椭球。在列文伯格提出的优化方法中，把D取成单位阵I，相当于直接把 ∆x*k* 约束在一个球中。随后，马夸尔特提出将D取成非负数对角阵——实际中通常用J*T*J 的对角元素平方根，使得在梯度小的维度上约束范围更大一些。

在列文伯格—马夸尔特优化中，需要求解这个子问题来获得梯度。

这个子问题是带不等式约束的优化问题，用拉格朗日乘子把约束项放到目标函数中，构成拉格朗日函数：

![视觉SLAM十四讲-163](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-163.jpg)

这里 *λ* 为拉格朗日乘子。类似于高斯牛顿法中的做法，令该拉格朗日函数关于∆x的导数为零，它的核心仍是计算增量的线性方程：

![视觉SLAM十四讲-164](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-164.jpg)     

可以看到，增量方程相比于高斯牛顿法，多了一项 *λ*D*T*D。考虑它的简化形式，即D = I，那么相当于求解：

(H + *λ*I)∆x*k* = g*.*

当参数λ比较小时，H占主要地位，这说明二次近似模型在该范围内是比较好的，列文伯格—马夸尔特方法更接近于高斯牛顿法。另一方面，当λ比较大时，λI占据主要地位，列文伯格—马夸尔特方法更接近于一阶梯度下降法（即最速下降），这说明附近的二次近似不够好。列文伯格—马夸尔特方法的求解方式，可在一定程度上避免线性方程组的系数矩阵的非奇异和病态问题，提供更稳定、更准确的增量 ∆x。

在实际中，还存在许多其他的方式来求解增量，例如

[31] J. Nocedal and S. Wright, Numerical Optimization. Springer Science & Business Media, 2006.

实际问题中，通常选择高斯牛顿法或列文伯格—马夸尔特方法其中之一作为梯度下降策略。当问题性质较好时，用高斯牛顿。如果问题接近病态，则用列文伯格—马夸尔特方法。

#### 小结

专门介绍数值优化的书籍

[31] J. Nocedal and S. Wright, Numerical Optimization. Springer Science & Business Media, 2006.

以高斯牛顿法和列文伯格—马夸尔特方法为代表的优化方法，在很多开源的优化库中都已经实现并提供给用户。最优化是处理许多实际问题的基本数学工具，不光在视觉 SLAM 中起着核心作用，在类似于深度学习等其他领域，它也是求解问题的核心方法之一（深度学习数据量很大，以一阶方法为主）。

无论是高斯牛顿法还是列文伯格—马夸尔特方法，在做最优化计算时，都需要提供变量的初始值。这个初始值不能随意设置。实际上非线性优化的所有迭代求解方案，都需要用户来提供一个良好的初始值。由于目标函数太复杂，导致在求解空间上的变化难以预测，对问题提供不同的初始值往往会导致不同的计算结果。这种情况是非线性优化的通病：大多数算法都容易陷入局部极小值。因此，无论是哪类科学问题，提供初始值都应该有科学依据，例如视觉 SLAM 问题中，会用 ICP、PnP 之类的算法提供优化初始值（视觉前端）。一个良好的初始值对最优化问题非常重要。

如何求解线性增量方程组呢？在视觉SLAM算法里，经常遇到∆x的维度很大，如果是要做大规模的视觉三维重建，就会经常发现这个维度可以轻易达到几十万甚至更高的级别。要对那么大个矩阵进行求逆是大多数处理器无法负担的，因此存在着许多针对线性方程组的数值求解方法。在不同的领域有不同的求解方式，但几乎没有一种方式是直接求系数矩阵的逆，会采用矩阵分解的方法来解线性方程，例如 QR、Cholesky 等分解方法。(工程数学)

视觉SLAM里这个矩阵往往有特定的稀疏形式，这为实时求解优化问题提供了可能性。利用稀疏形式的消元、分解，最后再进行求解增量，会让求解的效率大大提高。在很多开源的优化库上，比如GTSAM，维度为一万多的变量在一般的PC上就可以在几秒甚至更短的时间内被求解出来，其原因也是用了更加高级的数学工具。（因子图增量平滑优化）视觉SLAM算法现在能够实时地实现，多亏了系数矩阵是稀疏的，如果矩阵是稠密的，优化这类视觉SLAM算法不会被学界广泛采纳了。

### 6.3 实践：曲线拟合问题

#### 6.3.1手写高斯牛顿法

考虑一条满足以下方程的曲线：

![视觉SLAM十四讲-165](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-165.jpg)

其中 *a,b,c* 为曲线的参数，*w* 为高斯噪声，满足 *w* ∼ (0*,σ*2)。假设有 *N* 个关于 *x,y* 的观测数据点，根据这些数据点求出曲线的参数。那么，可以求解下面的最小二乘问题来估计曲线参数：

![视觉SLAM十四讲-166](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-166.jpg)

在这个问题中，待估计的变量是 *a,b,c*，而不是 *x*。程序里先根据模型生成 *x,y* 的真值，然后在真值中添加高斯分布的噪声。随后，使用高斯牛顿法来从带噪声的数据（x，y）拟合参数模型。定义误差为:

![视觉SLAM十四讲-167](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-167.jpg)

那么可以求出每个误差项对于状态变量的导数：  

![视觉SLAM十四讲-168](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-168.png)

于是

![视觉SLAM十四讲-169](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-169.jpg)

高斯牛顿法的增量方程为：

![视觉SLAM十四讲-170](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-170.jpg)              

也可以选择把所有的J*i* 排成一列，将这个方程写成矩阵形式，它的含义与求和形式是一致的。下面的代码演示了这个过程是如何进行的：slambook2/ch6/gaussNewton.cpp

在这个例子中演示了如何对一个简单的拟合问题进行迭代优化。该程序输出每一步迭代的目标函数值和更新量，如下：

![视觉SLAM十四讲-172](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-172.jpg)

整个问题的目标函数在迭代 9 次之后趋近收敛，更新量趋近于零。最终估计的值与真值接近，函数图像如下：

![视觉SLAM十四讲-173](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-173.jpg)

蓝色点为100个数据点，黑色线为理论模型，红色线为拟合的模型。

#### 6.3.2使用 Ceres 进行曲线拟合

**Ceres 简介**

Google Ceres 是一个广泛使用的最小二乘问题求解库。在 Ceres 中，只需按照一定步骤定义待解的优化问题，然后交给求解器计算即可。Ceres 求解的最小二乘问题最一般的形式如下（带边界的核函数最小二乘）：

![视觉SLAM十四讲-171](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-171.jpg)

在这个问题中，*x*1*,*··· *,xn* 为优化变量，又称参数块（Parameter blocks），*fi* 称为代价函数（Cost function），也称为残差块（Residual blocks），在 SLAM 中也可理解为误差项。*lj* 和 *uj* 为第 *j* 个优化变量的上限和下限。在最简单的情况下，取 *lj* = −∞*,uj* = ∞（不限制优化变量的边界）。此时，目标函数由许多平方项经过一个核函数 *ρ*(·) 之后求和组成。取 *ρ* 为恒等函数，那么目标函数即为许多项的平方和，就得到了无约束的最小二乘问题。为了让 Ceres 求解这个问题，需要做以下几件事：

1. 定义每个参数块。参数块通常为向量，但是在SLAM里也可以定义成四元数、李代数这种特殊的结构。如果是向量，那么需要为每个参数块分配一个 double 数组，来存储变量的值。

2. 然后定义残差块的计算方式。残差块通常关联若干个参数块，对它们进行一些自定义的计算，然后返回残差值。Ceres 对它们求平方和之后，作为目标函数的值。

3. 残差块往往也需要定义雅可比的计算方式。在 Ceres 中，可以使用自动求导功能，也可以手动指定雅可比的计算过程。如果要使用自动求导，那么残差块需要按照特定的写法来书写：残差的计算过程是一个带模板的括号运算符。

4. 最后，把所有的参数块和残差块加入 Ceres 定义的 Problem 对象中，调用 Solve 函数求解即可。求解之前，可以传入一些配置信息，例如迭代次数、终止条件等，也可以使用默认的配置。

**安装 Ceres**

Ceres 的 github 地址为：[https://github.com/ ](https://github.com/ceres-solver/ceres-solver)[ceres-solver/ceres-solver](https://github.com/ceres-solver/ceres-solver)，

安装依赖项

```shell
sudo apt−get install liblapack−dev libsuitesparse−dev libcxsparse3  libgflags−dev libgoogle−glog−dev libgtest−dev  
```

然后进入 Ceres 库目录下，使用 cmake 编译并安装。安装完成后，在/usr/local/include/ceres 下找到 Ceres 的头文件，并在/usr/local/lib/下找到名为 libceres.a 的库文件。有了这些文件，就可以使用 Ceres 进行优化计算了。

##### 使用 Ceres 拟合曲线

下面的代码演示了如何使用 Ceres 求解同样的问题：slambook/ch6/ceresCurveFitting.cpp

利用 OpenCV 的噪声生成器生成了 100 个带高斯噪声的数据，随后利用 Ceres 进行拟合。这里Ceres用法有如下几项：

1. 定义残差块的类。方法是书写一个类（或结构体），并在类中定义带模板参数的 () 运算符，这样该类就成为了一个拟函数（Functor），或者叫仿函数。这种定义方式使得 Ceres 可以像调用函数一样，对该类的某个对象（比如 a）调用 a<double>() 方法。Ceres 会把雅可比矩阵作为类型参数传入此函数，从而实现自动求导的功能。

2. 程序中的 double abc[3] 即为参数块，而对于残差块，对每一个数据构造 CURVE_FIT-TING_COST 对象，然后调用 AddResidualBlock 将误差项添加到目标函数中。由于优化需要梯度，有若干种选择：（1）使用 Ceres 的自动求导（Auto Diff）；（2）使用数值求导（Numeric Diff）；（3）自行推导解析的导数形式，提供给 Ceres。

3. 自动求导需要指定误差项和优化变量的维度。这里的误差是标量，维度为 1；优化的是 *a,b,c* 三个量，维度为 3。于是，在自动求导类 AutoDiffCostFunction 的模板参数中设定变量维度为 1、3。

4. 设定好问题后，调用 Solve 函数进行求解。可以在options里配置优化选项。例如，可以选择使用 Line Search 还是 Trust Region、迭代次数、步长，等等。可以查看 Options 的定义，看看有哪些优化方法可选，一般默认的配置已经可用于很广泛的问题了。

最终的优化值和手写的基本相同，但运行速度上 Ceres 要相对慢一些。

Ceres的优点是提供了自动求导工具，不必去计算很麻烦的雅可比矩阵。Ceres的自动求导是通过模板元实现的，在编译时期就可以完成自动求导工作，不过仍然是数值导数。此外，Ceres的优化过程配置也很丰富，使其适合很广泛的最小二乘优化问题，包括 SLAM 之外的各种问题。

注：自动求导也是用数值导数实现的。

#### 6.3.3使用 g2o 进行曲线拟合

g2o（General Graphic Optimization，G2O）是在SLAM领域广为使用的优化库。它是一个基于图优化的库。图优化是一种将非线性优化与图论结合起来的理论。

**图优化理论简介**

前面介绍了非线性最小二乘的求解方式。它们是由很多个误差项之和组成的。然而，仅有一组优化变量和许多个误差项，并不清楚它们之间的关联。比如，某个优化变量xj存在于多少个误差项中呢？能保证对它的优化是有意义的吗？进一步，希望能够直观地看到该优化问题长什么样。于是，就引出了图优化。

图优化，是把优化问题表现成图（Graph）的一种方式。这里的图是图论意义上的图。一个图由若干个顶点（Vertex），以及连接着这些顶点的边（Edge）组成。进而，用顶点表示优化变量，用边表示误差项。于是，对任意一个上述形式的非线性最小二乘问题，可以构建与之对应的一个图。可以简单地称它为图，也可以用概率图里的定义，称之为贝叶斯图或因子图。

如下图：

![视觉SLAM十四讲-174](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-174.jpg)

用三角形表示相机位姿节点，用圆形表示路标点，它们构成了图优化的顶点；同时，实线表示相机的运动模型，虚线表示观测模型，它们构成了图优化的边。最基本的图优化是用图模型来表达一个非线性最小二乘的优化问题，可以利用图模型的某些性质做更好的优化。

g2o 是一个通用的图优化库，可以在g2o里求解任何能够表示为图优化的最小二乘问题，包括曲线拟合问题。

**g2o 的编译与安装**

安装依赖：

```shell
sudo apt−get install qt5−qmake qt5−default libqglviewer−dev−qt5 libsuitesparse−dev libcxsparse3 libcholmod3
```

从GitHub下载并cmake安装：https://github.com/RainerKuemmerle/g2o

安装完成后， g2o 的头文件将位于/usr/local/g2o 下，库文件位于/usr/local/lib/下

**使用 g2o 拟合曲线**

首先要将曲线拟合问题抽象成图优化。这个过程中，节点为优化变量， 边为误差项。

在曲线拟合问题中，整个问题只有一个顶点：曲线模型的参数 a, b, c；而各个带噪声的数据点， 构成了一个个误差项，也就是图优化的边。这里的边是一元边（Unary Edge），即只连接一个顶点。事实上，图优化中一条边可以连接一个、两个或多个顶点，这主要反映每个误差与多少个优化变量有关。

主要步骤：

1. 定义顶点和边的类型。 
2. 构建图。 
3. 选择优化算法。 
4. 调用g2o进行优化，返回结果。 

程序：slambook/ch6/g2oCurveFitting.cpp

在这个程序中，从g2o派生出了用于曲线拟合的图优化顶点和边：CurveFittingVertex 和 CurveFittingEdge，这实质上扩展了g2o的使用方式。这两个类分别派生于BaseVertex和BaseUnaryEdge类。在派生类中，重写了重要的虚函数： 

1. 顶点的更新函数：oplusImpl。优化过程最重要的是增量∆x的计算，而该函数处理的是 xk+1 = xk + ∆x 的过程。在曲线拟合过程中，由于优化变量（曲线参数）本身位于向量空间中，这个更新计算就是简单的加法。但是当优化变量不在向量空间中时，比如x是相机位姿，它本身不一定有加法运算。这时就需要重新定义增量如何加到现有的估计上的行为了。 
2.  顶点的重置函数：setToOriginImpl。即把估计值置零即可。 
3. 边的误差计算函数：computeError。该函数需要取出边所连接的顶点的当前估计值，根据曲线模型，与它的观测值进行比较。这和最小二乘问题中的误差模型是一致的。 
4. 边的雅可比计算函数：linearizeOplus。这个函数里计算了每条边相对于顶点的雅可比。 
5. 存盘和读盘函数：read、write。

定义了顶点和边之后，在main函数里声明了一个图模型，然后按照生成的噪声数据，往图模型中添加顶点和边，最后调用优化函数进行优化。g2o 会给出优化的结果。

### 6.4 小结 

非线性优化问题：由许多个误差项平方和组成的最小二乘问题。讨论了两种主要的梯度下降方式：高斯牛顿法和列文伯格 —马夸尔特方法。在实践部分中，分别使用了手写高斯牛顿法、Ceres 和 g2o 两种优化库求解同一个 曲线拟合问题，发现结果相似。 特别地，如果用g2o来拟合曲线，必须先把问题转换为图优化，定义新的顶点和边。相比之下，Ceres 定义误差项求曲线拟合问题则自然了很多，因为它本身即是一个优化库。然而，在 SLAM 中更多的问题是，一个带有许多个相机位姿和许多个空间点的优化问题如何求解。特别地，当相机位姿以李代数表示时，误差项关于相机位姿的导数如何计算。g2o提供了大量现成的顶点和边，非常便于相机位姿估计问题。而在 Ceres 中， 不得不自己实现每一个Cost Function，有一些不便。 

Ceres 库提供了基于模板元的自动求导和运行时的数值求导，而 g2o 只提供了运行时数值求导这一种方式。但是对于大多数问题，如果能够推导出雅可比矩阵的解析形式并告诉优化库，就可以避免数值求导中的诸多问题。

### 习题

1. 证明线性方程 Ax = b 当系数矩阵 A 超定时，最小二乘解为 x = (ATA) −1ATb。 
2. 调研最速下降法、牛顿法、高斯牛顿法和列文伯格—马夸尔特方法各有什么优缺点。
3. 为什么高斯牛顿法的增量方程系数矩阵可能不正定？不正定有什么几何含义？为什么在这种情况下解就不稳定了？ 
4. DogLeg 是什么？它与高斯牛顿法和列文伯格—马夸尔特方法有何异同？
5.  阅读 Ceres 的教学材料（http://ceres-solver.org/tutorial.html）以更好地掌 握其用法。 
6. 阅读 g2o 自带的文档 
7. 更改曲线拟合实验中的曲线模型，并用 Ceres 和 g2o 进行优化实验。

## 第七章 视觉里程计 1

### 7.1特征点法

一个 SLAM 系统分为前端和后端,其中是视觉SLAM前端也称为视觉里程计(VO)。VO根据相邻图像的信息估计出粗略的相机运动,给后端提供较好的初始值。VO的算法主要分为两个大类:特征点法和直接法。基于特征点法的前端,被认为是视觉里程计的主流方法。它具有稳定,对光照、动态物体不敏感的优势,是目前比较成熟的解决方案。如何提取、匹配图像特征点,然后估计两帧之间的相机运动和场景结构,从而实现一个两帧间视觉里程计。这类算法也称为两视图几何(Two-view geometry)。

#### 7.1.1 特征点

VO的核心问题是如何根据图像来估计相机运动。然而,图像本身是一个由亮度和色彩组成的矩阵,如果直接从矩阵层面考虑运动估计,将会非常困难。所以,比较方便的做法是:首先,从图像中选取比较有代表性的点。这些点在相机视角发生少量变化后会保持不变,于是能在各个图像中找到相同的点。然后,在这些点的基础上,讨论相机位姿估计问题,以及这些点的定位问题。在经典SLAM模型中称这些点为路标(Landmark)。而在视觉 SLAM 中,路标则是指图像特征(Feature)。其定义为：一组与计算任务相关的信息,计算任务取决于具体的应用。简而言之,特征是图像信息的另一种数字表达形式。一组好的特征对于在指定任务上的最终表现至关重要。数字图像在计算机中以灰度值矩阵的方式存储,所以最简单的单个图像像素也是一种“特征”。但是,在视觉里程计中希望特征点在相机运动之后保持稳定,而灰度值受光照、形变、物体材质的影响严重,在不同图像间变化非常大,不够稳定。理想的情况是,当场景和相机视角发生少量改变时,算法还能从图像中判断哪些地方是同一个点。所以,仅凭灰度值是不够的,需要对图像提取特征点。
特征点是图像里一些特别的地方，可以把图像中的角点、边缘和区块都当成图像中有代表性的地方。不过更容易精确地指出,某两幅图像中出现了同一个角点;同一个边缘则稍微困难一些,因为沿着该边缘前进,图像局部是相似的;同一个区块则是最困难的。可以发现,图像中的角点、边缘相比于像素区块而言更加“特别”,在不同图像之间的辨识度更强。所以,一种直观的提取特征的方式就是在不同图像间辨认角点,确定它们的对应关系。在这种做法中,角点就是所谓的特征。角点的提取算法有很多,例如 Harris 角点、FAST 角点 、GFTT 角点等等。然而,在大多数应用中,单纯的角点依然不能满足很多需求。例如,从远处看上去是角点的地方,当相机走近之后,可能就不显示为角点了。或者,当旋转相机时,角点的外观会发生变化,也就不容易辨认出那是同一个角点。为此,就提出了许多更加稳定的局部图像特征,如著名的SIFT 、SURF 、ORB ,等等。相比于普通角点,这些人工设计的特征点能够拥有如下的性质:

1. 可重复性(Repeatability):相同的特征可以在不同的图像中找到。
2. 可区别性(Distinctiveness):不同的特征有不同的表达。
3. 高效率(Efficiency):同一图像中,特征点的数量应远小于像素的数量。
4. 本地性(Locality):特征仅与一小片图像区域相关。

特征点由关键点(Key-point)和描述子(Descriptor)两部分组成。关键点是指该特征点在图像里的位置,有些特征点还具有朝向、大小等信息。描述子通常是一个向量,按照某种人为设计的方式,描述了该关键点周围像素的信息。描述子是按照“外观相似的特征应该有相似的描述子”的原则设计的。因此,只要两个特征点的描述子在向量空间上的距离相近,就可以认为它们是同样的特征点。
SIFT(尺度不变特征变换,Scale-Invariant FeatureTransform)：充分考虑了在图像变换过程中出现的光照、尺度、旋转等变化,但随之而来的是极大的计算量。
FAST关键点则考虑适当降低精度和鲁棒性,以提升计算的速度，属于计算特别快的一种特征点(没有描述子)；而 ORB(Oriented FASTand Rotated BRIEF)特征则是改进了 FAST 检测子不具有方向性的问题,并采用速度极快的二进制描述子BRIEF,使整个图像特征提取的环节大大加速。根据作者在论文中所述测试,在同一幅图像中同时提取约 1000 个特征点的情况下,ORB 约要花费 15.3ms,SIFT 约花费 5228.7ms。由此可以看出,ORB在保持了特征子具有旋转、尺度不变性的同时,速度方面提升明显,对于实时性要求很高的SLAM来说是一个很好的选择。大部分特征提取都具有较好的并行性,可以通过 GPU 等设备来加速计算。经过GPU加速后的SIFT,就可以满足实时计算要求。但是,引入GPU将带来整个SLAM成本的提升。在目前的 SLAM方案中,ORB 是质量与性能之间较好的折中。

相关书籍和论文：

[39] E. Rosten and T. Drummond, “Machine learning for high-speed corner detection,” in Computer Vision–ECCV 2006, pp. 430–443, Springer, 2006.

[41] D. G. Lowe, “Distinctive image features from scale-invariant keypoints,” International Journal of Computer Vision, vol. 60, no. 2, pp. 91–110, 2004.

[43] E. Rublee, V. Rabaud, K. Konolige, and G. Bradski, “Orb: an efficient alternative to sift or surf,” in 2011 IEEE International Conference on Computer Vision (ICCV), pp. 2564–2571, IEEE, 2011.

[45] M. Nixon and A. S. Aguado, Feature extraction and image processing for computer vision. Academic Press, 2012.

#### 7.1.2 ORB 特征

ORB特征也由关键点和描述子两部分组成。它的关键点称为“Oriented FAST”,是一种改进的FAST角点。它的描述子称为BRIEF(Binary Robust Independent Elementary Feature)。提取 ORB 特征分为如下两个步骤:
1. FAST 角点提取:找出图像中的“角点”。相较于原版的 FAST,ORB中计算了特征点的主方向,为后续的BRIEF描述子增加了旋转不变特性。
2. BRIEF 描述子:对前一步提取出特征点的周围图像区域进行描述。ORB 对BRIEF进行了一些改进,主要是指在 BRIEF中使用了先前计算的方向信息。

**FAST 关键点**
FAST 是一种角点,主要检测局部像素灰度变化明显的地方,以速度快著称。它的思想是:如果一个像素与邻域的像素差别较大(过亮或过暗),那么它更可能是角点。相比于其他角点检测算法,FAST 只需比较像素亮度的大小,十分快捷。它的检测过程如下:

![视觉SLAM十四讲-175](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-175.jpg)

1. 在图像中选取像素p,假设它的亮度为Ip。

2. 设置一个阈值T (比如,Ip 的20%)。

3. 以像素p为中心,选取半径为3的圆上的16个像素点。

4. 假如选取的圆上有连续的N个点的亮度大于 Ip + T 或小于 Ip − T,那么像素p可以被认为是特征点(N通常取12,即为 FAST-12。其他常用的 N 取值为 9 和 11,它们分别被称为FAST-9 和 FAST-11)。

5. 循环以上四步,对每一个像素执行相同的操作。

在 FAST-12 算法中,为了更高效,可以添加一项预测试操作,以快速地排除绝大多数不是角点的像素。具体操作为,对于每个像素,直接检测邻域圆上的第 1, 5, 9, 13 个像素的亮度。只有当这 4个像素中有 3 个同时大于 Ip + T 或小于 Ip − T 时,当前像素才有可能是一个角点,否则应该直接排除。这样的预测试操作大大加速了角点检测。此外,原始的 FAST角点经常出现“扎堆”的现象。所以在第一遍检测之后，还需要用非极大值抑制（Non-maximal suppression），在一定区域内仅保留响应极大值的角点，避免角点集中的问题。

FAST特征点的计算仅仅是比较像素间亮度的差异，所以速度非常快，但它也有重复性不强，分布不均匀的缺点。此外，FAST角点不具有方向信息。同时，由于它固定取半径为3的圆，存在尺度问题：远处看着像是角点的地方，接近后看可能就不是角点了。针对FAST角点不具有方向性和尺度的弱点，ORB添加了尺度和旋转的描述。尺度不变性由构建图像金字塔，并在金字塔的每一层上检测角点来实现。而特征的旋转是由灰度质心法（Intensity Centroid）实现的。 金字塔是计算图视觉中常用的一种处理方法，如下图。

![视觉SLAM十四讲-176](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-176.jpg)

金字塔底层是原始图像。每往上一层，就对图像进行一个固定倍率的缩放，这样就有了不同分辨率的图像。较小的图像可以看成是远处看过来的场景。在特征匹配算法中可以匹配不同层上的图像，从而实现尺度不变性。 例如，如果相机在后退，那么应该能够在上一个图像金字塔的上层和下一个图像的下层中找到匹配。 在旋转方面，计算特征点附近的图像灰度质心。所谓质心是指以图像块灰度值作为权重的中心。其具体操作步骤如下： 

1. 在一个小的图像块B中，定义图像块的矩为

![视觉SLAM十四讲-177](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-177.jpg)

2. 通过矩可以找到图像块的质心：

![视觉SLAM十四讲-178](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-178.png)

3. 连接图像块的几何中心O与质心 C，得到一个方向向量 OC，于是特征点的方向可以定义为 

![视觉SLAM十四讲-179](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-179.jpg)

通过以上方法，FAST角点便具有了尺度与旋转的描述，从而大大提升了其在不同图像之间表述的鲁棒性。ORB 中，把这种改进后的FAST称为 Oriented FAST。

注：金字塔是指对图像进行不同层次的降采样，以获得不同分辨率的图像

**BRIEF 描述子** 

在提取 Oriented FAST 关键点后，对每个点计算其描述子。ORB使用改进的BRIEF特征描述。

BRIEF 是一种二进制描述子，其描述向量由许多个0和1组成，这里的0和1编码了关键点附近两个随机像素（比如 p 和 q）的大小关系：如果 p 比 q 大，则取 1，反之就取 0。如果取了 128 个这样的 p, q，最后就得到128维由 0、1 组成的向量。BRIEF 使用了随机选点的比较，速度非常快，而且由于使用了二进制表达，存储起来也十分方便，适用于实时的图像匹配。原始的 BRIEF描述子不具有旋转不变性，因此在图像发生旋转时容易丢失。而 ORB 在 FAST 特征点提取阶段计算了关键点的方向，所以可以利用方向信息，计算了旋转之后的“Steer BRIEF”特征使ORB 的描述子具有较好的旋转不变性。 由于考虑到了旋转和缩放，使得 ORB 在平移、旋转和缩放的变换下仍有良好的表现。同时，FAST和BREIF的组合也非常高效，使得ORB特征在实时SLAM中非常受欢迎。

#### 7.1.3 特征匹配

特征匹配是视觉 SLAM 中极为关键的一步，解决了SLAM中的数据关联问题（data association），即确定当前看到的路标与之前看到的路标之间的对应关系。通过对图像与图像或者图像与地图之间的描述子进行准确匹配，可以为后续的姿态估计、优化等操作减轻大量负担。然而，由于图像特征的局部特性，误匹配的情况广泛存在，成为视觉SLAM中制约性能提升的一大瓶颈。部分原因是场景中经常存在大量的重复纹理，使得特征描述非常相似。在这种情况下，仅利用局部特征解决误匹配是非常困难的。

考虑两个时刻的图像。如果在图像It中提取到特征点 

![视觉SLAM十四讲-180](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-180.jpg)

在图像 It+1 中也提取到特征点xnt+1。如何寻找这两个集合元素的对应关系？最简单的特征匹配方法就是暴力匹配（Brute-Force Matcher）。即对每一个特征点 xm t 与所有的 xn t+1 测量描述子的距离，然后排序，取最近的一 个作为匹配点。描述子距离表示了两个特征之间的相似程度，不过在实际运用中还可以取不同的距离度量范数。对于浮点类型的描述子，使用欧氏距离进行度量即可。而对于二进制的描述子（比如BRIEF），往往使用汉明距离（Hamming distance）作为度量——两个二进制串之间的汉明距离，指的是其不同位数的个数。 然而，当特征点数量很大时，暴力匹配法的运算量将变得很大，特别是当想要匹配某个帧和一张地图的时候。这不符合在SLAM中的实时性需求。此时快速近似最近邻（FLANN）算法更加适合于匹配点数量极多的情况。这些匹配算法理论已经成熟，实现上集成到OpenCV了。

相关文献：

[47] M. Muja and D. G. Lowe, “Fast approximate nearest neighbors with automatic algorithm configuration.,” in VISAPP (1), pp. 331–340, 2009.

### 7.2 实践：特征提取和匹配

OpenCV集成了多数主流的图像特征，可以很方便地进行调用。

#### 7.2.1 OpenCV 的 ORB 特征 

调用OpenCV来提取和匹配ORB。图像位于slambook2/ch7/下的1.png 和 2.png，可以看到相机发生了微小的运动。程序演示如何提取ORB特征并进行匹配：slambook/ch7/orb_cv.cpp

首先要安装g2o，github下载20200410版本的:https://github.com/RainerKuemmerle/g2o.git

对ch7文件夹中的cmake工程进行编译，报错：

```
undefined reference to `vtable for fmt::v7::format_error‘
```

链接上fmt 库：

```
target_link_libraries("可执行文件" ${Sophus_LIBRARIES} fmt)
```

运行结果：

![视觉SLAM十四讲-182](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-182.jpg)

未筛选的匹配中带有大量的误匹配。经过一次筛选之后，匹配数量减少了许多，大多数匹配都是正确的。这里筛选的依据是汉明距离小于最小距离的两倍，这是一种工程上的经验方法。尽管在示例图像中能够筛选出正确的匹配，但仍然不能保证在所有其他图像中得到的匹配都是正确的。因此，在后面的运动估计中，还需要使用去除误匹配的算法。

#### 7.2.2 手写 ORB 特征 

代码：slambook/ch7/orb_self.cpp

运行结果：

![视觉SLAM十四讲-183](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-183.jpg)

在计算中用256位的二进制描述，即对应到8个32位的unsigned int数据，用 typedef 将它表示成 DescType。然后计算 FAST 特征点的角度，再使用该角度计算描述子。此代码中通过三角函数的原理回避了复杂的 arctan 以及 sin、cos 计算，从而达到加速的效果。在 BfMatch 函数中还使用了 SSE 指令集中的_mm_popcnt_u32 函数来计算一个 unsigned int 变量中1的个数，从而达到计算汉明距离的效果。

这个程序中，通过一些简单的算法修改，对ORB的提取加速了数倍。如果对提取特征部分进一步并行化处理，算法还可以有加速的空间。

#### 7.2.3 计算相机运动 

有了匹配好的点对后要根据点对来估计相机的运动。

1. 当相机为单目时，只知道2D的像素坐标，因而问题是根据两组2D点估计运动。该问题用对极几何来解决。 
2. 当相机为双目、RGB-D 时，或者通过某种方法得到了距离信息，那么问题就是根据两组3D点估计运动。该问题通常用 ICP 来解决。 
3. 如果一组为 3D，一组为 2D，即得到了一些 3D 点和它们在相机的投影位置，也能估计相机的运动。该问题通过 PnP 求解。

### 7.3 2D−2D: 对极几何

#### 7.3.1对极约束

假设从两张图像中得到了一对配对好的特征点,如图:

![视觉SLAM十四讲-184](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-184.jpg)

如果有若干对这样的匹配点,就可以通过这些二维图像点的对应关系,恢复出在两帧之间摄像机的运动。求取两帧图像I1, I2之间的运动,设第一帧到第二帧的运动为R,t。两个相机中心分别为O1,O2。如果I1中有一个特征点p1 ,它在I2中对应着特征点p2，两者是通过特征匹配得到的。如果匹配正确,说明它们是同一个空间点在两个成像平面上的投影。首先,连线O1p1和连线O2p2在三维空间中会相交于点P。这时候点O1, O2 ,P三个点可以确定一个平面,称为极平面(Epipolar plane) 。O1O2连线与像平面I1, I2 的交点分别为 e1, e2 。e1, e2 称为极点(Epipoles) ,O1O2被称为基线(Baseline) 。称极平面与两个像平面 I1, I2之间的相交线 L1, L2极线(Epipolar line) 。

直观讲,从第一帧的角度看,射线O1p1是某个像素可能出现的空间位置——因为该射线上的所有点都会投影到同一个像素点。同时,如果不知道P的位置,那么当在第二幅图像上看时,连线e2p2(也就是第二幅图像中的极线)就是P可能出现的投影的位置,也就是射线O1p1在第二个相机中的投影。由于通过特征点匹配确定了p2的像素位置,所以能够推断P的空间位置,以及相机的运动。如果没有特征匹配，就没法确定p2到底在极线的哪个位置了。
从代数角度来看一下这里的几何关系。在第一帧的坐标系下,设P的空间位置为P = [X, Y, Z]T.根据针孔相机模型知道两个像素点 p1, p2的像素位置为：
![视觉SLAM十四讲-185](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-185.jpg)

这里K为相机内参矩阵,R, t 为两个坐标系的相机运动。具体来说,这里计算的是 R21和 t21 ,因为是把第一个坐标系下的坐标转换到第二个坐标系下。在使用齐次坐标时,一个向量等于它自身乘上任意的非零常数。这通常用于表达一个投影关系。例如 s1p1 和 p1 成投影关系,它们在齐次坐标的意义下是相等的。称这种相等关系为尺度意义下相等(equal up to a scale),记作:sp ≃ p.那么,上述两个投影关系可写为:
p1 ≃ KP ,p2 ≃ K(RP + t) .
现在取:

![视觉SLAM十四讲-186](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-186.jpg)

这里的x1,x2是两个像素点的归一化平面上的坐标。代入上式,得:x2 ≃ Rx1 + t.

两边同时左乘 t∧ 。根据∧的定义,这相当于两侧同时与t做外积:

![视觉SLAM十四讲-187](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-187.jpg)

然后,两侧同时左乘![视觉SLAM十四讲-188](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-188.jpg)

得：![视觉SLAM十四讲-189](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-189.jpg)

观察等式左侧,![视觉SLAM十四讲-190](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-190.jpg)是一个与 t 和x2都垂直的向量。把它再和x2做内积时,将得到0。由于等式左侧严格为零,那么乘以任意非零常数之后也为零,于是可以把 ≃ 写成通常的等号。因此,就得到了:![视觉SLAM十四讲-191](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-191.jpg)

重新代入 p1, p2 ,有:	

![视觉SLAM十四讲-192](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-192.png)

这两个式子都称为对极约束。它的几何意义是O1, P, O2三者共面。对极约束中同时包含了平移和旋转。把中间部分记作两个矩阵:基础矩阵(Fundamental Matrix)F和本质矩阵(Essential Matrix)E,于是可以进一步简化对极约束:

![视觉SLAM十四讲-193](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-193.png)

对极约束简洁地给出了两个匹配点的空间位置关系。于是,相机位姿估计问题变为以下两步:

1. 根据配对点的像素位置求出 E 或者 F 。
2. 根据E或者F求出 R, t。由于 E 和 F 只相差了相机内参,而内参在SLAM中通常是已知的,所以实践当中往往使用形式更简单的E。

#### 7.3.2本质矩阵

根据定义,本质矩阵 E = t∧R。它是一个3 × 3的矩阵,内有 9 个未知数。从E的构造方式上看,有以下值得注意的地方:
• 本质矩阵是由对极约束定义的。由于对极约束是等式为零的约束,所以对E乘以任意非零常数后,对极约束依然满足。这称为E在不同尺度下是等价的。
• 根据 E = t∧R,可以证明,本质矩阵E的奇异值必定是 [σ, σ, 0]T的形式。这称为本质矩阵的内在性质。
• 另一方面,由于平移和旋转各有 3 个自由度,故 t∧R 共有 6 个自由度。但由于尺度等价性,故 E 实际上有 5 个自由度。表明最少可以用5对点来求解E。但是,E的内在性质是一种非线性性质,在估计时会带来麻烦,因此,也可以只考虑它的尺度等价性,使用8对点来估计E——这就是经典的八点法(Eight-point-algorithm) 。八点法只利用了E的线性性质,因此可以在线性代数框架下求解。
考虑一对匹配点,它们的归一化坐标为 x1= [u1 , v1 , 1] T , x2 = [u2 , v2 , 1] T 。根据对极约束,有:

![视觉SLAM十四讲-194](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-194.png)

把矩阵 E 展开,写成向量的形式:
e = [e1 , e2 , e3 , e4 , e5 , e6 , e7 , e8 , e9 ] T ,
那么对极约束可以写成与e有关的线性形式:
[u1u2 , u1v2 , u1, v1u2, v1v2, v1, u2, v2, 1] ·e = 0.
同理,对于其他点对也有相同的表示。把所有点都放到一个方程中,变成线性方程组(ui , vi 表示第 i 个特征点，以此类推）：

![视觉SLAM十四讲-195](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-195.jpg)

这八个方程构成了一个线性方程组。它的系数矩阵由特征点位置构成，大小为 8 × 9。 e 位于该矩阵的零空间中。如果系数矩阵是满秩的（即秩为 8），那么它的零空间维数为 1， 也就是 e 构成一条线。这与 e 的尺度等价性是一致的。如果八对匹配点组成的矩阵满足秩为 8 的条件，那么E的各元素就可由上述方程解得。 

接下来的问题是如何根据已经估得的本质矩阵E，恢复出相机的运动 R, t。这个过程是由奇异值分解（SVD）得到的。设 E 的 SVD 分解为：![视觉SLAM十四讲-196](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-196.jpg)

其中 U,V 为正交阵，Σ 为奇异值矩阵。根据 E 的内在性质，可以知道 Σ = diag(σ, σ, 0)。 在 SVD 分解中，对于任意一个 E，存在两个可能的 t, R 与它对应：

![视觉SLAM十四讲-197](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-197.jpg)

其中![视觉SLAM十四讲-198](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-198.jpg)表示沿 Z 轴旋转 90 度得到的旋转矩阵。同时，由于 −E 和 E 等价，所以对任意一个 t 取负号，也会得到同样的结果。因此，从 E 分解到 t, R 时，一共存在四个可能的解。只有第一种解中，P 在两个相机中都具有正的深度。因此，只要把任意一点代入四种解中，检测该点在两个相机下的深度，就可以确定哪个解是正确的了。 

如果利用 E 的内在性质，那么它只有五个自由度。所以最小可以通过五对点来求解相机运动。然而这种做法形式复杂，从工程实现角度考虑，由于平时通常会有几十对乃至上百对的匹配点，从八对减至五对意义并不明显。剩下的问题还有一个：根据线性方程解出的 E，可能不满足 E 的内在性质——它的奇异值不一定为 σ, σ, 0 的形式。这时，在做 SVD 时会刻意地把 Σ 矩阵调整成上面的样子。通常的做法是，对八点法求得的 E 进行 SVD 分解后，会得到奇异值矩阵 Σ = diag(σ1, σ2, σ3)，不妨设 σ1 ≥ σ2 ≥ σ3。取：

![视觉SLAM十四讲-199](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-199.jpg)

这相当于是把求出来的矩阵投影到了 E 所在的流形上。更简单的做法是将奇异值矩阵取成 diag(1, 1, 0)，因为 E 具有尺度等价性，这样做也是合理的。

#### 7.3.3单应矩阵 

单应矩阵（Homography）H ，描述了两个平面之间的映射关系。若场景中的特征点都落在同一平面上（比如墙，地面等），则可以通过单应性来进行运动估计。这种情况在无人机携带的俯视相机，或扫地机携带的顶视相机中比较常见。单应矩阵通常描述处于共同平面上的一些点，在两张图像之间的变换关系。考虑在图像I1和I2有一对匹配好的特征点p1和p2。这些特征点落在某平面上。设这个平面满足方程： ![视觉SLAM十四讲-200](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-200.jpg)

然后代入![视觉SLAM十四讲-185](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-185.jpg)，得：

![视觉SLAM十四讲-201](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-201.jpg)

把直接描述图像坐标 p1 和 p2 之间的变换的中间这部分记为 H，于是 p2 = Hp1。它的定义与旋转、平移以及平面的参数有关。单应矩阵 H是一个 3 × 3 的矩阵，求解时的思路可以先根据匹配点计算 H，然后将它分解以计算旋转和平移。把上式展开，得：

![视觉SLAM十四讲-202](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-202.jpg)

注意这里的等号是在非零因子下成立的。在实际处理中，通常乘以一个非零因子使得 h9 = 1（在它取非零值时）。然后根据第三行，去掉这个非零因子，于是有：

![视觉SLAM十四讲-203](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-203.jpg)

这样一组匹配点对就可以构造出两项约束（事实上有三个约束，但是因为线性相关，只取前两个），于是自由度为 8 的单应矩阵可以通过 4 对匹配特征点算出（注意：这些特征点不能有三点共线的情况），即求解以下的线性方程组（当 h9 = 0 时，右侧为零）：

![视觉SLAM十四讲-204](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-204.jpg)

这种做法把H矩阵看成了向量，通过解该向量的线性方程来恢复H，又称直接线性变换法（Direct Linear Transform）。求出单应矩阵以后需要对其进行分解，才可以得到相应的旋转矩阵 R 和平移向量 t。分解的方法包括数值法与解析法 。单应矩阵的分解会返回四组旋转矩阵与平移向量，并且同时可以计算出它们分别对应的场景点所在平面的法向量。如果已知成像的地图点的深度全为正值（即在相机前方），则又可以排除两组解。最后仅剩两组解，这时需要通过更多的先验信息进行判断。通常可以通过假设已知场景平面的法向量来解决，如场景平面 与相机平面平行，那么法向量 n 的理论值为1。 

单应性在 SLAM 中具有重要意义。当特征点共面，或者相机发生纯旋转的时候，基础矩阵的自由度下降，这就出现了所谓的退化（degenerate）。现实中的数据总包含一些噪声， 这时候如果继续使用八点法求解基础矩阵，基础矩阵多余出来的自由度将会主要由噪 声决定。为了能够避免退化现象造成的影响，通常会同时估计基础矩阵 F 和单应矩阵 H，选择重投影误差比较小的那个作为最终的运动估计矩阵。 

### 7.4 实践：对极约束求解相机运动 

如何通过 Essential 矩阵求解相机运动？

使用匹配好的特征点来计算 E,F 和 H，进而分解 E 得 到 R, t。整个程序使用 OpenCV 提供的算法进行求解。

代码：slambook/ch7/pose_estimation_2d2d.cpp

pose_estimation_2d2d函数提供了从特征点求解相机运动的部分，在函数中输出了 E,F 和 H 的数值，然后验证对极约束是否成立，以及 t ∧R 和 E 在非零数乘下等价的事实。调用此程序的输出结果：

```shell
build/pose_estimation_2d2d 1.png 2.png
-- Max dist : 95.000000
-- Min dist : 4.000000
一共找到了 79 组匹配点
fundamental_matrix is
[4.844484382466111e-06, 0.0001222601840188731, -0.01786737827487386;
-0.0001174326832719333, 2.122888800459598e-05, -0.01775877156212593;
0.01799658210895528, 0.008143605989020664, 1]
essential_matrix is
[-0.0203618550523477, -0.4007110038118445, -0.03324074249824097;
0.3939270778216369, -0.03506401846698079, 0.5857110303721015;
-0.006788487241438284, -0.5815434272915686, -0.01438258684486258]
homography_matrix is
[0.9497129583105288, -0.143556453147626, 31.20121878625771;
0.04154536627445031, 0.9715568969832015, 5.306887618807696;
-2.81813676978796e-05, 4.353702039810921e-05, 1]
R is
[0.9985961798781875, -0.05169917220143662, 0.01152671359827873;
0.05139607508976055, 0.9983603445075083, 0.02520051547522442;
-0.01281065954813571, -0.02457271064688495, 0.9996159607036126]
t is
[-0.8220841067933337;
-0.03269742706405412;
0.5684264241053522]

t^R=
[0.02879601157010516, 0.5666909361828478, 0.04700950886436416;
-0.5570970160413605, 0.0495880104673049, -0.8283204827837456;
0.009600370724838804, 0.8224266019846683, 0.02034004937801349]
epipolar constraint = [0.002528128704106625]
epipolar constraint = [-0.001663727901710724]
epipolar constraint = [-0.0008009088410884102
```

可以看出，对极约束的满足精度约在 10−3 量级。OpenCV 使用三角化检测角点的深度是否为正，从而选出正确的解。 对极约束是从 x2 = Rx1 + t  得到的。这里的 R, t 组成的变换矩阵，是第一个图到第二个图的坐标变换矩阵： x2 = T21x1.

#### 7.4.1 讨论 

从演示程序中可以看到，输出的 E 和 F 相差了相机内参矩阵。从 E,F 和 H 都可以分解出运动，不过 H 需要假设特征点位于平面上。 由于 E 本身具有尺度等价性，它分解得到的 t, R 也有一个尺度等价性。而 R ∈ SO(3) 自身具有约束，所以认为 t 具有一个尺度。换言之，在分解过程中，对 t 乘以任意非零常数，分解都是成立的。因此通常把 t 进行归一化，让它的长度等于 1。 

**尺度不确定性**

对 t 长度的归一化，直接导致了单目视觉的尺度不确定性（Scale Ambiguity）。例如，程序中输出的 t 第一维约 0.822。这个 0.822 究竟是指 0.822 米呢，还是 0.822 厘米 呢，是没法确定的。因为对 t 乘以任意比例常数后，对极约束依然是成立的。换言之， 在单目 SLAM 中，对轨迹和地图同时缩放任意倍数，得到的图像依然是一样的。 在单目视觉中，对两张图像的 t 归一化，相当于固定了尺度。虽然不知道它的实际长度为多少，但以这时的 t 为单位 1，计算相机运动和特征点的 3D 位置。这被称为单目 SLAM 的初始化。在初始化之后，就可以用 3D-2D 来计算相机运动了。初始化之后的轨迹和地图的单位，就是初始化时固定的尺度。因此，单目 SLAM 有一步不可避免的初始化。初始化的两张图像必须有一定程度的平移，而后的轨迹和地图都将以此步的平移为单位。 除了对 t 进行归一化之外，另一种方法是令初始化时所有的特征点平均深度为 1，也 可以固定一个尺度。相比于令 t 长度为 1 的做法，把特征点深度归一化可以控制场景的规模大小，使计算在数值上更稳定些。

**初始化的纯旋转问题** 

从 E 分解到 R, t 的过程中，如果相机发生的是纯旋转，导致 t 为零，那么，得到的 E 也将为零，这将导致无从求解 R。不过，此时可以依靠 H 求取旋转，但仅有旋转时，无法用三角测量估计特征点的空间位置，于是，另一个结论是，单目初始化不能只有纯旋转，必须要有一定程度的平移。如果没有平移，单目将无法初始化。在实践当中，如果初始化时平移太小，会使得位姿求解与三角化结果不稳定， 从而导致失败。相对的，如果把相机左右移动而不是原地旋转，就容易让单目 SLAM 初始化。

**多于八对点的情况** 

当给定的点数多于八对时，可以计算一个最小二乘解。对于线性化后的对极约束，我们把左侧的系数矩阵记为 A： Ae = 0

对于八点法，A 的大小为 8 × 9。如果给定的匹配点多于 8，该方程构成一个超定方程，即不一定存在 e 使得上式成立。因此，可以通过最小化一个二次型来求： 

于是就求出了在最小二乘意义下的 E 矩阵。不过，当可能存在误匹配的情况时，会更倾向于使用随机采样一致性（Random Sample Concensus, RANSAC）来求，而不是最小二乘。RANSAC 是一种通用的做法，适用于很多带错误数据的情况，可以处理带有错误匹配的数据。

### 7.5 三角测量 

使用对极几何约束估计了相机运动之后，下一步需要用相机的运动估计特征点的空间位置。在单目SLAM中，仅通过单张图像无法获得像素的深度信息，需要通过三角测量（Triangulation）（或三角化）的方法来估计地图点的深度。 

三角测量是指，通过在两处观察同一个点的夹角，确定该点的距离。三角测量最早由高斯提出并应用于测量学中，它在天文学、地理学的测量中都有应用。例如可以通过不同季节观察到星星的角度，估计它离我们的距离。在SLAM中主要用三角化来估计像素点的距离。

![视觉SLAM十四讲-205](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-205.jpg)

考虑图像 I1 和 I2，以左图为参考，右图的变换矩阵为 T。相机光心 为 O1 和 O2。在 I1 中有特征点 p1，对应 I2 中有特征点 p2。理论上直线 O1p1 与 O2p2 在场景中会相交于一点 P，该点即是两个特征点所对应的地图点在三维场景中的位置。然而由于噪声的影响，这两条直线往往无法相交。因此可以通过最二小乘去求解。 按照对极几何中的定义，设 x1, x2 为两个特征点的归一化坐标，那么它们满足： s1x1 = s2Rx2 + t. 

已知 R, t，想要求解的是两个特征点的深度 s1, s2。这两个深度是可以分开求的，先算s2，那么先对上式两侧左乘一个![视觉SLAM十四讲-207](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-207.jpg)，得： 

![视觉SLAM十四讲-206](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-206.jpg)

该式左侧为零，右侧可看成 s2 的一个方程，可以根据它直接求得 s2。有了 s2，s1也可以求出。于是就得到了两个帧下的点的深度，确定了它们的空间坐标。由于噪声的存在，估得的 R, t不一定精确使方程为0，所以更常见的做法求最小二乘解而不是零解。 

### 7.6 实践：三角测量 

#### 7.6.1 三角测量代码 

代码演示了对之前根据对极几何求解的相机位姿通过三角化求出特征点的空间位置。调用 OpenCV 提供的 triangulation 函数进行三角化。 

代码为：slambook/ch7/triangulation.cpp

同时在 main 函数中增加三角测量部分，并验证重投影关系。

代码输出某特征点的信息：

```shell
point in the first camera frame: [0.0844072, -0.0734976]
point projected from 3D [0.0843702, -0.0743606], d=14.9895
point in the second camera frame: [0.0431343, -0.0459876]
point reprojected from second frame: [0.04312769812378599, -0.04515455276163744, 1]
```

打印了每个空间点在两个相机坐标系下的投影坐标与像素坐标。这里对于这个特征点来说，通过两个图像下的一对像素坐标的三角化，能得到这个特征点的空间坐标，d为深度，利用这个空间坐标重投影能得到一个投影坐标，跟原始的像素坐标相比有一定的误差。由于误差的存在，它们会有一些微小的差异。误差的量级大约在小数点后第三位。可以看到，三角化特征点的距离大约为 15。但由于尺度不确定性，并不知道这里的 15 究竟是多少米。 

#### 7.6.2 讨论 

三角测量是由平移得到的，有平移才会有对极几何中的三角形，才谈的上三角测量。因此，纯旋转是无法使用三角测量的，因为对极约束将永远满足。在平移存在的情况下，还要关心三角测量的不确定性，这会引出一个三角测量的矛盾。

![视觉SLAM十四讲-208](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-208.jpg)

如图所示。当平移很小时，像素上的不确定性将导致较大的深度不确定性。也就是说，如果特征点运动一个像素 δx，使得视线角变化了一个角度 δθ，那么测量到深度值将有 δd 的变化。从几何关系可以看到，当 t 较大时，δd 将明显变小，这说明平移较大时， 在同样的相机分辨率下，三角化测量将更精确。对该过程的定量分析可以使用正弦定理得到，但这里先考虑定性分析。 因此，要增加三角化的精度，其一是提高特征点的提取精度，也就是提高图像分辨率 ——但这会导致图像变大，提高计算成本。另一方式是使平移量增大。但是，平移量增大会导致图像的外观发生明显的变化，比如箱子原先被挡住的侧面显示出来了，比如反射光发生变化了，等等。外观变化会使得特征提取与匹配变得困难。总而言之，增大平移，会导致匹配失效；而平移太小，则三角化精度不够——这就是三角化的矛盾。 

定量地计算每个特征点的位置及不确定性。假设特征点服从高斯分布，并且对它不断地进行观测，在信息正确的情况下，就能够期望它的方差会不断减小乃至收敛。这就得到了一个滤波器，称为深度滤波器（Depth Filter）。

### 7.7 3D-2D: PnP 

PnP（Perspective-n-Point）是求解 3D 到 2D 点对运动的方法。它描述了当知道 n 个 3D 空间点以及它们的投影位置时，如何估计相机所在的位姿。前面的2D-2D 的对极几何方法需要八个或八个以上的点对（以八点法为例），且存在着初始化、纯旋转和尺度的问题。然而，如果两张图像中，其中一张特征点的 3D 位置已知，那么最少只需三个点对（需要至少一个额外点验证结果）就可以估计相机运动。特征点的 3D 位置可以由三角化，或者由RGB-D 相机的深度图确定。因此，在双目或 RGB-D 的视觉里程计中， 可以直接使用 PnP 估计相机运动。而在单目视觉里程计中，必须先进行初始化，然后才能使用 PnP。3D-2D 方法不需要使用对极约束，又可以在很少的匹配点中获得较好的运动估计，是最重要的一种姿态估计方法。 PnP 问题有很多种求解方法，例如用三对点估计位姿的P3P，直接线性变换（DLT）， EPnP（Efficient PnP），UPnP等等。此外，还能用非线性优化的方式，构建最小二乘问题并迭代求解，也就是万金油式的 Bundle Adjustment。

#### 7.7.1 直接线性变换 

考虑某个空间点 P，它的齐次坐标为 P = (X, Y, Z, 1)T。在图像 I1 中，投影到特征点 x1 = (u1, v1, 1)T（以归一化平面齐次坐标表示）。此时相机的位姿 R, t 是未知的。与单应矩阵的求解类似，定义增广矩阵 [R|t] 为一个 3 × 4 的矩阵，包含了旋转与平移信息。展开形式列写如下：

![视觉SLAM十四讲-209](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-209.jpg)

用最后一行把 s 消去，得到两个约束： 

![视觉SLAM十四讲-210](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-210.jpg)

定义 T 的行向量： t1 = (t1, t2, t3, t4) T , t2 = (t5, t6, t7, t8) T , t3 = (t9, t10, t11, t12) T , 于是有：

![视觉SLAM十四讲-211](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-211.jpg)

![视觉SLAM十四讲-212](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-212.jpg)

t 是待求的变量，可以看到每个特征点提供了两个关于 t 的线性约束。假设一共有 N 个特征点，可以列出线性方程组： 

![视觉SLAM十四讲-213](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-213.jpg)

由于 t 一共有 12 维，因此最少通过六对匹配点，即可实现矩阵 T 的线性求解，这种方法称为直接线性变换（Direct Linear Transform，DLT）。当匹配点大于六对时，可以使用 SVD 等方法对超定方程求最小二乘解。 在 DLT 求解中，直接将 T 矩阵看成了 12 个未知数，忽略了它们之间的联系。因为旋转矩阵 R ∈ SO(3)，用 DLT 求出的解不一定满足该约束，它是一个一般矩阵。平移向量属于向量空间。对于旋转矩阵 R，必须针对 DLT 估计的 T 的左边寻找一个最好的旋转矩阵对它进行近似。这可以由 QR 分解完成， 相当于把结果从矩阵空间重新投影到 SE(3) 流形上，转换成旋转和平移两部分。 这里的 x1 使用了归一化平面坐标，去掉了内参矩阵 K 的影响 ——这是因为内参 K 在 SLAM 中通常假设为已知。如果内参未知，也能用 PnP 去估计 K, R, t 三个量。然而由于未知量的增多，效果会差一些。 

#### 7.7.2 P3P 

P3P仅使用三对匹配点，对数据要求较少，需要利用给定的三个点的几何关系。它的输入数据为三对 3D-2D 匹配点。记 3D 点为 A, B, C，2D 点为 a, b, c，其中小写字母代表的点为大写字母在相机成像平面上的投影，如图所示。

![视觉SLAM十四讲-214](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-214.jpg)

此外，P3P 还需要使用一对验证点，以从可能的解出选出正确的那一个（类似于对极几何情形）。记验证点对为 D − d，相机光心为 O。注意，预知的是 A, B, C 在世界坐标系中的坐标，而不是在相机坐标系中的坐标。一旦3D 点在相机坐标系下的坐标能够算出，就得到了 3D-3D 的对应点，把 PnP 问题转换为了 ICP 问题。三角形之间存在对应关系： 

![视觉SLAM十四讲-215](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-215.jpg)

利用余弦定理，有： 

![视觉SLAM十四讲-216](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-217.jpg)

对上面三式全体除以![视觉SLAM十四讲-218](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-218.jpg),并且记 x = OA/OC, y = OB/OC，得：

![视觉SLAM十四讲-219](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-219.jpg)

记![视觉SLAM十四讲-220](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-220.jpg)

有： ![视觉SLAM十四讲-221](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-221.jpg)

化为：![视觉SLAM十四讲-222](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-222.jpg)

由于知道 2D 点的图像位置，三个余弦角 cos⟨a, b⟩, cos⟨b, c⟩, cos⟨a, c⟩ 是已知的。同时，![视觉SLAM十四讲-223](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-223.jpg)可以通过 A, B, C 在世界坐标系下的坐标算出，变换到相机坐标系下之后，并不改变这个比值。该式中的 x, y 是未知的，随着相机移动会发生变化。因此，该方程组是关于 x, y 的一个二元二次方程（多项式方程）。解析地求解该方程组是一个复杂的过程，需要用吴消元法。类似于分解 E 的情况，该方程最多可能得到四个解，但可以用验证点来计算最可能的解，得到 A, B, C 在相机坐标系下的 3D 坐标。然后，根据 3D-3D 的点对，计算相机的运动 R, t。

从 P3P 的原理上可以看出，为了求解 PnP，利用了三角形相似性质，求解投影点 a, b, c 在相机坐标系下的 3D 坐标，最后把问题转换成一个 3D 到 3D 的位姿估计问题。

P3P 也存在着一些问题： 

1. P3P 只利用三个点的信息。当给定的配对点多于 3 组时，难以利用更多的信息。 
2. 如果 3D 点或 2D 点受噪声影响，或者存在误匹配，则算法失效。 所以后续人们还提出了许多别的方法，如 EPnP、UPnP 等。它们利用更多的信息，而且用迭代的方式对相机位姿进行优化，以尽可能地消除噪声的影响。在 SLAM 当中，通常的做法是先使用 P3P/EPnP 等方法估计相机位姿，然后构建最小二乘优化问题对估计值进行调整（Bundle Adjustment）。

#### 7.7.3 Bundle Adjustment 

除了使用线性方法之外，可以把 PnP 问题构建成一个定义于李代数上的非线性最小二乘问题。前面说的线性方法，往往是先求相机位姿，再求空间点位置，而非线性优化则是把它们都看成优化变量，放在一起优化。这是一种非常通用的求解方式，可以用它对 PnP 或 ICP 给出的结果进行优化。在 PnP 中， 这个 Bundle Adjustment 问题，是一个最小化重投影误差（Reprojection error）的问题。

考虑 n 个三维空间点 P 和它们的投影 p，希望计算相机的位姿 R, t，它的李代数表示为 ξ。假设某空间点坐标为 Pi = [Xi , Yi , Zi ]T，其投影的像素坐标为 ui = [ui , vi ]T。 像素位置与空间点位置的关系如下：

![视觉SLAM十四讲-225](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-225.jpg)

写成矩阵形式是：

![视觉SLAM十四讲-226](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-226.jpg)

中间隐含着齐次坐标到非齐次的转换，否则按矩阵的乘法来说，维度是不对的。因为exp (ξ∧) Pi 结果是 4 × 1 的，而它左侧的 K 是 3 × 3 的，所以必须把 exp (ξ ∧) Pi 的前三维取出来，变成三维的非齐次坐标。 现在，由于相机位姿未知以及观测点的噪声，该等式存在一个误差。因此把误差求和，构建最小二乘问题，然后寻找最好的相机位姿，使它最小化：

![视觉SLAM十四讲-227](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-227.jpg)

该问题的误差项，是将像素坐标（观测到的投影位置）与 3D 点按照当前估计的位姿进行投影得到的位置相比较得到的误差，所以称之为重投影误差。使用齐次坐标时，这个误差有 3 维。不过，由于u最后一维为 1，该维度的误差一直为零，因而更多时候使用非齐次坐标，于是误差就只有 2 维了。

如图所示，通过特征匹配，知道了 p1 和 p2 是同一个空间点 P 的投影，但是不知道相机的位姿。在初始值中，P 的投影 pˆ2 与实际的 p2 之间有一定的距离。于是调整相机的位姿，使得这个距离变小。不过，由于这个调整需要考虑很多个点，所以最后每个点的误差通常都不会精确为零。

![视觉SLAM十四讲-228](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-228.jpg)

使用李代数，可以构建无约束的优化问题， 很方便地通过 G-N, L-M 等优化算法进行求解。不过，在使用 G-N 和 L-M 之前，需要知道每个误差项关于优化变量的导数，也就是线性化： e(x + ∆x) ≈ e(x) + J∆x. 

这里的 J 的形式是关键所在。固然可以使用数值导数，但如果能够推导解析形式时，会优先考虑解析导数。现在，当 e 为像素坐标误差 （2 维），x 为相机位姿（6 维）时，J 将是一个 2 × 6 的矩阵。

使用扰动模型来求李代数的导数：

首先，记变换到相机坐标系下的空间点坐标为 P′，并且将其前三维取出来： P′ = (exp (ξ∧)P)1:3 = [X′,Y′,Z′]T

相机投影模型相对于P′则为： su = KP′ 

展开：![视觉SLAM十四讲-229](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-229.jpg)

利用第 3 行消去 s（实际上就是 P′ 的距离），得：

![视觉SLAM十四讲-230](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-230.jpg)

这与相机模型是一致的。当求误差时，可以把这里的 u, v 与实际的测量值比较，求差。定义了中间变量后对 ξ∧左乘扰动量δξ，然后考虑 e 的变化关于扰动量的导数。利用链式法则，可以列写如下：

![视觉SLAM十四讲-231](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-231.jpg)

这里的⊕指李代数上的左乘扰动。第一项是误差关于投影点的导数，得：

![视觉SLAM十四讲-232](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-232.jpg)

第二项为变换后的点关于李代数的导数，得：

![视觉SLAM十四讲-233](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-233.jpg)

而在 P′ 的定义中，取出了前三维，于是得：

![视觉SLAM十四讲-234](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-234.jpg)

将这两项相乘，就得到了 2 × 6 的雅可比矩阵：

![视觉SLAM十四讲-235](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-235.jpg)

这个雅可比矩阵描述了重投影误差关于相机位姿李代数的一阶变化关系。保留了前面的负号，因为这是由于误差是由观测值减预测值定义的。它当然也可反过来，定义成 “预测减观测”的形式。在这种情况下，只要去掉前面的负号即可。此外，如果 se(3) 的定义方式是旋转在前，平移在后时，只要把这个矩阵的前三列与后三列对调即可。 

另一方面，除了优化位姿，还希望优化特征点的空间位置。因此，需要讨论 e 关于空间点 P 的导数。仍利用链式法则，有：

![视觉SLAM十四讲-236](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-236.jpg)

第二项，按照定义 P′ = exp(ξ∧)P = RP + t. 

发现 P′ 对 P 求导后只剩下 R。于是:

![视觉SLAM十四讲-237](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-237.jpg)

以上是观测相机方程关于相机位姿与特征点的两个导数矩阵。它们能够在优化过程中提供重要的梯度方向，指导优化的迭代。 

### 7.8 实践：求解 PnP 

#### 7.8.1 使用 EPnP 求解位姿

首先用 OpenCV 提供的 EPnP 求 解 PnP 问题，然后通过 g2o 对结果进行优化。由于 PnP 需要使用 3D 点，为了避免初始化带来的麻烦，使用了 RGB-D 相机中的深度图（1_depth.png），作为特征点的 3D 位置。

代码为: slambook/ch7/pose_estimation_3d2d.cpp

在例程中，得到配对特征点后，在第一个图的深度图中寻找它们的深度，并求出空间位置。以此空间位置为 3D 点，再以第二个图像的像素位置为 2D 点，调用 EPnP 求解 PnP 问题。程序输出后可以看到，在有3D信息时，估计的 R 几乎是相同的，而 t 相差的较多。这是由于引入了新的深度信息所致。 不过，由于 Kinect 采集的深度图本身会有一些误差，所以这里的 3D 点也不是准确的。后面会希望把位姿 ξ 和所有三维特征点 P 同时优化。

#### 7.8.2 使用 BA 优化 

使用前一步的估计值作为初始值，把问题建模成一个最小二乘的图优化问题，如图所示。

![视觉SLAM十四讲-224](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-224.jpg)在这个图优化中，节点和边的选择为： 

1. 节点：第二个相机的位姿节点 ξ ∈ se(3)，以及所有特征点的空间位置 P ∈ R3。 
2. 边：每个 3D 点在第二个相机中的投影，以观测方程来描述： zj = h(ξ, Pj ). 

由于第一个相机位姿固定为零，没有把它写到优化变量里。现在根据一组 3D 点和第二个图像中的 2D 投影，估计第二个相机的位姿。所以把第一个相机画成虚线，表明不希望考虑它。 g2o 提供了许多关于 BA 的节点和边，不必自己从头实现所有的计算。在 g2o/types/sba/types_six_dof_expmap.h 中提供了李代数表达的节点和边。文件中有VertexSE3Expmap（李代数位姿）、VertexSBAPointXYZ（空间点位置） 和 EdgeProjectXYZ2UV（投影方程边）这三个类。

VertexSE3Expmap的类定义：

```c++
class G2O_TYPES_SBA_API VertexSE3Expmap : public BaseVertex<6, SE3Quat>{
 public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 VertexSE3Expmap();

 bool read(std::istream& is);

 bool write(std::ostream& os) const;

 virtual void setToOriginImpl() {
 _estimate = SE3Quat();
 }

 virtual void oplusImpl(const double∗ update_) {
 Eigen::Map<const Vector6d> update(update_);
 setEstimate( SE3Quat::exp(update)∗estimate());
 }
 };
```

模板参数：

1. 第一个参数 6 表示它内部存储的优化变量维度，可以看到这是一个 6 维的李代数。
2. 第二参数是优化变量的类型，这里使用了 g2o 定义的相机位姿：SE3Quat。 这个类内部使用了四元数加位移向量来存储位姿，但同时也支持李代数上的运算，例如对数映射（log 函数）和李代数上增量（update 函数）等操作。
3. 空间点位置类的维度为 3，类型是 Eigen 的 Vector3D。另一方面，边 EdgeProjectXYZ2UV 连接了两个前面说的两个顶点，它的观测值为 2 维，由 Vector2D 表示，实际上就是空间点的像素坐标。它的误差计算函数表达了投影方程的误差计算方法，也就是前面提到的 z − h(ξ, P ) 的方式。 

EdgeProjectXYZ2UV 的 linearizeOplus 函数的实现用到了前面推导的雅可比矩阵：

```c++
void EdgeProjectXYZ2UV::linearizeOplus() {
 VertexSE3Expmap ∗ vj = static_cast<VertexSE3Expmap ∗>(_vertices[1]);
 SE3Quat T(vj−>estimate());
 VertexSBAPointXYZ∗ vi = static_cast<VertexSBAPointXYZ∗>(_vertices[0]);
 Vector3D xyz = vi−>estimate();
 Vector3D xyz_trans = T.map(xyz);

 double x = xyz_trans[0];
 double y = xyz_trans[1];
 double z = xyz_trans[2];
 double z_2 = z∗z;

 const CameraParameters ∗ cam = static_cast<const CameraParameters ∗>(parameter(0));

 Matrix<double,2,3,Eigen::ColMajor> tmp;
 tmp(0,0) = cam−>focal_length;
 tmp(0,1) = 0;
 tmp(0,2) = −x/z∗cam−>focal_length;

 tmp(1,0) = 0;
 tmp(1,1) = cam−>focal_length;
 tmp(1,2) = −y/z∗cam−>focal_length;

 _jacobianOplusXi = −1./z ∗ tmp ∗ T.rotation().toRotationMatrix();

 _jacobianOplusXj(0,0) = x∗y/z_2 ∗cam−>focal_length;
 _jacobianOplusXj(0,1) = −(1+(x∗x/z_2)) ∗cam−>focal_length;
 _jacobianOplusXj(0,2) = y/z ∗cam−>focal_length;
 _jacobianOplusXj(0,3) = −1./z ∗cam−>focal_length;
 _jacobianOplusXj(0,4) = 0;
 _jacobianOplusXj(0,5) = x/z_2 ∗cam−>focal_length;

 _jacobianOplusXj(1,0) = (1+y∗y/z_2) ∗cam−>focal_length;
 _jacobianOplusXj(1,1) = −x∗y/z_2 ∗cam−>focal_length;
 _jacobianOplusXj(1,2) = −x/z ∗cam−>focal_length;
 _jacobianOplusXj(1,3) = 0;
 _jacobianOplusXj(1,4) = −1./z ∗cam−>focal_length;
 _jacobianOplusXj(1,5) = y/z_2 ∗cam−>focal_length;
 }
```

它与公式是一致的。成员变量“_- jacobianOplusXi”是误差到空间点的导数，“_jacobianOplusXj”是误差到相机位姿的导数，以李代数的左乘扰动表达。稍有差别的是，g2o 的相机里用 f 统一描述 fx, fy，并且 李代数定义顺序不同（g2o 是旋转在前，平移在后），所以矩阵前三列和后三列与上面的定义是颠倒的。 

们在上一个 PnP 例程的基础上，加上 g2o 提供的 Bundle Adjustment：

首先声明了 g2o 图优化，配置优化求解器和梯度下降方法，然后根据估计到的特征点，将位姿和空间点放到图中。最后调用优化函数进 行求解。

优化结果：迭代 11 轮后，LM 发现优化目标函数接近不变，于是停止了优化。输出了最后得到位姿变换矩阵 T，对比之前直接做 PnP 的结果，大约在小数点后第三位发生了一些变化。这主要是由于同时优化了特征点和相机位姿导致的。

Bundle Adjustment 是一种通用的做法。它可以不限于两个图像。可以放入多个图像匹配到的位姿和空间点进行迭代优化，甚至可以把整个 SLAM 过程放进来。那种做法规模较大，主要在后端使用。在前端，我们通常 考虑局部相机位姿和特征点的小型 Bundle Adjustment 问题，希望实时对它进行求解和优化。

### 7.9 3D−3D：ICP 

假设有一组配对好的 3D 点（比如对两幅 RGB-D 图像进行了匹配）： 

![视觉SLAM十四讲-238](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-238.jpg)

想要找一个欧氏变换 R, t，使得：

![视觉SLAM十四讲-239](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-239.jpg)

这个问题可以用迭代最近点（Iterative Closest Point，ICP）求解。3D−3D 位姿估计问题和相机模型并没有关系。在激光 SLAM 中也会碰到 ICP，不过由于激光数据特征不够丰富，无从知道两个点集之间的匹配关系，只能认为距离最近的两个点为同一个，所以这个方法称为迭代最近点。而在视觉中，特征点提供了较好的匹配关系，所以整个问题就变得更简单了。在 RGB-D SLAM 中， 可以用这种方式估计相机位姿。ICP 的求解也分为两种方式：利用线性代数的求解（主要是 SVD），以及利用非线性优化方式的求解（类似于 Bundle Adjustment）。

#### 7.9.1 SVD 方法 

定义第 i 对点的误差项：

![视觉SLAM十四讲-240](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-240.jpg)

然后构建最小二乘问题，求使误差平方和达到极小的R, t：

![视觉SLAM十四讲-241](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-241.jpg)

求解:

首先，定义两组点的质心：

![视觉SLAM十四讲-242](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-242.jpg)

随后，在误差函数中做如下的处理：

![视觉SLAM十四讲-243](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-243.jpg)

交叉项部分中![视觉SLAM十四讲-244](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-244.jpg)

在求和之后为零，因此优化目标函数可以简化为

![视觉SLAM十四讲-245](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-245.jpg)

左边只和旋转矩阵 R 相关，而右边既有 R 也有 t，但只和质心相关。只要获得了R，令第二项为零就能得到 t。

于是ICP可以分为以下三个步骤求解：

1. 计算两组点的质心位置 p, p′，然后计算每个点的去质心坐标：

![视觉SLAM十四讲-246](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-246.jpg)

2. 根据以下优化问题计算旋转矩阵：

![视觉SLAM十四讲-247](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-247.jpg)

3. 根据第 2 步的 R 计算 t： t∗ = p − Rp′ 

求出了两组点之间的旋转，平移量很容易得到。

展开关于 R 的误差项，得： 

![视觉SLAM十四讲-248](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-248.jpg)

第一项和 R 无关，第二项由于 RTR = I，也与 R 无关。因此实际上优化目标函数变为

![视觉SLAM十四讲-249](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-249.jpg)

下面通过 SVD 解出上述问题中最优的 R：

先定义矩阵： 

![视觉SLAM十四讲-250](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-250.jpg)

W 是一个 3 × 3 的矩阵，对 W 进行 SVD 分解，得： W = UΣVT

其中，Σ 为奇异值组成的对角矩阵，对角线元素从大到小排列，而 U 和 V 为对角矩阵。当 W 满秩时，R = UVT 

然后按t∗ = p − Rp′ 求解 t。

#### 7.9.2 非线性优化方法 

以李代数表达位姿时，目标函数可以写成

![视觉SLAM十四讲-251](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-251.jpg)

使用李代数扰动模型：

![视觉SLAM十四讲-252](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E8%A7%86%E8%A7%89SLAM%E5%8D%81%E5%9B%9B%E8%AE%B2/pic/视觉SLAM十四讲-252.jpg)

在非线性优化中只需不断迭代，就能找到极小值。ICP 问题存在唯一 解或无穷多解的情况。在唯一解的情况下，只要能找到极小值解，那么这个极小值就是全局最优值，不会遇到局部极小的情况。这意味着已匹配点时求解 ICP可以任意选定初始值。

这里的ICP是指已由图像特征给定了匹配的情况下进行位姿估计的问题。 在匹配已知的情况下，这个最小二乘问题具有解析解，所以并没有必要进行迭代优化。对于深度已知的特征点，建模它们的 3D−3D 误差；对于深度未知的特征点，则建模 3D−2D 的重投影误差。可以将所有的误差放在同一个问题中考虑，使得求解更加方便。 

### 7.10 实践：求解 ICP 

#### 7.10.1 SVD 方法 

使用两幅 RGB-D 图像，通过特征匹配获取两组 3D 点，用 ICP 计算它们的位姿变换。

代码：slambook/ch7/pose_estimation_3d3d.cpp

调用 Eigen 进行 SVD，然后计算 R, t 矩阵。输出匹配后的结果。

对比ICP 与 PnP、对极几何的运动估计结果之间的差异，在这个过程中使用了越来越多的信息，从没有深度到有一个图的深度到有两个图的深度。因此在深度准确的情况下，得到的估计也将越来越准确。但是由于 Kinect 的深度图存在噪声，而且有可能存在数据丢失的情况，使得不得不丢弃一些没有深度数据的特征点。这可能导致 ICP 的估计不够准确，并且，如果特征点丢弃得太多，可能引起由于特征点太少，无法进行运动估计的情况，即退化的情况。

#### 7.10.2 非线性优化方法 

使用李代数来表达相机位姿，与 SVD 思路不同的地方在于，在优化中不仅考虑相机的位姿，同时会优化 3D 点的空间位置。RGB-D 相机每次可以观测到路标点的三维位置，从而产生一个 3D 观测数据。g2o/sba 中没有提 供 3D 到 3D 的边，所以自定义一 种这样的边，并向 g2o 提供解析求导方式。

代码：slambook/ch7/pose_estimation_3d3d.cpp

这是一个一元边，写法类似于g2o::EdgeSE3ProjectXYZ，不过观测量从 2 维变成了 3 维，内部没有相机模型，并且只关联到一个节点。雅可比矩阵给出了关于相机位姿的导数，是一个 3 × 6 的矩阵。 调用 g2o 进行优化的代码是相似的，设定好图优化的节点和边即可。

优化的结果： 

只迭代一次后总体误差稳定不变，说明仅在一次迭代之后算法即已收敛。从位姿求解的结果可以看出，它和前面 SVD 给出的位姿结果几乎一模一样，这说明 SVD 已经给出了优化问题的解析解。所以可以认为 SVD 给出的结果是相机位姿的最优值。 在这个ICP 中，使用了在两个图都有深度读数的特征点。然而事实上只要其中一个图深度确定，就能用类似于 PnP 的误差方式，把它们也加到优化中来。同时，除了相机位姿之外，将空间点也作为优化变量考虑，也是一种解决问题的方式。实际的求解是非常灵活的，不必拘泥于某种固定的形式。如果同时考虑点和相机，整个问题就变得更自由了，可能会得到其他的解。比如可以让相机少转一些角度，而把点多移动一些。这从另一侧面反映出，在 Bundle Adjustment 里面，会希望有尽可能多的约束，因为多次观测会带来更多的信息，更准确地估计每个变量。 

### 7.11 小结 

基于特征点的视觉里程计中的几个重要的问题，包括： 

1. 特征点如何提取并匹配
2.  如何通过 2D−2D 的特征点估计相机运动。 
3. 如何从 2D−2D 的匹配估计一个点的空间位置。 
4. 3D−2D 的 PnP 问题，其线性解法和 Bundle Adjustment 解法。 
5. 3D−3D 的 ICP 问题，其线性解法和 Bundle Adjustment 解法。 

这省略了大量关于某些特殊情况的讨论。例如，如果在对极几何求解过程中给定的特征点共面，会发生什么情况？共线又会发生什么情况？在 PnP 和 ICP 中若给定这样的解，又会导致什么情况？求解算法能否识别这些特殊的情况，并报告所得的解可能不可靠？

在工程实现中，这些情况很少出现。

### 习题 

1. 除了ORB 特征点，SIFT 或 SURF 的原理，并对比它们与 ORB 之间的优劣。 
2. 设计程序调用 OpenCV 中的其他种类特征点。统计在提取 1000 个特征点时在机器上所用的时间。 
3. OpenCV 提供的 ORB 特征点在图像当中分布不够均匀。有没让特征点分布更加均匀的方法？ 
4. 研究 FLANN 为何能够快速处理匹配问题。除了 FLANN 之外，还有哪些可以加速匹配的手段？ 
5. 在 PnP 优化中，将第一个相机的观测也考虑进来，程序应如何书写？最后结果会有何变化？ 
6. 在 ICP 程序中，将空间点也作为优化变量考虑进来，程序应如何书写？最后结果会有何变化？ 
7. 在特征点匹配过程中，不可避免地会遇到误匹配的情况。如果把错误匹配输入到 PnP 或 ICP 中，会发生怎样的情况？能想到哪些避免误匹配的方法？ 
8. 使用 Sophus 的 SE3 类，自己设计 g2o 的节点与边，实现 PnP 和 ICP 的优化。 
9. 在 Ceres 中实现 PnP 和 ICP 的优化。

### 总结

**特征点进化表格：**

| **特征类型** | **图像块**     | **角点、边缘**                         | 人工设计的特征点       |
| ------------ | -------------- | -------------------------------------- | ---------------------- |
| **原理**     | **灰度值不同** | **考虑灰度值辨识度更强**               | **考虑尺度和旋转问题** |
| **性能**     | **不稳定**     | **存在尺度和旋转问题，鲁棒性不强**     | **鲁棒性强，效率高**   |
| **举例**     |                | **Harris 角点、FAST 角点 、GFTT 角点** | **ORB特征**            |

**ORB特征点性能改进表**

**第七章内容总结图**