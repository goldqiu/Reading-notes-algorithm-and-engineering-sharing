### 董靖博士公开课：

董靖博士：佐治亚理工博士

讲座分成三部分：

#### 第一部分：

##### 因子图模型在机器人领域（感知）：

Factor graph model in robotics

在讲因子图的具体模型之前，首先先定义一个简单的机器人问题（SLAM）。

![因子图1](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图1.jpg)

在样例中，假设有一个机器人在往前运动，过程中能观察到两个路标点，定义了三个时间，三个时间中有路标点的观测量和机器人自己运动的估计量（轮速计）

用贝叶斯网络（Bayes Net）来描述SLAM建模问题（因子图）。

![因子图2](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图2.jpg)

在贝叶斯网络中有一些变量，这些变量通过有向的边来连接。图中定义了两类四种变量。蓝色是路标节点的变量，黄色是机器人的状态变量，都属于状态变量。红色是机器人对路标点的观测，绿色是机器人对自己运动的观测，都属于第二类变量-观测变量。这个贝叶斯网络实际上描述了状态变量和观测变量联合的概率模型。

概率模型的定义是假设知道了系统的状态变量（机器人所在位置、路标点所在位置），可以推测出得到的观测量是什么（因为观测量是通过某些传感器得到的，而传感器是有已知模型的，可以通过数据手册得到），即已知机器人的状态量和传感器的模型，就可以推算出机器人观测量。

![因子图3](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图3.jpg)

在公式中，用X来表示机器人状态量，用Z来表示观测量。

假设知道系统状态的分布和给定系统状态下的观测量的条件概率（通过传感器的概率模型），相乘得到的联合概率就是系统状态变量和观测变量的联合概率。

观测变量的条件概率的特点：路标点1的观测变量只跟时间1的机器人的位置（状态变量）和路标点1的位置有关，跟路标点2无关。图中也可以很清晰的看到，路标点1的观测变量通过两个边连接到机器人的位置变量和路标点1的位置变量。这就说明这个观测变量是由这两个边连接到机器人的位置变量和路标点1的位置变量单独决定的。 每个系统的观测变量都是这样的特点。这说明是由极少的状态变量决定每一个单独的观测变量（相互独立）。所以观测变量的条件概率是以一个乘积的形式存在的。这个乘积里面，对于每一个观测变量有自己单独的系统状态的子集（来决定每个的观测变量）。在示例中公式的红色部分，三个观测变量(条件概率)相互独立，相乘起来。同样绿色部分是两个odometry的观测变量，也相乘起来。（这个条件概率可以分解）

生成模型generative model：如果用一个贝叶斯网络来描述系统模型，在已知状态量的情况下，如何得到观测量。

现实生活中需要求解的问题是知道观测量，需要求解状态量，是一个状态估计的问题（SLAM问题）。而生成模型做的是相反的工作。

因子图是一类reference model （网络）。同样应用贝叶斯定律（Bayes rule），给定Z，求解X的概率正比于给定X,求解Z的概率(生成模型)

![因子图4](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图4.jpg)

分母的Z先验值是跟X无关的(相互独立)，可以省略。

对于求解的状态估计问题，就是给定系统观测量，求解系统状态量，使得条件概率最大。

![因子图5](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图5.jpg)

就是求解MAP（最大后验分布 maximum posterior inference）， 公式中右边项的X是先验概率（prior），一般是上一帧的最优后验概率，P(Z|X)是似然概率（likelihood，由传感器模型给定） ，左边就是后验概率。

每一个观测变量在贝叶斯网络里都是单独求解的（相互独立），所以所有的条件概率是乘积的形式，且可分解，在因子图里面，分解的每一个项就是一个因子，乘积乘在一起用图的形式来描述就是因子图。

![因子图6](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图6.jpg)

Modeling a SLAM Problem by a Factor Graph

![因子图7](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图7.jpg)

因子图里面包括两类节点和边

节点：状态变量（空心圆圈），是inference要求解的变量。

观测节点（就是因子，用方块标定），每一个因子表示每一个得到的观测量。

因子图里面还会包括一个先验因子（prior factor），就是公式中X的先验值(先验概率)，用来固定整个系统的解（避免数值多解，达到数值可解 ）。

![因子图8](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图8.jpg)

整个因子图实际上就是每个因子单独的乘积。红色对应观测量因子，绿色对应状态量之间的因子（轮速），紫色是先验因子。求解因子图就是将这些因子乘起来，求一个最大值，得到的系统状态就是概率上最可能的系统状态。

![因子图9](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图9.jpg)

实际上，每个因子都是用指数函数来定义的，因为每一个因子描述的是一个观测变量（IMU或者相机），根据中心极限定理，绝大多数传感器的噪音是符合高斯分布的(所以选择指数函数），指数函数对应了error function，包括两个部分：系统状态量和观测量，error function实际上表示的是用状态量去推测的观测量与实际观测量的区别。

以odometry factor为例，它的error function是：后一个位置减去前一个位置（这是预估的量，因为后一个位置是预估的），再减去实际观测到的量，其实就是预估的量减去观测的量。

Landmark factor的error function也一样，是用Xi和Lj推测出来观测点的位置量减去实际的观测点位置量。

error是越小越好的，这说明观测值和预测值是接近的。

套到MAP里面来看，因子图的求解是要所有因子的乘积最大化。而对于指数函数形式（负指数），每一个乘积最大化代表里面的fx最小化。这正好符合希望误差函数的误差最小。即系统状态与观测值越吻合，error越小，那φ就越大（MAP最大），那么就是希望能找到这么一组状态变量能尽可能和观测量吻合。

现在进入到具体的求解问题。

对定义的因子图乘起来的函数，希望能够最大化，假设所有因子都是指数函数形式，对函数取负对数，则指数函数最大化问题实际上等于一个非线性最小二乘问题。

![因子图10](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图10.jpg)

那么就是给定一个因子图的情况下，如何求解因子图（通过非线性最小二乘），常用方法是迭代法，如高斯牛顿。

##### 如何求解因子图（通过求解非线性最小二乘）：

Solve factor graph by solving non-linear least squares

给定一个初始值，通过某种办法或猜测一组可能的系统状态变量，去求解一个修改量（dx），让最小二乘值尽可能变小，求出dx再加上原来的x（修改了初始值），再代回原来的函数，迭代iteration调整初始值，达到一定的停止条件后，求出最优的一个修改值。

![因子图11](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图11.jpg)

中间步骤每一步求解增量是通过线性化来做（对非线性函数做一阶泰勒展开Taylor expansion），得到线性最小二乘问题linear least square。

![因子图12](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图12.jpg)

有两个方法:一可以直接求解，二可以通过normal equation来求解（常见）

![因子图13](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图13.jpg)

![因子图14](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图14.jpg)

JtJ矩阵是正定矩阵（Jacobian满足列满秩column full rank），做Cholesky分解（Cholesky factorization）， 将JtJ分解成RtR，而R是上三角阵，比较简单，实际上是求解两个上三角阵的线性方程组。

![因子图15](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图15.jpg)

对于直接求解，通过QR分解，如果给定只有J，不计算Jt，对J进行QR分解，可以直接求解出R。

QR分解的速度会慢一些，但Cholesky分解的数值稳定性要差，一般是要根据所求解系统的性质来决定。

![因子图16](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图16.jpg)

通过SLAM案例：对于Jacobian矩阵，每一行（block行）对应的是一个因子，比如说紫色部分表示的是prior因子，绿色表示的是odometry因子，红色表示路标点观测因子。紫色一行只有一个(只在X1有非零项)是因为先验因子只影响了一个状态（X1）, 同样对应里程计因子，只有X1和X2有连接，在J矩阵中第二行只有X1,X2有非零项。 

JtJ：information matrix

J矩阵具有稀疏的性质，因为每一个观测量相关的系统状态量很少（每一行只填了很少的的block），对于求解比较有利，稀疏性最好，求解速度越快。因子图生成的线性代数的概率模型有很好的稀疏性，所以因子图求解这类问题效率很高。

有一个问题的说明：对于J矩阵，每一个变量摆在哪里，实际上变量的顺序对于求解的性能有很大的关系，这就是线性系统求解的ordering的问题（Select the correct column ordering does matter since it decide the sparsity of information matrix）。对于不同的ordering，求解出来的上三角阵的稀疏程度不一样。通过调整变量元素的顺序（ordering），我们可以得到稀疏性比较好的矩阵，对于直接方法，元素的ordering是很重要的。找最好的ordering是NP hard的问题， 现实生活中不会去找，因为非常慢。有些线性代数领域的专家，开发了找近似最优approximated best ordering的方法，比如COLAMD。

##### 总结：

如何定义这个模型，

如何求解

在机器人运行的过程(Graph grows while robotis moving)，因子图往往是逐渐成长和增大，逐渐在往外延伸的一个因子图， 比如说在往前移动的过程中，加了一组节点，随着观测的数据越来越多，加了越来越多的因子进来。 每一次求解都是比上次大一点，而且大部分因子图跟之前因子图是基本一致的，这是incremental inference（增量推理），假设因子图只加了一点点， 其他都没变(Most factors leaves unchanged)，如果从零开始求解，矩阵会越来越来，速度会越来越慢，绝大数都是重复性的工作。

如何避免？(How can we effectively update the solution?)如何能够在求解状态变量越来越多的情况下能够引出一个近似常数。

最常用的办法是isam，有isam1和isam2

isam1实际上做的就是增量QR分解(Incremental QR Factorization)， 给定一个J矩阵，可以分解成Q和R，假设因子图其他都不变，之前的因子图还在，加了一些新的东西，每一个因子对应的是J矩阵的每一行，所以新加了几个因子，就是在J矩阵增加了几行，如果在已知分解出来的QR的情况下，J加了几行，如何快速算出R矩阵。

![因子图17](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图17.jpg)

ISAM1论文给的公式，A就是图中的J，如果A矩阵下面加一行（Wt），左边乘一个扩展的正交矩阵，右下角标一个1，就能够将增量的那一行加到R项。通过这个就能够将增量QR分解转化成增量三角化R的问题，就是如果在R下面加了几列，如何将矩阵重新变成三角阵（通过正交阵）。

![因子图18](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图18.jpg)

通过given rotation的办法：如果在R矩阵的下一列加了一个非零元素，左乘given旋转矩阵，可以将非零元素变成零元素，对应的代价是左乘后的矩阵0后面加了一些非零元素。（By applying multiple Givens rotation can re-triangulate the R matrix）

一步一步做这个步骤，将每一个元素右边的元素逐渐消为0，消到最后一个元素，实际得到的矩阵是一个密集的三角阵(Lower-right part of R will be fully-filled)。大小是跟从哪里开始消元相关的。(How large the fully-filled part is decided by variable ordering )

例子：比如机器人顺时针转圈，元素按时间的ordering来排，假如新加了一个元素，建立了回环，看到一开始第一帧第二帧看到的东西，变量会影响最开始的元素，会增加很大的密集三角阵。这样处理时间会变慢，内存会比较大，所以需要定期进行reordering，重新排序，重新做QR分解等，会得到比较稀疏的R矩阵，计算速度才会比较快。（Periodically re-ordering/re-linearization needed）

ISAM1是讨论线性系统，求解因子图是用的迭代方法，用的是线性代数的方法，实际上是增量做了每一步线性化求解的步骤，也就是做线性增量QR分解的过程中，线性化点没有变过，线性化点是影响系统求解精度（如求解卡尔曼滤波问题，线性化点很重要），而增量QR分解本身是不解决重线性化的问题，对于isam1，需要定期重排序，重线性化（定期更新），确保解有一定精度。

isam2（利用贝叶斯树）线性因子图推理：将因子图转换成树状结构，如果是操作了少量因子（少量系统变量），只需要动少许树上的节点，不需要重新求解整个树，速度比较快。

如果遇到很大的回环，因为大部分变量都有触及，贝叶斯树中很多元素会被移动，有大量节点需要重新计算，

无论isam1的增量QR分解和isam2的贝叶斯树，遇到大回环情况，也是需要重新求解。

贝叶斯树是理论最优的方法，isam1是偏经验的方法，贝叶斯实现起来比较难。可以根据需求选择isam1还是isam2。

#### 第二部分：

miniSAM库在求解因子图优化问题的使用

方便初学者学习因子图（特别只会python），参照gtsam，首次提供了python和numpy的API，提供了线性代数求解器（包括QR分解）3rd party sparse linear solvers

ceres，g2o：通用的线性最小二乘库

gtsam：因子图优化

minisam：目前还没有自动求导

给定最简单的位姿图，有5个节点，有回环，里程计

![因子图19](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图19.jpg)

第一步，建因子图，首先要定义传感器的噪音模型，因为每一个因子对应的是观测的量，概率模型是传感器概率模型来定的。

![因子图21](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图21.jpg)

Define the error distribution (measurements noise model)：

Gaussian models given covariances

Robust models (Huber, Cauchy, etc.)

第二步：加先验因子，固定求解位置。Insert a prior factor

具体编写：graph是一个容器，容器中加了先验的因子，构建每个因子过程中，有三项主要输入，

![因子图22](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图22.jpg)

第一项是所连的状态量，表示办法是key（一个数字加一个字符），第二个是观测量，传感器实际给出的（A measurement, for 2D pose is a SE(2)）。

第三个是误差函数的分布（传感器模型）An error distribution。

对于先验因子：是实际的位姿减去观测的位姿。

对于里程计因子：比如X2 - X1- Z，即实际的里程计值减去观测的里程计值。

![因子图23](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图23.jpg)

![因子图20](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图20.jpg)

第二步还要给变量初始化值（Add Initial Values of Variables），因为用的是迭代法求解，最重要的是有初始值， 在minisam中有一个容器存储， 告诉变量需要从这个值开始求解。

![因子图24](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图24.jpg)

第三步：选择非线性最小二乘求解器（Gauss-Newton Levenberg-Marquardt Dogleg），还有线性求解器，如QR分解、Cholesky。

![因子图25](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图25.jpg)

考虑新的传感器，相机、雷达，自定义传感器模型。自定义因子如何做？

![因子图26](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图26.jpg)

定义一个类，定义error function，输入是系统状态（给定系统状态），返回误差值。

误差函数的J矩阵，如果有求解好的解析式就直接放上去，或者通过数值差分求解J矩阵。ceres支持自动微分求解。解析解比较快，数据解比较慢（Analytic Jacobian is optional if use numerical Jacobian）。

minisam速度跟ceres g2o 差不多，比GTSAM快一些。为方便做迁移，提供了python接口。

因子图优点：求解速度快，编程方便。

#### 第三部分：

三维四维重建在农田中的应用（3D crop monitoring : over time）和如何用因子图解决机器人运动规划的问题。

使用因子图的思路：

农业场景作物监测中，如何用因子图加速重建的过程。

希望得到精确的时序信息，如何将三维重建应用到连续变化的持续信息中（3D reconstruction of continuously changing temporal sequences），引出四维重建，加上了时间维度。

align multiple 3D reconstructions in a single frame，3D reconstructions are from different coordinate frames，Although with GPS sensor they are all in global frame, error exists

通过GPS或者点云配准得到的坐标系是不准的。

如果若干三维重建在同一个坐标系，那么不同时间的三维空间的同个点位置都是一样的，就可以得到在同个点上三维重建的时序信息。

实际上第一步做的是三维重建，包括前端和后端（front-end and back-end），前端是原始传感器的处理，做点的匹配，将传感器送给因子图的后端，后端包括了若干种因子（extract SIFT feature from images, match by FLANN, and output visual correspondences），包括structure-less vision factor ，

Back-end: factor graph optimization: 

Visual landmarks by structure-less vision factor

IMU by pre-integrated IMU factor 

因为空间中有很多点，单独求解太消耗内存，所以就引入了路标点因子，不需要求解路标点，就可以求解系统状态。求解了系统状态再求解路标点，这样节省内存，不需要显性存在路标点。

四维重建：建立若干三维重建在空间中的关系，利用因子图去优化，得到空间中连续的四维重建。

给定两个三维重建，找某个空间中的三维点，用图像特征点匹配的方法，将两个三维重建的三维点进行匹配，匹配后只剩下一个点，就是空间中同个三维点，为公共的目标点，通过这种办法将两个目标点合并成一个点， 变成公共的因子，就将两个三维重建合并为一个四维重建，类推可以建立出大的四维重建，对因子图进行整体优化， 通过公共的坐标点将所有的三维重建限制在了一个坐标系下。三维重建中例如ICP点云匹配是有误差的，每一组点云都会有误差，直接匹配的话是会匹配不上甚至有变形的， 如果去做四维重建的话，因为有对应的公共路标点去约束，再去做四维重建， 也会有误差，只是通过公共路标点的限制下误差是相同的，因为是在公用的大的因子图里面去优化，得到的误差也是一样的。（Data association is performed by image matching, then find matched landmark in each 3D reconstruction，Matched landmarks are identified as the same landmarks, which are unchanged 3D structures cross sessions，By merging shared landmarks, two 3D reconstructions can be merged as a single one）

![因子图27](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图27.jpg)

四维重建帮助农作物进行可视化，是非常明显的。

![因子图28](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图28.jpg)

第二个案例，运动规划。motion planning in robotics

因子图有比较好的特性，在其他领域也有应用。加速运动规划的过程。

运动规划问题：有一个机器人在有障碍物的环境里，知道初始位置和障碍物，希望到达一个位置，找到一个轨迹（避开所有障碍物），但并不是在线做规避，是用优化或采样的办法提前给出轨迹。

![因子图29](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图29.jpg)

轨迹特点和要求：1.不能跟障碍物有碰撞，feasibility

2.没有碰撞的情况，尽可能最优（时间最短，路径最短，能量最低，尽量平滑）

两类方法：1.采样法

2.优化法：速度快

套用因子图定义的联合概率分布，求解一个最大概率，来实现运动规划。

MAP最大后验分布包括两部分，prior 分布（先验）（对应第二个要求，最优，尽可能平滑enforce smoothness），似然分布Collision-free likelihood:（第一个要求，不能有碰撞，enforce feasibility，相当于SLAM里面的观测量，就是没有碰撞的概率尽可能高）

![因子图30](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图30.jpg)

乘起来求解最优化问题，就是求解运动规划问题。即希望离障碍物越远，碰撞的概率越低。 

设置成指数函数：假设是高斯分布的形式，这是因子图所要求的。让函数平滑，好优化。

计算过程中将机器人的形状用若干球体去近似。球体与空间中点的距离比较好算：点与球心的距离减去半径。

这个机器人碰撞的概率就是若干球体碰撞的概率的乘积。

对于轨迹的圆滑，用的是高斯过程先验GP prior，是定速先验值（整个过程速度变化小，加速度最低，施加的力是最小的，最优的一个情况）

对于每一个时间状态，每一个时间节点计算与环境中碰撞的概率，两个节点间计算加速度最小高斯过程先验值，加起来就是大的因子图去优化，得到的就是最优的没有碰撞的轨迹。

![因子图31](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图31.jpg)

优化办法常遇到的问题是success rate（遇到局部最低值（最优），很难前进到全局最优，很常卡在一个有碰撞的地方，这个时候就优化失败了）

因子图的优势：计算快。

扩展：如何将增量扩展应用到重规划Replanning的问题中， 假设有初始位置和目标位置，规划了一个轨迹，执行到中间，换了目标位置，需要重新做轨迹的规划（重新做优化 given current state and a new goal），如果将这个问题构建成因子图的话，重规划的问题相当于只改变了两个因子， 包括目标节点因子和当前状态的因子（加一个当前状态的约束，作为先验），其他因子不需要变，实际上优化的是变化量非常小的一个图，给使用增量因子图推理很大的空间，如果用isam处理少量变化的图，比从0计算后半段的图效率更高。用贝叶斯树的办法， 如果只改变最后的节点和加一个当前位置的约束，贝叶斯树只是少量的变化， 需要改变的位置越靠后，需要重新规划的轨迹越短，贝叶斯树改变的越少，计算量越低。因子图只有少量的变化，增量规划可以获得大的提升。

![因子图32](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图32.jpg)

答疑：

对于概率来讲，在贝叶斯网络， 这些概率（有效的概率）的积分加起来是1（概率为1的要求）。而因子是乘起来。 对于因子没有概率为1的要求。

因子图中的每个因子的Fi是具体的函数，比如传感器是自己决定的，对于内部的值来讲，观测量Z是传感器获得的，变化的是X，是求解的状态，在求解过程中逐渐迭代。

实时做运动规划：地图在实时变的话， 目标函数（collision free probability）也要实时变的，J矩阵每一个error在时间点上需要重新求，要有实时的因子图优化过程。

双机协同SLAM因子图：DDF-SAM，分布式因子图， 每个机器人有自己处理自己的数据，处理自己的因子图，还有个公共图，发送压缩过的数据，量比较小。

集中式因子图就是若干个机器人产生的因子汇总到一个大的后端服务器里面进行优化，有一个整体的大的图。

单纯的增量求解对reordering没有要求的，一直增量求解的矩阵会越来越密，需要reordering保证性能。

ceres：最小二乘优化求解器。

因子图求解的过程转换成一个最小二乘。只是描述问题的一种方式，描述成了概率问题。因子图里面只需要定义传感器的概率特性，但计算和其他优化器没有什么区别。

也可以自己处理概率问题，然后再用ceres求解最小二乘。

贝叶斯树只是因子图的一种表示形式。树状结构，只操作了少量变量，只需要操作树的少许节点。

因子图本身表示的是J矩阵，是分解前的形式，贝叶斯树表示的是分解后的形式，分解后的R矩阵对应的是树状结构。贝叶斯树存储的是分解后的因子图，只操作了少许变量，对应的R矩阵也动了一点点，反映到树上也是少许。对于回环问题，整个R矩阵都要变，贝叶斯树也一样。

GP指的是高斯过程。GP factor约束的是两个系统状态之间的加速度。速度差越大，GP prior产生越大的error， 概率就越低， error function 的cost就会增加。

因子图并不适合所有的问题，有内存随着时间增长消耗增多的问题，单纯的IMU用卡尔曼滤波可以解决。

四维重建中匹配问题: 图上使用重投影误差进行约束，或者GPS或者其他先验信息。根据遇到的问题方案不一样。

对于整个图来讲，先验信息是第一个时间的信息或者最后一个。对于运动规划来讲，先验信息是整个过程的加速度最小的信息。SLAM中是某个时间的信息。用高斯模型是方便数学处理，假设是指数函数会比较简单，实际上可以用鲁棒的模型。用高斯模型会比较直观。

重定位中重投影误差最小化可以用因子图。

minisam：求解优化的时候有GPU加速。

噪音模型的单位跟测量量的单位是一致的，跟协方差也是一个单位。 

边缘化:滑窗优化和增量优化：

内存小可以用滑窗优化，增量优化需要保存所有数据（存在树里面，随着图逐渐增长，图是在逐渐增长的），滑窗优化随着优化滑窗的过去，数据就边缘化掉了。	

增量优化的优点是考虑历史信息，精度更高，但内存也消耗更大。在很大回环的情况下，增量优化求解可能会有很大的问题。

滑窗优化优点：低计算量，低内存，没有历史信息

因子图不能直接处理传感器的数据，对于点云得先做好匹配，因子图只是后端方案。

根据需求选择合适的求解器方案，设计好对应的因子（建模包装成因子图的形式），多读论文，熟悉这种思维方式。

通过选择合适的鲁棒噪音模型，出现错误的因子，在优化中占的比重就会继续降低，不产生cost function，不对结果做出贡献。



### GTSAM 4.0 Tutorial

#### Theory

SLAM as a Bayes Net

1. SLAM属于链状的贝叶斯网络，当前的状态只跟上一个状态有关，路标点的测量值只跟当前状态和所对应路标点位置有关。

2. 先验分布或者先验值可以预设或者通过GPS等有绝对位置测量值的传感器来得到。

3. 最大后验概率正比于所有因子图相乘的最大概率值。

4. 线性化之后得到的J矩阵每一列对应了因子图中一个节点，是要估计的状态（包括机器人自身的状态和路标点的状态）。每一行对应的是每一个测量值（包括路标点测量值和系统运动测量值），也就是因子。转换为线性代数问题后，J矩阵是稀疏的，所以是比较容易求解的，收敛效率比较高，解算比较快。有两种办法求解这个线性代数问题：1. QR分解 （慢，求解稳定性好，特征值近似0，矩阵接近奇异还能求解）2.Cholesky分解（稳定性差，矩阵接近奇异可能会发散，快）

5. ordering：选择合适的J矩阵的列排序。GTSAM调用了COLAMD算法能求出近似最优ordering。

6. isam2：通过贝叶斯树的结构，增加了因子后重新优化不需要从初始值开始做起，利用之前已有的解、之前已经线性化好的贝叶斯网络，重优化的时间比较短，有些优化工作不用重新做，特别是在graph变化不是特别大时（比如机器人建图往前走，没有闭环检测）

7. 先验因子（prior factor）或初始分布的作用：固定地图的位置，不让地图在空间中进行平移和旋转，即求解唯一。

   

#### Programming

C++：

1.Build factor graph

设置noiseModel：传感器假设是高斯分布，对应的协方差矩阵就是noiseModel

BetweenFactor：里程计或系统运动学模型。里程计测量值相当于系统控制输入。 

2.Give initial values

设置初值很重要，关系到能否跳出局部最优值，达到全局最优值，一般设置为比较接近最后的解的初始值

3.Optimize

4.(Optional) Post process, like calculate marginal distributions



Write your own factor

Design a cost function to minimize

自定义因子：

1. 定义输出的误差向量e决定了factor输出的cost function

2. 定义J矩阵：最困难的部分

3. 如何求J矩阵：GTSAM自动微分，因为误差向量形式很复杂，有很多函数嵌套。在GTSAM只需要提供每一步的函数和每一步的J矩阵，用嵌套的形式去自动求解。（通过自定义模板类的方式）



Matlab：

GTSAM在matlab有个toolbox，可以进行c++代码生成matlab代码（通过生成自定义matlab toolbox）

安装：

下载gtsam：https://github.com/borglab/gtsam.git 

找到GTSAM_INSTALL_MATLAB_TOOLBOX,把OFF改为ON，然后编译安装

ubuntu下安装matlab，安装好后根据GTSAM安装路径配置toolbox（使用matlab设置路径添加该文件夹）

测试test_gtsam出现以下则成功：

![因子图33](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图33.png)

下载demo程序：https://github.com/dongjing3309/gtsam-examples

运行如下：

![因子图34](https://github.com/goldqiu/Reading-notes-algorithm-and-engineering-sharing/tree/main/%E5%9B%A0%E5%AD%90%E5%9B%BE%E4%BC%98%E5%8C%96/%E5%9B%BE%E7%89%87/因子图34.png)



Applications

IMU ：预积分转换成关键帧之间的IMU factor

Structure-less factor by Schur complement ：四维重建

Multi-Robot SLAM

Distributed real-time cooperative localization and mapping using an uncertainty-aware expectation maximization approach



### 三川博客

#### 算法发展脉络

1、塞巴斯蒂安最开始就是研究图优化的，只不过换了一种模型表达方式，叫做“平方根平滑建图”，这篇文章提出的概念就是批量更新。这里埋下一个伏笔，因为后来他们发现这个模型竟然有一个彩蛋。因为他们通过平方根的方法构建好了矩阵后，新添加的变量都是从最后一行。

2.后来Michael觉得，每个矩阵才添加了几行就得更新，能不能简化一点呢？因为每次都是在最后面添加的，所以只要把后面的那几行“搞搞”就行了，就找到了一种Givens 旋转的方法就可以消元了（数值方法必考知识点）。这时候作者就发了第二篇文章“iSAM”（增量平滑建图），这里面呢，作者用的思路是，正常情况下，用Givens增量消元，如果误差太大，那就固定隔几分钟来个全部优化（又用到了论文[1]）。

3、又过了一段时间，作者发现，为什么要分“增量优化”和“全局优化”两种模式呢？ 有没有这样一种方法呢，局部更新如果没有影响全局图，就更新局部；如果影响了全局，就更新全局？或者说有没有一种方法让我影响了多大区域就更新多大面积呢？

4、后来他就想到树结构，从树的根部开始，受影响了多大范围，我就更新多大范围。这个树该怎么构建呢？

找到了一种贝叶斯树的结构。这次呢就不用分“批量”和“增量”两种模式了，而是智能调节，所有的数据都在树结构上，新加进来的放在树根部，受影响的区域肯定是在刚加进来的地方。把受影响的区域提取出来。这就成了最终的贝叶斯树构建的因子图优化了，即iSAM2[3].

### SLAM后端优化发展：

1. 滤波阶段：局部信息融合

2. 图优化阶段：全局一致性优化
3. 增量平滑阶段：用贝叶斯树维护一个动态更新的数，兼具滤波的速度和图优化的精度

在slam的后端优化问题中，通常会通过⼀些传感器的观测，比如视觉特征点，IMU预积分量，Lidar面点和边缘点（角点）的约束去构建一个优化问题，求解状态量（如位姿、速度等）。这个时候存在一个问题，当给这个系统新增⼀个约束时，就会重新建立所有的约束对状态量的优化问题进行求解；当优化模型增大时，显然进行一次优化的时间也会增加很多；一方面实时性遭遇了挑战，另一方面，很久之前的状态也没有继续更新的必要。为了解决这个问题，⼀种方式是使用滑动窗口来控制优化问题的规模，而滑动窗口需要处理好边缘化的问题；另一种方式，可以使用因子图的模型来解决这个问题。