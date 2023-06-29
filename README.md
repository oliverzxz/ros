    我们的组号是第二组，成员有李孟鲜 李拾生 胡洋 张煊泽 龙云涵 翁嘉陆

    我们机器人的导航是基于tianbot_mini功能包开发的，主要的开发文件在master这个branchs中的src/tianbot_mini文件夹中，主要的文件有get_pos.py、obstacle_avoidance.py、send_goal.py,
还有launch文件夹中的move_forward.launch、obstacle_avoidance.launch、send_goal.launch
原本的实现思路是调用雷达数据实现沿着障碍物绕行避障，在起点处算的终点的位置，然后在臂章的过程中加入判断，如果已经成功避障，则停止绕行，向目标前进。但是实现起来非常复杂，由于时间有限，我们实现了其中的一部分，对应的代码在obstacle_avoidance.py和obstacle_avoidance.launch两个文件中。之后我们尝试采用tianbot_mini开发的导航算法，结合坐标计算，进行边走边建图的方式，逐步到达目标点，最终的实现部分对应的代码是send_goal.py和send_goal.launch
其余文件是在实验过程中实现部分功能的尝试。

小组分工：

任务一：写说明文档指导对小车不熟悉的人完成基本配置（张煊泽）
任务二：完成小车的相对初始状态的位置，并对输出做出解释（胡洋）
任务三：获取任意封闭环境的地图，将地图与实际环境进行对比（翁嘉陆）
任务四：避障实现（李拾生 李孟鲜 龙云涵）
