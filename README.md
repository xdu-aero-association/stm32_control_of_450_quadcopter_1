# 工程训练大赛无人机组投放机构控制代码

PX4飞控控制投放装置原理跟相机触发方式一样，都是通过映射辅助AUX通道实现对应的信号发送。

相机触发方法参考如下链接：

[Camer Trigger](https://docs.px4.io/master/en/peripherals/camera.html)

触发方法有很多种，这里我们使用GPIO触发的方法，具体方法如下：

飞控响应来自地面站的快门mavlink数据（直接在地面站上面点击拍照按键即可触发），并在设定好的AUX通道（默认是56，可以修改，这里修改为1）上面输出电平跳变，单片机对飞控发出的电平跳变信号进行捕获，进而进入中断函数执行舵机投放。

注：

1. 该方案不适用于遥控器对通道的直接控制，如果需要遥控器直接通道控制触发，需要进行AUX通道映射。（在Radio菜单对应的AUX passthrough选项中修改即可）
2. mavlink快门触发优先级高于遥控器通道映射触发，如果同时设置AUX通道映射和mavlink触发到同一个通道的话，会导致飞控无法响应这个通道来自遥控器的数据。
3. mavlink快门触发可以通过mavros对应的msg实现offboard外部控制。具体方法参考下图，这里以官方的vio触发为例：
    ![https://s3-us-west-2.amazonaws.com/secure.notion-static.com/c170918b-ed05-4d40-8eaa-e663f8142209/Untitled.png](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/c170918b-ed05-4d40-8eaa-e663f8142209/Untitled.png)
4. mavlink快门触发不管映射多少个通道，都是同时触发，无法实现指定通道触发。这里因为比赛需要轮流投放三个物体，因此具体的数据处理部分在单片机上面利用状态机，实现轮流投放对应的物块。

负载添加方法参考如下链接：

[Payloads adn Cameras](https://docs.px4.io/master/en/payloads/)
