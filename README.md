# FRC2022CommandBaseCode
自己编写的使用CommandBase模式控制的5737 FRC2022板版车的程序

# 2023.7.13 周四 抛物线计算
![抛物线计算](./Image/抛物线计算.png)

![角度](./Image/角度计算.png)

```java
 public double calculate_degree(double distance) {
    double x1 = distance;
    double y1 = PhotonVisionConstant.Camera_height - PhotonVisionConstant.Target_height;
    double x2 = distance - 0.2; // the 0.2 is the distance between Target and parabola top
    double y2 = y1 + 0.1; // the 0.1 is the distance between Target and parabola top

    double parabola_a = ((y1 * x2 / x1) - y2) / ((x1 * x2) - (x2 * x2));
    double parabola_b = (y1 - (x1 * x1 * parabola_a)) / x1;

    double vertex_x = -parabola_b / (2 * parabola_a);
    double vertex_y = (-parabola_b * parabola_b) / (4 * parabola_a);
    double ang = (Math.atan((2 * vertex_y) / vertex_x)) * 180 / 3.14;
    return ang;
  }
```
# 2023.7.10 周一 抛物线计算

$ V_x = V*cosθ $

$ V_y = V*sinθ = gt $

$ X_2 = V_xt $

$ Y_2 = {1\over2} gt^2 $


# 2023.7.5 周三 ShuffleBoard学习
今天更新了Shooter里的shuffle功能。

```java

Shuffleboard.getTab("Numbers").add("Pi", 3.14);
//更新数据有两种方法，一种是在periodic里面直接使用add方法更新。
//这种方法不知道为什么总是报错，导致机器人断联。
//还有一种方法是使用网络表的形式，如下
```

``` java
//创建标签页
ShuffleboardTab ShooterTab = Shuffleboard.getTab("ShooterTab");
//获取标签页,返回标签页对象.这个语句不知道为什么也总报错
//Shooterlayout = Shuffleboard.selectTab("ShooterTab")

//创建Layout
ShuffleboardLayout Shooterlayout = ShooterTab
  .getLayout("Shooter", BuiltInLayouts.kGrid);
  .withSize(2, 2)
  .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands;


public NetworkTableEntry ShooterSpeed = ShooterTab
      .add("Shooter Speed", 0)                          //添加想要显示的部件，初始化部件名称和初始值
      .withWidget(BuiltInWidgets.kGraph)                //设置显示模式为Graph(折线图)
      .withProperties(Map.of("min", 0, "max", 10000))   //配置最大值和最小值
      .withSize(3,3)                                    //配置组件大小
      .withPosition(0, 0)                               //组件位置
      .getEntry();                                      //配置为可更新的


public ShooterSubsystem() {

}

public void periodic(){
      ShooterSpeed.setDouble(RawSensorUnittoRPM(m_shooter_left_falcon.getSelectedSensorVelocity()));
   //更新网络表内容
}
 ```

 这里没太看懂：
 https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/layouts-with-code/organizing-widgets.html

![将小部件添加到布局](./Image/将小部件添加到布局.png)