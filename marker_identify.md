

# 项目背景

本次测试中所用靶标为平面贴纸形状，每个靶标上均有八个追踪点。由八个追踪点的相对位置关系，可唯一确定靶标 id。测试过程中，双目内窥镜捕捉靶标图像，自动识别：__靶标 id，及靶标坐标系在相机坐标系下的位姿。__

<img src="/img/marker.png" width=45%>
<img src="/img/markerNum.png" width=45%>

<font color="red">总共可以有 64 种 id。
下方的五个圆圈用于确定x轴和y轴。</font>

e.g.
|id|000|123|
|---|---|---|
|marker|<img src="/img/id000.png">|<img src="/img/id123.png">|

# 算法流程

```dot {engine="dot"}
digraph T1 {
    形态学滤波 [shape=egg];
    高斯滤波 -> 阈值分割;
    阈值分割 -> 形态学滤波 [label="先开运算：消除黑色背景中的白色噪声。
    再闭运算：消除白色圆圈中的黑色噪声。"];
    形态学滤波 -> canny [label="提取边缘信息"];
    canny -> findContours [label="保留连续的边缘作为轮廓"];
    findContours -> 筛选矩形框
    筛选矩形框 -> 确定目标圆 [label="在矩形框内检测：
    先根据面积大小进行一轮筛选。
    再根据距离进行第二轮筛选。
    得到 8 个目标轮廓。"];
    确定目标圆 -> fitEllipse [label="将所有轮廓拟合成椭圆"]; 
}
```

```c++ {.line-numbers}
bool findCoordinateX(vector<point2f> points) {
    for (const auto& a:points) {
        for (const auto& b:points) {
            for (const auto& c:points) {
                if () {  // 条件1：{向量ab}=={向量bc}
                    continue;
                }
                if () {  // 条件2：其余所有点均位于{向量ac}的逆时针方向
                    continue;
                }
                if () {  // 条件3：a的最大夹角==c的最小夹角，记为θ
                    continue;
                }
                if () {  // 条件4：c在θ方向上只有1个点
                    continue;
                }
                if () {  // 条件5：a,b在θ方向上有等距的2个点
                    continue;
                }
                return true;
            }
        }
    }
    return false;
}
```
