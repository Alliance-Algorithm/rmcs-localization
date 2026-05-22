# rmcs_localization

基于点云配准的重定位节点。输入为 `odom` 坐标系下的 SLAM 累积点云，输出为静态 `world -> odom` 变换。

## 快速开始

### 1. 构建

```bash
zsh -lc "build-rmcs"
```

### 2. 检查配置

默认配置文件位于 `config/localization.yaml`，需要重点确认以下参数：

- `map_path`：待匹配地图的 `.pcd` 文件路径
- `frames.world`：全局坐标系，默认 `world`
- `frames.odom`：SLAM 点云所在坐标系，默认 `odom`
- `subscription.pointcloud`：输入点云话题，默认 `/cloud_registered_undistort`
- `initial_world_to_odom.red` / `initial_world_to_odom.blue`：红蓝方的初始 `world -> odom` 位姿

当前实现假设输入点云已经在 `odom` 系下完成累计，不依赖 `base_link` 的 TF 输入。

### 3. 启动节点

```bash
ros2 launch rmcs_localization launch.py
```

节点启动后会：

- 加载 `map_path` 指向的 PCD 地图
- 订阅 `subscription.pointcloud` 指定的话题
- 等待重定位服务触发

### 4. 触发重定位

红方：

```bash
ros2 service call /rmcs_localization/relocalize/red std_srvs/srv/Trigger
```

蓝方：

```bash
ros2 service call /rmcs_localization/relocalize/blue std_srvs/srv/Trigger
```

`/rmcs_localization/relocalize/lost` 当前仅为占位实现，不执行实际重定位。

### 5. 查看输出

重定位成功后，节点会发布：

- 静态 TF：`world -> odom`
- 调试点云：`/rmcs_localization/origin_pointcloud`
- 调试点云：`/rmcs_localization/mapped_pointcloud`

其中：

- `origin_pointcloud` 是按初始位姿中心截取出的局部地图
- `mapped_pointcloud` 是配准完成后映射到 `world` 系下的输入点云

## 算法流程

### 1. 读取初始解

收到 `/rmcs_localization/relocalize/red` 或 `/rmcs_localization/relocalize/blue` 请求后，节点从配置读取对应的 `initial_world_to_odom` 作为初始解。

如果已经存在上一轮成功的重定位结果，则优先使用当前保存的 `world -> odom` 作为下一次配准初值。

### 2. 收集输入点云

节点进入收集状态后，会在固定时间窗口内累积 `subscription.pointcloud` 的输入点云。当前窗口长度为 `2.0s`。

这里的输入点云默认是 SLAM 已经累计完成、并且已经处于 `odom` 坐标系下的点云，因此不再额外查询 `odom -> base_link`。

### 3. 裁剪局部地图

节点使用初始解的平移分量作为中心，从全局 PCD 地图中按 `registration.initial_map_radius` 裁剪局部地图，降低后续配准的搜索范围。

### 4. 执行点云配准

配准模块会：

- 对局部地图和输入点云做预处理
- 使用配置中的迭代次数、体素分辨率、距离阈值等参数进行粗配准和精配准
- 输出最终的刚体变换结果

由于输入点云本身就在 `odom` 系，配准结果直接解释为 `world -> odom`。

### 5. 发布重定位结果

配准成功后，节点会：

- 保存新的 `world -> odom`
- 发布 `world -> odom` 静态 TF
- 发布局部地图与配准后的调试点云

后续再次触发重定位时，这个结果可以继续作为下一次的初始解。
