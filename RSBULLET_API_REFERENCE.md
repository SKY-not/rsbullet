# rsbullet / PyBullet API 对照表

> 说明：本表基于 `rsbullet-core/src/client.rs`、`rsbullet-sys/bullet3/examples/pybullet/pybullet.c` 以及 `REPORT-2025-10-10.md`。
>
> - **Rust API 名称**：`PhysicsClient` 及关联模块中暴露的 Rust 方法。
> - **PyBullet API 名称**：PyBullet 文档/脚本使用的调用标识（非 C 层函数名）。
> - **状态**：Implemented、Pending、Optional（仅在编译选项满足、或为遗留/低优先级时实现）。
> - **说明**：简要含义及功能。
> - **其他**：Rust/Python 行为差异、相似 API 区分等补充信息。

## Connection & Session

| Rust API 名称 | PyBullet API 名称 | 状态 | 说明 | 其他 |
| --- | --- | --- | --- | --- |
| `PhysicsClient::connect` | `connect` | Implemented | 连接指定模式（Direct/GUI/SharedMemory/TCP 等）的物理服务器。 | Rust 返回 `PhysicsClient`; Python 返回整数句柄。 |
| `PhysicsClient::disconnect` | `disconnect` | Implemented | 主动断开当前会话。 | Rust 在 `Drop` 自动调用；Python 需手动调用。 |
| `PhysicsClient::get_connection_info` | `getConnectionInfo` | Implemented | 查询当前连接配置、端口、状态等。 | Rust 返回 JSON 字符串；Python 返回 `dict`。 |
| `PhysicsClient::is_connected` | `isConnected` | Implemented | 判断客户端是否仍可提交指令。 | Rust 缓存命令可用性，避免重复 FFI。 |
| `get_num_physics_clients` | `getNumPhysicsClients` | Implemented | 获取当前进程中的客户端数量。 | 仅统计 `PhysicsClient` 活跃实例。 |
| `get_num_gui_physics_clients` | `getNumGuiPhysicsClients` | Implemented | 获取当前 GUI 客户端数。 | GUI 模式互斥，Rust 侧有原子守卫。 |
| `PhysicsClient::can_submit_command` | `canSubmitCommand` | Implemented | 检测命令队列是否可用。 | Rust 标记为内部方法。 |

## Simulation Parameters

| Rust API 名称 | PyBullet API 名称 | 状态 | 说明 | 其他 |
| --- | --- | --- | --- | --- |
| `reset_simulation` | `resetSimulation` | Implemented | 清除世界，重置到初始状态。 | — |
| `step_simulation` | `stepSimulation` | Implemented | 执行单步动力学更新。 | Rust 返回 `Result<()>`; Python 返回 `None`。 |
| `perform_collision_detection` | `performCollisionDetection` | Implemented | 仅碰撞/AABB 更新，不积分。 | — |
| `set_gravity` | `setGravity` | Implemented | 设置全局重力向量。 | Rust 接受任意 `Into<[f64;3]>`。 |
| `set_time_step` | `setTimeStep` | Implemented | 设置模拟步长。 | Rust 文档强调尽量保持常量步长。 |
| `set_default_contact_erp` | `setDefaultContactERP` | Implemented | 修改接触 ERP。 | — |
| `set_real_time_simulation` | `setRealTimeSimulation` | Implemented | 启用/关闭实时模式。 | Direct 模式下无效。 |
| `set_physics_engine_parameter` | `setPhysicsEngineParameter` | Implemented | 批量更新物理参数。 | Rust 使用结构体封装选项。 |
| `get_physics_engine_parameters` | `getPhysicsEngineParameters` | Implemented | 查询当前物理参数。 | 强类型结果。 |
| `set_internal_sim_flags` | `setInternalSimFlags` | Implemented | 设置内部调试标志。 | 高级功能，慎用。 |
| `set_physics_engine_parameter` (多次调用) | `changeDynamics`（单项重载） | Implemented | 见 Dynamics。 | — |

## World Authoring & Persistence

| Rust API 名称 | PyBullet API 名称 | 状态 | 说明 | 其他 |
| --- | --- | --- | --- | --- |
| `load_urdf` | `loadURDF` | Implemented | 加载 URDF 模型。 | Rust 通过 `UrdfOptions` 配置 base pose 等。 |
| `load_sdf` | `loadSDF` | Implemented | 加载 SDF 场景，返回 body 列表。 | — |
| `load_mjcf` | `loadMJCF` | Implemented | 加载 MJCF 文件。 | — |
| `set_additional_search_path` | `setAdditionalSearchPath` | Implemented | 添加资源检索路径。 | Rust 确保路径存在。 |
| `load_bullet` | `loadBullet` | Implemented | 导入 `.bullet` 快照。 | Rust 返回载入的 body ID 列表。 |
| `save_bullet` | `saveBullet` | Implemented | 保存世界为 `.bullet`。 | — |
| `save_world` | `saveWorld` | Implemented | 导出 Python 脚本。 | — |
| `save_state` | `saveState` | Implemented | 保存内存状态。 | 返回状态 ID。 |
| `restore_state` | `restoreState` | Implemented | 恢复状态快照。 | Rust 使用 `RestoreStateOptions`。 |
| `remove_state` | `removeState` | Implemented | 删除指定状态。 | — |
| `load_soft_body` | `loadSoftBody` | Optional | 加载软体对象。 | 需启用软体编译；Rust 未暴露。 |
| `create_soft_body_anchor` | `createSoftBodyAnchor` | Optional | 创建软体锚点。 | 同上。 |

## Asset Creation & Mutation

| Rust API 名称 | PyBullet API 名称 | 状态 | 说明 | 其他 |
| --- | --- | --- | --- | --- |
| `create_collision_shape` | `createCollisionShape` | Implemented | 创建单个碰撞体。 | Rust 用枚举描述几何类型。 |
| `create_collision_shape_array` | `createCollisionShapeArray` | Implemented | 创建复合碰撞体。 | — |
| `remove_collision_shape` | `removeCollisionShape` | Implemented | 清理碰撞体资源。 | — |
| `create_visual_shape` | `createVisualShape` | Implemented | 创建视觉形状。 | — |
| `create_visual_shape_array` | `createVisualShapeArray` | Implemented | 批量视觉形状。 | — |
| `create_multi_body` | `createMultiBody` | Implemented | 程序化创建多体刚体。 | Rust 提供建造者式参数。 |
| `create_constraint` | `createConstraint` | Implemented | 创建约束。 | — |
| `change_constraint` | `changeConstraint` | Implemented | 更新约束属性。 | — |
| `remove_constraint` | `removeConstraint` | Implemented | 移除约束。 | — |
| `enable_joint_force_torque_sensor` | `enableJointForceTorqueSensor` | Implemented | 启用关节力矩传感器。 | — |
| `remove_body` | `removeBody` | Implemented | 删除世界中的 body。 | — |
| `get_num_constraints` | `getNumConstraints` | Implemented | 获取约束数量。 | — |
| `get_constraint_info` | `getConstraintInfo` | Implemented | 查询约束信息。 | — |
| `get_constraint_state` | `getConstraintState` | Implemented | 查询约束状态/力。 | — |
| `get_constraint_unique_id` | `getConstraintUniqueId` | Implemented | 获取约束 ID。 | — |
| `change_visual_shape` | `changeVisualShape` | Implemented | 动态修改视觉属性。 | — |
| `load_texture` | `loadTexture` | Implemented | 加载纹理。 | Rust 接受 `Cow<str>`。 |
| `change_texture` | `changeTexture` | Implemented | 切换纹理。 | — |
| `get_mesh_data` | `getMeshData` | Pending | 查询网格信息。 | 暂未实现。 |
| `get_tetra_mesh_data` | `getTetraMeshData` | Pending | 软体用网格数据。 | — |
| `reset_mesh_data` | `resetMeshData` | Optional | 软体/可变形网格重置。 | — |
| `reset_visual_shape_data` | `resetVisualShapeData` | Pending | 视觉重置。 | PyBullet 遗留 API。 |

## Bodies, Joints & Base State

| Rust API 名称 | PyBullet API 名称 | 状态 | 说明 | 其他 |
| --- | --- | --- | --- | --- |
| `get_num_bodies` | `getNumBodies` | Implemented | 返回世界体数量。 | Rust 返回 `usize`。 |
| `get_body_unique_id` | `getBodyUniqueId` | Implemented | 通过索引获取 body ID。 | — |
| `get_body_info` | `getBodyInfo` | Implemented | 查询 body 名称等信息。 | Rust 返回结构体。 |
| `compute_dof_count` | `computeDofCount` | Implemented | 统计总自由度。 | — |
| `sync_body_info` | `syncBodyInfo` | Implemented | 同步 body 信息至多客户端。 | 多客户端模式使用。 |
| `sync_user_data` | `syncUserData` | Implemented | 同步用户数据。 | — |
| `add_user_data` | `addUserData` | Implemented | 添加用户数据条目。 | Rust 使用 `Cow<'_, str>`。 |
| `remove_user_data` | `removeUserData` | Implemented | 删除用户数据。 | — |
| `get_user_data` | `getUserData` | Implemented | 获取用户数据。 | — |
| `get_user_data_id` | `getUserDataId` | Implemented | 查询用户数据 ID。 | — |
| `get_num_user_data` | `getNumUserData` | Implemented | 查询用户数据条数。 | — |
| `get_user_data_info` | `getUserDataInfo` | Implemented | 列出数据详情。 | — |
| `get_base_position_and_orientation` | `getBasePositionAndOrientation` | Implemented | 读取 base 坐标与姿态。 | Rust 返回 `Isometry3`. |
| `get_base_velocity` | `getBaseVelocity` | Implemented | 获取 base 线/角速度。 | — |
| `reset_base_position_and_orientation` | `resetBasePositionAndOrientation` | Implemented | 设置 base pose。 | — |
| `reset_base_velocity` | `resetBaseVelocity` | Implemented | 设置 base 速度。 | — |
| `get_aabb` | `getAABB` | Implemented | 查询 AABB 包围盒。 | Rust 返回 `[f64; 6]`。 |
| `get_num_joints` | `getNumJoints` | Implemented | 获取关节数。 | — |
| `get_joint_info` | `getJointInfo` | Implemented | 获取关节元数据。 | Rust 结构体封装字段。 |
| `get_joint_state` | `getJointState` | Implemented | 查询单关节状态。 | — |
| `get_joint_states` | `getJointStates` | Implemented | 批量查询关节状态。 | — |
| `get_joint_state_multi_dof` | `getJointStateMultiDof` | Implemented | 多自由度关节状态。 | — |
| `get_joint_states_multi_dof` | `getJointStatesMultiDof` | Implemented | 批量多自由度状态。 | — |
| `reset_joint_state` | `resetJointState` | Implemented | 设定单关节初始状态。 | — |
| `reset_joint_state_multi_dof` | `resetJointStateMultiDof` | Implemented | 多自由度单关节重置。 | — |
| `reset_joint_states_multi_dof` | `resetJointStatesMultiDof` | Implemented | 多自由度批量重置。 | — |
| `unsupported_change_scaling` | `changeScaling` | Optional | PyBullet 遗留缩放 API。 | Rust 标记为 `unsupported`。 |

## Dynamics & Control

| Rust API 名称 | PyBullet API 名称 | 状态 | 说明 | 其他 |
| --- | --- | --- | --- | --- |
| `change_dynamics` | `changeDynamics` | Implemented | 修改刚体动力学属性。 | Rust 与 `set_physics_engine_parameter` 不同；前者单刚体参数更新。 |
| `get_dynamics_info` | `getDynamicsInfo` | Implemented | 查询刚体动力学信息。 | — |
| `set_joint_motor_control` | `setJointMotorControl` | Optional | 旧版单关节控制。 | 建议改用 `_2`。 |
| `set_joint_motor_control2` | `setJointMotorControl2` | Implemented | 主控制 API。 | Rust 使用 `JointMotorControl2Options`。 |
| `set_joint_motor_control_multi_dof` | `setJointMotorControlMultiDof` | Implemented | 多自由度单关节控制。 | — |
| `set_joint_motor_control_multi_dof_array` | `setJointMotorControlMultiDofArray` | Implemented | 多自由度批量控制。 | — |
| `set_joint_motor_control_array` | `setJointMotorControlArray` | Implemented | 多关节批量控制。 | — |
| `apply_external_force` | `applyExternalForce` | Implemented | 对 body/链接施加外力。 | Rust 接受世界/局部 frame。 |
| `apply_external_torque` | `applyExternalTorque` | Implemented | 施加外部力矩。 | — |
| `calculate_inverse_dynamics` | `calculateInverseDynamics` | Implemented | 逆动力学求解。 | 返回 `Vec<f64>`。 |
| `calculate_jacobian` | `calculateJacobian` | Implemented | 计算雅可比矩阵。 | Rust 返回 `Jacobian` 结构。 |
| `calculate_mass_matrix` | `calculateMassMatrix` | Implemented | 计算质量矩阵。 | — |
| `calculate_inverse_kinematics` | `calculateInverseKinematics` | Implemented | 逆解单目标。 | Rust 支持约束/权重。 |
| `calculate_inverse_kinematics2` | `calculateInverseKinematics2` | Implemented | 多目标逆解。 | — |

## Collision Queries & Contact Data

| Rust API 名称 | PyBullet API 名称 | 状态 | 说明 | 其他 |
| --- | --- | --- | --- | --- |
| `get_contact_points` | `getContactPoints` | Implemented | 返回接触点列表。 | Rust 映射为结构体。 |
| `get_closest_points` | `getClosestPoints` | Implemented | 计算最近距离。 | — |
| `get_overlapping_objects` | `getOverlappingObjects` | Implemented | AABB 交叠查询。 | — |
| `set_collision_filter_pair` | `setCollisionFilterPair` | Implemented | 控制两 body 是否碰撞。 | — |
| `set_collision_filter_group_mask` | `setCollisionFilterGroupMask` | Implemented | 设置碰撞组/掩码。 | — |
| `ray_test` | `rayTest` | Implemented | 单根光线检测。 | — |
| `ray_test_batch` | `rayTestBatch` | Implemented | 批量光线检测。 | Rust 接受迭代器。 |

## Rendering, Debug & Visualization

| Rust API 名称 | PyBullet API 名称 | 状态 | 说明 | 其他 |
| --- | --- | --- | --- | --- |
| — | `renderImage` | Pending | 渲染图像到缓冲区。 | 需图像缓冲拷贝，尚未实现。 |
| — | `getCameraImage` | Pending | 获取摄像机图像。 | 计划集成。 |
| — | `computeViewMatrix` | Pending | 视图矩阵计算。 | 将以 `nalgebra` 提供。 |
| — | `computeViewMatrixFromYawPitchRoll` | Pending | 视图矩阵辅助。 | — |
| — | `computeProjectionMatrix` | Pending | 投影矩阵计算。 | — |
| — | `computeProjectionMatrixFOV` | Pending | 基于视场的投影矩阵。 | — |
| — | `addUserDebugLine` | Pending | 调试线段渲染。 | — |
| — | `addUserDebugPoints` | Pending | 调试点云渲染。 | — |
| — | `addUserDebugText` | Pending | 调试文本渲染。 | — |
| — | `addUserDebugParameter` | Pending | 调试 GUI 控制。 | — |
| — | `readUserDebugParameter` | Pending | 读取调试 GUI 值。 | — |
| — | `removeUserDebugItem` | Pending | 删除调试项。 | — |
| — | `removeAllUserDebugItems` | Pending | 清空调试项。 | — |
| — | `removeAllUserParameters` | Pending | 清空调试 GUI。 | — |
| — | `setDebugObjectColor` | Pending | 设定调试颜色。 | — |
| — | `getDebugVisualizerCamera` | Pending | 读取调试相机参数。 | — |
| — | `configureDebugVisualizer` | Pending | 配置调试显示器。 | — |
| — | `resetDebugVisualizerCamera` | Pending | 重置调试相机。 | — |
| — | `getVisualShapeData` | Pending | 查询视觉形状数据。 | — |
| — | `getCollisionShapeData` | Pending | 查询碰撞形状数据。 | — |

## Math & Transform Utilities

| Rust API 名称 | PyBullet API 名称 | 状态 | 说明 | 其他 |
| --- | --- | --- | --- | --- |
| — | `getQuaternionFromEuler` | Pending | 欧拉角转四元数。 | 计划提供 `nalgebra` 版本。 |
| — | `getEulerFromQuaternion` | Pending | 四元数转欧拉角。 | — |
| — | `multiplyTransforms` | Pending | 变换矩阵乘法。 | — |
| — | `invertTransform` | Pending | 变换逆。 | — |
| — | `getMatrixFromQuaternion` | Pending | 四元数转矩阵。 | — |
| — | `getQuaternionSlerp` | Pending | 球面插值。 | — |
| — | `getQuaternionFromAxisAngle` | Pending | 轴角转四元数。 | — |
| — | `getAxisAngleFromQuaternion` | Pending | 四元数转轴角。 | — |
| — | `getDifferenceQuaternion` | Pending | 求差四元数。 | — |
| — | `getAxisDifferenceQuaternion` | Pending | 轴差四元数。 | — |
| — | `calculateVelocityQuaternion` | Pending | 四元数速度计算。 | — |
| — | `rotateVector` | Pending | 旋转向量。 | — |

## VR, Input, Logging, Plugins & Misc

| Rust API 名称 | PyBullet API 名称 | 状态 | 说明 | 其他 |
| --- | --- | --- | --- | --- |
| — | `getVREvents` | Pending | VR 事件读取。 | 需启用 VR 编译。 |
| — | `setVRCameraState` | Pending | 设置 VR 相机。 | — |
| — | `getKeyboardEvents` | Pending | 键盘事件读取。 | — |
| — | `getMouseEvents` | Pending | 鼠标事件读取。 | — |
| — | `startStateLogging` | Pending | 开始状态日志。 | 依赖回放框架。 |
| — | `stopStateLogging` | Pending | 停止状态日志。 | — |
| — | `loadPlugin` | Pending | 加载插件。 | 需插件 ABI 支持。 |
| — | `unloadPlugin` | Pending | 卸载插件。 | — |
| — | `executePluginCommand` | Pending | 执行插件命令。 | — |
| — | `submitProfileTiming` | Pending | 提交性能分析标记。 | — |
| — | `setTimeOut` | Pending | 设置通信超时。 | Python 中默认值为秒。 |
| — | `getAPIVersion` | Pending | 返回 API 魔数。 | Python 返回 `SHARED_MEMORY_MAGIC_NUMBER`。 |

## 额外说明

1. **错误处理差异**：Rust 所有公开方法返回 `Result<T, BulletError>`，确保错误必须显式处理；Python 使用异常或返回 `None`。
2. **参数封装**：Rust 倾向使用结构体（如 `UrdfOptions`、`JointMotorControl2Options`）以规避 Python 中位置/关键字混搭的错误。
3. **可选功能**：软体、调试可视化、VR/输入等功能依赖 Bullet 构建宏（如 `BT_ENABLE_ENET`、`BT_ENABLE_VHACD`）。Rust 将依据需求和平台支持逐步开放。
4. **未来工作**：优先补充 Debug & Visualization 以及 Math Helpers，以提升可视化调试能力，并对齐 PyBullet 的便捷数学接口。
