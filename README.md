# rsim-driver

RSim 驾驶员模型 plugin 仓库 (C++)。当前提供:

- `default_driver` -- 最小恒速 plugin (P1 阶段)。后续会接入 IDM/Frenet/MOBIL/红绿灯/路由等子模块,演化为完整 default-driver。

> Internal use, license TBD.

## 工程结构

```
rsim-driver/
├── CMakeLists.txt
├── third_party                        -> ~/work/third_party-feature-plugin (软链)
├── source/
│   └── plugins/
│       └── default_driver/
│           ├── DefaultDriverController.cpp
│           ├── plugin.yaml
│           └── CMakeLists.txt
└── examples/
    └── 01_default_driver_basic/       (P1 阶段加入)
```

`third_party` 必须指向 **feature-plugin 分支的 third_party worktree**, 因为 `IPluginController` SDK 头 (`rsim/worldsim_plugin/PluginInterface.hpp`) 只在那条分支存在。仓库不直接追踪该软链, 各机器自行建立。

## 前置依赖

- `~/work/third_party-feature-plugin` 已存在 (feature-plugin 分支的 third_party worktree)
- `~/work/rsim-package` 已切到 `feature-driver` 分支 (其 `rsim/linux/bin/scene_runner` 与 `plugin_loader_cpp` 支持 plugin 子系统)

切换工作空间到 feature-driver 项目: 见 `$RSIM_WORKSPACE/docs/projects.md` 的 "feature-driver" 章节。

## 编译

```bash
ln -s ~/work/third_party-feature-plugin third_party    # 仅首次

cmake -G Ninja -S . -B Build -DCMAKE_BUILD_TYPE=Release
cmake -G Ninja -S . -B Build -DCMAKE_BUILD_TYPE=Debug
cmake --build Build
```

产出: `Build/source/plugins/default_driver/libdefault_driver.so`

## 跑 example (P1)

详见 `examples/01_default_driver_basic/README.md`。简要:

```bash
cd examples/01_default_driver_basic
python3 run_demo.py
```

`run_demo.py` 会:

1. 检查 `libdefault_driver.so` 已编译
2. 把 xosc 中 `__PLUGIN_SO__` 替换为 `.so` 的绝对路径
3. 调用 `~/work/rsim-package/rsim/linux/bin/scene_runner --osc <runtime.xosc>`
4. 读取输出 CSV, 断言 ego 位置随时间增长 (动起来即 PASS)

## 后续路线 (不在第一版范围)

- P2: 抽取 IDM / Frenet / MOBIL / Perception / TrafficRule / MapHelper 子模块
- P3: default_driver 接入子模块, 实现 IDM 跟车
- P4: MOBIL 换道
- P5: 红绿灯 + route following
- P6: 5 场景对齐 `WorldSim-feature-default-driver-route` 基线
