# rsim-driver 任务规则

- 规划阶段，必须输出 markdown 格式的计划书，包含 Summary、Implementation Changes、Test Plan 三个部分。
- 本仓库是 RSim 驾驶员模型 plugin 工程, 主要代码为 C++17 + CMake; 改动应优先遵循现有 `source/lib` 与 `source/plugins` 的模块边界。
- 不要修改或提交 `third_party` 软链及外部 worktree 内容; 需要 SDK 时默认使用 `third_party/RSimAPI/source`, 或通过 `RSIM_PLUGIN_SDK_INCLUDE` 指定。
- 构建命令优先使用:
  `cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release`
  `cmake --build build`
- 修改 planner、helper 或 plugin 行为后, 至少运行相关 smoke test 或示例; 基础示例为 `conda run -n engine python3 examples/01_default_driver_basic/run_demo.py`。
- 新增代码保持小范围、可读、可测试; 避免引入与当前阶段无关的大型抽象或重构。
- 输出文件、日志、编译产物应留在 `build/` 或 example 的 `output/` 下, 不作为源码变更提交。


