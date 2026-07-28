# rsim-driver 任务规则

- 当使用plan模式时，必须输出 markdown 格式的计划书到/home/ubzjx/proj/qc_intern/work/rsim_driver/Dission_md，包含 Summary、Implementation Changes、Test Plan 三个部分，审批通过，才能执行。
- 当使用plan模式输出计划，如果一次修改的计划过大，分成子计划，然后顺序请求审批，顺序执行每个子计划
- 当使用规划模式编写代码之前，查看有无代码更改却未提交，如有，提醒我，并告知commit的message，即简短的标题；如无，编写代码
- Fail Fast / Errors Never Pass Silently：不要在代码里藏兜底逻辑来吞掉错误、隐藏问题。出了问题就应该让它爆出来，否则你永远找不到真实问题。
- Fix the Cause, Not the Symptom / Don't Paper Over Bugs：当一个问题出现时，不要用各种 small fix、针对性补丁来掩盖它。必须定位真实根因，彻底修复。在 bug 上糊纸只会让系统积累你不知道的危险暗病。
- 本仓库是 RSim 驾驶员模型 plugin 工程, 主要代码为 C++17 + CMake; 改动应优先遵循现有 `source/lib` 与 `source/plugins` 的模块边界。
- 不要修改或提交 `third_party` 软链及外部 worktree 内容; 需要 SDK 时默认使用 `third_party/RSimAPI/source`, 或通过 `RSIM_PLUGIN_SDK_INCLUDE` 指定。
- 构建命令优先使用:
  `cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release`
  `cmake --build build`
- 修改 planner、helper 或 plugin 行为后, 至少运行相关 smoke test 或示例; 基础示例为 `conda run -n engine python3 examples/01_default_driver_basic/run_demo.py`。
- 新增代码保持小范围、可读、可测试; 避免引入与当前阶段无关的大型抽象或重构。
- 输出文件、日志、编译产物应留在 `build/` 或 example 的 `output/` 下, 不作为源码变更提交。


