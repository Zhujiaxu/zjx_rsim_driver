#!/usr/bin/env python3
"""
06_emplanner_rsim_driver — 插件全局路径获取验证

============================================================================
一、demo 脚本职责

  本脚本做两件事:
  1. prepare() — 将静态 scene.xosc 改造为 runtime.xosc (注入插件属性)
  2. main()   — 启动两个子进程 + Python rsim 客户端, 跑短仿真验证

============================================================================
二、静态 XOSC vs 运行时 XOSC 的差异

  静态 scene.xosc (来自 resouce/xosc/scene.xosc) 只包含场景描述,
  prepare() 在其中注入以下内容, 生成 runtime.xosc:

  [A] 路径替换 — 将相对路径替换为绝对路径:
      - LogicFile filepath            → 地图 .xodr 的绝对路径
      - CataLogs/Vehicles/ 等目录     → 资源目录的绝对路径

  [B] ObjectController (注入到 ego <ScenarioObject> 内) ← 核心差异
      ┌──────────────────────────────────────────────────────┐
      │ <ObjectController>                                   │
      │   <Controller name="RSimDriver">                     │
      │     <Properties>                                     │
      │       esminiController=PluginController              │
      │       pluginCapability=RSimDriver                    │
      │       pluginSoPath=librsim_driver.so                 │
      │       pluginManifest=plugin.yaml                     │
      │       xodrPath=...      routeXoscPath=...            │
      │       routeCsvPath=...  setSpeed=...  entityName=... │
      │     </Properties>                                    │
      │   </Controller>                                      │
      │ </ObjectController>                                  │
      └──────────────────────────────────────────────────────┘

  [C] TeleportAction + ActivateControllerAction (注入到 ego Init/Private)
      - TeleportAction: 将 ego 传送到 FollowTrajectoryAction 的起点
      - ActivateControllerAction: 激活插件控制器 (lateral + longitudinal)

============================================================================
三、进程通信关系 (树状图)

  run_demo.py (Python 主进程)
  │
  ├─[spawn]── scene_runner (C++ 仿真引擎)
  │             │
  │             ├── 监听 TCP :9080         ← SceneRunnerClient 连接端口
  │             │     ▲ rsim RPC 协议
  │             │     │  · load_xodr()     — 加载场景, 触发各插件 Init()
  │             │     │  · get_state()     — 查询仿真状态 (PREPARING/READY/...)
  │             │     │  · tick(dt)        — 推进仿真步进
  │             │     │  · get_actor_transform/velocity() — 读取 actor 状态
  │             │     │  · finish()        — 结束仿真
  │             │
  │             └── 监听 TCP :9081         ← 3D 同步端口 (本 demo 不使用)
  │
  ├─[spawn]── plugin_loader_cpp (插件加载器)
  │             │
  │             ├── 连接 TCP :9080 ──→ scene_runner
  │             │     · 注册插件, 接收 actor 分配
  │             │
  │             └── dlopen("librsim_driver.so")
  │                   └── RSimDriverPlugin::Init()
  │                         └── global_path 静态库 → global_path_world_points.csv
  │
  └─[主线程]── rsim.SceneRunnerClient (Python RPC 客户端)
                 │
                 └── TCP :9080 ──→ scene_runner
                      · load_xodr / get_state / tick / finish

  通信方向总结:
    Python ──RPC──→ scene_runner          (控制仿真)
    Python ←──RPC── scene_runner          (查询状态)
    plugin_loader_cpp ──内部协议──→ scene_runner  (注册/数据交换)
    plugin_loader_cpp ──dlopen──→ librsim_driver.so  (加载插件)
    scene_runner ──触发──→ plugin_loader_cpp ──→ RSimDriverPlugin::Step()

============================================================================
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from xml.sax.saxutils import quoteattr

try:
    import rsim
except ImportError:
    sys.stderr.write("ERROR: cannot 'import rsim'. Run with: conda run -n qc_work python3 run_demo.py\n")
    sys.exit(2)


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
BUILD_DIR = os.environ.get("RSIM_DRIVER_BUILD_DIR", "build")
SO_PATH = REPO_ROOT / BUILD_DIR / "source" / "plugins" / "rsim_driver" / "librsim_driver.so"
MANIFEST_PATH = REPO_ROOT / "source" / "plugins" / "rsim_driver" / "plugin.yaml"
XODR_PATH = HERE / "resource" / "xodr" / "map.xodr"
SOURCE_XOSC = HERE / "resource" / "xosc" / "scene.xosc"
CATALOG_ROOT = HERE / "resource" / "xosc" / "CataLogs"
OUTPUT_DIR = HERE / "output"
CSV_DIR=OUTPUT_DIR / "csv"
RUNTIME_XOSC = OUTPUT_DIR / CSV_DIR/"scene.runtime.xosc"
GLOBAL_PATH_CSV = OUTPUT_DIR / CSV_DIR/"global_path_world_points.csv"
REFERENCE_LINE_CSV = OUTPUT_DIR / CSV_DIR/"reference_line_motion.csv"
OBSTACLE_CSV = OUTPUT_DIR / CSV_DIR/"obstacles.csv"
CSV_PATH = OUTPUT_DIR / CSV_DIR/"scene.csv"
LOG_DIR = OUTPUT_DIR / "logs"
PACKAGE_DIR = OUTPUT_DIR / "package"


PORT = 9080
STREAMING_PORT = 9081
STEP = 0.05
SIM_DURATION = 20.0  # 仿真 — 验证插件沿参考线推动车辆并记录参考线

# ---- RSimDriver 插件配置 -------------------------------------------------
PLUGIN_PROPS = {
    # 插件加载元信息 (必须)
    "esminiController":  "PluginController",
    "pluginCapability":  "RSimDriver",
    "pluginPath":        str(SO_PATH),
    "pluginManifest":    str(MANIFEST_PATH),

    # 基本参数
    "xodrPath":          str(XODR_PATH),
    "routeXoscPath":     str(RUNTIME_XOSC),
    "routeCsvPath":      str(GLOBAL_PATH_CSV),     # 插件 Init 时写入全局路径 CSV
    "referenceLineCsvPath": str(REFERENCE_LINE_CSV),
    "obstacleCsvPath":   str(OBSTACLE_CSV),
    "setSpeed":          "8",
    "pointStep":         "2",
    "entityName":        "ego",
}
# -------------------------------------------------------------------------


def fail(msg):
    print(f"FAIL: {msg}", file=sys.stderr)
    sys.exit(1)


def resolve_bin(name):
    cs = []
    rp = os.environ.get("RSIM_PATH")
    if rp:
        cs.append(Path(rp) / name)
    cs.append(Path.home() / "proj" / "qc_intern" / "work" / "rsim-package" / "rsim" / "linux" / "bin" / name)
    for c in cs:
        if c.is_file() and os.access(c, os.X_OK):
            return c
    fail(f"{name} not found.")
    return Path()


def prepare():
    """从静态 scene.xosc 生成 runtime.xosc。

    对原始 XOSC 做三类修改:
      [A] 路径替换 — 相对路径 → 绝对路径
      [B] ObjectController 注入 — 告诉 scene_runner 加载哪个插件及参数
      [C] TeleportAction + ActivateControllerAction 注入 — 设置 ego 起点 + 激活控制器
    """
    if not SO_PATH.is_file():
        fail(f".so not built: {SO_PATH}")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    CSV_DIR.mkdir(parents=True, exist_ok=True)  

    text = SOURCE_XOSC.read_text(encoding="utf-8")

    # ====================================================================
    # [A] 路径替换: 将静态 XOSC 中的相对路径替换为绝对路径
    #     这些行覆盖了原始 XOSC 中对应的相对路径引用
    # ====================================================================
    text = text.replace(
        '<LogicFile filepath="../xodr/map.xodr"/>',
        f'<LogicFile filepath={quoteattr(str(XODR_PATH))}/>',
    )
    text = text.replace(
        '<Directory path="CataLogs/Vehicles/"/>',
        f'<Directory path={quoteattr(str(CATALOG_ROOT / "Vehicles"))}/>',
    )
    text = text.replace(
        '<Directory path="CataLogs/Pedestrians/"/>',
        f'<Directory path={quoteattr(str(CATALOG_ROOT / "Pedestrians"))}/>',
    )
    text = text.replace(
        '<Directory path="CataLogs/MiscObjects/"/>',
        f'<Directory path={quoteattr(str(CATALOG_ROOT / "MiscObjects"))}/>',
    )
    text = text.replace(
        '<Directory path="CataLogs/Controllers/"/>',
        f'<Directory path={quoteattr(str(CATALOG_ROOT / "Controllers"))}/>',
    )

    # ====================================================================
    # [B] ObjectController 注入
    #     在 ego 的 <CatalogReference/> 之后插入 <ObjectController> 块。
    #     原始静态 XOSC 没有这段 —— scene_runner 通过它知道要加载哪个 .so
    #     插件及配置参数。
    # ====================================================================
    props_lines = []
    for key, value in PLUGIN_PROPS.items():
        props_lines.append(
            f'                        <Property name={quoteattr(key)} value={quoteattr(value)}/>'
        )

    controller_xml = (
        '            <ObjectController>\n'
        '                <Controller name="RSimDriver">\n'
        '                    <Properties>\n'
        + '\n'.join(props_lines) + '\n'
        '                    </Properties>\n'
        '                </Controller>\n'
        '            </ObjectController>'
    )

    ego_start = text.find('<ScenarioObject name="ego">')
    if ego_start == -1:
        fail("cannot find ego ScenarioObject")
    cat_end = text.find('/>', text.find('<CatalogReference', ego_start))
    if cat_end == -1:
        fail("cannot find ego CatalogReference")
    insert_pos = cat_end + 2
    # ↓ 在此处注入 [B]
    text = text[:insert_pos] + '\n' + controller_xml + text[insert_pos:]

    # ====================================================================
    # [C] TeleportAction + ActivateControllerAction 注入
    #     在 ego 的 Init/Private 块最前面插入两个 action:
    #       TeleportAction  — 传送 ego 到 FollowTrajectoryAction 的起点
    #       ActivateControllerAction — 激活插件控制器 (必须, 否则插件不生效)
    #     提取 FollowTrajectory 第一个 RoadPosition 作为传送目标。
    # ====================================================================
    ego_private_start = text.find('<Private entityRef="ego">')
    if ego_private_start == -1:
        fail("cannot find ego Init <Private>")
    ego_private_end = text.find('</Private>', ego_private_start)
    if ego_private_end == -1:
        fail("cannot find ego Init </Private>")

    # 提取 FollowTrajectoryAction 的第一个 RoadPosition 作为 Teleport 目标
    first_road_pos = text.find('<RoadPosition', ego_private_start, ego_private_end)
    if first_road_pos == -1:
        fail("cannot find ego route first RoadPosition")
    first_road_pos_end = text.find('</RoadPosition>', first_road_pos, ego_private_end)
    if first_road_pos_end == -1:
        fail("cannot find ego route first </RoadPosition>")
    first_road_pos_end += len('</RoadPosition>')
    first_road_position = text[first_road_pos:first_road_pos_end]
    first_road_position = '\n'.join(
        '                                ' + line.strip()
        for line in first_road_position.splitlines()
        if line.strip()
    )

    first_action = text.find('<PrivateAction>', ego_private_start)
    if first_action == -1:
        fail("cannot find ego Init <PrivateAction>")
    teleport_action = (
        '                    <PrivateAction>\n'
        '                        <TeleportAction>\n'
        '                            <Position>\n'
        f'{first_road_position}\n'
        '                            </Position>\n'
        '                        </TeleportAction>\n'
        '                    </PrivateAction>\n'
    )
    activate_action = (
        '                    <PrivateAction>\n'
        '                        <ActivateControllerAction lateral="true" longitudinal="true"/>\n'
        '                    </PrivateAction>\n'
    )
    # ↓ 在此处注入 [C]
    text = text[:first_action] + teleport_action + activate_action + text[first_action:]

    RUNTIME_XOSC.write_text(text, encoding="utf-8")

    # ---- 准备 package 目录 (插件 .so + plugin.yaml) ----
    if PACKAGE_DIR.exists():
        shutil.rmtree(PACKAGE_DIR)
    PACKAGE_DIR.mkdir(parents=True)
    (PACKAGE_DIR / SO_PATH.name).symlink_to(SO_PATH)
    shutil.copy2(MANIFEST_PATH, PACKAGE_DIR / "plugin.yaml")

    print(f"[FILE_Prepare] runtime.xosc written ({RUNTIME_XOSC.stat().st_size} bytes)")
    print(f"[FILE_Prepare] package dir ready: {PACKAGE_DIR}")


def spawn(cmd, label):
    print(f"[{label}]", " ".join(cmd))
    if os.environ.get("RSIM_DEBUG_CAPTURE_CHILD_OUTPUT") == "1":
        stdout, stderr = subprocess.PIPE, subprocess.PIPE
    else:
        stdout, stderr = None, None
    return subprocess.Popen(cmd, stdout=stdout, stderr=stderr, cwd=HERE, preexec_fn=os.setsid)


def kill_pg(p):
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def wait_for_debug_attach():
    if os.environ.get("RSIM_DEBUG_WAIT_FOR_ATTACH") != "1":
        return
    input("[debug] Attach VSCode/gdb to plugin_loader_cpp, then press Enter to continue...")


def main():
    sr_bin = resolve_bin("scene_runner")
    ld_bin = resolve_bin("plugin_loader_cpp")
    prepare()

    if CSV_PATH.exists():
        CSV_PATH.unlink()
    if GLOBAL_PATH_CSV.exists():
        GLOBAL_PATH_CSV.unlink()
    if REFERENCE_LINE_CSV.exists():
        REFERENCE_LINE_CSV.unlink()
    if OBSTACLE_CSV.exists():
        OBSTACLE_CSV.unlink()

    sr = spawn(
        [str(sr_bin), "--scene_runner_port", str(PORT),
         "--scene_runner_streaming_port", str(STREAMING_PORT),
         "--csv_logger", str(CSV_PATH), "--logfile_dir", str(LOG_DIR)],
        "scene_runner",
    )
    for _ in range(60):
        if sr.poll() is not None:
            break
        time.sleep(0.1)
    if sr.poll() is not None:
        _, err = sr.communicate(timeout=2)
        fail(f"scene_runner died early\nstderr: {(err or b'').decode(errors='replace')[:400]}")

    ld = spawn(
        [str(ld_bin), "--host", "127.0.0.1", "--port", str(PORT), "--package", str(PACKAGE_DIR)],
        "plugin_loader_cpp",
    )

    try:
        client = rsim.SceneRunnerClient("127.0.0.1", PORT)
        client.set_need_sync_3d(False)
        print(f"[client] 客户端连接场景器 connected to 127.0.0.1:{PORT}; loading xosc {RUNTIME_XOSC}")
        client.load_xodr(str(RUNTIME_XOSC))
        print("[client] 客户端调用场景器的 RUNTIME_XOS 加载服务 load_xodr returned; waiting for READY (max 15s)...")

        last_state = None
        ready = False
        for i in range(150):
            st = client.get_state()
            if st != last_state:
                print(f"[client] 客户端加载状态 state -> {st} (i={i}, t={i * 0.1:.1f}s)")
                if st == rsim.ModuleState.READY:
                    last_state = st
                    ready = True
                    break
                last_state = st
            if st == rsim.ModuleState.READY:
                ready = True
                print(f"[client] 客户端加载状态 state -> {st} (i={i}, t={i * 0.1:.1f}s)")
                break
            if i % 10 == 9:
                sr_alive = sr.poll() is None
                ld_alive = ld.poll() is None
                print(f"[client] still waiting i={i} state={st} "
                      f"scene_runner_alive={sr_alive} plugin_loader_alive={ld_alive}")
                if not sr_alive or not ld_alive:
                    fail(f"subprocess died: sr_rc={sr.poll()} ld_rc={ld.poll()}")
            time.sleep(0.1)
        if not ready:
            fail(f"客户端与场景器断连 READY timeout (last state={last_state})")

        wait_for_debug_attach()

        erect = client.get_actor_transform("ego")
        print(f"[client] 场景器加载完成，客户端立即获得主车位置 ego initial: x={erect.location.x:.2f} y={erect.location.y:.2f}")
        print(f"[client] 仿真周期与步长 ticking for {SIM_DURATION:.1f}s at dt={STEP:.2f}s")

        for step in range(int(SIM_DURATION / STEP)):
            client.tick(STEP)
            if step % 40 == 39:
                transform = client.get_actor_transform("ego")
                print(f"[client] 仿真中 step={step + 1:>4} 位置信息  x={transform.location.x:8.2f} " 
                      f"y={transform.location.y:8.2f}")
            if client.get_state() == rsim.ModuleState.FINISHED:
                print(f"[client] 仿真完成 FINISHED at step={step + 1}")
                break

        try:
            client.finish()
        except Exception:
            pass

        print("PASS")
    finally:
        kill_pg(ld)
        kill_pg(sr)


if __name__ == "__main__":
    main()
