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
    sys.stderr.write("ERROR: cannot 'import rsim'. Run with: conda run -n engine python3 run_demo.py\n")
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
RUNTIME_XOSC = OUTPUT_DIR / "scene.runtime.xosc"
CSV_PATH = OUTPUT_DIR / "scene.csv"
LOG_DIR = OUTPUT_DIR / "logs"
PACKAGE_DIR = OUTPUT_DIR / "package"

PORT = 9080
STREAMING_PORT = 9081
STEP = 0.05
SIM_DURATION = 21.0

# ---- RSimDriver 插件配置 -------------------------------------------------
# 这些值会被注入到 runtime.xosc 中 ego 的 <ObjectController> 里,
# scene_runner 将它们原样传给 RSimDriverPlugin::Init(properties, ...)。
# 修改这里即可切换规划器参数, 无需手动编辑 xosc。
#
# 重要: esminiController=PluginController 是 RSim 插件系统的约定,
# pluginSoPath / pluginManifest / pluginCapability 三个属性告诉 scene_runner
# 如何加载和匹配插件。
PLUGIN_PROPS = {
    # ---- 插件加载元信息 (必须, 不要改动) ----
    "esminiController":  "PluginController",       # RSim 插件系统约定值
    "pluginCapability":  "RSimDriver",             # 必须与 plugin.yaml 中 capability name 一致
    # pluginSoPath / pluginManifest 的值在 prepare() 中动态注入

    # ---- 规划器参数 ----
    "xodrPath":          str(XODR_PATH),
    "routeXoscPath":     str(RUNTIME_XOSC),        # 优先从 runtime xosc 的 ego FollowTrajectory 取全局路径
    "plannerType":       "em",                     # "sampling" 或 "em"
    "setSpeed":          "8",                      # 期望巡航速度 m/s
    "maxAccel":          "3.0",                    # 最大加速度 m/s²
    "maxDecel":          "4.0",                    # 最大减速度 m/s²
    "planningHorizonSec": "5.0",                   # 规划时域 秒
    "enableDebugLog":    "true",                   # 打印调试日志到 stderr
    "sceneRunnerHost":   "127.0.0.1",              # GetRoute RPC 主机
    "sceneRunnerPort":   str(PORT),                # GetRoute RPC 端口 (与 scene_runner 一致)
    "entityName":        "ego",                    # GetRoute 实体名
}
# -------------------------------------------------------------------------


def fail(msg):
    print(f"FAIL: {msg}", file=sys.stderr); sys.exit(1)


def resolve_bin(name):
    cs = []
    rp = os.environ.get("RSIM_PATH")
    if rp: cs.append(Path(rp) / name)
    cs.append(Path.home() / "work" / "rsim-package" / "rsim" / "linux" / "bin" / name)
    for c in cs:
        if c.is_file() and os.access(c, os.X_OK): return c
    fail(f"{name} not found.")
    return Path()


def prepare():
    """从静态 scene.xosc 生成 runtime.xosc, 并准备插件 package 目录。

    用字符串替换在 ego 的 <CatalogReference> 之后直接注入 <ObjectController>,
    格式与示例 02 的 working xosc 完全一致, 避免 XML 库的 whitespace 差异。
    """
    if not SO_PATH.is_file():
        fail(f".so not built: {SO_PATH}")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    text = SOURCE_XOSC.read_text(encoding="utf-8")
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

    # ---- 构建 <ObjectController> 的 XML 字符串 ----
    # 格式严格匹配 RSim 插件系统的约定 (参考 examples/02_rsim_driver_cruise/cruise.xosc)
    props_lines = []
    all_props = dict(PLUGIN_PROPS)
    all_props["pluginSoPath"] = str(SO_PATH)
    all_props["pluginManifest"] = str(MANIFEST_PATH)
    for key, value in all_props.items():
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

    # ---- 在 ego 的 <CatalogReference .../> 之后注入 ----
    # 找到 ego 的 CatalogReference 结束标签位置
    ego_start = text.find('<ScenarioObject name="ego">')
    if ego_start == -1:
        fail("cannot find ego ScenarioObject")
    cat_end = text.find('/>', text.find('<CatalogReference', ego_start))
    if cat_end == -1:
        fail("cannot find ego CatalogReference")
    insert_pos = cat_end + 2  # 跳过 />

    text = text[:insert_pos] + '\n' + controller_xml + text[insert_pos:]

    # ---- 在 ego 的 Init Private 块中注入 <ActivateControllerAction> ----
    # esmini 需要 ActivateControllerAction 来激活已分配的控制器,
    # 仅有 <ObjectController> 不够 (参考 examples/02/cruise.xosc)
    ego_private_start = text.find('<Private entityRef="ego">')
    if ego_private_start == -1:
        fail("cannot find ego Init <Private>")
    # 在 <Private> 块内第一个 action 之前插入
    first_action = text.find('<PrivateAction>', ego_private_start)
    if first_action == -1:
        fail("cannot find ego Init <PrivateAction>")
    activate_action = ('                    <PrivateAction>\n'
                       '                        <ActivateControllerAction lateral="true" longitudinal="true"/>\n'
                       '                    </PrivateAction>\n')
    text = text[:first_action] + activate_action + text[first_action:]

    RUNTIME_XOSC.write_text(text, encoding="utf-8")

    # ---- 准备 package 目录 ----
    if PACKAGE_DIR.exists():
        shutil.rmtree(PACKAGE_DIR)
    PACKAGE_DIR.mkdir(parents=True)
    (PACKAGE_DIR / SO_PATH.name).symlink_to(SO_PATH)
    shutil.copy2(MANIFEST_PATH, PACKAGE_DIR / "plugin.yaml")

    print(f"[prepare] runtime.xosc written ({RUNTIME_XOSC.stat().st_size} bytes)")
    print(f"[prepare] package dir ready: {PACKAGE_DIR}")


def spawn(cmd, label):
    print(f"[{label}]", " ".join(cmd))
    if os.environ.get("RSIM_DEBUG_CAPTURE_CHILD_OUTPUT") == "1":
        stdout, stderr = subprocess.PIPE, subprocess.PIPE
    else:
        stdout, stderr = None, None
    return subprocess.Popen(cmd, stdout=stdout, stderr=stderr,
                            cwd=HERE, preexec_fn=os.setsid)


def kill_pg(p):
    if p is None or p.poll() is not None: return
    try: os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError: return
    try: p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError: pass


def wait_for_debug_attach():
    if os.environ.get("RSIM_DEBUG_WAIT_FOR_ATTACH") != "1":
        return
    input("[debug] Attach VSCode/gdb to plugin_loader_cpp, then press Enter to continue...")


def main():
    sr_bin = resolve_bin("scene_runner")
    ld_bin = resolve_bin("plugin_loader_cpp")
    prepare()
    if CSV_PATH.exists(): CSV_PATH.unlink()
    sr = spawn([str(sr_bin), "--scene_runner_port", str(PORT),
                "--scene_runner_streaming_port", str(STREAMING_PORT),
                "--csv_logger", str(CSV_PATH), "--logfile_dir", str(LOG_DIR)], "scene_runner")
    for _ in range(60):
        if sr.poll() is not None: break
        time.sleep(0.1)
    if sr.poll() is not None:
        out, err = sr.communicate(timeout=2)
        stderr = "" if err is None else err.decode(errors="replace")[:400]
        stdout = "" if out is None else out.decode(errors="replace")[:400]
        fail(f"scene_runner died early\nstdout: {stdout}\nstderr: {stderr}")
    ld = spawn([str(ld_bin), "--host", "127.0.0.1", "--port", str(PORT), "--package", str(PACKAGE_DIR)], "plugin_loader_cpp")

    try:
        client = rsim.SceneRunnerClient("127.0.0.1", PORT)
        client.set_need_sync_3d(False)
        print(f"[client] connected to 127.0.0.1:{PORT}; loading xosc {RUNTIME_XOSC}")
        client.load_xodr(str(RUNTIME_XOSC))
        print("[client] load_xodr returned; waiting for READY (max 15s)...")
        last_state = None
        ready = False
        for i in range(150):
            st = client.get_state()
            if st != last_state:
                print(f"[client] state -> {st} (i={i}, t={i*0.1:.1f}s)")
                last_state = st
            if st == rsim.ModuleState.READY:
                ready = True
                break
            if i % 10 == 9:
                sr_alive = sr.poll() is None
                ld_alive = ld.poll() is None
                print(f"[client] still waiting i={i} state={st} "
                      f"scene_runner_alive={sr_alive}(rc={sr.poll()}) "
                      f"plugin_loader_alive={ld_alive}(rc={ld.poll()})")
                if not sr_alive or not ld_alive:
                    fail(f"subprocess died during READY wait: "
                         f"sr_rc={sr.poll()} ld_rc={ld.poll()}")
            time.sleep(0.1)
        if not ready:
            fail(f"READY timeout (last state={last_state}, "
                 f"sr_alive={sr.poll() is None}, ld_alive={ld.poll() is None})")

        wait_for_debug_attach()

        ego_name = PLUGIN_PROPS["entityName"]
        try:
            names = list(client.get_ego_names())
            if ego_name not in names and names:
                ego_name = names[0]
        except Exception:
            pass

        print(f"[client] route source hint: routeXoscPath={RUNTIME_XOSC}")
        print(f"[client] ticking simulation for {SIM_DURATION:.1f}s at dt={STEP:.2f}s (ego={ego_name})")
        traj = []
        for step in range(int(SIM_DURATION / STEP)):
            client.tick(STEP)
            transform = client.get_actor_transform(ego_name)
            velocity = client.get_actor_velocity(ego_name)
            speed = (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5
            traj.append({
                "x": transform.location.x,
                "y": transform.location.y,
                "speed": speed,
            })
            if step % 40 == 39:
                print(f"  step={step + 1:>4} "
                      f"x={transform.location.x:8.2f} "
                      f"y={transform.location.y:8.2f} "
                      f"speed={speed:5.2f}")
            if client.get_state() == rsim.ModuleState.FINISHED:
                print(f"[client] FINISHED at step={step + 1}")
                break

        try:
            client.finish()
        except Exception:
            pass

        if len(traj) < 2:
            fail("no ego trajectory samples recorded after READY")

        first, last = traj[0], traj[-1]
        dx = last["x"] - first["x"]
        dy = last["y"] - first["y"]
        dist = (dx ** 2 + dy ** 2) ** 0.5
        max_speed = max(p["speed"] for p in traj)
        print(f"[client] ego summary: samples={len(traj)} "
              f"pos=({first['x']:.2f},{first['y']:.2f})->({last['x']:.2f},{last['y']:.2f}) "
              f"dist={dist:.2f}m max_speed={max_speed:.2f}m/s last_speed={last['speed']:.2f}m/s")
        if dist < 1.0 and max_speed < 0.5:
            fail("ego did not move after plugin activation")
        print("PASS")
    finally:
        kill_pg(ld); kill_pg(sr)


if __name__ == "__main__":
    main()
