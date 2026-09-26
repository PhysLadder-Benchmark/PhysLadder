import argparse
import json
from pathlib import Path

from .backends.isaacsim import IsaacSimBackend
from .backends.mock import MockBackend, MockRobot, MockTask, NoopPolicy
from .config import load_config
from .paths import PROJECT_ROOT
from .policies.gpt import GPTPolicy
from .policies.model_config import load_gpt_settings
from .policies.responses import ModelError
from .policies.scripted import ScriptedPolicy
from .registry import TASKS, get_spec
from .robots.adapters import R5Adapter
from .robots.audit import audit_robot, require_robot_ready
from .robots.profile import load_robot_profile
from .runner import run_episode


def main(argv: list[str] | None = None) -> int:
    from .runtime import configure_windows_storage
    configure_windows_storage()
    parser = argparse.ArgumentParser(description="完整机器人任务框架；当前实际任务均为待实现模板")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="列出任务模板及真实实现状态")
    motion = sub.add_parser("check-r5", help="运行真实 R5 仿真关节/夹爪验证，保存录像与测量结果")
    motion.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs/checks/r5_motion.toml")
    motion.add_argument("--output", type=Path, default=Path("runs"))
    motion.add_argument("--gui", action="store_true", help="显示 Isaac Sim 窗口")
    cartesian = sub.add_parser("check-r5-cartesian", help="验证 R5 TCP、IK、实际到达误差与不可达目标拒绝")
    cartesian.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs/checks/r5_cartesian.toml")
    cartesian.add_argument("--output", type=Path, default=Path("runs"))
    cartesian.add_argument("--gui", action="store_true")
    interaction = sub.add_parser("check-r5-interaction", help="验证 R5 抓取、抬升、搬运、释放及物体稳定性")
    interaction.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs/checks/r5_interaction.toml")
    interaction.add_argument("--output", type=Path, default=Path("runs"))
    interaction.add_argument("--gui", action="store_true")
    gpt = sub.add_parser("check-r5-gpt", help="真实 GPT 图像输入、R5 仿真动作和反馈闭环校验")
    gpt.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs/checks/r5_gpt.toml")
    gpt.add_argument("--output", type=Path, default=Path("runs"))
    gpt.add_argument("--gui", action="store_true")
    cameras = sub.add_parser("check-r5-cameras", help="校验三路 RGB、R5 腕部跟随及临时相机参数")
    cameras.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs/checks/r5_cameras.toml")
    cameras.add_argument("--output", type=Path, default=Path("runs"))
    cameras.add_argument("--gui", action="store_true")
    api = sub.add_parser("check-gpt-api", help="一次图像/API/动作格式校验；不启动机器人")
    api.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs/checks/r5_gpt.toml")
    api.add_argument("--model-config", type=Path, default=PROJECT_ROOT / "configs/models/gpt.toml")
    api.add_argument("--output", type=Path, default=Path("runs"))
    validate = sub.add_parser("validate-robot", help="离线校验 R5 资产、关节映射和配置完整性，不启动仿真")
    validate.add_argument("--profile", type=Path, required=True)
    validate.add_argument("--require-ready", action="store_true", help="未确认运行参数也返回非零退出码")
    validate.add_argument("--output", type=Path, help="可选 JSON 校验报告路径")
    inspect = sub.add_parser("inspect", help="校验并查看任务配置，不启动仿真")
    inspect.add_argument("--config", type=Path, required=True)
    dry = sub.add_parser("dry-run", help="纯占位流程自检，不运行物理、不产生任务成功率")
    dry.add_argument("--config", type=Path, required=True)
    dry.add_argument("--steps", type=int, default=3)
    dry.add_argument("--output", type=Path, default=Path("runs"))
    run = sub.add_parser("run", help="真实仿真入口；当前遇到未实现接口会明确报错")
    run.add_argument("--config", type=Path, required=True)
    run.add_argument("--policy", choices=["scripted", "gpt"], default="scripted")
    run.add_argument("--model-config", type=Path, default=PROJECT_ROOT / "configs/models/gpt.toml")
    run.add_argument("--output", type=Path, default=Path("runs"))
    args = parser.parse_args(argv)
    try:
        checks = {"check-r5": "joint", "check-r5-cartesian": "cartesian", "check-r5-interaction": "interaction",
                  "check-r5-gpt": "gpt", "check-r5-cameras": "cameras"}
        if args.command in checks:
            from .integrationtest.simulation_process import run_motion_process
            _, passed = run_motion_process(args.config, args.output.resolve(), gui=args.gui,
                                            check=checks[args.command])
            return 0 if passed else 2
        if args.command == "check-gpt-api":
            from .integrationtest.gpt_api_check import run_gpt_api_check
            directory, result = run_gpt_api_check(args.config, args.model_config, args.output.resolve())
            print(json.dumps({"output": str(directory), **result}, ensure_ascii=False, indent=2))
            return 0 if result["passed"] else 2
        if args.command == "list":
            for spec in TASKS.values():
                status = "implemented" if spec.implemented else "template_only"
                print(f"{spec.task_id} | {spec.name} | {status} | {spec.evidence}")
            return 0
        if args.command == "validate-robot":
            profile = load_robot_profile(args.profile)
            audit = audit_robot(profile)
            report = {"robot": profile.model, "profile": str(profile.source),
                      "available_arms": profile.available_arms, **audit.as_dict()}
            serialized = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
            if args.output is not None:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(serialized + "\n", encoding="utf-8")
            print(serialized)
            return 0 if (audit.ready if args.require_ready else audit.valid) else 2
        config = load_config(args.config)
        spec = get_spec(config)
        if args.command == "inspect":
            audit = audit_robot(config.robot)
            print(json.dumps({
                "task_id": spec.task_id, "name": spec.name,
                "configuration_valid": audit.valid,
                "task_implemented": spec.implemented,
                "robot": config.robot_model, "available_arms": config.robot.available_arms,
                "active_arms": config.arms, "cameras": config.cameras,
                "robot_audit": audit.as_dict(),
                "asset_path": str(config.asset_path) if config.asset_path else None,
                "asset_exists": config.asset_path.exists() if config.asset_path else False,
                "pending": ["现场布局与相机标定参数", "任务中的末端控制与路径验证",
                            "任务场景和复位", "任务脚本与 GPT 测评协议", "任务评分", "任务录像接入"],
                "evidence": spec.evidence,
            }, ensure_ascii=False, indent=2))
            return 0 if audit.valid else 2
        dry_run = args.command == "dry-run"
        if dry_run:
            backend, robot, task = MockBackend(), MockRobot(), MockTask()
            policy = NoopPolicy(config.control_dt_s)
        else:
            require_robot_ready(config.robot)
            backend = IsaacSimBackend()
            robot = R5Adapter()
            task = spec.factory()
            policy = (ScriptedPolicy() if args.policy == "scripted"
                      else GPTPolicy(config, load_gpt_settings(args.model_config)))
        directory, result = run_episode(
            config, backend, robot, task, policy, args.output.resolve(),
            dry_run=dry_run, step_limit=args.steps if dry_run else None,
        )
        print(json.dumps({"output": str(directory), **result}, ensure_ascii=False, indent=2))
        return 0 if result["status"] in {"finished", "dry_run_complete"} else 2
    except (ValueError, KeyError, OSError, TypeError, ModelError) as exc:
        parser.exit(2, f"Configuration/framework error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
