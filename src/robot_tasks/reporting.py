"""Render recorded evidence into a report in the same run directory.

This module never scores a task, starts a simulator, or calls a model. result.json
remains authoritative; reports can be regenerated for old or interrupted runs.
"""

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics
from urllib.parse import quote


def _cell(value) -> str:
    if value is None:
        return "未记录"
    if isinstance(value, bool):
        return "是" if value else "否"
    if isinstance(value, float):
        return f"{value:.6g}"
    if isinstance(value, (dict, list, tuple)):
        value = json.dumps(value, ensure_ascii=False, allow_nan=False)
    return str(value).replace("|", "\\|").replace("\r", "").replace("\n", "<br>")


def _table(headers, rows) -> list[str]:
    return ["| " + " | ".join(map(_cell, headers)) + " |",
            "| " + " | ".join("---" for _ in headers) + " |",
            *("| " + " | ".join(map(_cell, row)) + " |" for row in rows), ""]


def _number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _flatten(data: dict, prefix=""):
    for name, value in data.items():
        key = f"{prefix}.{name}" if prefix else name
        if isinstance(value, dict):
            yield from _flatten(value, key)
        else:
            yield key, value


def collect_run_evidence(directory: Path) -> dict:
    """Stream logs; distinguish measured action tracking from task-object error."""
    warnings = []

    def read(name):
        path = directory / name
        if not path.exists():
            return {}
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(value, dict):
                raise ValueError("expected an object")
            return value
        except (OSError, ValueError) as exc:
            warnings.append(f"{name} 无法读取：{exc}")
            return {}

    result, manifest, process = read("result.json"), read("manifest.json"), read("process.json")
    if not result:
        warnings.append("没有完整的 result.json，不能判定任务结果。")
    errors = defaultdict(lambda: defaultdict(list))
    rejections, calls, latencies = Counter(), 0, []
    provider = {}
    log = directory / "episode.jsonl"
    if log.exists():
        with log.open(encoding="utf-8") as stream:
            for number, line in enumerate(stream, 1):
                try:
                    event = json.loads(line)
                except ValueError:
                    warnings.append(f"episode.jsonl 第 {number} 行不完整，未纳入日志汇总。")
                    continue
                if event.get("event") == "policy_configuration":
                    metadata = event.get("configuration", {}).get("provider", {})
                    provider = {key: metadata.get(key) for key in ("model", "reasoning_effort")}
                elif event.get("event") == "model_exchange":
                    calls += 1
                    latency = event.get("exchange", {}).get("latency_s")
                    if _number(latency):
                        latencies.append(latency)
                elif event.get("event") == "action_rejected":
                    feedback = event.get("observation", {}).get("feedback", {})
                    rejections[feedback.get("metrics", {}).get("reason_code", "unspecified")] += 1
                elif event.get("event") == "transition":
                    feedback = event.get("observation", {}).get("feedback", {})
                    for arm, values in feedback.get("metrics", {}).get("tcp", {}).items():
                        for field in ("position_error_m", "orientation_error_rad"):
                            value = values.get(field)
                            if _number(value):
                                errors[arm][field].append(value)
    tracking = {arm: {field: {"samples": len(values), "mean": statistics.mean(values), "max": max(values)}
                      for field, values in fields.items()} for arm, fields in errors.items()}
    return {"result": result, "manifest": manifest, "process": process, "provider": provider,
            "tracking": tracking, "rejections": dict(rejections), "logged_model_exchanges": calls,
            "api_latency_total_s": sum(latencies) if latencies else None,
            "api_latency_samples": len(latencies), "warnings": warnings}


def write_run_report(directory: Path) -> Path:
    """Create/refresh report.md without changing raw logs or success criteria."""
    directory = Path(directory)
    if not directory.is_dir():
        raise ValueError(f"Run directory does not exist: {directory}")
    data = collect_run_evidence(directory)
    result, manifest, process = data["result"], data["manifest"], data["process"]
    evaluated = result.get("task_evaluated") is True
    outcome = ("成功" if result.get("success") is True else "失败" if result.get("success") is False
               else "未确定") if evaluated else "未评估（工程检查、演练或异常回合不计为任务成功／失败）"
    lines = ["# 任务测评报告", "", "根据本回合已有记录生成；不重新评分。单回合结果不代表任务成功率。", ""]
    lines += _table(("项目", "结果"), [
        ("任务", result.get("task_id", manifest.get("task_id", directory.name))),
        ("运行模式", result.get("mode", manifest.get("kind", manifest.get("mode")))),
        ("运行状态", result.get("status")), ("任务评价", outcome),
        ("工程检查通过", result.get("passed")), ("终止原因", result.get("termination_reason")),
        ("模型", data["provider"].get("model")), ("推理强度", data["provider"].get("reasoning_effort")),
        ("随机种子", manifest.get("seed")),
    ])
    if process:
        lines += _table(("进程检查", "结果"), [("退出码", process.get("returncode")),
                          ("监控超时", process.get("timed_out")), ("进程与结果联合校验通过", process.get("passed"))])
        expected_check_failure = (process.get("returncode") == 2 and result.get("status") == "completed"
                                  and result.get("passed") is False)
        if (process.get("returncode") not in (None, 0) and not expected_check_failure) or process.get("timed_out"):
            lines += ["进程退出异常或超时；请结合 process.json 与 process.log 复核，不能作为正常完成的回合。", ""]
    if result.get("error"):
        lines += [f"运行异常：{_cell(result['error'])}", ""]
    if data["warnings"]:
        lines += ["记录缺项：", "", *(f"- {_cell(w)}" for w in data["warnings"]), ""]
    lines += ["## 执行统计", ""]
    lines += _table(("指标", "数值"), [
        ("模型调用次数", result.get("model_calls")), ("已记录模型交互", data["logged_model_exchanges"]),
        ("执行动作数", result.get("executed_actions")), ("动作拒绝数", result.get("rejected_actions")),
        ("动作超时数", result.get("action_timeouts")), ("错误宣告完成", result.get("false_completion")),
        ("现实耗时（秒）", result.get("wall_time_s")), ("仿真时间（秒）", result.get("simulation_time_s")),
        ("有时延记录的 API 耗时合计（秒）", data["api_latency_total_s"]),
        ("API 时延记录条数", data["api_latency_samples"]),
    ])
    lines += ["API 耗时仅汇总已记录的调用；未返回／未记录时延的请求不补记为零。录像按仿真时间播放。", "",
              "## 动作跟踪误差", "",
              "统计动作结束时反馈中的实测 TCP 与请求目标之差，包含动作超时。单位为米／弧度；这不是物体放置误差，也不是 IK 数学求解误差。", ""]
    rows = [(arm, field, stats["samples"], stats["mean"], stats["max"])
            for arm, fields in data["tracking"].items() for field, stats in fields.items()]
    if rows:
        lines += _table(("机械臂", "误差指标", "样本数", "均值", "最大值"), rows)
    else:
        lines += ["未记录可汇总的动作结束 TCP 误差；不按零误差处理。", ""]
    if data["rejections"]:
        lines += ["## 动作拒绝原因", ""]
        lines += _table(("原因代码", "次数"), data["rejections"].items())
    assessment = result.get("assessment", {})
    lines += ["## 任务指标与判定", ""]
    if assessment:
        lines += [f"评分器状态：{_cell(assessment.get('status'))}；{_cell(assessment.get('reason'))}", ""]
        metrics = list(_flatten(assessment.get("metrics", {})))
        if "lift_confirmed" in assessment.get("metrics", {}):
            lines += ["抓放指标说明：lift_confirmed 表示本回合曾经满足持续抬升要求；lift.* 是末尾滑动窗口的状态。放下后 lift.passed 为否是正常现象，最终放置看 placement.*，整体结论看任务评价。", ""]
        if metrics:
            lines += _table(("指标（原始字段／单位）", "测量值"), metrics)
        lines += ["评分参数见 config.toml。物体误差、接触、稳定性等指标由各任务评分器定义；本报告只呈现其记录，不改变成功条件。", ""]
    else:
        # Older engineering checks expose their own case tables, not task scores.
        checks = {key: result[key] for key in ("pose_checks", "rejection_checks", "cases", "reset_checks", "tolerances") if key in result}
        if checks:
            lines += ["工程检查明细（不作为模型任务成功率）：", "", "```json",
                      json.dumps(checks, ensure_ascii=False, indent=2, allow_nan=False), "```", ""]
        else:
            lines += ["本回合没有任务评分明细，参见原始 result.json。", ""]
    lines += ["## 录像与原始记录", ""]
    paths = [path for name in ("result.json", "episode.jsonl", "motion.jsonl", "manifest.json", "instruction.txt",
                               "config.toml", "robot_profile.toml", "camera_profile.toml", "scene.json",
                               "process.json", "process.log", "frames", "analysis")
             if (path := directory / name).exists()]
    paths += sorted(directory.glob("*.mp4")) + sorted(directory.glob("video/**/*.mp4"))
    for path in paths:
        relative = path.relative_to(directory).as_posix()
        lines.append(f"- [{_cell(relative)}](<{quote(relative, safe='/')}>)")
    lines.append("")
    destination = directory / "report.md"
    temporary = directory / "report.md.tmp"
    temporary.write_text("\n".join(lines), encoding="utf-8")
    temporary.replace(destination)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+", help="Existing run directories; no simulation or API call")
    args = parser.parse_args()
    for directory in args.runs:
        print(write_run_report(directory))


if __name__ == "__main__":
    main()
