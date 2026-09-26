"""Separate application initialization from task/asset errors."""
import json
import sys
import time
from pathlib import Path
from runtime_config import simulation_config

output = Path(sys.argv[1])
output.mkdir(parents=True, exist_ok=True)
config = simulation_config(640, 480)
(output / "config.json").write_text(json.dumps(config, indent=2))
started = time.time()
from isaacsim import SimulationApp

app = SimulationApp(config)
for _ in range(20):
    app.update()
(output / "summary.json").write_text(json.dumps({
    "startup_passed": True, "updates": 20,
    "elapsed_seconds": time.time() - started, "config": config,
}, indent=2))
print("STARTUP_SMOKE_PASSED", flush=True)
app.close()
