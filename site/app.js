(() => {
  "use strict";
  const tasks = window.BENCHMARK_TASKS;
  const $ = id => document.getElementById(id);
  let selected = tasks.find(t => t.id === "L2-03"), view = "overview", setting = "original", level = "all";
  const toast = message => { $("toast").textContent = message; $("toast").classList.add("show"); setTimeout(() => $("toast").classList.remove("show"), 2200); };
  async function copy(text) {
    try {
      if (navigator.clipboard && window.isSecureContext) await navigator.clipboard.writeText(text);
      else {
        const input = document.createElement("textarea");
        input.value = text; input.style.position = "fixed"; input.style.opacity = "0";
        document.body.append(input); input.select();
        const copied = document.execCommand("copy"); input.remove();
        if (!copied) throw new Error("Copy unavailable");
      }
      toast("Copied to clipboard");
    } catch (_) { toast("Copy unavailable — select the command to copy it"); }
  }
  function renderList() {
    $("task-list").replaceChildren();
    tasks.filter(t => level === "all" || t.level === level).forEach(task => {
      const button = document.createElement("button");
      button.className = "task-button" + (task.id === selected.id ? " active" : "");
      button.setAttribute("aria-pressed", String(task.id === selected.id));
      const id = document.createElement("small"), title = document.createElement("span");
      id.textContent = task.id; title.textContent = task.title;
      button.append(id, title);
      button.addEventListener("click", () => { selected = task; render(); renderList(); });
      $("task-list").append(button);
    });
  }
  function render() {
    $("task-id").textContent = selected.id;
    $("task-title").textContent = selected.title;
    for (const state of ["original", "variant01"]) {
      const image = $(state === "original" ? "original-image" : "variant-image");
      image.src = selected.settings[state].images[view];
      image.alt = `${selected.id} ${selected.title}: ${state} simulation initial state, ${view.replace("_", " ")}`;
    }
    const config = selected.settings[setting];
    $("task-prompt").textContent = config.instruction;
    $("task-command").textContent = `python tools/reproduce.py --tasks ${config.id}`;
    $("budget").textContent = `Seed ${config.seed} · ${config.max_calls || "Uncapped"} calls · ${config.wall_time_s ? config.wall_time_s / 3600 + " h" : "No time cap"}`;
    $("config-link").href = `https://github.com/TuojingAI/GPT_Benchmark/blob/master/${config.config}`;
    $("comparison").style.aspectRatio = view === "overview" ? "1100/820" : "4/3";
  }
  document.querySelectorAll("[data-level]").forEach(button => button.addEventListener("click", () => {
    level = button.dataset.level;
    document.querySelectorAll("[data-level]").forEach(b => { b.classList.toggle("active", b === button); b.setAttribute("aria-pressed", String(b === button)); });
    if (level !== "all" && selected.level !== level) selected = tasks.find(t => t.level === level);
    renderList(); render();
  }));
  for (const key of ["view", "setting"]) document.querySelectorAll(`[data-${key}]`).forEach(button => button.addEventListener("click", () => {
    if (key === "view") view = button.dataset.view; else setting = button.dataset.setting;
    document.querySelectorAll(`[data-${key}]`).forEach(b => { b.classList.toggle("active", b === button); b.setAttribute("aria-pressed", String(b === button)); });
    render();
  }));
  $("comparison-slider").addEventListener("input", event => $("comparison").style.setProperty("--split", `${event.target.value}%`));
  $("copy-command").addEventListener("click", () => copy($("task-command").textContent));
  $("copy-setup").addEventListener("click", () => copy($("setup-command").textContent));
  document.querySelectorAll("[data-select]").forEach(link => link.addEventListener("click", () => {
    selected = tasks.find(t => t.id === link.dataset.select);
    document.querySelector('[data-level="all"]').click(); renderList(); render();
  }));
  renderList(); render();
})();
