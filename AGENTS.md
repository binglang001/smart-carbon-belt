# smart-sports-anklet

## Scope
- This `AGENTS.md` applies to the entire repository.

## Project overview
- `client/` contains the device-side program and sensor algorithms.
- `client/main.py` is the main client entry and currently concentrates most runtime logic, state, and threads.
- `client/sensors/` contains reusable sensor access and motion-detection logic.
- `client/utils/` contains shared helpers such as logging and debug utilities.
- `client/tools/` contains offline collection, plotting, and analysis scripts; these are support tools, not the main runtime path.
- `server/server.py` is the Flask backend entry.
- `server/control.html` is the web control panel and dashboard.
- `server/data/` stores runtime JSON data and should be treated as persisted application state.

## Working rules
- Keep changes minimal and targeted; avoid broad refactors unless the user explicitly asks for them.
- Prefer fixing root causes over adding superficial patches.
- Preserve the current architecture unless there is a clear reason to change it: this repo relies on module-level state, background threads, and JSON persistence.
- Keep user-facing text, comments, and docs consistent with the existing Chinese-first style.
- Preserve UTF-8 encoding when editing files.
- Do not edit generated runtime data in `server/data/` unless the task specifically requires schema or seed-data changes.
- Do not touch `.venv/`, `docs/论文/`, or other research/reference files unless the task explicitly targets them.

## Client-specific guidance
- Reuse `client/utils/logger.py` for logging changes instead of introducing ad-hoc print-based logging.
- High-frequency sensor, pace, sampling, or detection traces should not stay at `INFO` level unless they are strongly rate-limited; prefer `DEBUG` or throttled summaries.
- Be careful with hardware-coupled code in `client/main.py` and `client/sensors/icm20689.py`; desktop environments may not have the board-specific runtime.
- When adjusting runtime behavior, consider thread safety because the client uses multiple long-running threads and shared globals.

## Web UI guidance
- Keep desktop and mobile behavior aligned.
- Avoid duplicating mode definitions in multiple places; if a UI control reflects mode count, derive it from the actual mode list whenever practical.
- For controls like sliders, tabs, and carousels, ensure the number of visual states matches the actual number of supported modes.
- Keep changes in `server/control.html` focused and test both normal and narrow/mobile layouts when possible.

## Server-specific guidance
- Preserve JSON schema compatibility in `server/data/*.json` where possible.
- Keep persistence atomic; `server/server.py` already uses an atomic-save pattern and delayed save scheduling.
- Avoid adding blocking work to request handlers without a clear need.

## Validation guidance
- Prefer the smallest useful validation first.
- Safe baseline validation is `python -m py_compile` on the touched Python files.
- For Flask-side changes, validate `server/server.py` syntax locally when possible.
- Do not claim full end-to-end verification for client runtime behavior unless hardware-dependent paths were actually exercised.

## Review focus areas
- Mobile UI state consistency in `server/control.html`.
- Logging noise and log-level discipline in `client/main.py` and sensor-related modules.
- Shared-state/thread-safety issues in the client.
- Persistence, API robustness, and data validation in the server.
