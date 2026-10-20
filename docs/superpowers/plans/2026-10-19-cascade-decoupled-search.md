# Cascade Decoupled Search Implementation Plan

> **For the coordinating agent:** This plan is your dispatch list. You do not implement the tasks. Use superpowers:subagent-driven-development for the dispatch mechanics. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Four programs tune one cascade loop each. The next program runs only after the previous loop has converged and saved its best result.

**Architecture:** `search_knobs` is one coordinate pass over a fixed factor list. `calibration/tune_rate.py`, `tune_angle.py`, `tune_velocity.py`, and `tune_position.py` are separate programs. Rate freezes geometry, fits hover thrust, then searches motor lag and roll/pitch rate P. Each later program loads the previous loop's saved best, adds roll-rate and pitch-rate RMS to its score, and searches only its own gains. A loop has converged when its program exits 0 and `calibration/results/cascade/stages/<step>/best.json` exists with `"step"` equal to that loop. There is no RMS threshold.

**Tech Stack:** Python 3, pytest, numpy, plotly. Unit tests call the `fly` fake and must not start Gazebo, Docker, or PX4. The live convergence run of a loop is a separate command, after that loop's tests and review.

**Spec:** This plan. It supersedes `docs/superpowers/specs/2026-10-16-iris-cascade-calibration-design.md` and `docs/superpowers/plans/2026-10-16-iris-cascade-calibration.md` where those keep every `MC_*RATE_*` gain frozen and where they search rotor x/y. The 2026-10-16 documents stay on disk unchanged.

## Global Constraints

- No git commit or push unless the user explicitly asks. Do not add a commit step.
- You are the coordinating agent. You dispatch. You do not edit the cascade code yourself.
- Every implementer and every reviewer is a fresh subagent on `grok-4.7-high`. No fast mode and no other model family. Ignore any skill advice to use a cheaper or faster model.
- An implementer does one task and does not dispatch anyone, including a reviewer.
- Pytest must not start Gazebo, Docker, or PX4.
- Do not kill a sim that is already running. If a sim is running, do not dispatch a live tune command and do not dispatch the next loop.
- Update `calibration/README.md` and put a new `UPDATES.md` entry on top when the cascade behavior changes. No other new markdown.
- Stage ids stay `rate`, `angle`, `velocity`, `position`. "Angle rate" in the request is the `rate` step.
- Closeness stays unshifted per-axis RMS of `(sim - log)` via `axis_rms` and `rms_sum`. Convergence is the saved best file, not an RMS cutoff.
- `MPC_THR_HOVER` stays 0.5. Yaw gains, rate I/D/FF/K, mass, and inertia are not knobs.
- A knob is kept only when `proposed < current`.
- Factors, relative to the knob value at the start of that knob, are exactly `0.7`, `0.85`, `1.15`, `1.3`. One pass. No second round.
- `timeConstantDown = timeConstantUp * (down0 / up0)` using the two values read from the model before the search.

## Review Focus

- A rejected factor is the last flight of a knob, so the jinja and the gain file can be left on that reject. After the stage, both files must match the best trial.
- An angle, velocity, or position change that improves its own axes and worsens roll or pitch rate must be rejected.
- `timeConstantDown` must stay on the original ratio when `timeConstantUp` moves.
- A target sitting at `base * 0.7 * 0.7` must not be reached. The pass stops at `base * 0.7`.
- `save_stage` on a step that has no trials must raise `ValueError` and write nothing.

## Dispatch

You work the tasks in order. The implementer's report is not acceptance. After every report you dispatch a reviewer, and you wait for that reviewer before the next dispatch.

For each task:

1. Dispatch a fresh implementer. Give it that task's text, the Global Constraints, and the interfaces the task says it consumes. Tell it to write the tests, make them pass, and report the test command and the pass/fail output. Tell it not to dispatch a reviewer and not to start the next task.
2. When the implementer reports, dispatch a fresh reviewer. Give it the task text, the implementer's report, and the diff. The reviewer checks the result of the job: the named tests passed, the diff matches the task's interfaces, and the Review Focus items that this task owns still hold. The reviewer does not edit the code.
3. If the reviewer reports a defect, dispatch a fresh implementer with that report and the task text. When that implementer reports, dispatch a fresh reviewer again. Repeat until a reviewer accepts.
4. Only then mark the task done and dispatch the next one.

Tasks 1–4 are the rate loop. Tasks 5, 6, and 7 are each a new implementer that must not edit an earlier loop's program. Do not dispatch Task 5 until a reviewer has accepted the live rate run. The same gate applies to velocity after angle, and to position after velocity.

After the reviewer accepts Task 4, dispatch a fresh subagent whose only job is `python3 -m calibration.tune_rate` from the repo root. When it reports, dispatch a reviewer to verify exit code 0 and `calibration/results/cascade/stages/rate/best.json` with `"step": "rate"`. A pytest fixture does not count. If the reviewer rejects the live result, dispatch a fresh implementer on Task 4 with the review, then a fresh reviewer, then run the live command again. Do the same after Tasks 5, 6, and 7 with `tune_angle`, `tune_velocity`, and `tune_position`, and with `stages/angle/best.json`, `stages/velocity/best.json`, and `stages/position/best.json`.

After the position live run is accepted, dispatch one fresh whole-branch reviewer on the full diff. If that reviewer reports a defect, dispatch one fresh implementer to fix it, then one fresh reviewer. Stop when that reviewer accepts.

## File structure

- `calibration/cascade_search.py` — one-pass coordinate line search.
- `calibration/cascade_gains.py` — which gains `gain_file` and `search_keys` accept.
- `calibration/cascade_session.py` — `Session.save_stage`.
- `calibration/cascade_stage.py` — the stable convergence directory and the previous-loop check.
- `calibration/cascade_tune.py` — shared flight wiring used by the four programs.
- `calibration/tune_rate.py`, `tune_angle.py`, `tune_velocity.py`, `tune_position.py` — one program per loop.
- `calibration/run_cascade.py` — no longer tunes. Its `main` prints the four commands and returns 2.
- `.vscode/launch.json` — one debug config per program, `cwd` `${workspaceFolder}/calibration`.
- Tests beside those modules. `calibration/README.md` and `UPDATES.md` in Task 7.

---

### Task 1: One-pass line search

**Files:**
- Modify: `calibration/cascade_search.py`
- Test: `calibration/tests/test_cascade_search.py`

**Interfaces:**
- Consumes: nothing
- Produces: `FACTORS: tuple[float, ...] = (0.7, 0.85, 1.15, 1.3)`. `keep(current: float, proposed: float) -> bool`. `search_knobs(start: dict[str, float], evaluate: Callable[[dict[str, float]], float]) -> tuple[dict[str, float], float]`. One pass. For each key, every factor multiplies the value from the start of that key. Keep the strict minimum. Later keys see the kept value. Do not run another round.

- [ ] **Step 1: Write the failing test**

Replace `test_search_moves_only_the_knob_that_gets_closer` with these two tests. Leave `test_keep_requires_a_strict_drop` as it is.

```python
def test_search_keeps_the_factor_that_lands_on_the_target():
    def evaluate(knobs: dict[str, float]) -> float:
        return abs(knobs["arm_x"] - 0.13 * 1.15) + abs(knobs["arm_y"] - 0.21)

    best, score = search_knobs({"arm_x": 0.13, "arm_y": 0.21}, evaluate)
    assert best["arm_y"] == pytest.approx(0.21)
    assert best["arm_x"] == pytest.approx(0.13 * 1.15)
    assert score == pytest.approx(0.0)


def test_search_does_not_compound_a_second_pass():
    calls = {"n": 0}

    def evaluate(knobs: dict[str, float]) -> float:
        calls["n"] += 1
        return abs(knobs["arm_x"] - 0.13 * 0.7 * 0.7)

    best, score = search_knobs({"arm_x": 0.13}, evaluate)
    assert best["arm_x"] == pytest.approx(0.13 * 0.7)
    assert score == pytest.approx(abs(0.13 * 0.7 - 0.13 * 0.7 * 0.7))
    assert calls["n"] == 1 + len(FACTORS)
```

Import `FACTORS` from `calibration.cascade_search`.

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_search.py -q`

Expected: FAIL. The current factors are `1.1` and `0.9`, and the loop can take a second pass.

- [ ] **Step 3: Implement the one-pass search in `calibration/cascade_search.py`**

```python
FACTORS: tuple[float, ...] = (0.7, 0.85, 1.15, 1.3)

def search_knobs(start, evaluate):
    knobs = dict(start)
    score = evaluate(dict(knobs))
    for name in tuple(knobs):
        base = knobs[name]
        best = dict(knobs)
        best_score = score
        for factor in FACTORS:
            trial = dict(knobs)
            trial[name] = base * factor
            trial_score = evaluate(trial)
            if keep(best_score, trial_score):
                best = trial
                best_score = trial_score
        knobs = best
        score = best_score
    return knobs, score
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_search.py -q`

Expected: PASS

---

### Task 2: Rate P gains are searchable

**Files:**
- Modify: `calibration/cascade_gains.py`
- Test: `calibration/tests/test_cascade_gains.py`

**Interfaces:**
- Consumes: nothing
- Produces: `RATE_P_KEYS: tuple[str, ...] = ("MC_ROLLRATE_P", "MC_PITCHRATE_P")`. `search_keys("rate")` returns `RATE_P_KEYS`. `gain_file` accepts those two overrides. `gain_file` still raises `ValueError` for `MC_ROLLRATE_I`, `MC_ROLLRATE_D`, `MC_ROLLRATE_FF`, `MC_ROLLRATE_K`, the matching `MC_PITCHRATE_*` fields, yaw, and `MPC_THR_HOVER`.

- [ ] **Step 1: Write the failing test**

Change `assert search_keys("rate") == ()` to `assert search_keys("rate") == RATE_P_KEYS`. Import `RATE_P_KEYS`.

Add:

```python
def test_rate_p_override_is_accepted_and_rate_i_is_refused():
    initial = {
        "MC_ROLLRATE_P": 0.08,
        "MC_ROLLRATE_I": 0.2,
        "MC_PITCHRATE_P": 0.08,
        "MC_PITCHRATE_I": 0.2,
        "MC_ROLL_P": 12.0,
    }
    gained = gain_file(initial, {"MC_ROLLRATE_P": 0.05, "MC_PITCHRATE_P": 0.06})
    assert gained["MC_ROLLRATE_P"] == pytest.approx(0.05)
    assert gained["MC_PITCHRATE_P"] == pytest.approx(0.06)
    assert gained["MC_ROLLRATE_I"] == pytest.approx(0.2)
    assert gained["MPC_THR_HOVER"] == pytest.approx(0.5)
    with pytest.raises(ValueError):
        gain_file(initial, {"MC_ROLLRATE_I": 0.1})
    with pytest.raises(ValueError):
        gain_file(initial, {"MC_ROLLRATE_D": 0.01})
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_gains.py::test_rate_p_override_is_accepted_and_rate_i_is_refused calibration/tests/test_cascade_gains.py::test_search_keys_and_hover_restore_and_yaw_refusal -q`

Expected: FAIL because `search_keys("rate")` is `()` and `gain_file` rejects `MC_ROLLRATE_P`.

- [ ] **Step 3: Implement `RATE_P_KEYS` and admit those two names in `_SEARCHABLE` and `_STEPS["rate"]`**

Do not add I, D, FF, or K.

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_gains.py -q`

Expected: PASS

---

### Task 3: Save the best trial of a stage

**Files:**
- Modify: `calibration/cascade_session.py`
- Test: `calibration/tests/test_cascade_session.py`

**Interfaces:**
- Consumes: `Session.record`
- Produces: `Session.save_stage(self, step: str) -> Path`. Writes `directory/stages/<step>/best.json` and `directory/stages/<step>/comparison.html`. Returns that directory. `best.json` keys are `step`, `trial`, `score`, `parameters`, `log`. The chosen trial is the lowest score for that step; a tie keeps the earliest. The HTML shows that trial's sim traces (the page title contains `trial <n>` with the real trial number). `_cycle_script` keys cycles by the RMS point's trial number so a click still selects that flight. `save_stage` raises `ValueError` when the step has no trials and creates no directory.

- [ ] **Step 1: Write the failing test**

```python
def test_save_stage_writes_the_best_trial_and_refuses_an_empty_step(tmp_path: Path):
    session = open_session(tmp_path, Path("calibration/log_2_UnknownDate.ulg"))
    flight = _flight()
    session.record("rate", {"MC_ROLLRATE_P": 0.08}, 2.0, flight, _series([0.2, 0.4, 0.6]))
    session.record("rate", {"MC_ROLLRATE_P": 0.056}, 1.0, flight, _series([0.05, 0.9, 0.55]))
    session.record("rate", {"MC_ROLLRATE_P": 0.092}, 3.0, flight, _series([4.0, 4.0, 4.0]))
    with pytest.raises(ValueError):
        session.save_stage("angle")
    assert not (session.directory / "stages" / "angle").exists()
    stage = session.save_stage("rate")
    saved = json.loads((stage / "best.json").read_text(encoding="utf-8"))
    assert saved["step"] == "rate"
    assert saved["trial"] == 2
    assert saved["score"] == 1.0
    assert saved["parameters"]["MC_ROLLRATE_P"] == pytest.approx(0.056)
    html = (stage / "comparison.html").read_text(encoding="utf-8")
    assert "trial 2" in html
    assert "trial 3" not in html
    assert "0.9" in html
```

Import `pytest` in this test module.

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_cascade_session.py::test_save_stage_writes_the_best_trial_and_refuses_an_empty_step -q`

Expected: FAIL with `AttributeError` on `save_stage`.

- [ ] **Step 3: Implement `Session.save_stage`**

Filter `self._trials` by `step`. Raise `ValueError` if the filter is empty. Choose the minimum `(score, trial)`. Build the HTML from that step's trials and flights, displaying the chosen flight. Key the click script by `trial["trial"]`.

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_cascade_session.py -q`

Expected: PASS, including the existing click-script test.

---

### Task 4: Rate program

**Files:**
- Create: `calibration/cascade_stage.py`
- Create: `calibration/cascade_tune.py`
- Create: `calibration/tune_rate.py`
- Modify: `calibration/run_cascade.py` (`main` only)
- Modify: `.vscode/launch.json`
- Test: `calibration/tests/test_tune_rate.py`
- Test: `calibration/tests/test_run_cascade.py`

**Interfaces:**
- Consumes: `search_knobs`, `RATE_P_KEYS`, `gain_file`, `read_plant`, `read_arms`, `write_model`, `Session.save_stage`
- Produces: `PREVIOUS = {"rate": None, "angle": "rate", "velocity": "angle", "position": "velocity"}`. `converged_path(stage_root: Path, step: str) -> Path` returns `stage_root/stages/<step>/best.json`. `load_converged(stage_root: Path, step: str) -> dict` raises `FileNotFoundError` when the file is missing and `ValueError` when `step` does not match. `tune_rate(...) -> dict[str, float]` flies only `"rate"`. `python3 -m calibration.tune_rate` writes the best to `calibration/results/cascade/stages/rate/best.json` and exits 0. `python3 -m calibration.run_cascade` prints the four `python3 -m calibration.tune_*` commands and returns 2.

The rate program reads arms, prop radius, and `max_rot_velocity` and never searches them. It writes hover `motorConstant` at that frozen radius. It searches `time_constant_up` to completion, then `MC_ROLLRATE_P` and `MC_PITCHRATE_P`. `timeConstantDown` uses the original ratio. It then rewrites the model and the gain file to the lowest rate score, even when the last flight was a rejected factor. It does not fly angle, velocity, or position. Score axes are `roll_rate` and `pitch_rate` only.

- [ ] **Step 1: Write the failing test**

```python
def test_rate_step_freezes_geometry_and_restores_the_best_lag(tmp_path: Path):
    model = tmp_path / "iris.sdf.jinja"
    source = (ROOT / "calibration/models/iris/iris.sdf.jinja").read_text(encoding="utf-8")
    model.write_text(source, encoding="utf-8")
    before_arms = read_arms(model)
    before = read_plant(model)
    logged_p = load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg").initial_parameters["MC_ROLLRATE_P"]
    seen: list[tuple[str, float, float]] = []

    def fly(step, flight, model_path, gains_path):
        plant = read_plant(model_path)
        gains = json.loads(Path(gains_path).read_text(encoding="utf-8"))
        seen.append((step, plant["time_constant_up"], gains["MC_ROLLRATE_P"]))
        level = 1.0
        if plant["time_constant_up"] == pytest.approx(before["time_constant_up"] * 0.85):
            level = 0.0
        if gains["MC_ROLLRATE_P"] != pytest.approx(logged_p):
            level = 10.0
        return {name: ([0.0], [level]) for name in (
            "roll_rate", "pitch_rate", "roll", "pitch", "vx", "vy", "vz", "x", "y", "z")}

    from calibration.tune_rate import tune_rate

    tune_rate(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        model,
        tmp_path / "iris_controller.json",
        fly,
        tmp_path / "cascade_report.json",
        tmp_path / "cascade",
    )
    assert read_arms(model)[0] == pytest.approx(before_arms[0])
    assert read_arms(model)[1] == pytest.approx(before_arms[1])
    after = read_plant(model)
    assert after["prop_radius"] == pytest.approx(before["prop_radius"])
    assert after["max_rot_velocity"] == pytest.approx(before["max_rot_velocity"])
    assert after["time_constant_up"] == pytest.approx(before["time_constant_up"] * 0.85)
    assert after["time_constant_down"] == pytest.approx(
        after["time_constant_up"] * before["time_constant_down"] / before["time_constant_up"]
    )
    saved_gains = json.loads((tmp_path / "iris_controller.json").read_text(encoding="utf-8"))
    assert saved_gains["MC_ROLLRATE_P"] == pytest.approx(logged_p)
    assert saved_gains["MC_ROLLRATE_I"] == pytest.approx(
        load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg").initial_parameters["MC_ROLLRATE_I"]
    )
    rate_lags = [lag for step, lag, _p in seen if step == "rate"]
    assert any(lag == pytest.approx(before["time_constant_up"] * 0.7) for lag in rate_lags)
    assert {step for step, _lag, _p in seen} == {"rate"}
    best = json.loads((tmp_path / "cascade" / "stages" / "rate" / "best.json").read_text(encoding="utf-8"))
    assert best["step"] == "rate"
    assert best["parameters"]["time_constant_up"] == pytest.approx(before["time_constant_up"] * 0.85)
    assert best["parameters"]["MC_ROLLRATE_P"] == pytest.approx(logged_p)
```

Add `test_run_cascade_prints_the_four_programs` in `test_run_cascade.py`: `main(["log.ulg"])` returns 2 and stderr contains `python3 -m calibration.tune_rate`, `tune_angle`, `tune_velocity`, and `tune_position`.

Replace `test_outer_step_does_not_move_motor_constant_or_arms`. It currently expects one `run_cascade` call to fly angle, velocity, and position. Those flights now belong to later programs. Point the geometry assertions at `tune_rate` and assert the fly steps are only `"rate"`.

Import `read_arms` and `read_plant` from `calibration.cascade_model`, and `load_flight` from `calibration.cascade_flight`. Put the test in `calibration/tests/test_tune_rate.py`. A sim sample of `0.0` is closer to the log than `1.0` or `10.0`, so the lag search keeps `0.85` and every rate-P factor is refused. The last rate flight is one of those refused factors. The model, the gain file, and `best.json` must still show the kept lag and the logged rate P.

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_tune_rate.py -q`

Expected: FAIL with `ModuleNotFoundError` for `calibration.tune_rate`.

- [ ] **Step 3: Implement the rate program**

`tune_rate` searches `{"time_constant_up": plant["time_constant_up"]}`, then the two `RATE_P_KEYS` from `flight.initial_parameters`. Remember the lowest rate score across both calls. Write that lag and those two P gains back. Write `stages/rate/best.json` through `save_stage` into the stage root. Add a `tune_rate` launch config with `cwd` `${workspaceFolder}/calibration`. Change `run_cascade.main` so it does not fly.

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_tune_rate.py calibration/tests/test_run_cascade.py calibration/tests/test_cascade_gains.py -q`

Expected: PASS

You dispatch the live rate run only after a reviewer has accepted this task. Dispatch a fresh subagent to run `python3 -m calibration.tune_rate` from the repo root, unless a sim is already running. When it reports, dispatch a reviewer. Acceptance is exit 0 and `calibration/results/cascade/stages/rate/best.json` with `"step": "rate"`. That acceptance is the gate for Task 5.

---

### Task 5: Angle program

**Dispatch:** a new implementer, only after a reviewer has accepted the live rate result. The implementer does not dispatch a reviewer.

**Files:**
- Create: `calibration/tune_angle.py`
- Modify: `.vscode/launch.json`
- Test: `calibration/tests/test_tune_angle.py`

**Interfaces:**
- Consumes: `load_converged`, `PREVIOUS`, `search_knobs`, `search_keys("angle")`, the rate program's restored model and gains
- Produces: `python3 -m calibration.tune_angle`. With no `stages/rate/best.json` it prints `rate has not converged; run python3 -m calibration.tune_rate` and returns 2 without flying. With that file, it searches only `MC_ROLL_P` and `MC_PITCH_P`. The score axes are `roll_rate`, `pitch_rate`, `roll`, `pitch`. It does not change `time_constant_up`, arms, prop radius, `max_rot_velocity`, `MC_ROLLRATE_P`, or `MC_PITCHRATE_P`. It writes `stages/angle/best.json` and restores the best angle gains before exiting.

- [ ] **Step 1: Write the failing test**

```python
def test_angle_refuses_to_start_before_rate_converges(tmp_path: Path):
    code = tune_angle_main(stage_root=tmp_path / "cascade")
    assert code == 2


def test_angle_rejects_a_gain_that_breaks_rate(tmp_path: Path):
    model = tmp_path / "iris.sdf.jinja"
    model.write_text(
        (ROOT / "calibration/models/iris/iris.sdf.jinja").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    stage = tmp_path / "cascade"
    rate_best = stage / "stages" / "rate"
    rate_best.mkdir(parents=True)
    (rate_best / "best.json").write_text(
        json.dumps({"step": "rate", "trial": 1, "score": 1.0, "parameters": {}, "log": None}) + "\n",
        encoding="utf-8",
    )
    flight = window_flight(load_flight(ROOT / "calibration" / "log_2_UnknownDate.ulg"), 50.0, 80.0)
    initial_p = flight.initial_parameters["MC_ROLL_P"]
    before = read_plant(model)
    seen: list[str] = []

    def fly(step, flight, model_path, gains_path):
        seen.append(step)
        gains = json.loads(Path(gains_path).read_text(encoding="utf-8"))
        series = {name: ([0.0, 1.0], [0.0, 0.0]) for name in (
            "roll_rate", "pitch_rate", "roll", "pitch", "vx", "vy", "vz", "x", "y", "z")}
        if gains["MC_ROLL_P"] == pytest.approx(initial_p * 0.7):
            series["roll"] = (list(flight.roll.t_s), list(flight.roll.response))
            series["roll_rate"] = ([0.0], [1.0e6])
        return series

    tune_angle(
        ROOT / "calibration" / "log_2_UnknownDate.ulg",
        model,
        tmp_path / "iris_controller.json",
        fly,
        tmp_path / "cascade_report.json",
        stage,
    )
    assert set(seen) == {"angle"}
    saved = json.loads((tmp_path / "iris_controller.json").read_text(encoding="utf-8"))
    assert saved["MC_ROLL_P"] == pytest.approx(initial_p)
    assert read_plant(model)["time_constant_up"] == pytest.approx(before["time_constant_up"])
    best = json.loads((stage / "stages" / "angle" / "best.json").read_text(encoding="utf-8"))
    assert best["step"] == "angle"
```

`tune_angle_main` is the function `python3 -m calibration.tune_angle` calls. Give it a `stage_root` argument so the refusal test does not read `calibration/results/cascade`. The `0.7` case improves roll RMS and sets roll-rate sim to `1e6`. With rate in the score, `MC_ROLL_P` stays at the logged value.

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_tune_angle.py -q`

Expected: FAIL with `ModuleNotFoundError` for `calibration.tune_angle`.

- [ ] **Step 3: Implement `tune_angle`**

Check `load_converged(stage_root, "rate")` before any flight. Search only `search_keys("angle")`. Score `_AXES["angle"] = ("roll_rate", "pitch_rate", "roll", "pitch")`. Restore the best gains, write `stages/angle/best.json`, and add a `tune_angle` launch config.

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_tune_angle.py calibration/tests/test_tune_rate.py -q`

Expected: PASS

After a reviewer accepts this task, you dispatch a fresh subagent to run `python3 -m calibration.tune_angle` unless a sim is already running. When it reports, dispatch a reviewer. Acceptance is exit 0 and `calibration/results/cascade/stages/angle/best.json` with `"step": "angle"`.

---

### Task 6: Velocity program

**Dispatch:** a new implementer, only after a reviewer has accepted the live angle result. The implementer does not dispatch a reviewer.

**Files:**
- Create: `calibration/tune_velocity.py`
- Modify: `.vscode/launch.json`
- Test: `calibration/tests/test_tune_velocity.py`

**Interfaces:**
- Consumes: `load_converged(stage_root, "angle")`, `search_keys("velocity")`
- Produces: `python3 -m calibration.tune_velocity`. Missing angle convergence returns 2 and prints `angle has not converged; run python3 -m calibration.tune_angle`, with no flight. Otherwise it searches only `MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_I_ACC`, `MPC_XY_VEL_D_ACC`, `MPC_Z_VEL_P_ACC`, `MPC_Z_VEL_I_ACC`, `MPC_Z_VEL_D_ACC`. Score axes are `roll_rate`, `pitch_rate`, `vx`, `vy`, `vz`. It writes `stages/velocity/best.json`.

- [ ] **Step 1: Write the failing test**

Same shape as the angle refusal test, for `tune_velocity_main` and the message `angle has not converged`. A second test plants `stages/angle/best.json`, flies a fake, and asserts every flown step is `"velocity"`, the score used for a trial that sets `roll_rate` sim to `1e6` does not keep that velocity gain, and `stages/velocity/best.json` has `"step": "velocity"`. Use `MPC_XY_VEL_P_ACC * 0.7` as the gain that improves `vx` by copying the windowed `vx` response and ruins `roll_rate`.

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_tune_velocity.py -q`

Expected: FAIL with `ModuleNotFoundError` for `calibration.tune_velocity`.

- [ ] **Step 3: Implement `tune_velocity` and its launch config**

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest calibration/tests/test_tune_velocity.py -q`

Expected: PASS

After a reviewer accepts this task, you dispatch a fresh subagent to run `python3 -m calibration.tune_velocity` unless a sim is already running. When it reports, dispatch a reviewer. Acceptance is exit 0 and `calibration/results/cascade/stages/velocity/best.json` with `"step": "velocity"`.

---

### Task 7: Position program

**Dispatch:** a new implementer, only after a reviewer has accepted the live velocity result. The implementer does not dispatch a reviewer.

**Files:**
- Create: `calibration/tune_position.py`
- Modify: `.vscode/launch.json`
- Modify: `calibration/README.md`
- Modify: `UPDATES.md`
- Test: `calibration/tests/test_tune_position.py`

**Interfaces:**
- Consumes: `load_converged(stage_root, "velocity")`, `search_keys("position")`
- Produces: `python3 -m calibration.tune_position`. Missing velocity convergence returns 2 and prints `velocity has not converged; run python3 -m calibration.tune_velocity`, with no flight. Otherwise it searches only `MPC_XY_P` and `MPC_Z_P`. Score axes are `roll_rate`, `pitch_rate`, `x`, `y`, `z`. It writes `stages/position/best.json`.

- [ ] **Step 1: Write the failing test**

Same shape as Task 6, using `MPC_XY_P * 0.7`, the windowed `x` response, and a ruined `roll_rate`. Assert the program does not fly without `stages/velocity/best.json`, flies only `"position"` with that file, keeps the logged `MPC_XY_P`, and writes `stages/position/best.json`.

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest calibration/tests/test_tune_position.py -q`

Expected: FAIL with `ModuleNotFoundError` for `calibration.tune_position`.

- [ ] **Step 3: Implement `tune_position` and its launch config**

- [ ] **Step 4: Update the docs**

In `calibration/README.md`, replace the single `run_cascade` sequence. The four commands are `python3 -m calibration.tune_rate`, `tune_angle`, `tune_velocity`, and `tune_position`. Rate freezes rotor x/y, prop radius, and `maxRotVelocity`, fits hover `motorConstant`, then searches `timeConstantUp` and `MC_ROLLRATE_P` / `MC_PITCHRATE_P`. Each later command refuses to fly until `calibration/results/cascade/stages/<previous>/best.json` exists. Angle, velocity, and position scores include roll-rate and pitch-rate RMS. Factors are `0.7`, `0.85`, `1.15`, and `1.3`, once per knob.

Add a new top `UPDATES.md` entry dated `2026-10-19` with those same facts.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `python3 -m pytest calibration/tests/test_cascade_search.py calibration/tests/test_cascade_gains.py calibration/tests/test_cascade_session.py calibration/tests/test_tune_rate.py calibration/tests/test_tune_angle.py calibration/tests/test_tune_velocity.py calibration/tests/test_tune_position.py calibration/tests/test_run_cascade.py -q`

Expected: PASS. Then run `python3 -m pytest calibration/tests -q` and expect PASS. If a failure is an existing test that still expects one process to fly every loop, point it at the program that owns that loop. Do not weaken the rate-RMS rejection tests.

After a reviewer accepts this task, you dispatch a fresh subagent to run `python3 -m calibration.tune_position` unless a sim is already running. When it reports, dispatch a reviewer. The cascade is complete when that reviewer accepts exit 0 and `calibration/results/cascade/stages/position/best.json` with `"step": "position"`. You then dispatch the whole-branch reviewer from the Dispatch section.
