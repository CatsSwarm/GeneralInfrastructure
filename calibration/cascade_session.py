"""One cascade run: parameter history from the ULG, and a target-vs-sim plot."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Every comparison page shows these panels, whichever cascade step is in flight.
_PLOT_LABELS: tuple[tuple[str, str], ...] = (
    ("roll_rate", "roll rate (rad/s)"),
    ("pitch_rate", "pitch rate (rad/s)"),
    ("yaw_rate", "yaw rate (rad/s)"),
    ("roll", "roll (rad)"),
    ("pitch", "pitch (rad)"),
    ("yaw", "yaw (rad)"),
    ("vx", "velocity X (m/s)"),
    ("vy", "velocity Y (m/s)"),
    ("vz", "velocity Z (m/s)"),
    ("x", "position X (m)"),
    ("y", "position Y (m)"),
    ("z", "position Z (m)"),
)


class Session:
    def __init__(self, directory: Path, ulg: Path):
        self.directory = directory
        self.ulg = str(ulg)
        self.history_path = directory / "parameter_history.jsonl"
        self.state_path = directory / "state.json"
        self.plot_path = directory / "comparison.html"
        self._best: dict[str, float] = {}
        self._trials: list[dict] = []
        self._flights: list[dict] = []
        self._logged = None
        self._per_stage = False

    def log_file(self, step: str | None = None) -> Path:
        """Path for the PX4 log of the flight that is about to start.

        A stage session keeps that log under ``stages/<step>/logs`` so a
        later step cannot reuse the same ``trial_NNN.ulg`` name.
        """
        if self._per_stage:
            if not step:
                raise ValueError("stage session log requires a step")
            base = self.directory / "stages" / step
            number = sum(1 for trial in self._trials if trial["step"] == step) + 1
        else:
            base = self.directory
            number = len(self._trials) + 1
        path = base / "logs" / f"trial_{number:03d}.ulg"
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    def _history_for(self, step: str) -> Path:
        if not self._per_stage:
            return self.history_path
        path = self.directory / "stages" / step / "parameter_history.jsonl"
        path.parent.mkdir(parents=True, exist_ok=True)
        if not any(trial["step"] == step for trial in self._trials):
            path.write_text("", encoding="utf-8")
        return path

    def record(
        self,
        step: str,
        parameters: dict,
        score: float,
        flight,
        series: dict,
        log_path: Path | None = None,
    ) -> None:
        """Append one trial and refresh the comparison plot."""
        score = float(score)
        improved = step in self._best and score < self._best[step]
        if step not in self._best or score < self._best[step]:
            self._best[step] = score
        trial = {
            "trial": len(self._trials) + 1,
            "step": step,
            "ulg": self.ulg,
            "score": score,
            "improved": improved,
            "parameters": {str(name): float(value) for name, value in parameters.items()},
            "log": None if log_path is None else str(log_path),
        }
        history = self._history_for(step)
        self._trials.append(trial)
        self._flights.append(series)
        self._logged = flight
        with history.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(trial) + "\n")
        self.state_path.write_text(
            json.dumps(
                {
                    "ulg": self.ulg,
                    "step": step,
                    "trial": trial["trial"],
                    "best_score": dict(self._best),
                    "parameters": trial["parameters"],
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        _comparison_plot(self.plot_path, self._trials, self._flights, step, flight)

    def save_stage(self, step: str) -> Path:
        """Write the lowest-score trial of ``step`` and a plot of that step."""
        chosen = [
            (trial, series)
            for trial, series in zip(self._trials, self._flights)
            if trial["step"] == step
        ]
        if not chosen:
            raise ValueError(f"no trials for step {step}")
        best_at = min(
            range(len(chosen)),
            key=lambda index: (chosen[index][0]["score"], chosen[index][0]["trial"]),
        )
        best = chosen[best_at][0]
        stage = self.directory / "stages" / step
        stage.mkdir(parents=True, exist_ok=True)
        (stage / "best.json").write_text(
            json.dumps(
                {
                    "step": best["step"],
                    "trial": best["trial"],
                    "score": best["score"],
                    "parameters": best["parameters"],
                    "log": best["log"],
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        _comparison_plot(
            stage / "comparison.html",
            [trial for trial, _series in chosen],
            [series for _trial, series in chosen],
            step,
            self._logged,
            shown=best_at,
        )
        return stage


def open_session(root: Path, ulg: Path) -> Session:
    """Create a new session directory and point ``latest.txt`` at it."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    directory = Path(root) / f"{stamp}_{Path(ulg).stem}"
    directory.mkdir(parents=True)
    (Path(root) / "latest.txt").write_text(str(directory) + "\n", encoding="utf-8")
    return Session(directory, ulg)


def _legend_name(row: int) -> str:
    return "legend" if row == 1 else f"legend{row}"


def _place_legends(fig, n_rows: int) -> None:
    """One legend in the top-right corner of each subplot."""
    placed = {}
    for row in range(1, n_rows + 1):
        axis = "yaxis" if row == 1 else f"yaxis{row}"
        _y0, y1 = fig.layout[axis].domain
        placed[_legend_name(row)] = dict(
            x=0.99,
            y=y1,
            xanchor="right",
            yanchor="top",
            xref="paper",
            yref="paper",
            bgcolor="rgba(255,255,255,0.75)",
        )
    fig.update_layout(**placed)


def _comparison_plot(
    path: Path,
    trials: list[dict],
    flights: list[dict],
    step: str,
    flight,
    shown: int | None = None,
) -> None:
    """Interactive HTML. Click an RMS point to show that flight. Every sample is kept."""
    if shown is None:
        shown = len(trials) - 1
    series = flights[shown]
    chosen = trials[shown]
    titles = ["RMS sum"] + [label for _name, label in _PLOT_LABELS]
    fig = make_subplots(rows=len(titles), cols=1, subplot_titles=titles, vertical_spacing=0.04)
    fig.add_trace(
        go.Scatter(
            x=[item["trial"] for item in trials],
            y=[item["score"] for item in trials],
            mode="lines+markers",
            name="RMS sum",
            legend=_legend_name(1),
            marker={"size": 10},
            hovertemplate="trial %{x}<br>RMS %{y}<br>click to show this flight<extra></extra>",
        ),
        row=1,
        col=1,
    )
    fig.update_xaxes(title_text="trial", row=1, col=1)
    fig.update_yaxes(title_text="RMS sum", row=1, col=1)
    for row, (name, label) in enumerate(_PLOT_LABELS, start=2):
        logged = getattr(flight, name)
        airborne = np.asarray(logged.airborne, dtype=bool)
        legend = _legend_name(row)
        fig.add_trace(
            go.Scatter(
                x=np.asarray(logged.t_s, dtype=float)[airborne],
                y=np.asarray(logged.response, dtype=float)[airborne],
                mode="lines",
                name="target (ULG response)",
                legend=legend,
            ),
            row=row,
            col=1,
        )
        sim_t, sim = series.get(name, ([], []))
        fig.add_trace(
            go.Scatter(
                x=np.asarray(sim_t, dtype=float),
                y=np.asarray(sim, dtype=float),
                mode="lines",
                name="current (sim)",
                legend=legend,
            ),
            row=row,
            col=1,
        )
        fig.update_xaxes(title_text="time (s)", row=row, col=1)
        fig.update_yaxes(title_text=label, row=row, col=1)
    fig.update_layout(
        title=f"Target vs current — {step} — trial {chosen['trial']} — {Path(chosen['ulg']).name}",
        height=320 * len(titles),
        hovermode="x unified",
        dragmode="zoom",
    )
    _place_legends(fig, len(titles))
    html = fig.to_html(
        include_plotlyjs=True,
        full_html=True,
        div_id="comparison",
        config={"scrollZoom": True, "displaylogo": False},
    )
    html = html.replace("</body>", _cycle_script(trials, flights) + "\n</body>", 1)
    path.write_text(html, encoding="utf-8")


def _cycle_script(trials: list[dict], flights: list[dict]) -> str:
    """Swap the sim traces when an RMS point is clicked. Keys are trial numbers."""
    names = [name for name, _label in _PLOT_LABELS]
    cycles = {}
    for trial, series in zip(trials, flights):
        key = str(trial["trial"])
        cycles[key] = {}
        for name in names:
            times, values = series.get(name, ([], []))
            cycles[key][name] = {
                "t": [float(value) for value in times],
                "y": [float(value) for value in values],
            }
    payload = json.dumps({"names": names, "cycles": cycles})
    return f"""<script>
var CYCLE = {payload};
var comparison = document.getElementById("comparison");
comparison.on("plotly_click", function (event) {{
  var point = event.points && event.points[0];
  if (!point || point.curveNumber !== 0) return;
  var cycle = CYCLE.cycles[String(point.x)];
  if (!cycle) return;
  var idx = [];
  var xs = [];
  var ys = [];
  for (var i = 0; i < CYCLE.names.length; i++) {{
    var samples = cycle[CYCLE.names[i]];
    idx.push(2 * (i + 1));
    xs.push(samples.t);
    ys.push(samples.y);
  }}
  Plotly.restyle(comparison, {{x: xs, y: ys}}, idx);
}});
</script>"""
