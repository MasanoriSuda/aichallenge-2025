"""Self-contained JSON and HTML report generation."""

from __future__ import annotations

from html import escape
import json
import math
from pathlib import Path
from typing import Any

from .models import RunAnalysis


COLORS = {
    "Trajectory CSV": "#94a3b8",
    "Runtime trajectory": "#f59e0b",
    "EKF": "#2563eb",
    "GNSS": "#dc2626",
    "Baseline EKF": "#2563eb",
    "Candidate EKF": "#16a34a",
    "Baseline GNSS": "#7c3aed",
    "Candidate GNSS": "#dc2626",
    "Vehicle speed": "#2563eb",
    "Twist speed": "#7c3aed",
    "EKF speed": "#16a34a",
    "Command speed": "#64748b",
    "IMU raw": "#dc2626",
    "IMU corrected": "#f59e0b",
    "Vehicle heading rate": "#2563eb",
    "EKF yaw rate": "#16a34a",
    "Actual steering": "#dc2626",
    "Command steering": "#2563eb",
    "GNSS covariance X": "#2563eb",
    "GNSS covariance Y": "#dc2626",
}

METRIC_LABELS = {
    "actual_speed_mean_mps": "Actual speed mean [m/s]",
    "actual_speed_p95_mps": "Actual speed P95 [m/s]",
    "actual_speed_max_mps": "Actual speed max [m/s]",
    "ekf_cross_track_abs_mean_m": "EKF cross-track |mean| [m]",
    "ekf_cross_track_abs_p95_m": "EKF cross-track |P95| [m]",
    "ekf_cross_track_abs_max_m": "EKF cross-track |max| [m]",
    "ekf_yaw_abs_p95_deg": "EKF yaw error |P95| [deg]",
    "gnss_cross_track_abs_p95_m": "GNSS cross-track |P95| [m]",
    "gnss_ekf_distance_p95_m": "GNSS-EKF distance P95 [m]",
    "stationary_speed_abs_p95_mps": "Stationary speed |P95| [m/s]",
    "stationary_imu_yaw_rate_std_radps": "Stationary IMU yaw-rate stddev [rad/s]",
    "vehicle_ekf_speed_difference_p95_mps": (
        "Vehicle-EKF speed difference P95 [m/s]"
    ),
    "imu_vehicle_yaw_rate_difference_p95_radps": (
        "IMU-vehicle yaw-rate difference P95 [rad/s]"
    ),
    "steering_tracking_difference_p95_rad": (
        "Actual-command steering difference P95 [rad]"
    ),
}


def _format(value: Any, digits: int = 4) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        if not math.isfinite(value):
            return "N/A"
        return f"{value:.{digits}f}"
    if isinstance(value, (dict, list)):
        return escape(json.dumps(value, ensure_ascii=False))
    return escape(str(value))


def _downsample(points: list[list[float]], maximum: int = 1800) -> list[list[float]]:
    if len(points) <= maximum:
        return points
    step = max(1, len(points) // maximum)
    sampled = points[::step]
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def _svg_chart(
    series: list[tuple[str, list[list[float]]]],
    *,
    x_label: str,
    y_label: str,
    equal_axes: bool = False,
) -> str:
    valid_series = [
        (name, _downsample(points))
        for name, points in series
        if points
        and all(
            len(point) >= 2
            and math.isfinite(float(point[0]))
            and math.isfinite(float(point[1]))
            for point in points
        )
    ]
    if not valid_series:
        return '<div class="empty">No data available</div>'

    all_x = [float(point[0]) for _, points in valid_series for point in points]
    all_y = [float(point[1]) for _, points in valid_series for point in points]
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    if x_max - x_min < 1e-9:
        x_min -= 0.5
        x_max += 0.5
    if y_max - y_min < 1e-9:
        y_min -= 0.5
        y_max += 0.5
    if equal_axes:
        span = max(x_max - x_min, y_max - y_min)
        x_mid = (x_min + x_max) * 0.5
        y_mid = (y_min + y_max) * 0.5
        x_min, x_max = x_mid - span * 0.5, x_mid + span * 0.5
        y_min, y_max = y_mid - span * 0.5, y_mid + span * 0.5

    width, height = 960.0, 500.0
    left, right, top, bottom = 72.0, 24.0, 30.0, 58.0
    plot_width = width - left - right
    plot_height = height - top - bottom

    def sx(value: float) -> float:
        return left + (value - x_min) / (x_max - x_min) * plot_width

    def sy(value: float) -> float:
        return top + (y_max - value) / (y_max - y_min) * plot_height

    chunks = [
        f'<svg class="chart" viewBox="0 0 {width:.0f} {height:.0f}" role="img">',
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" '
        'fill="none" stroke="#cbd5e1"/>',
    ]
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        x = left + plot_width * fraction
        y = top + plot_height * fraction
        x_value = x_min + (x_max - x_min) * fraction
        y_value = y_max - (y_max - y_min) * fraction
        chunks.extend(
            [
                f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_height}" '
                'stroke="#e2e8f0"/>',
                f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" '
                'stroke="#e2e8f0"/>',
                f'<text x="{x:.2f}" y="{height - 32}" text-anchor="middle">{x_value:.2f}</text>',
                f'<text x="{left - 10}" y="{y + 4:.2f}" text-anchor="end">{y_value:.2f}</text>',
            ]
        )
    for name, points in valid_series:
        coordinates = " ".join(
            f"{sx(float(point[0])):.2f},{sy(float(point[1])):.2f}" for point in points
        )
        color = COLORS.get(name, "#475569")
        chunks.append(
            f'<polyline points="{coordinates}" fill="none" stroke="{color}" '
            'stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>'
        )
    chunks.extend(
        [
            f'<text x="{left + plot_width / 2}" y="{height - 7}" text-anchor="middle">'
            f"{escape(x_label)}</text>",
            f'<text x="18" y="{top + plot_height / 2}" text-anchor="middle" '
            f'transform="rotate(-90 18 {top + plot_height / 2})">{escape(y_label)}</text>',
            "</svg>",
            '<div class="legend">',
        ]
    )
    for name, _ in valid_series:
        chunks.append(
            f'<span><i style="background:{COLORS.get(name, "#475569")}"></i>'
            f"{escape(name)}</span>"
        )
    chunks.append("</div>")
    return "".join(chunks)


def _base_html(title: str, body: str) -> str:
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{escape(title)}</title>
<style>
:root {{ color-scheme: light dark; font-family: system-ui, sans-serif; }}
body {{ margin: 0; background: #f8fafc; color: #0f172a; }}
main {{ max-width: 1180px; margin: 0 auto; padding: 24px; }}
h1 {{ margin-bottom: 4px; }}
h2 {{ margin-top: 30px; }}
.muted {{ color: #64748b; }}
.grid {{ display: grid; grid-template-columns: repeat(auto-fit,minmax(190px,1fr)); gap: 12px; }}
.card, section {{ background: white; border: 1px solid #e2e8f0; border-radius: 10px;
  padding: 16px; box-shadow: 0 1px 2px rgba(15,23,42,.04); }}
.card .value {{ font-size: 1.45rem; font-weight: 650; }}
section {{ margin-top: 14px; overflow-x: auto; }}
table {{ border-collapse: collapse; width: 100%; }}
th, td {{ border-bottom: 1px solid #e2e8f0; padding: 8px; text-align: left; }}
th {{ background: #f1f5f9; }}
.chart {{ width: 100%; min-width: 680px; height: auto; }}
.chart text {{ font-size: 12px; fill: #64748b; }}
.legend {{ display:flex; gap:14px; flex-wrap:wrap; margin:6px 0 4px; }}
.legend i {{ display:inline-block; width:18px; height:3px; margin-right:6px;
  vertical-align:middle; }}
.empty {{ padding: 36px; text-align:center; color:#64748b; background:#f8fafc; }}
.warning {{ border-left: 4px solid #f59e0b; padding: 8px 12px; background:#fffbeb; margin:8px 0; }}
.改善 {{ color:#15803d; font-weight:650; }}
.悪化 {{ color:#b91c1c; font-weight:650; }}
.実質差なし {{ color:#475569; font-weight:650; }}
.判定不能 {{ color:#a16207; font-weight:650; }}
pre {{ white-space:pre-wrap; word-break:break-word; font-size:12px; }}
details {{ margin-top:10px; }}
.controls {{ display:flex; flex-wrap:wrap; gap:12px; align-items:end; }}
.controls label {{ display:flex; flex-direction:column; gap:4px; font-weight:600; }}
button, select {{ font:inherit; padding:7px 10px; border:1px solid #94a3b8;
  border-radius:7px; background:white; color:#0f172a; }}
button.active {{ background:#2563eb; color:white; border-color:#2563eb; }}
@media (prefers-color-scheme: dark) {{
  body {{ background:#0f172a; color:#e2e8f0; }}
  .card, section {{ background:#111827; border-color:#334155; }}
  th {{ background:#1e293b; }} th, td {{ border-color:#334155; }}
  .warning {{ background:#422006; }}
  .chart text {{ fill:#94a3b8; }}
  button, select {{ background:#1e293b; color:#e2e8f0; }}
}}
</style>
</head>
<body><main>{body}</main></body>
</html>
"""


def _warning_block(warnings: list[str]) -> str:
    if not warnings:
        return ""
    return "<section><h2>Warnings</h2>" + "".join(
        f'<div class="warning">{escape(item)}</div>' for item in warnings
    ) + "</section>"


def single_html(analysis: RunAnalysis) -> str:
    summary = analysis.summary
    metrics = summary["metrics"]
    cards = []
    for key in (
        "actual_speed_p95_mps",
        "ekf_cross_track_abs_p95_m",
        "gnss_cross_track_abs_p95_m",
        "gnss_ekf_distance_p95_m",
    ):
        cards.append(
            '<div class="card"><div class="muted">'
            + escape(METRIC_LABELS[key])
            + '</div><div class="value">'
            + _format(metrics.get(key))
            + "</div></div>"
        )

    topic_rows = "".join(
        "<tr>"
        f"<td>{escape(row['key'])}</td><td>{escape(row['topic'])}</td>"
        f"<td>{_format(row['type'])}</td><td>{row['count']}</td>"
        f"<td>{_format(row['rate_hz'], 2)}</td>"
        f"<td>{_format(row['period_stddev_sec'], 5)}</td>"
        f"<td>{_format(row['max_gap_sec'], 5)}</td>"
        f"<td>{_format(row['record_source_offset_stddev_sec'], 5)}</td>"
        f"<td>{'available' if row['available'] else 'missing'}</td></tr>"
        for row in summary["topics"]
    )
    speed_rows = "".join(
        "<tr>"
        f"<td>{escape(row['name'])}</td><td>{_format(row['lower_mps'])}</td>"
        f"<td>{_format(row['upper_mps'])}</td><td>{row['sample_count']}</td>"
        f"<td>{_format(row['cross_track_abs_p95_m'])}</td>"
        f"<td>{_format(row['yaw_abs_p95_deg'])}</td></tr>"
        for row in summary["speed_bands"]
    )
    metric_rows = "".join(
        f"<tr><td>{escape(METRIC_LABELS.get(key, key))}</td><td>{_format(value)}</td></tr>"
        for key, value in metrics.items()
    )
    plots = analysis.plots
    body = f"""
<h1>Localization Scope</h1>
<p class="muted">{escape(summary['display_name'])} · single-run report</p>
<div class="grid">{''.join(cards)}</div>
{_warning_block(analysis.warnings)}
<section><h2>Trajectory overlay</h2>
{_svg_chart([
    ("Trajectory CSV", plots["trajectory_xy"]),
    ("Runtime trajectory", plots["runtime_trajectory_xy"]),
    ("EKF", plots["ekf_xy"]),
    ("GNSS", plots["gnss_xy"]),
], x_label="map x [m]", y_label="map y [m]", equal_axes=True)}</section>
<section><h2>Speed</h2>
{_svg_chart([
    ("Vehicle speed", plots["vehicle_speed_time"]),
    ("Twist speed", plots["twist_speed_time"]),
    ("EKF speed", plots["ekf_speed_time"]),
    ("Command speed", plots["command_speed_time"]),
],
            x_label="bag time [s]", y_label="speed [m/s]")}</section>
<section><h2>Yaw rate</h2>
{_svg_chart([
    ("IMU raw", plots["imu_raw_yaw_rate_time"]),
    ("IMU corrected", plots["imu_corrected_yaw_rate_time"]),
    ("Vehicle heading rate", plots["vehicle_yaw_rate_time"]),
    ("EKF yaw rate", plots["ekf_yaw_rate_time"]),
], x_label="bag time [s]", y_label="yaw rate [rad/s]")}</section>
<section><h2>Steering tracking</h2>
{_svg_chart([
    ("Actual steering", plots["actual_steering_time"]),
    ("Command steering", plots["command_steering_time"]),
], x_label="bag time [s]", y_label="steering tire angle [rad]")}</section>
<section><h2>GNSS covariance</h2>
{_svg_chart([
    ("GNSS covariance X", plots["gnss_cov_x_time"]),
    ("GNSS covariance Y", plots["gnss_cov_y_time"]),
], x_label="bag time [s]", y_label="position covariance [m²]")}</section>
<section><h2>Cross-track error</h2>
{_svg_chart([
    ("EKF", plots["ekf_cross_track"]),
    ("GNSS", plots["gnss_cross_track"]),
], x_label="trajectory s [m]", y_label="signed cross-track [m]")}</section>
<section><h2>GNSS-EKF distance</h2>
{_svg_chart([("GNSS", plots["gnss_ekf_distance"])],
            x_label="bag time [s]", y_label="distance [m]")}</section>
<section><h2>Metrics</h2><table><tbody>{metric_rows}</tbody></table></section>
<section><h2>Speed bands</h2><table><thead><tr><th>Band</th><th>Lower [m/s]</th>
<th>Upper [m/s]</th><th>Samples</th><th>Cross-track P95 [m]</th>
<th>Yaw P95 [deg]</th></tr></thead><tbody>{speed_rows}</tbody></table></section>
<section><h2>Topics</h2><table><thead><tr><th>Role</th><th>Topic</th><th>Type</th>
<th>Count</th><th>Rate [Hz]</th><th>Period std [s]</th><th>Max gap [s]</th>
<th>Stamp offset std [s]</th><th>Status</th></tr></thead>
<tbody>{topic_rows}</tbody></table></section>
<section><h2>Manifest</h2><details><summary>Show resolved inputs</summary>
<pre>{escape(json.dumps(analysis.manifest, indent=2, ensure_ascii=False))}</pre>
</details></section>
"""
    return _base_html(f"Localization Scope - {summary['display_name']}", body)


def comparison_html(
    baseline: RunAnalysis,
    candidate: RunAnalysis,
    comparison: dict[str, Any],
) -> str:
    condition_rows = "".join(
        "<tr>"
        f"<td>{escape(row['condition'])}</td><td>{_format(row['baseline'])}</td>"
        f"<td>{_format(row['candidate'])}</td>"
        f"<td>{'same' if row['same'] else 'changed'}</td></tr>"
        for row in comparison["conditions"]
    )
    metric_rows = "".join(
        "<tr>"
        f"<td>{escape(METRIC_LABELS.get(row['metric'], row['metric']))}</td>"
        f"<td>{_format(row['baseline'])}</td><td>{_format(row['candidate'])}</td>"
        f"<td>{_format(row['delta'])}</td><td>{_format(row['tolerance'])}</td>"
        f"<td class=\"{row['verdict']}\">{row['verdict']}</td></tr>"
        for row in comparison["metrics"]
    )
    warnings = comparison["warnings"] + baseline.warnings + candidate.warnings
    baseline_plots = baseline.plots
    candidate_plots = candidate.plots
    body = f"""
<h1>Localization Scope Comparison</h1>
<p class="muted">Baseline: {escape(comparison['baseline'])}<br>
Candidate: {escape(comparison['candidate'])}</p>
{_warning_block(warnings)}
<section><h2>Comparison conditions</h2><table><thead><tr><th>Condition</th>
<th>Baseline</th><th>Candidate</th><th>Status</th></tr></thead>
<tbody>{condition_rows}</tbody></table></section>
<section><h2>Metric deltas</h2><table><thead><tr><th>Metric</th><th>Baseline</th>
<th>Candidate</th><th>Delta</th><th>Tolerance</th><th>Verdict</th></tr></thead>
<tbody>{metric_rows}</tbody></table></section>
<section><h2>Trajectory overlay</h2>
{_svg_chart([
    ("Trajectory CSV", baseline_plots["trajectory_xy"]),
    ("Baseline EKF", baseline_plots["ekf_xy"]),
    ("Candidate EKF", candidate_plots["ekf_xy"]),
    ("Baseline GNSS", baseline_plots["gnss_xy"]),
    ("Candidate GNSS", candidate_plots["gnss_xy"]),
], x_label="map x [m]", y_label="map y [m]", equal_axes=True)}</section>
<section><h2>EKF cross-track comparison</h2>
{_svg_chart([
    ("Baseline EKF", baseline_plots["ekf_cross_track"]),
    ("Candidate EKF", candidate_plots["ekf_cross_track"]),
], x_label="trajectory s [m]", y_label="signed cross-track [m]")}</section>
"""
    return _base_html(
        f"Localization Scope - {comparison['baseline']} vs {comparison['candidate']}",
        body,
    )


def _safe_script_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).replace(
        "</", "<\\/"
    )


def catalog_html(
    runs: list[RunAnalysis],
    comparisons: dict[str, dict[str, Any]],
) -> str:
    """Build a browser selector for single-run and two-run views."""

    payload = [
        {
            "id": str(index),
            "manifest": run.manifest,
            "summary": run.summary,
            "plots": run.plots,
        }
        for index, run in enumerate(runs)
    ]
    labels = _safe_script_json(METRIC_LABELS)
    data = _safe_script_json(payload)
    pair_data = _safe_script_json(comparisons)
    body = f"""
<h1>Localization Scope Catalog</h1>
<p class="muted">Select one run or compare a Baseline and Candidate.</p>
<section>
  <div class="controls">
    <div>
      <button id="single-mode" class="active" type="button">Single</button>
      <button id="compare-mode" type="button">Baseline vs Candidate</button>
    </div>
    <label>Run / Baseline<select id="baseline"></select></label>
    <label id="candidate-label" hidden>
      Candidate<select id="candidate"></select>
    </label>
  </div>
</section>
<div id="catalog-view"></div>
<script>
const runs = {data};
const comparisons = {pair_data};
const metricLabels = {labels};
const colors = {{
  trajectory: "#94a3b8",
  runtime: "#f59e0b",
  baseline: "#2563eb",
  candidate: "#16a34a",
  gnss: "#dc2626"
}};
let mode = "single";
const baselineSelect = document.getElementById("baseline");
const candidateSelect = document.getElementById("candidate");
const candidateLabel = document.getElementById("candidate-label");
const view = document.getElementById("catalog-view");

function formatValue(value, digits = 4) {{
  if (value === null || value === undefined) return "N/A";
  if (typeof value === "number") {{
    if (!Number.isFinite(value)) return "N/A";
    return value.toFixed(digits);
  }}
  if (typeof value === "object") return JSON.stringify(value);
  return String(value);
}}

function escapeHtml(value) {{
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}}

function options() {{
  return runs.map(run => {{
    const label = run.summary.display_name || `Run ${{run.id}}`;
    return `<option value="${{run.id}}">${{escapeHtml(label)}}</option>`;
  }}).join("");
}}

function svgChart(series) {{
  const usable = series.filter(item => item.points && item.points.length);
  if (!usable.length) return '<div class="empty">No data available</div>';
  const points = usable.flatMap(item => item.points);
  const xs = points.map(point => Number(point[0]));
  const ys = points.map(point => Number(point[1]));
  let xMin = Math.min(...xs), xMax = Math.max(...xs);
  let yMin = Math.min(...ys), yMax = Math.max(...ys);
  let span = Math.max(xMax - xMin, yMax - yMin, 1e-6);
  const xMid = (xMin + xMax) / 2;
  const yMid = (yMin + yMax) / 2;
  xMin = xMid - span / 2;
  xMax = xMid + span / 2;
  yMin = yMid - span / 2;
  yMax = yMid + span / 2;
  const width = 960, height = 500;
  const left = 55, top = 20, plotWidth = 875, plotHeight = 430;
  const sx = x => left + (x - xMin) / (xMax - xMin) * plotWidth;
  const sy = y => top + (yMax - y) / (yMax - yMin) * plotHeight;
  const paths = usable.map(item => {{
    const step = Math.max(1, Math.floor(item.points.length / 1800));
    const sampled = item.points.filter((_, index) => index % step === 0);
    const coords = sampled.map(point =>
      `${{sx(Number(point[0])).toFixed(2)}},` +
      `${{sy(Number(point[1])).toFixed(2)}}`
    ).join(" ");
    return `<polyline points="${{coords}}" fill="none" ` +
      `stroke="${{item.color}}" stroke-width="2"/>`;
  }}).join("");
  const legend = usable.map(item =>
    `<span><i style="background:${{item.color}}"></i>` +
    `${{escapeHtml(item.name)}}</span>`
  ).join("");
  return `<svg class="chart" viewBox="0 0 ${{width}} ${{height}}">` +
    `<rect x="${{left}}" y="${{top}}" width="${{plotWidth}}" ` +
    `height="${{plotHeight}}" fill="none" stroke="#cbd5e1"/>` +
    paths + "</svg>" + `<div class="legend">${{legend}}</div>`;
}}

function warnings(items) {{
  if (!items || !items.length) return "";
  return "<section><h2>Warnings</h2>" + items.map(item =>
    `<div class="warning">${{escapeHtml(item)}}</div>`
  ).join("") + "</section>";
}}

function metricTable(metrics) {{
  const rows = Object.entries(metrics).map(([key, value]) =>
    `<tr><td>${{escapeHtml(metricLabels[key] || key)}}</td>` +
    `<td>${{formatValue(value)}}</td></tr>`
  ).join("");
  return `<section><h2>Metrics</h2><table>${{rows}}</table></section>`;
}}

function renderSingle() {{
  const run = runs[Number(baselineSelect.value)];
  const plots = run.plots;
  view.innerHTML =
    `<h2>${{escapeHtml(run.summary.display_name)}}</h2>` +
    warnings(run.summary.warnings) +
    `<section><h2>Trajectory overlay</h2>${{svgChart([
      {{name:"Trajectory CSV", points:plots.trajectory_xy,
        color:colors.trajectory}},
      {{name:"Runtime trajectory", points:plots.runtime_trajectory_xy,
        color:colors.runtime}},
      {{name:"EKF", points:plots.ekf_xy, color:colors.baseline}},
      {{name:"GNSS", points:plots.gnss_xy, color:colors.gnss}}
    ])}}</section>` +
    metricTable(run.summary.metrics);
}}

function renderComparison() {{
  const baselineId = baselineSelect.value;
  const candidateId = candidateSelect.value;
  if (baselineId === candidateId) {{
    view.innerHTML =
      '<div class="warning">Select two different runs.</div>';
    return;
  }}
  const baseline = runs[Number(baselineId)];
  const candidate = runs[Number(candidateId)];
  const comparison = comparisons[`${{baselineId}}:${{candidateId}}`];
  if (!comparison) {{
    view.innerHTML = '<div class="warning">Comparison is unavailable.</div>';
    return;
  }}
  const conditionRows = comparison.conditions.map(row =>
    `<tr><td>${{escapeHtml(row.condition)}}</td>` +
    `<td>${{escapeHtml(formatValue(row.baseline))}}</td>` +
    `<td>${{escapeHtml(formatValue(row.candidate))}}</td>` +
    `<td>${{row.same ? "same" : "changed"}}</td></tr>`
  ).join("");
  const metricRows = comparison.metrics.map(row =>
    `<tr><td>${{escapeHtml(metricLabels[row.metric] || row.metric)}}</td>` +
    `<td>${{formatValue(row.baseline)}}</td>` +
    `<td>${{formatValue(row.candidate)}}</td>` +
    `<td>${{formatValue(row.delta)}}</td>` +
    `<td class="${{row.verdict}}">${{row.verdict}}</td></tr>`
  ).join("");
  view.innerHTML =
    `<h2>${{escapeHtml(comparison.baseline)}} vs ` +
    `${{escapeHtml(comparison.candidate)}}</h2>` +
    warnings(comparison.warnings) +
    `<section><h2>Conditions</h2><table><thead><tr>` +
    `<th>Condition</th><th>Baseline</th><th>Candidate</th><th>Status</th>` +
    `</tr></thead><tbody>${{conditionRows}}</tbody></table></section>` +
    `<section><h2>Metric deltas</h2><table><thead><tr>` +
    `<th>Metric</th><th>Baseline</th><th>Candidate</th><th>Delta</th>` +
    `<th>Verdict</th></tr></thead><tbody>${{metricRows}}</tbody></table>` +
    `</section><section><h2>Trajectory overlay</h2>${{svgChart([
      {{name:"Trajectory CSV", points:baseline.plots.trajectory_xy,
        color:colors.trajectory}},
      {{name:"Baseline EKF", points:baseline.plots.ekf_xy,
        color:colors.baseline}},
      {{name:"Candidate EKF", points:candidate.plots.ekf_xy,
        color:colors.candidate}}
    ])}}</section>`;
}}

function render() {{
  if (mode === "single") renderSingle();
  else renderComparison();
}}

baselineSelect.innerHTML = options();
candidateSelect.innerHTML = options();
if (runs.length > 1) candidateSelect.value = "1";
document.getElementById("single-mode").addEventListener("click", () => {{
  mode = "single";
  candidateLabel.hidden = true;
  document.getElementById("single-mode").classList.add("active");
  document.getElementById("compare-mode").classList.remove("active");
  render();
}});
document.getElementById("compare-mode").addEventListener("click", () => {{
  mode = "compare";
  candidateLabel.hidden = false;
  document.getElementById("compare-mode").classList.add("active");
  document.getElementById("single-mode").classList.remove("active");
  render();
}});
baselineSelect.addEventListener("change", render);
candidateSelect.addEventListener("change", render);
render();
</script>
"""
    return _base_html("Localization Scope Catalog", body)


def write_single_report(analysis: RunAnalysis, output_dir: Path) -> tuple[Path, Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output_dir / "run-manifest.json"
    summary_path = output_dir / "summary.json"
    report_path = output_dir / "report.html"
    manifest_path.write_text(
        json.dumps(analysis.manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    summary_path.write_text(
        json.dumps(analysis.summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    report_path.write_text(single_html(analysis), encoding="utf-8")
    return manifest_path, summary_path, report_path


def write_comparison_report(
    baseline: RunAnalysis,
    candidate: RunAnalysis,
    comparison: dict[str, Any],
    output_dir: Path,
) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "comparison-summary.json"
    report_path = output_dir / "comparison.html"
    summary_path.write_text(
        json.dumps(comparison, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    report_path.write_text(
        comparison_html(baseline, candidate, comparison),
        encoding="utf-8",
    )
    return summary_path, report_path


def write_catalog(
    runs: list[RunAnalysis],
    comparisons: dict[str, dict[str, Any]],
    output_dir: Path,
) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    index_path = output_dir / "runs-index.json"
    catalog_path = output_dir / "catalog.html"
    index = {
        "schema_version": "1.0",
        "runs": [
            {"manifest": run.manifest, "summary": run.summary} for run in runs
        ],
        "comparisons": comparisons,
    }
    index_path.write_text(
        json.dumps(index, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    catalog_path.write_text(catalog_html(runs, comparisons), encoding="utf-8")
    return index_path, catalog_path
