#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.experiments import load_baselines
from mobile_ros.policies import NetworkSnapshot, build_stream_policy


HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MobileROS SLAM Dashboard</title>
<style>
:root {
  color-scheme: dark;
  --bg: #101314;
  --panel: #171c1f;
  --line: #2e3a40;
  --text: #e8edf0;
  --muted: #93a1a8;
  --green: #4fd18b;
  --amber: #f2b84b;
  --red: #f06c64;
  --blue: #6ab9ff;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--text); font-family: Inter, Segoe UI, Arial, sans-serif; }
main { display: grid; grid-template-columns: minmax(520px, 1fr) 360px; min-height: 100vh; }
#map { width: 100%; height: 100vh; display: block; background: #0d1011; }
aside { border-left: 1px solid var(--line); background: var(--panel); padding: 18px; overflow: auto; }
h1 { font-size: 18px; margin: 0 0 14px; font-weight: 650; letter-spacing: 0; }
.status { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.metric { border: 1px solid var(--line); padding: 10px; border-radius: 6px; min-height: 68px; }
.label { color: var(--muted); font-size: 12px; }
.value { font-size: 24px; margin-top: 6px; font-weight: 650; }
.unit { color: var(--muted); font-size: 12px; margin-left: 4px; }
.timeline { margin-top: 16px; display: grid; gap: 8px; }
.event { border-left: 3px solid var(--blue); padding: 8px 10px; background: #111619; font-size: 13px; color: var(--text); }
.mode { margin: 14px 0; padding: 10px; border: 1px solid var(--line); border-radius: 6px; color: var(--muted); }
.mode strong { color: var(--text); }
canvas.chart { width: 100%; height: 92px; margin-top: 12px; border: 1px solid var(--line); border-radius: 6px; background: #101517; }
@media (max-width: 860px) {
  main { grid-template-columns: 1fr; }
  #map { height: 62vh; }
  aside { border-left: 0; border-top: 1px solid var(--line); }
}
</style>
</head>
<body>
<main>
  <canvas id="map"></canvas>
  <aside>
    <h1>MobileROS SLAM</h1>
    <div class="status">
      <div class="metric"><div class="label">PRB</div><div class="value" id="prb">--<span class="unit">%</span></div></div>
      <div class="metric"><div class="label">Latency</div><div class="value" id="latency">--<span class="unit">ms</span></div></div>
      <div class="metric"><div class="label">Throughput</div><div class="value" id="throughput">--<span class="unit">Mbps</span></div></div>
      <div class="metric"><div class="label">Keyframes</div><div class="value" id="keyframes">--<span class="unit">Hz</span></div></div>
    </div>
    <div class="mode">Policy: <strong id="policy">waiting</strong></div>
    <canvas class="chart" id="chart"></canvas>
    <div class="timeline" id="events"></div>
  </aside>
</main>
<script>
const map = document.getElementById('map');
const ctx = map.getContext('2d');
const chart = document.getElementById('chart');
const cctx = chart.getContext('2d');
const state = { frames: [], latest: null };

function resize() {
  map.width = map.clientWidth * devicePixelRatio;
  map.height = map.clientHeight * devicePixelRatio;
  chart.width = chart.clientWidth * devicePixelRatio;
  chart.height = chart.clientHeight * devicePixelRatio;
}
addEventListener('resize', resize);
resize();

function xy(p) {
  const w = map.width, h = map.height;
  return [w * (0.12 + p.x * 0.76), h * (0.82 - p.y * 0.68)];
}

function drawGrid() {
  ctx.strokeStyle = '#1b2428';
  ctx.lineWidth = 1 * devicePixelRatio;
  for (let i = 0; i <= 10; i++) {
    const x = map.width * i / 10;
    const y = map.height * i / 10;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, map.height); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(map.width, y); ctx.stroke();
  }
}

function drawPath(points, color, width) {
  if (points.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width * devicePixelRatio;
  ctx.beginPath();
  points.forEach((p, i) => {
    const [x, y] = xy(p);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

function drawMap() {
  ctx.clearRect(0, 0, map.width, map.height);
  drawGrid();
  const frames = state.frames;
  const gt = frames.map(f => f.ground_truth);
  const est = frames.map(f => f.estimate);
  drawPath(gt, '#4fd18b', 2.5);
  drawPath(est, '#6ab9ff', 2.0);
  const latest = state.latest;
  if (!latest) return;
  ctx.fillStyle = latest.prb < 45 ? '#f06c64' : latest.prb < 75 ? '#f2b84b' : '#4fd18b';
  for (const point of latest.points) {
    const [x, y] = xy(point);
    ctx.globalAlpha = point.a;
    ctx.fillRect(x, y, 2.2 * devicePixelRatio, 2.2 * devicePixelRatio);
  }
  ctx.globalAlpha = 1;
  const [rx, ry] = xy(latest.estimate);
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.arc(rx, ry, 6 * devicePixelRatio, 0, Math.PI * 2);
  ctx.fill();
}

function drawChart() {
  cctx.clearRect(0, 0, chart.width, chart.height);
  const frames = state.frames.slice(-120);
  if (frames.length < 2) return;
  const series = [
    ['throughput', '#6ab9ff', 12],
    ['latency', '#f2b84b', 160],
    ['packet_loss', '#f06c64', 10]
  ];
  for (const [key, color, max] of series) {
    cctx.strokeStyle = color;
    cctx.lineWidth = 1.5 * devicePixelRatio;
    cctx.beginPath();
    frames.forEach((f, i) => {
      const x = chart.width * i / Math.max(1, frames.length - 1);
      const y = chart.height - (Math.min(max, f[key]) / max) * chart.height * 0.85 - chart.height * 0.075;
      if (i === 0) cctx.moveTo(x, y); else cctx.lineTo(x, y);
    });
    cctx.stroke();
  }
}

function updatePanel(f) {
  document.getElementById('prb').innerHTML = f.prb.toFixed(0) + '<span class="unit">%</span>';
  document.getElementById('latency').innerHTML = f.latency.toFixed(0) + '<span class="unit">ms</span>';
  document.getElementById('throughput').innerHTML = f.throughput.toFixed(1) + '<span class="unit">Mbps</span>';
  document.getElementById('keyframes').innerHTML = f.keyframe_rate.toFixed(2) + '<span class="unit">Hz</span>';
  document.getElementById('policy').textContent = `${f.quality} / ${f.slice_action} / q${f.jpeg_quality}`;
  if (f.event) {
    const e = document.createElement('div');
    e.className = 'event';
    e.textContent = f.event;
    const list = document.getElementById('events');
    list.prepend(e);
    while (list.children.length > 8) list.removeChild(list.lastChild);
  }
}

const events = new EventSource('/events');
events.onmessage = (msg) => {
  const f = JSON.parse(msg.data);
  state.latest = f;
  state.frames.push(f);
  if (state.frames.length > 300) state.frames.shift();
  updatePanel(f);
  drawMap();
  drawChart();
};
</script>
</body>
</html>
"""


def load_conditions() -> List[Dict[str, float]]:
    baselines = load_baselines()
    rows = baselines["table_iii_orb_slam3_prb"]["rows"]
    return [
        {
            "name": "low PRB",
            "prb": 30.0,
            "throughput": rows["low_prb_30"]["link_rate_mbps"][0],
            "latency": rows["low_prb_30"]["ul_latency_ms"][0],
            "packet_loss": rows["low_prb_30"]["packet_loss_rate_pct"][0],
            "jitter": rows["low_prb_30"]["jitter_ms"][0],
            "pose_error": rows["low_prb_30"]["mean_abs_pose_error_m"][0],
            "keyframe_rate": rows["low_prb_30"]["keyframe_rate_hz"][0],
        },
        {
            "name": "medium PRB",
            "prb": 60.0,
            "throughput": rows["medium_prb_60"]["link_rate_mbps"][0],
            "latency": rows["medium_prb_60"]["ul_latency_ms"][0],
            "packet_loss": rows["medium_prb_60"]["packet_loss_rate_pct"][0],
            "jitter": rows["medium_prb_60"]["jitter_ms"][0],
            "pose_error": rows["medium_prb_60"]["mean_abs_pose_error_m"][0],
            "keyframe_rate": rows["medium_prb_60"]["keyframe_rate_hz"][0],
        },
        {
            "name": "high PRB",
            "prb": 90.0,
            "throughput": rows["high_prb_90"]["link_rate_mbps"][0],
            "latency": rows["high_prb_90"]["ul_latency_ms"][0],
            "packet_loss": rows["high_prb_90"]["packet_loss_rate_pct"][0],
            "jitter": rows["high_prb_90"]["jitter_ms"][0],
            "pose_error": rows["high_prb_90"]["mean_abs_pose_error_m"][0],
            "keyframe_rate": rows["high_prb_90"]["keyframe_rate_hz"][0],
        },
    ]


def frame_at(step: int) -> Dict[str, object]:
    conditions = load_conditions()
    segment = (step // 80) % len(conditions)
    c = conditions[segment]
    t = step / 35.0
    x = (step % 260) / 260.0
    y = 0.46 + 0.28 * math.sin(t * 0.75)
    drift = c["pose_error"] * 0.035
    estimate = {"x": x + drift * math.sin(t * 1.7), "y": y + drift * math.cos(t * 1.3)}
    ground_truth = {"x": x, "y": y}
    density = int(35 + c["prb"] * 1.1)
    points = []
    for i in range(density):
        angle = (i * 2.399 + t) % (math.pi * 2)
        radius = 0.015 + (i % 19) * 0.0035
        points.append(
            {
                "x": max(0.02, min(0.98, estimate["x"] + math.cos(angle) * radius)),
                "y": max(0.05, min(0.95, estimate["y"] + math.sin(angle) * radius)),
                "a": 0.25 + min(0.65, c["prb"] / 120.0),
            }
        )

    snapshot = NetworkSnapshot(
        throughput_mbps=c["throughput"],
        latency_ms=c["latency"],
        packet_loss_pct=c["packet_loss"],
        jitter_ms=c["jitter"],
        prb_allocation_pct=c["prb"],
    )
    policy = build_stream_policy(snapshot, task_criticality=0.85 if segment == 0 and step % 80 > 45 else 0.45)
    event = ""
    if step % 80 == 0:
        event = f"{c['name']} condition: throughput {c['throughput']} Mbps, latency {c['latency']} ms"
    if segment == 0 and step % 80 == 46:
        event = "slice promote requested for SLAM continuity"

    return {
        "step": step,
        "ground_truth": ground_truth,
        "estimate": estimate,
        "points": points,
        "prb": c["prb"],
        "throughput": c["throughput"],
        "latency": c["latency"],
        "packet_loss": c["packet_loss"],
        "keyframe_rate": c["keyframe_rate"],
        "quality": policy.quality.value,
        "slice_action": policy.slice_action.value,
        "jpeg_quality": policy.jpeg_quality,
        "event": event,
    }


class Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/" or self.path.startswith("/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/events"):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            step = 0
            while True:
                payload = json.dumps(frame_at(step), separators=(",", ":"))
                try:
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break
                step += 1
                time.sleep(0.08)
            return
        self.send_error(404)

    def log_message(self, fmt: str, *args: object) -> None:
        return


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the MobileROS SLAM/network dashboard")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    try:
        print(f"dashboard http://{args.host}:{args.port}", flush=True)
    except OSError:
        pass
    server.serve_forever()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        error_path = Path(__file__).resolve().with_name("dashboard_error.log")
        error_path.write_text(f"{type(exc).__name__}: {exc}\n", encoding="utf-8")
        raise
