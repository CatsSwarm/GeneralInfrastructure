#!/usr/bin/env python3
"""Crop hil_city.world to ~25% and write simple_hil.world.

Keeps downtown buildings/cars/actors. Drops city_terrain, ocean, sidewalk
polylines, and Prius. Shifts the x=-15 road onto the empty-world spawn
line (x=0) and drops z by 5.01 onto ground_plane.
"""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent
GI = ROOT.parent
SRC = GI / "city_assets" / "worlds" / "hil_city.world"
OUT = ROOT / "simple_hil.world"

# Downtown box ≈ 25% of include count. Shift so road_y_2 (x=-15) is x=0.
XMIN, XMAX = -42.0, 48.0
YMIN, YMAX = -42.0, 52.0
DX, DY, DZ = 15.0, 0.0, -5.01
SKIP_URI = {
    "model://city_terrain",
    "model://ocean",
    "model://pier",
    "model://prius_hybrid",
}


def _strip_comments(text: str) -> str:
    return re.sub(r"<!--.*?-->", "", text, flags=re.S)


def _in_actor(text: str, pos: int) -> bool:
    last_open = text.rfind("<actor", 0, pos)
    last_close = text.rfind("</actor>", 0, pos)
    return last_open > last_close


def _pose6(raw: str) -> list[float]:
    nums = [float(x) for x in raw.split()]
    while len(nums) < 6:
        nums.append(0.0)
    return nums[:6]


def _xf(x: float, y: float, z: float) -> tuple[float, float, float]:
    return x + DX, y + DY, z + DZ


def _in_box(x: float, y: float) -> bool:
    return XMIN <= x <= XMAX and YMIN <= y <= YMAX


def _rewrite_poses(xml: str) -> str:
    def repl(m: re.Match[str]) -> str:
        p = _pose6(m.group(1))
        x, y, z = _xf(p[0], p[1], p[2])
        return f"<pose>{x:.4f} {y:.4f} {z:.4f} {p[3]:.4f} {p[4]:.4f} {p[5]:.4f}</pose>"

    return re.sub(r"<pose>\s*([^<]+)</pose>", repl, xml, flags=re.S)


def _rewrite_actor_poses(xml: str) -> str:
    """Translate world poses. Leave nested include pose 0 0 0 (actor-local)."""

    def repl(m: re.Match[str]) -> str:
        p = _pose6(m.group(1))
        if abs(p[0]) < 0.05 and abs(p[1]) < 0.05 and abs(p[2]) < 0.05:
            return m.group(0)
        x, y, z = _xf(p[0], p[1], p[2])
        return f"<pose>{x:.4f} {y:.4f} {z:.4f} {p[3]:.4f} {p[4]:.4f} {p[5]:.4f}</pose>"

    return re.sub(r"<pose>\s*([^<]+)</pose>", repl, xml, flags=re.S)


def _clip_road(xml: str) -> str | None:
    pts = []
    for m in re.finditer(r"<point>\s*([^<]+)</point>", xml):
        nums = [float(x) for x in m.group(1).split()]
        pts.append(nums)
    if len(pts) < 2:
        return None
    clipped = []
    for n in pts:
        x, y = n[0], n[1]
        z = n[2] if len(n) > 2 else 5.02
        if _in_box(x, y):
            xx, yy, zz = _xf(x, y, z)
            clipped.append((xx, yy, zz))
        else:
            # Keep the in-box projection of long N/S or E/W roads.
            cx = min(max(x, XMIN), XMAX)
            cy = min(max(y, YMIN), YMAX)
            if abs(x - cx) < 1.0 or abs(y - cy) < 1.0:
                xx, yy, zz = _xf(cx, cy, z)
                clipped.append((xx, yy, zz))
    # unique consecutive
    out_pts: list[tuple[float, float, float]] = []
    for p in clipped:
        if not out_pts or (abs(p[0] - out_pts[-1][0]) > 0.05 or abs(p[1] - out_pts[-1][1]) > 0.05):
            out_pts.append(p)
    if len(out_pts) < 2:
        return None
    width_m = re.search(r"<width>\s*([^<]+)</width>", xml)
    name_m = re.search(r'<road name="([^"]+)"', xml)
    mat_m = re.search(r"(<material>.*?</material>)", xml, flags=re.S)
    name = name_m.group(1) if name_m else "road"
    width = width_m.group(1).strip() if width_m else "7.4"
    body = "\n".join(f"      <point>{x:.3f} {y:.3f} {z:.3f}</point>" for x, y, z in out_pts)
    mat = f"\n      {mat_m.group(1)}\n" if mat_m else ""
    return f'    <road name="{name}">\n      <width>{width}</width>\n{body}{mat}    </road>\n'


def _include_uri(xml: str) -> str:
    m = re.search(r"<uri>\s*([^<]+)</uri>", xml)
    return m.group(1).strip() if m else ""


def _include_xy(xml: str) -> tuple[float, float, float]:
    m = re.search(r"<pose>\s*([^<]+)</pose>", xml, flags=re.S)
    if not m:
        return 0.0, 0.0, 0.0
    p = _pose6(m.group(1))
    return p[0], p[1], p[2]


def main() -> None:
    if not SRC.is_file():
        raise SystemExit(f"missing {SRC} (run fetch_hil_city.sh)")
    raw = _strip_comments(SRC.read_text(encoding="utf-8"))

    includes: list[str] = []
    for m in re.finditer(r"<include>.*?</include>", raw, flags=re.S):
        if _in_actor(raw, m.start()):
            continue
        xml = m.group(0)
        uri = _include_uri(xml)
        if uri in SKIP_URI:
            continue
        x, y, _z = _include_xy(xml)
        if not _in_box(x, y):
            continue
        includes.append(_rewrite_poses(xml))

    roads: list[str] = []
    for m in re.finditer(r"<road\b.*?</road>", raw, flags=re.S):
        clipped = _clip_road(m.group(0))
        if clipped:
            roads.append(clipped)

    actors: list[str] = []
    for m in re.finditer(r"<actor\b.*?</actor>", raw, flags=re.S):
        xml = m.group(0)
        name_m = re.search(r'<actor name="([^"]+)"', xml)
        name = name_m.group(1) if name_m else ""
        if name.startswith("car_") and name not in ("car_626",):
            # One looping car on the pad road (original x=-15). Skip the rest.
            continue
        pm = re.search(r"<pose>\s*([^<]+)</pose>", xml, flags=re.S)
        if not pm:
            continue
        p = _pose6(pm.group(1))
        # Keep if start or any waypoint is in the box.
        wps = [_pose6(w) for w in re.findall(r"<pose>\s*([^<]+)</pose>", xml, flags=re.S)]
        if not any(_in_box(w[0], w[1]) for w in wps):
            continue
        actors.append(_rewrite_actor_poses(xml))

    people = [
        ('hil_person_1', 5.0, 1.5, 0.0, 1.57),
        ('hil_person_2', 5.5, 6.0, 0.0, 0.0),
        ('hil_person_3', 4.8, 10.5, 0.0, 3.14),
        ('hil_person_4', 6.2, 14.0, 0.0, -0.6),
    ]
    extra = []
    for name, x, y, z, yaw in people:
        extra.append(
            f"""    <include>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <uri>model://person_standing</uri>
    </include>"""
        )

    header = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <!-- 25% crop of hil_city downtown. Generated by crop_simple_hil.py.
       No city_terrain / sidewalk polylines / Prius. Road x=-15 shifted to x=0. -->
  <world name="simple_hil">
    <gui>
      <camera name="user_camera">
        <pose>-18 8 14 0 0.42 0</pose>
      </camera>
    </gui>
    <scene>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
      <ambient>0.592 0.624 0.635 1</ambient>
      <background>0.35 0.35 0.35 1.0</background>
      <shadows>false</shadows>
    </scene>
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -9.8066</gravity>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>47.397742</latitude_deg>
      <longitude_deg>8.545594</longitude_deg>
      <elevation>488.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <light type="directional" name="sun">
      <pose>0 0 1000 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.4</direction>
    </light>
    <include>
      <uri>model://ground_plane</uri>
    </include>
"""
    footer = "  </world>\n</sdf>\n"
    parts = [header, "    <!-- roads -->\n", *roads, "    <!-- downtown includes -->\n",
             *[f"    {b}\n" for b in includes], "    <!-- hailo sidewalk people -->\n",
             *[f"{b}\n" for b in extra], "    <!-- actors -->\n", *[f"    {a}\n" for a in actors],
             footer]
    OUT.write_text("".join(parts), encoding="utf-8")
    print(
        f"wrote {OUT} includes={len(includes)} roads={len(roads)} "
        f"actors={len(actors)} people={len(people)}"
    )


if __name__ == "__main__":
    main()
