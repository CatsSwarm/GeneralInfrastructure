#!/usr/bin/env bash
# Download OSRF citysim + gazebo_models it includes (Gazebo Classic 11).
# Host-only. Output: multidrone/city_assets/ (gitignored).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASSETS="${ROOT}/city_assets"
MODELS="${ASSETS}/models"
WORLDS="${ASSETS}/worlds"
SHARE="${ASSETS}/share"
CACHE="${ROOT}/.city_cache"
mkdir -p "${MODELS}" "${WORLDS}" "${SHARE}" "${CACHE}"

MODELS_NEEDED=(
  ambulance apartment asphalt_plane bus cafe_table cardboard_box
  cinder_block cinder_block_2 collapsed_industrial construction_barrel
  construction_cone drc_practice_angled_barrier_45
  drc_practice_blue_cylinder drc_practice_orange_jersey_barrier
  drc_practice_white_jersey_barrier dumpster fast_food fire_hydrant
  fire_station fire_truck fountain gas_station gazebo grocery_store
  hatchback hatchback_blue hatchback_red house_1 house_2 house_3
  lamp_post law_office oak_tree osrf_first_office parking_garage
  person_standing person_walking pier pine_tree playground police_station
  post_office postbox powerplant prius_hybrid radio_tower reactor
  robocup_3Dsim_field salon school stop_light stop_light_post stop_sign suv
  telephone_pole thrift_shop tower_crane truss_bridge pickup
)

if [ ! -d "${CACHE}/gazebo_models/.git" ]; then
  git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/osrf/gazebo_models.git "${CACHE}/gazebo_models"
fi
(
  cd "${CACHE}/gazebo_models"
  git sparse-checkout set --no-cone "${MODELS_NEEDED[@]}" || \
    git sparse-checkout set "${MODELS_NEEDED[@]}"
  git checkout
)

if [ ! -d "${CACHE}/citysim/.git" ]; then
  git clone --depth 1 https://github.com/osrf/citysim.git "${CACHE}/citysim"
fi

echo "Copying gazebo_models …"
missing=0
for m in "${MODELS_NEEDED[@]}"; do
  if [ -d "${CACHE}/gazebo_models/${m}" ]; then
    rm -rf "${MODELS}/${m}"
    cp -a "${CACHE}/gazebo_models/${m}" "${MODELS}/${m}"
    echo "  ${m}"
  else
    echo "  MISSING ${m}"
    missing=$((missing + 1))
  fi
done

echo "Copying citysim terrain / ocean / actors / media …"
for m in city_terrain ocean actor; do
  rm -rf "${MODELS}/${m}"
  cp -a "${CACHE}/citysim/models/${m}" "${MODELS}/${m}"
done
# actor/model.sdf references model://actor_walk/...; same meshes.
rm -rf "${MODELS}/actor_walk"
cp -a "${MODELS}/actor" "${MODELS}/actor_walk"
rm -rf "${SHARE}/media"
cp -a "${CACHE}/citysim/media" "${SHARE}/media"

# People models in gazebo_models are dynamic; freeze them so they do not fall.
for m in person_standing person_walking; do
  sdf="${MODELS}/${m}/model.sdf"
  if [ -f "${sdf}" ] && ! grep -q '<static>' "${sdf}"; then
    python3 - "${sdf}" <<'PY'
import sys
from pathlib import Path
p = Path(sys.argv[1])
t = p.read_text()
t = t.replace("<model name=\"person_standing\">", "<model name=\"person_standing\">\n    <static>true</static>", 1)
t = t.replace("<model name=\"person_walking\">", "<model name=\"person_walking\">\n    <static>true</static>", 1)
p.write_text(t)
PY
  fi
done

# Heightmap visual is too heavy for in-container llvmpipe; keep the street photo as a plane.
mkdir -p "${MODELS}/city_terrain/materials/scripts"
cat > "${MODELS}/city_terrain/materials/scripts/city_terrain.material" <<'MAT'
material CityTerrain/Ortho
{
  technique
  {
    pass
    {
      lighting off
      texture_unit
      {
        texture city_terrain.jpg
        filtering anisotropic
        max_anisotropy 8
      }
    }
  }
}
MAT
python3 - "${MODELS}/city_terrain/model.sdf" <<'PY'
from pathlib import Path
import re, sys
p = Path(sys.argv[1])
t = p.read_text()
plane = """      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <pose>40 -20 0 0 0 0</pose>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>500 500</size>
          </plane>
        </geometry>
        <material>
          <script>
            <uri>model://city_terrain/materials/scripts</uri>
            <uri>model://city_terrain/materials/textures</uri>
            <name>CityTerrain/Ortho</name>
          </script>
        </material>
      </visual>"""
t = re.sub(r"      <visual name=\"visual\">.*?</visual>", plane, t, count=1, flags=re.S)
p.write_text(t)
print("patched city_terrain visual")
PY

python3 - "${CACHE}/citysim/worlds/simple_city.world" "${WORLDS}/hil_city.world" <<'PY'
import re, sys
src, dst = sys.argv[1], sys.argv[2]
text = open(src, encoding="utf-8", errors="replace").read()
text = re.sub(r"\s*<plugin name=\"joy\".*?</plugin>", "", text, flags=re.S)
text = re.sub(r"\s*<plugin name=\"keyboard\".*?</plugin>", "", text, flags=re.S)
text = re.sub(r"\s*<plugin name=\"TrafficLights\".*?</plugin>", "", text, flags=re.S)
# Self-closing Classic plugins. Do not use [^>]*/> — that lets / get eaten and
# `>.*?</plugin>` then swallows the rest of the world file.
text = re.sub(r"\s*<plugin name=\"bloom\"[^/]*/>", "", text)
text = re.sub(r"\s*<plugin name=\"lensflare\"[^/]*/>", "", text)
text = re.sub(r"\s*<plugin name=\"prius\".*?</plugin>", "", text, flags=re.S)
text = text.replace("model://prius_hybrid_sensors", "model://prius_hybrid")
text = text.replace("<cast_shadows>true</cast_shadows>", "<cast_shadows>false</cast_shadows>")
# SkyX volumetric clouds (humidity default 0.6) wash out the north-field camera.
text = re.sub(
    r"<sky>.*?</sky>",
    "<sky>\n        <clouds>\n          <speed>0</speed>\n"
    "          <humidity>0</humidity>\n          <mean_size>0</mean_size>\n"
    "        </clouds>\n      </sky>",
    text,
    count=1,
    flags=re.S,
)
text = re.sub(
    r"<latitude_deg>0</latitude_deg>",
    "<latitude_deg>47.397742</latitude_deg>",
    text,
    count=1,
)
text = re.sub(
    r"<longitude_deg>0</longitude_deg>",
    "<longitude_deg>8.545594</longitude_deg>",
    text,
    count=1,
)
text = re.sub(r"<elevation>0</elevation>", "<elevation>488.0</elevation>", text, count=1)
px4_physics = """    <physics name='default_physics' default='0' type='ode'>
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>"""
text = re.sub(
    r"<physics name='default_physics'.*?</physics>",
    px4_physics,
    text,
    count=1,
    flags=re.S,
)
open(dst, "w", encoding="utf-8").write(text)
import xml.etree.ElementTree as ET
ET.parse(dst)
print("wrote", dst, "xml ok")
PY

echo "missing models: ${missing}"
echo "Done."
du -sh "${ASSETS}" "${WORLDS}/hil_city.world"
