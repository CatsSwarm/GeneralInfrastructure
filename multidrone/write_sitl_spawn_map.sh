#!/bin/bash
# Write host-side SITL map frame so ObservationBoard LLA matches Gazebo XY.
# Gazebo X = East, Y = North; PX4 reprojects (0,0) → Zurich Irchel Park
# 47.397742, 8.545594 (sitl_gazebo-classic/include/common.h).
#
# Emits the **per-drone** spawn table, not just first + spacing: GUI XY (and the
# Randomize button) need not be an arithmetic line, and extrapolating
# first + (id-1)*spacing put drone 3/4 map icons tens of metres from their
# Gazebo models. Index i of spawn_east_m/spawn_north_m is drone i+1.
#
# Mirrors sitl_multiple_run.sh: same hil_city origin-shift heuristic, and the
# same no-XY default for drones past the end of the positions file.
#
# Usage: CATSWARM_WORLD=hil_city POSITIONS_FILE=... NUM_DRONES=4 ./write_sitl_spawn_map.sh
# CATSWARM_WORLD=simple_hil (and any non-hil_city world) keeps the empty-world
# parking line. City pad / origin-shift apply only when world is hil_city.

write_sitl_spawn_map() {
    local spawn_json="${CATSWARM_SITL_SPAWN_FILE:-/tmp/catswarm_sitl_spawn.json}"
    local spawn_env="/tmp/catswarm_sitl_spawn.env"
    local world="${CATSWARM_WORLD:-hil_city}"
    local pos_file="${POSITIONS_FILE:-}"
    local num_drones="${NUM_DRONES:-0}"
    local xs=()
    local ys=()

    if [ -n "${pos_file}" ] && [ -f "${pos_file}" ]; then
        while IFS=' ' read -r x y z || [ -n "$x" ]; do
            [[ -z "$x" || "$x" =~ ^# ]] && continue
            xs+=("$x")
            ys+=("$y")
        done < "${pos_file}"
    fi

    # simple_hil downtown crop ends near y=52. A leftover hil_city pad
    # (y≈108) must not become this world's map frame. Same rule as
    # sitl_multiple_run.sh. hil_city keeps the north field. Outdoor LLA
    # does not read this file unless the process is actually on SITL.
    if [ "${world}" = "simple_hil" ] && [ ${#ys[@]} -gt 0 ]; then
        local north_field=0
        local y
        for y in "${ys[@]}"; do
            if awk -v y="${y}" 'BEGIN { exit !((y+0) > 60) }'; then
                north_field=1
                break
            fi
        done
        if [ "${north_field}" = "1" ]; then
            local i n
            for i in "${!xs[@]}"; do
                n=$((i + 1))
                xs[$i]=0.000
                ys[$i]=$(awk -v n="${n}" 'BEGIN { printf "%.3f", 3*n }')
            done
        fi
    fi

    # Parking-lot / origin XY (0,0 / 0,3 / …) is inside the city shops.
    # sitl_multiple_run.sh nudges the whole set onto the open north field.
    if [ "${world}" = "hil_city" ] && [ ${#xs[@]} -gt 0 ]; then
        local first_x="${xs[0]}"
        local first_y="${ys[0]}"
        local awk_origin='BEGIN { x="'"${first_x}"'"; y="'"${first_y}"'"; if (x+0 >= -8 && x+0 <= 8 && y+0 >= -20 && y+0 <= 20) exit 0; exit 1 }'
        if awk "${awk_origin}"; then
            local i
            for i in "${!xs[@]}"; do
                xs[$i]=$(awk -v x="${xs[$i]}" 'BEGIN { printf "%.3f", x-15 }')
                ys[$i]=$(awk -v y="${ys[$i]}" 'BEGIN { printf "%.3f", y+108 }')
            done
        fi
    fi

    # Drones past the positions file get sitl_multiple_run.sh's spawn_model
    # default: hil_city pad (-15, 108+4*(n-1)), empty world (0, 3*n), n 1-based.
    local n
    for ((n = ${#xs[@]} + 1; n <= num_drones; n++)); do
        if [ "${world}" = "hil_city" ]; then
            xs+=("-15.000")
            ys+=("$(awk -v n="${n}" 'BEGIN { printf "%.3f", 108 + 4*(n-1) }')")
        else
            xs+=("0.000")
            ys+=("$(awk -v n="${n}" 'BEGIN { printf "%.3f", 3*n }')")
        fi
    done

    if [ ${#xs[@]} -eq 0 ]; then
        if [ "${world}" = "hil_city" ]; then
            xs=(-15)
            ys=(108)
        else
            xs=(0)
            ys=(0)
        fi
    fi

    # Legacy first + spacing fields: still consumed for ids past the table.
    local east="${xs[0]}"
    local north="${ys[0]}"
    local de="0.000"
    local dn="3.000"
    if [ ${#xs[@]} -ge 2 ]; then
        de=$(awk -v a="${xs[1]}" -v b="${xs[0]}" 'BEGIN { printf "%.3f", a-b }')
        dn=$(awk -v a="${ys[1]}" -v b="${ys[0]}" 'BEGIN { printf "%.3f", a-b }')
    elif [ "${world}" = "hil_city" ]; then
        dn="4.000"
    fi

    # Gazebo world Z: city_terrain_1 is a plane posed at 5.01; empty-world
    # ground_plane is 0. spawn_z is what sitl_multiple_run.sh spawns a landed
    # iris at, so teleports can keep the same clearance above the ground.
    local ground_z="0.000"
    local spawn_z="0.830"
    if [ "${world}" = "hil_city" ]; then
        ground_z="5.010"
        spawn_z="5.350"
    fi

    local east_list
    local north_list
    local xy_list
    east_list=$(IFS=,; echo "${xs[*]}")
    north_list=$(IFS=,; echo "${ys[*]}")
    xy_list=""
    for n in "${!xs[@]}"; do
        xy_list+="${xs[$n]},${ys[$n]};"
    done

    cat > "${spawn_json}" << EOF
{"east_m": ${east}, "north_m": ${north}, "spacing_east_m": ${de}, "spacing_north_m": ${dn},
 "ground_z_m": ${ground_z}, "spawn_z_m": ${spawn_z},
 "spawn_east_m": [${east_list}], "spawn_north_m": [${north_list}]}
EOF
    cat > "${spawn_env}" << EOF
CATSWARM_WORLD=${world}
CATSWARM_SITL_SPAWN_EAST_M=${east}
CATSWARM_SITL_SPAWN_NORTH_M=${north}
CATSWARM_SITL_SPAWN_SPACING_EAST_M=${de}
CATSWARM_SITL_SPAWN_SPACING_NORTH_M=${dn}
CATSWARM_SITL_SPAWN_XY=${xy_list}
CATSWARM_SITL_GROUND_Z_M=${ground_z}
CATSWARM_SITL_SPAWN_Z_M=${spawn_z}
EOF
    echo "SITL map frame: ${#xs[@]} pads, first east=${east} north=${north}, ground_z=${ground_z} (${spawn_json})"
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    write_sitl_spawn_map
fi
