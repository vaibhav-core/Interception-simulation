# 🚁 Drone Intercept Simulation — CoppeliaSim

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Language](https://img.shields.io/badge/Language-Lua-blue.svg)](https://www.lua.org/)
[![Simulator](https://img.shields.io/badge/Simulator-CoppeliaSim-orange.svg)](https://www.coppeliarobotics.com/)

A CoppeliaSim simulation where a drone predicts and intercepts a moving target using quadratic intercept mathematics.

*(Optional: Add a GIF/Screenshot of your simulation running here by replacing this line!)* 📸

---

## Quick Start

The easiest way to run the simulation is using the provided `.ttt` scene file:
1. Clone this repository or download the ZIP.
2. Open CoppeliaSim.
3. Open the scene `single_drone_single_target/single_drone_single_target.ttt`.
4. Press **Play**!

---

## Requirements

- CoppeliaSim **v4.3 or later** (developed and tested on v4.10)
- No external plugins required

---

## Scene Setup

### Object Hierarchy

```
Scene
├── Target
│     └── script    ← Target movement script
└── Drone
      └── script    ← Drone intercept script
```

### Steps to Set Up

1. Open CoppeliaSim and create a **new scene**
2. Add a shape for the **Drone**:
   - Menu → Add → Primitive Shape → Cuboid (or any shape)
   - Rename it to `Drone`
3. Add a shape for the **Target**:
   - Menu → Add → Primitive Shape → Sphere (or any shape)
   - Rename it to `Target`
4. Add a script to **Target**:
   - Right-click `Target` in hierarchy → Add → Script → Simulation Script->lua
   - Paste the Target script (see below)
5. Add a script to **Drone**:
   - Right-click `Drone` in hierarchy → Add → Script → Simulation Script
   - Paste the Drone script (see below)
6. Press **Play** to run the simulation

---

## Scripts

### Target Script

Moves the Target at a constant velocity every simulation step.

```lua
function sysCall_init()
    targetObj = sim.getObjectHandle("/Target")
    velocity = {1.0, 0.5, 0.0} -- velocity in x, y, z (units/sec)
end

function sysCall_actuation()
    local pos = sim.getObjectPosition(targetObj, sim.handle_world)
    local dt = sim.getSimulationTimeStep()
    local newPos = {
        pos[1] + velocity[1]*dt,
        pos[2] + velocity[2]*dt,
        pos[3] + velocity[3]*dt
    }
    sim.setObjectPosition(targetObj, sim.handle_world, newPos)
end
```

### Drone Script

Estimates the target's velocity and predicts an intercept point using quadratic math, then flies toward it.

```lua
function sysCall_init()
    droneObj = sim.getObjectHandle("/Drone")
    target = sim.getObjectHandle("/Target")

    if droneObj == -1 then error("Could not find '/Drone'") end
    if target == -1 then error("Could not find '/Target'") end

    prevTargetPos = sim.getObjectPosition(target, sim.handle_world)
    droneSpeed = 2.0 -- units/sec
end

function sysCall_actuation()
    local dronePos = sim.getObjectPosition(droneObj, sim.handle_world)
    local targetPos = sim.getObjectPosition(target, sim.handle_world)
    local dt = sim.getSimulationTimeStep()

    -- Estimate target velocity from position change
    local targetVel = {
        (targetPos[1] - prevTargetPos[1]) / dt,
        (targetPos[2] - prevTargetPos[2]) / dt,
        (targetPos[3] - prevTargetPos[3]) / dt
    }

    -- Relative position (target - drone)
    local relPos = {
        targetPos[1] - dronePos[1],
        targetPos[2] - dronePos[2],
        targetPos[3] - dronePos[3]
    }

    -- Solve quadratic for intercept time
    local a = targetVel[1]^2 + targetVel[2]^2 + targetVel[3]^2 - droneSpeed^2
    local b = 2*(relPos[1]*targetVel[1] + relPos[2]*targetVel[2] + relPos[3]*targetVel[3])
    local c = relPos[1]^2 + relPos[2]^2 + relPos[3]^2

    local discriminant = b*b - 4*a*c
    local t = 0
    if discriminant > 0 and math.abs(a) > 1e-6 then
        local t1 = (-b + math.sqrt(discriminant)) / (2*a)
        local t2 = (-b - math.sqrt(discriminant)) / (2*a)
        t = math.max(t1, t2)
        if t < 0 then t = 0 end
    end

    -- Predicted intercept point
    local interceptPoint = {
        targetPos[1] + targetVel[1]*t,
        targetPos[2] + targetVel[2]*t,
        targetPos[3] + targetVel[3]*t
    }

    -- Direction toward intercept point
    local dir = {
        interceptPoint[1] - dronePos[1],
        interceptPoint[2] - dronePos[2],
        interceptPoint[3] - dronePos[3]
    }
    local dist = math.sqrt(dir[1]^2 + dir[2]^2 + dir[3]^2)
    if dist > 1e-6 then
        dir = {dir[1]/dist, dir[2]/dist, dir[3]/dist}
    else
        dir = {0, 0, 0}
    end

    -- Move drone to desired position
    local newPos = {
        dronePos[1] + dir[1]*droneSpeed*dt,
        dronePos[2] + dir[2]*droneSpeed*dt,
        dronePos[3] + dir[3]*droneSpeed*dt
    }
    sim.setObjectPosition(droneObj, sim.handle_world, newPos)

    prevTargetPos = targetPos
end
```

---

## How the Code Works

### Target
The target moves in a straight line at constant velocity. Each frame it adds `velocity × dt` to its position, where `dt` is the simulation time step. This keeps movement smooth and frame-rate independent.

### Drone — Intercept Logic
The drone does **not** simply chase the target. Instead it:

1. **Estimates target velocity** by comparing the target's current and previous position divided by `dt`
2. **Solves a quadratic equation** to find how many seconds `t` until the drone can intercept the target given both their speeds
3. **Predicts the intercept point** as `targetPosition + targetVelocity × t`
4. **Flies toward that point** at a constant speed each frame

This means the drone cuts off the target rather than chasing it from behind — much more efficient.

### Quadratic Intercept Math
The equation being solved is:

```
|relPos + targetVel*t|² = (droneSpeed*t)²
```

Expanding gives `at² + bt + c = 0`, solved with the quadratic formula. The larger positive root is taken as the future intercept time.

---

## Customization

| Variable | Location | What it controls |
|---|---|---|
| `velocity` | Target script | Target's speed and direction |
| `droneSpeed` | Drone script | How fast the drone moves |

---

## Troubleshooting

### "object does not exist" error
- Check that your objects are named exactly `Drone` and `Target` (case-sensitive)
- If objects are nested inside other objects, update the path e.g. `"/Parent/Drone"`
- If using just the name without `/`, make sure names are unique in the scene

### Drone doesn't move
- Make sure `droneSpeed` is greater than `0`
- Check the Drone script is attached under the `Drone` object

### Scripts conflict or run twice
- Make sure there is only **one script** under each object
- Delete any duplicate scripts in the hierarchy

### Wrong CoppeliaSim version
- This simulation requires **v4.3+**
- `sim.handle_world` replaces the old `-1` argument
- `sim.getObjectHandle` replaces direct use of `sim.handle_self` for child scripts

---

## Notes

- Scripts in CoppeliaSim 4.10 are always added as **child nodes** in the hierarchy — this is normal
- Use `sim.getObjectParent(sim.handle_self)` as an alternative to `sim.getObjectHandle` if you want the script to work regardless of the object's name
