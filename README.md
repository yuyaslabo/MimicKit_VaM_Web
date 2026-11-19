# MimicKit_VaM_Web

**Web-based inference and physics simulation for Virt-A-Mate (VaM), derived from [MimicKit_VaM](https://github.com/yuyaslabo/MimicKit_VaM).**

[Open Web Application](https://yuyaslabo.github.io/MimicKit_VaM_Web/) · [Windows Bridge Releases](https://github.com/yuyaslabo/MimicKit_VaM_Web/releases) · [Original Newton Project](https://github.com/yuyaslabo/MimicKit_VaM)

## Abstract

**MimicKit_VaM_Web** is a web-based adaptation of **[MimicKit_VaM](https://github.com/yuyaslabo/MimicKit_VaM)**, a project that runs MimicKit-trained physical AI inside **Virt-A-Mate (VaM)** and enables physical interaction through VR controllers.

The original project uses **Newton** for physics simulation and neural-network inference. This version replaces that runtime with **MuJoCo WebAssembly (WASM)** for physics and **ONNX Runtime Web** for policy inference, allowing the AI simulation to run directly in a web browser rather than in the original Newton/Python inference environment.

A lightweight **Windows Bridge** maintains communication with VaM: it retargets the simulated humanoid's pose to VaM control points and relays VR controller, HMD, and furniture data between VaM and the web application. The aim is to preserve the original project's **physical interaction and player-following behavior** while moving simulation and inference to the web.

The web application handles physical AI simulation and inference; VaM renders and hosts the interactive character and scene.

## Architecture

```text
                  Web Browser (MuJoCo WASM + ONNX Runtime Web)
                  ┌───────────────────────────────────────────┐
                  │ G1 humanoid physics + RL policy inference │
                  │ VR interaction / HMD target / furniture  │
                  └─────────────────────┬─────────────────────┘
                                        │ WebSocket :8765
                                        │
                  ┌─────────────────────▼─────────────────────┐
                  │          Windows Python Bridge            │
                  │  Pose retargeting + UDP/WS forwarding     │
                  └─────────────────────┬─────────────────────┘
                                        │
                         Virt-A-Mate (VaM) plugins

   Browser → Bridge → VaM     UDP :9998   Character control-point poses
   VaM → Bridge → Browser     UDP :9999   VR controllers + HMD + triggers
   VaM → Bridge → Browser     UDP :9001   Furniture transforms
```

The web version loads **pre-registered furniture collider XML files** at startup; unlike the original Newton receiver, it does **not** require live shape registration over UDP 9002.

## Features

- **Web inference and simulation:** MimicKit-trained ONNX policy with a 29-DoF G1 humanoid, simulated using MuJoCo WASM.
- **VaM pose synchronization:** Pose retargeting in the local Bridge and real-time UDP output to VaM.
- **VR interaction:** Left/right hand and HMD tracking, physical hand colliders, and trigger-controlled grabbing/pulling of the humanoid.
- **Direction control:** Right double-trigger selects a world-fixed facing direction; a downward double-trigger returns to HMD-facing mode.
- **Follow / Stay:** Left double-trigger switches between a fixed target and HMD-driven target tracking.
- **Furniture synchronization:** Load existing compound-box MJCF colliders and update their transforms from VaM, enabling obstacle contact.
- **Obstacle-aware targets:** Local target placement with a 1 m agent-centered search radius and basic furniture clearance checks. The advanced Newton rescue/pathfinding system is not fully reproduced.
- **Simulation controls:** Physics and actuator settings, target controls, collision-shape inspection, and optional correlated action-output noise.

## Requirements

**For VaM integration:**

- Windows PC with a modern browser supporting WebAssembly and WebGL.
- **Virt-A-Mate**, with the compatible receiver, VRHand/HMD and furniture-sync plugins described in the [original MimicKit_VaM project](https://github.com/yuyaslabo/MimicKit_VaM).
- **VaM Web Bridge** installed locally on the PC running the browser. See [Releases](https://github.com/yuyaslabo/MimicKit_VaM_Web/releases) for available installer packages.
- Working local-network connectivity between the Bridge and VaM. A single-PC or dual-PC setup is possible, subject to network configuration.

The browser runs MuJoCo and ONNX inference; the Windows Bridge does **not** run the neural-network policy or the physics simulation.

## Setup — Connect to VaM

1. Load the following plugins in Virt-A-Mate.
    - `Receiver_mimickit_public.cs` — **Person plugin** for receiving AI character motion.
    - `VRhand_mimickit_public.cs` — **Scene plugin** for VR controller/HMD data and controls.
    - `Atom_sync_mimickit_public.cs` — **Scene plugin** for furniture Atom transform and collider shape synchronization.
2. Install and run the **VaM Web Bridge** on the same PC as your browser.
3. Open **[MimicKit_VaM_Web](https://yuyaslabo.github.io/MimicKit_VaM_Web/)**.
4. Enable **VaM WebSocket / UDP - Stream enabled** and verify that its status displays **Connected**.


### Bridge destination IP

Bridge sends the retargeted humanoid poses to VaM via UDP port 9998. To change the destination IP address, please edit the `bridge_config.json` file, which is located in the same directory as the Bridge executable, `VaM_Web_Bridge.exe`.

```json
{
  "udp_host": "127.0.0.1"
}
```

For a dual-PC configuration, replace `127.0.0.1` with the IP address of the **VaM PC**, then restart the Bridge.

### Ports

| Port | Direction | Purpose |
| --- | --- | --- |
| TCP 8765 | Browser ↔ Bridge (WebSocket) | Simulation poses and relayed tracking/furniture data |
| UDP 9998 | Bridge → VaM | Humanoid control-point poses |
| UDP 9999 | VaM → Bridge | Left/right controller positions, rotations, triggers and HMD tracking |
| UDP 9001 | VaM → Bridge | Furniture Atom positions and rotations |

**UDP 9002 is not used by this web implementation.** Register furniture collider shapes in the original Newton project first, then place the exported XML assets in the web project.

> **Hosted-page networking:** The GitHub Pages application is served over HTTPS, while the local Bridge uses WebSocket on `127.0.0.1`. Browser security and local-network permissions may affect the connection. Check the **VaM WebSocket / UDP** status and browser console if it cannot connect.

## VR Controls

| Input | Behavior |
| --- | --- |
| Hold a trigger near an agent body | Grab and pull the nearby body using physical forces |
| Left double-trigger (without grabbing) | Toggle **STAY / FOLLOW** |
| Right double-trigger (without grabbing) | Set an absolute world-space facing point from the controller direction |
| Right downward double-trigger | Clear the world-fixed facing point and return to HMD-facing direction |

**STAY** holds the agent target fixed in world space. **FOLLOW** updates the target from HMD tracking, subject to target-distance logic and basic local obstacle avoidance. The right-hand facing control is independent of the left-hand target-follow toggle.

## Furniture Collision Synchronization

The web application uses the `atom_<ID>.xml` compound-collider assets previously generated by the Newton version. Place them alongside a matching `manifest.json` under:

```text
public/vam_env_boxes/
├── manifest.json
├── atom_11967007.xml
└── atom_*.xml
```

The Atom IDs in XML filenames must match the IDs sent by the VaM furniture plugin over UDP 9001. At runtime, only the **position and rotation** of registered furniture are synchronized.

New furniture shapes and scale changes require updating the XML assets and rebuilding/redeploying the web app. Collider geometry is box-based: rounded/curved furniture does not retain an exact curved collision surface. Furniture provides collision and obstacle information, **not furniture manipulation**.

The GUI's **Furniture / XML diagnostics** section shows registered Atom IDs, received IDs, unregistered IDs, and active collider state.

## Run From Source

This repository contains the Vite web application. Install Node.js and then run:

```bash
npm install
npm run dev
```

For the production build:

```bash
npm run build
npm run preview
```

The build process copies the required MuJoCo WASM and ONNX Runtime WASM/JSEP assets into `public/`. The Vite `base` path is configured for `/MimicKit_VaM_Web/` on GitHub Pages.

### Relevant files

| File / directory | Role |
| --- | --- |
| `src/main.js` | Main simulation loop, ONNX inference, MuJoCo controls and GUI |
| `src/vam_raw_pose_ws.js` | Browser/Bridge WebSocket transport |
| `src/vam_vr_web.js` | VRHand and HMD tracking, physical interaction and trigger controls |
| `src/vam_target_web.js` | HMD-driven target and STAY/FOLLOW behavior |
| `src/vam_furniture_web.js` | XML furniture colliders and pose synchronization |
| `public/g1_collision.xml` | Simulated humanoid MJCF, contact shapes, inertias and actuators |
| `public/g1.xml`, `public/meshes/` | Visual-only G1 model assets |
| `public/vam_env_boxes/` | Registered furniture collision assets |
| `bridge/` | Windows Bridge source and installer build scripts |

## Troubleshooting

- **Bridge status not Connected:** Confirm the Bridge is running, that TCP 8765 is available, and that browser local-network/WebSocket policies permit the connection.
- **VaM character does not move:** Check the Bridge's `udp_host` value, the UDP 9998 receiver plugin and firewall settings.
- **VRHand/HMD does not track:** Verify UDP 9999 is reaching the Bridge and **Tracking & physics** is enabled.
- **Furniture not active:** Check XML filenames, `manifest.json`, UDP 9001, and the **Furniture / XML diagnostics** panel for unmatched IDs.
- **ONNX or MuJoCo fails to initialize:** Inspect browser console errors, WASM asset paths, and the supplied ONNX file.
- **Physical behavior differs from Newton:** Because Newton and MuJoCo employ different mechanisms for physics computation and contact solvers, discrepancies arise even when using identical policy inputs or similar servo settings.

## Project Origin and Credits

This project is derived from **[yuyaslabo/MimicKit_VaM](https://github.com/yuyaslabo/MimicKit_VaM)**, itself based on **[MimicKit](https://github.com/xbpeng/MimicKit)**. It retains the concept of real-time physically simulated AI interaction with Virt-A-Mate while moving the simulator and policy inference into the browser.

Core technologies include **[MuJoCo](https://mujoco.org/)**, **[ONNX Runtime Web](https://onnxruntime.ai/)**, and **[Virt-A-Mate](https://www.virtamate.com/)**.

This is an unofficial community project and is not affiliated with the upstream projects or Virt-A-Mate's developers. Refer to the upstream repositories and applicable licenses for copyright and redistribution requirements.

**Developed by [LocalAIVRLab / yuyaslabo](https://github.com/yuyaslabo).**
