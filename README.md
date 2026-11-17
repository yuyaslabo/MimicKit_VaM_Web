# MimicKit VaM Web

**Physics-based AI in your browser — with optional Virt-A-Mate (VaM) VR interaction.**

Run an ONNX reinforcement-learning policy on a 29-DoF G1 humanoid inside **MuJoCo WebAssembly**. The simulation runs locally in your browser. With the optional Windows Bridge, you can stream the simulated character into **Virt-A-Mate**, interact through VR controllers, and synchronize furniture collision geometry.

**[Launch Web App](https://yuyaslabo.github.io/MimicKit_VaM_Web/)** · **[GitHub Repository](https://github.com/yuyaslabo/MimicKit_VaM_Web)** · **[Releases](https://github.com/yuyaslabo/MimicKit_VaM_Web/releases)**

> **Experimental project.** Browser behavior, frame rate, and VR integration depend on the device, browser, and scene. The browser demo does not require VaM or the Python Bridge.

## Features

- **Browser-based physics:** MuJoCo WASM runs the humanoid simulation; ONNX Runtime Web executes the trained controller.
- **29-DoF G1 humanoid:** Newton-style joint position servos with adjustable drive settings.
- **Real-time controls:** Target position/direction, camera, simulation controls, and optional action-output noise.
- **VaM pose streaming:** The Windows Bridge converts browser poses into VaM control-point data using the project's retargeting pipeline.
- **VR interaction:** Tracked left/right controllers and HMD, physical hand interaction, and trigger-based body grabbing.
- **Right double-trigger:** Point to set a world-fixed facing target; point downward and double-trigger to return to HMD-facing mode.
- **Left double-trigger:** Toggle between **STAY** (fixed target) and **FOLLOW** (HMD target tracking).
- **Furniture synchronization:** Load previously registered collider XML files and update furniture positions/orientations from VaM.
- **Obstacle-aware target placement:** A nearby safe-target search is available; the browser implementation does not reproduce every Newton pathfinding/rescue feature.

## 1. Try the Web Demo

Open **https://yuyaslabo.github.io/MimicKit_VaM_Web/** in a desktop browser that supports WebAssembly and WebGL.

The app runs the G1 simulation in-browser. No Python, Conda, or Virt-A-Mate installation is necessary for this standalone mode. The first launch may take longer while the ONNX model, WASM runtimes, and character assets load.

Use the on-screen GUI to inspect the policy, adjust targets, change camera/physics settings, or enable action noise. Performance depends on browser and hardware; WASM execution does not imply GPU-accelerated neural-network inference.

## 2. Connect to Virt-A-Mate (Windows)

**Requirements:**

- A working installation of **Virt-A-Mate** and the compatible VaM receiver/sender plugins from the [Newton/VaM integration project](https://github.com/yuyaslabo/VaM_Physical_AI_from_Mimickit).
- The **VaM Web Bridge** on the PC running the browser (Windows EXE/installer when available in [Releases](https://github.com/yuyaslabo/MimicKit_VaM_Web/releases)).
- Network access enabled for the VaM plugins, with the appropriate firewall rules if using separate PCs.

**Setup:**

1. Install or extract the VaM Web Bridge and launch `VaM_Web_Bridge.exe`.
2. Open the [Web App](https://yuyaslabo.github.io/MimicKit_VaM_Web/) on the **same PC as the Bridge**.
3. In the app, enable **VaM WebSocket / UDP** streaming and check that its status becomes **Connected**.
4. Enable **VRHand / HMD sync → Tracking & physics** to use tracked controllers and the HMD.
5. In VaM, configure the matching receiver, VRHand, and furniture-sync plugins. Align the VaM scene origin with the simulation as described in the original integration project's setup instructions.

### Network ports

| Port | Direction | Purpose |
| --- | --- | --- |
| TCP 8765 | Browser → local Bridge (WebSocket) | Simulated pose data and VR/furniture forwarding |
| UDP 9998 | Bridge → VaM | Retargeted control-point poses |
| UDP 9999 | VaM → Bridge | Left/right VR controllers, triggers, and HMD tracking |
| UDP 9001 | VaM → Bridge | Furniture Atom position/orientation updates |

**UDP 9002 is not needed by the Web version.** Furniture shapes are loaded from pre-existing XML assets rather than registered live.

### Set a different VaM destination IP

Edit `bridge_config.json` next to the Bridge executable, then restart the Bridge:

```json
{
  "udp_host": "127.0.0.1"
}
```

For a two-PC setup, replace `127.0.0.1` with the IP address of the **VaM PC**. This setting changes the UDP 9998 destination; it does not change the Bridge's local WebSocket listener.

> **Browser security:** The hosted page uses HTTPS while the Bridge uses a local `ws://` connection. Browser permissions and security policies vary. If the page works but Bridge connection fails, check the browser's local-network access policy and the Bridge console before changing the retargeting configuration.

## 3. VR Controller Operations

| Input | Action |
| --- | --- |
| Hold a trigger near the humanoid | Grab and physically pull a nearby body |
| Left double-trigger (when not grabbing) | Switch **STAY ↔ FOLLOW** |
| Right double-trigger (when not grabbing) | Set a world-fixed direction to face using the controller pointing direction |
| Right downward double-trigger | Clear the fixed direction and face the HMD again |

Hand positions are physically tracked with position-PD control; their orientations follow the tracked controller orientation. HMD tracking is also used to estimate a torso collider and target/facing directions.

In **FOLLOW** mode, HMD motion updates the target. A local safe-target search attempts to keep the goal near the character (typically within 1 m), while avoiding registered furniture. In **STAY** mode, the target remains fixed and HMD-driven recovery/avoidance is bypassed.

## 4. Furniture Collision Assets

The Web app uses already registered Newton-style compound-collider MJCF assets (`atom_<ID>.xml`), typically stored under:

```text
public/vam_env_boxes/
  manifest.json
  atom_11967007.xml
  atom_....xml
```

The Atom ID in each filename must match the ID transmitted by the VaM furniture-sync plugin on UDP 9001. Furniture poses update at runtime, but **new shapes or scale changes require regenerating the XML assets and rebuilding/redeploying the Web app**. The current collider representation uses oriented boxes; curved meshes are not reproduced as exact curved surfaces.

Use **Furniture / XML diagnostics** in the GUI to inspect registered XML IDs, received IDs, unmatched IDs, and active colliders.

> **Privacy / publishing:** Do not publish scene-specific furniture assets, scene data, or third-party models unless you have permission to distribute them.

## 5. Run Locally / Develop

Install a supported Node.js LTS version, then run:

```bash
npm install
npm run dev
```

Build and preview the production version:

```bash
npm run build
npm run preview
```

The `prebuild` step copies the local MuJoCo and ONNX Runtime WASM/JSEP runtime assets into `public/`. The Vite `base` setting is configured for the GitHub Pages repository path `/MimicKit_VaM_Web/`. If you deploy under a different repository name or domain, update `base` accordingly.

### Key project files

| Location | Purpose |
| --- | --- |
| `src/main.js` | MuJoCo simulation, ONNX inference, GUI, and main loop |
| `src/vam_vr_web.js` | VR tracking, hand physics, and grab/facing controls |
| `src/vam_target_web.js` | HMD target tracking and STAY/FOLLOW behavior |
| `src/vam_furniture_web.js` | Furniture pose sync and XML-based colliders |
| `src/vam_raw_pose_ws.js` | Bridge WebSocket communication |
| `public/g1_collision.xml` | Humanoid physics model and collision geometry |
| `public/g1.xml` / `public/meshes/` | G1 appearance assets |
| `public/amp_actor.onnx` | Trained policy (if included in your distribution) |

Some paths may vary between revisions; check the actual repository tree before modifying the app.

## 6. Troubleshooting

- **App loads, but the character does not move:** Inspect the browser console for ONNX/WASM loading errors and confirm that required model assets exist under the configured Vite base path.
- **Bridge does not connect:** Confirm the Bridge is running, the browser's **VaM WebSocket / UDP** status shows **Connected**, and TCP port 8765 is available on the browser PC.
- **VaM receives no poses:** Confirm UDP 9998 destination IP and VaM plugin networking permissions.
- **VR hands/HMD not moving:** Verify UDP 9999 transmission and enable **Tracking & physics**.
- **Furniture missing:** Use **Furniture / XML diagnostics**. Check registered IDs against received IDs, and confirm UDP 9001 data reaches the Bridge.
- **Different behavior than Newton:** MuJoCo and Newton use different physics implementations. Similar actuator settings do not guarantee identical trajectories or contact responses.

## 7. Project Background and Credits

This is an independent experimental Web/VaM integration built around a policy trained with **[MimicKit](https://github.com/xbpeng/MimicKit)** and simulated in **[MuJoCo](https://mujoco.org/)**, with inference via **[ONNX Runtime Web](https://onnxruntime.ai/)**.

The original Newton-based VaM integration is maintained separately at **[VaM_Physical_AI_from_Mimickit](https://github.com/yuyaslabo/VaM_Physical_AI_from_Mimickit)**.

MimicKit and other upstream assets retain their respective copyrights and licenses. Check the repository's `LICENSE` and the licenses of bundled models, meshes, and dependencies before redistribution. No license for original project content is asserted by this README.

**Developer:** [LocalAIVRLab / yuyaslabo](https://github.com/yuyaslabo)
