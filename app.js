/*
  ====================================================================================
  5-DOF ROBOTIC ARM - 3D SIMULATION & REAL-TIME CONTROLLER
  ====================================================================================
  Three.js Kinematics, WebSockets, Web Serial API, Motion Choreographer,
  and Universal Browser / Node.js SSR Guard.
*/

// --- Browser Environment Detection & SSR / Node.js Guard ---
const isBrowser = typeof window !== 'undefined' && typeof document !== 'undefined';
const globalScope = isBrowser ? window : (typeof globalThis !== 'undefined' ? globalThis : (typeof global !== 'undefined' ? global : {}));
const getNow = () => (typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now());

// Safe Storage abstraction
const safeStorage = {
  getItem: (key) => {
    try {
      if (typeof localStorage !== 'undefined') return localStorage.getItem(key);
    } catch (e) {}
    return null;
  },
  setItem: (key, val) => {
    try {
      if (typeof localStorage !== 'undefined') localStorage.setItem(key, val);
    } catch (e) {}
  },
  removeItem: (key) => {
    try {
      if (typeof localStorage !== 'undefined') localStorage.removeItem(key);
    } catch (e) {}
  }
};

// --- State & Angle Variables ---
const NUM_JOINTS = 5;
// J1: Base Pitch, J2: Elbow Pitch, J3: Wrist Pitch, J4: Wrist Roll/Yaw, J5: Gripper
let targetAngles = [90, 90, 90, 90, 20];
let currentAngles = [90, 90, 90, 90, 20];
let jointTrims = [0, 0, 0, 0, 0];
let isInverted = [false, false, false, false, false];
let motionSpeed = 40; // deg/sec

// Presets
const PRESETS = {
  home:  [90, 90, 90, 90, 20],
  rest:  [35, 145, 140, 90, 10],
  reach: [110, 65, 45, 90, 45],
  pick:  [135, 45, 30, 90, 85],
  wave:  [80, 110, 120, 140, 60]
};

// Physical dimensions in centimeters (scaled to Three.js units)
const DIM = {
  baseHeight: 34,
  baseRadius: 11,
  link1Length: 42, // Lower tube
  link2Length: 40, // Middle tube
  link3Length: 16, // Upper tube
  gripperLength: 12
};

// --- Three.js Globals ---
let scene, camera, renderer, controls, transformControls;
let robotArm = {};
let targetGhostArm = {};
let showGhost = true;
let gridHelper, axesHelper;
let ikTargetMesh;

// --- Communication Globals ---
let ws = null;
let serialPort = null;
let serialWriter = null;
let serialReader = null;
let isConnected = false;
let connMode = 'wifi'; // 'wifi' or 'serial'
let lastSendTime = 0;
let pingStartTime = 0;

// --- Sequencer Globals ---
let keyframes = [
  { name: "1. Standoff", angles: [90, 90, 90, 90, 80], duration: 1500 },
  { name: "2. Descend", angles: [125, 55, 35, 90, 80], duration: 2000 },
  { name: "3. Grab Object", angles: [125, 55, 35, 90, 5], duration: 1000 },
  { name: "4. Lift High", angles: [75, 105, 80, 90, 5], duration: 2000 },
  { name: "5. Deposit", angles: [60, 80, 90, 90, 75], duration: 1500 }
];
let isPlayingSeq = false;
let seqIndex = 0;
let seqTimer = null;

// Motion Trail Globals
const MAX_TRAIL_POINTS = 120;
let trailPoints = [];
let trailLine = null;

// --- Initialize App Entry Point ---
function initApp() {
  if (!isBrowser) return;
  initThree();
  initUIEvents();
  renderKeyframesList();
  updateAnglesDisplay();
  animate();
}

if (isBrowser) {
  if (document.readyState === 'loading') {
    window.addEventListener('DOMContentLoaded', initApp);
  } else {
    initApp();
  }
}

// ====================================================================================
// THREE.JS 3D ROBOT RIGGING & SCENE BUILDER
// ====================================================================================
function initThree() {
  if (!isBrowser || typeof THREE === 'undefined') return;

  const container = document.getElementById('canvas-container');
  if (!container) return;
  const width = container.clientWidth || 800;
  const height = container.clientHeight || 600;

  // Scene
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0e17);
  scene.fog = new THREE.FogExp2(0x0a0e17, 0.0035);

  // Camera
  camera = new THREE.PerspectiveCamera(45, width / height, 1, 1000);
  camera.position.set(130, 90, 140);

  // Renderer
  renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
  renderer.setSize(width, height);
  const pixelRatio = (typeof window !== 'undefined' && window.devicePixelRatio) ? Math.min(window.devicePixelRatio, 2) : 1;
  renderer.setPixelRatio(pixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  // OrbitControls
  if (typeof THREE.OrbitControls !== 'undefined') {
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxPolarAngle = Math.PI / 2 - 0.02; // Don't go below floor
    controls.target.set(0, 55, 0);
  }

  // Lights
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
  scene.add(ambientLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
  dirLight.position.set(80, 150, 100);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  dirLight.shadow.camera.near = 10;
  dirLight.shadow.camera.far = 400;
  dirLight.shadow.camera.left = -100;
  dirLight.shadow.camera.right = 100;
  dirLight.shadow.camera.top = 100;
  dirLight.shadow.camera.bottom = -100;
  scene.add(dirLight);

  const fillLight = new THREE.DirectionalLight(0x38bdf8, 0.4);
  fillLight.position.set(-80, 60, -60);
  scene.add(fillLight);

  // Ground Floor & Grid
  const floorGeo = new THREE.PlaneGeometry(300, 300);
  const floorMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.85, metalness: 0.1 });
  const floor = new THREE.Mesh(floorGeo, floorMat);
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);

  gridHelper = new THREE.GridHelper(300, 30, 0x38bdf8, 0x1e293b);
  gridHelper.position.y = 0.1;
  scene.add(gridHelper);

  axesHelper = new THREE.AxesHelper(30);
  axesHelper.position.y = 0.2;
  scene.add(axesHelper);

  // Glowing Motion Trajectory Trail Line
  const trailGeo = new THREE.BufferGeometry();
  const trailPositions = new Float32Array(MAX_TRAIL_POINTS * 3);
  trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
  const trailMat = new THREE.LineBasicMaterial({
    color: 0x38bdf8,
    transparent: true,
    opacity: 0.75,
    linewidth: 2
  });
  trailLine = new THREE.Line(trailGeo, trailMat);
  scene.add(trailLine);

  // IK Target Gizmo (semi-transparent glowing sphere)
  const ikGeo = new THREE.SphereGeometry(2.5, 16, 16);
  const ikMat = new THREE.MeshStandardMaterial({
    color: 0x38bdf8,
    emissive: 0x0284c7,
    emissiveIntensity: 0.6,
    transparent: true,
    opacity: 0.75,
    wireframe: true
  });
  ikTargetMesh = new THREE.Mesh(ikGeo, ikMat);
  ikTargetMesh.position.set(35, 60, 0);
  scene.add(ikTargetMesh);

  // 3D Draggable Transform Gizmo for IK
  if (typeof THREE.TransformControls !== 'undefined') {
    transformControls = new THREE.TransformControls(camera, renderer.domElement);
    transformControls.size = 0.75;
    transformControls.attach(ikTargetMesh);
    scene.add(transformControls);

    transformControls.addEventListener('dragging-changed', (event) => {
      if (controls) controls.enabled = !event.value;
    });

    transformControls.addEventListener('change', () => {
      if (transformControls.dragging) {
        const pos = ikTargetMesh.position;
        const sx = document.getElementById('slider_ik_x');
        const sy = document.getElementById('slider_ik_y');
        const sz = document.getElementById('slider_ik_z');
        const vx = document.getElementById('val_ik_x');
        const vy = document.getElementById('val_ik_y');
        const vz = document.getElementById('val_ik_z');
        const sp = document.getElementById('slider_ik_pitch');

        if (sx) sx.value = Math.round(pos.x);
        if (sy) sy.value = Math.round(pos.y);
        if (sz) sz.value = Math.round(pos.z);
        if (vx) vx.textContent = Math.round(pos.x);
        if (vy) vy.textContent = Math.round(pos.y);
        if (vz) vz.textContent = Math.round(pos.z);
        const pitch = sp ? parseFloat(sp.value) : 0;
        solveAnalyticalIK(pos.x, pos.y, pos.z, pitch, true);
      }
    });
  }

  // Build the 3D Robot Arm Assembly
  buildRobotHierarchy();

  // Resize handler
  if (typeof window !== 'undefined') {
    window.addEventListener('resize', onWindowResize);
  }
}

function onWindowResize() {
  if (!isBrowser) return;
  const container = document.getElementById('canvas-container');
  if (!container || !camera || !renderer) return;
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}

// ====================================================================================
// 3D PROCEDURAL ROBOT RIG (Authentic to User Photos)
// ====================================================================================
function buildRobotHierarchy() {
  if (typeof THREE === 'undefined') return;

  // Shared Materials
  const darkPvcMat = new THREE.MeshStandardMaterial({ color: 0x27272a, roughness: 0.6, metalness: 0.2 });
  const whitePlateMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, roughness: 0.4 });
  const aluminumMat = new THREE.MeshStandardMaterial({ color: 0xd4d4d8, roughness: 0.25, metalness: 0.85 });
  const pinkServoMat = new THREE.MeshStandardMaterial({ color: 0xe11d48, roughness: 0.35, metalness: 0.8 }); // 150kg servo anodized body
  const blackBracketMat = new THREE.MeshStandardMaterial({ color: 0x18181b, roughness: 0.5, metalness: 0.6 });
  const blackServoMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.6, metalness: 0.3 }); // MG996R

  // 1. BASE STAND
  const baseGroup = new THREE.Group();
  scene.add(baseGroup);

  // Bottom Square Platform
  const platformGeo = new THREE.BoxGeometry(32, 2.5, 32);
  const platform = new THREE.Mesh(platformGeo, whitePlateMat);
  platform.position.y = 1.25;
  platform.receiveShadow = true;
  baseGroup.add(platform);

  // Tall Cylindrical Drum (Dark PVC)
  const drumGeo = new THREE.CylinderGeometry(DIM.baseRadius, DIM.baseRadius, DIM.baseHeight, 32);
  const drum = new THREE.Mesh(drumGeo, darkPvcMat);
  drum.position.y = 1.25 + DIM.baseHeight / 2;
  drum.castShadow = true;
  drum.receiveShadow = true;
  baseGroup.add(drum);

  // Metal Brackets at base of drum
  for (let a = 0; a < 4; a++) {
    const angle = (a * Math.PI) / 2;
    const bGeo = new THREE.BoxGeometry(1.5, 6, 4);
    const bMesh = new THREE.Mesh(bGeo, blackBracketMat);
    bMesh.position.set(Math.cos(angle) * (DIM.baseRadius + 0.5), 4, Math.sin(angle) * (DIM.baseRadius + 0.5));
    bMesh.rotation.y = -angle;
    baseGroup.add(bMesh);
  }

  // Top White Plate
  const topPlateGeo = new THREE.CylinderGeometry(DIM.baseRadius + 0.5, DIM.baseRadius + 0.5, 1.8, 32);
  const topPlate = new THREE.Mesh(topPlateGeo, whitePlateMat);
  topPlate.position.y = 1.25 + DIM.baseHeight + 0.9;
  topPlate.castShadow = true;
  baseGroup.add(topPlate);

  const pivotY0 = 1.25 + DIM.baseHeight + 1.8;

  // Helper to create a 150kg High-Torque Servo with dual U-bracket
  function create150kgServoMesh() {
    const sGroup = new THREE.Group();
    // Pink Aluminum Servo Body
    const bodyGeo = new THREE.BoxGeometry(6.5, 3.2, 5.8);
    const body = new THREE.Mesh(bodyGeo, pinkServoMat);
    body.castShadow = true;
    sGroup.add(body);

    // Black U-Bracket surrounding it
    const uGeo = new THREE.BoxGeometry(7.2, 3.6, 1.2);
    const u1 = new THREE.Mesh(uGeo, blackBracketMat);
    u1.position.z = 2.6;
    const u2 = new THREE.Mesh(uGeo, blackBracketMat);
    u2.position.z = -2.6;
    sGroup.add(u1, u2);

    // Circular Output Horn
    const hornGeo = new THREE.CylinderGeometry(1.6, 1.6, 0.6, 24);
    const horn = new THREE.Mesh(hornGeo, blackBracketMat);
    horn.rotation.x = Math.PI / 2;
    horn.position.z = 3.2;
    sGroup.add(horn);

    return sGroup;
  }

  // Helper to create tubular link with flared 3D-printed mounting cups
  function createTubeLink(length, radius) {
    const linkGroup = new THREE.Group();
    
    // Silver Tube
    const tubeGeo = new THREE.CylinderGeometry(radius, radius, length, 24);
    const tube = new THREE.Mesh(tubeGeo, aluminumMat);
    tube.position.y = length / 2;
    tube.castShadow = true;
    linkGroup.add(tube);

    // Bottom Flared Collar
    const collarBotGeo = new THREE.CylinderGeometry(radius * 1.5, radius * 1.8, 4, 16);
    const collarBot = new THREE.Mesh(collarBotGeo, blackBracketMat);
    collarBot.position.y = 2;
    collarBot.castShadow = true;
    linkGroup.add(collarBot);

    // Top Flared Collar
    const collarTopGeo = new THREE.CylinderGeometry(radius * 1.8, radius * 1.5, 4, 16);
    const collarTop = new THREE.Mesh(collarTopGeo, blackBracketMat);
    collarTop.position.y = length - 2;
    collarTop.castShadow = true;
    linkGroup.add(collarTop);

    return linkGroup;
  }

  // ---------------------------------------------------------
  // JOINT 1: BASE PITCH / SHOULDER (150kg)
  // ---------------------------------------------------------
  const j1Group = new THREE.Group();
  j1Group.position.set(0, pivotY0 + 2.5, 0);
  baseGroup.add(j1Group);

  const j1Mount = new THREE.Mesh(new THREE.BoxGeometry(7.5, 4, 6), blackBracketMat);
  j1Mount.position.y = -1;
  j1Group.add(j1Mount);

  const j1Rot = new THREE.Group();
  j1Group.add(j1Rot);

  const servo1 = create150kgServoMesh();
  servo1.position.y = 2.5;
  j1Rot.add(servo1);

  // LINK 1: Lower Aluminum Tube
  const link1 = createTubeLink(DIM.link1Length, 1.4);
  link1.position.y = 5;
  j1Rot.add(link1);

  // ---------------------------------------------------------
  // JOINT 2: ELBOW PITCH (150kg)
  // ---------------------------------------------------------
  const j2Group = new THREE.Group();
  j2Group.position.y = DIM.link1Length + 5;
  j1Rot.add(j2Group);

  const j2Rot = new THREE.Group();
  j2Group.add(j2Rot);

  const servo2 = create150kgServoMesh();
  servo2.position.y = 2;
  j2Rot.add(servo2);

  // LINK 2: Forearm Aluminum Tube
  const link2 = createTubeLink(DIM.link2Length, 1.4);
  link2.position.y = 4.5;
  j2Rot.add(link2);

  // ---------------------------------------------------------
  // JOINT 3: WRIST PITCH (150kg)
  // ---------------------------------------------------------
  const j3Group = new THREE.Group();
  j3Group.position.y = DIM.link2Length + 4.5;
  j2Rot.add(j3Group);

  const j3Rot = new THREE.Group();
  j3Group.add(j3Rot);

  const servo3 = create150kgServoMesh();
  servo3.position.y = 2;
  j3Rot.add(servo3);

  // LINK 3: Upper Extension Tube
  const link3 = createTubeLink(DIM.link3Length, 1.2);
  link3.position.y = 4.5;
  j3Rot.add(link3);

  // ---------------------------------------------------------
  // JOINT 4: WRIST ROLL / YAW (MG996R)
  // ---------------------------------------------------------
  const j4Group = new THREE.Group();
  j4Group.position.y = DIM.link3Length + 4.5;
  j3Rot.add(j4Group);

  const j4Rot = new THREE.Group();
  j4Group.add(j4Rot);

  const mg996Body = new THREE.Mesh(new THREE.BoxGeometry(4.2, 4.8, 2.2), blackServoMat);
  mg996Body.position.y = 2.4;
  mg996Body.castShadow = true;
  j4Rot.add(mg996Body);

  // ---------------------------------------------------------
  // JOINT 5: GRIPPER CLAW (MG996R Driven)
  // ---------------------------------------------------------
  const gripperBase = new THREE.Group();
  gripperBase.position.y = 5.2;
  j4Rot.add(gripperBase);

  const palmPlate = new THREE.Mesh(new THREE.BoxGeometry(5.5, 0.8, 3.5), blackBracketMat);
  palmPlate.castShadow = true;
  gripperBase.add(palmPlate);

  const leftJawGroup = new THREE.Group();
  leftJawGroup.position.set(-1.8, 0.4, 0);
  gripperBase.add(leftJawGroup);

  const leftJawMesh = new THREE.Mesh(new THREE.BoxGeometry(0.8, 8, 2.2), blackBracketMat);
  leftJawMesh.position.set(0, 4, 0);
  leftJawMesh.castShadow = true;
  leftJawGroup.add(leftJawMesh);

  const rightJawGroup = new THREE.Group();
  rightJawGroup.position.set(1.8, 0.4, 0);
  gripperBase.add(rightJawGroup);

  const rightJawMesh = new THREE.Mesh(new THREE.BoxGeometry(0.8, 8, 2.2), blackBracketMat);
  rightJawMesh.position.set(0, 4, 0);
  rightJawMesh.castShadow = true;
  rightJawGroup.add(rightJawMesh);

  robotArm = {
    j1Rot,
    j2Rot,
    j3Rot,
    j4Rot,
    leftJawGroup,
    rightJawGroup,
    endEffectorNode: gripperBase
  };
}

// Update 3D Robot Pose from Angles (degrees)
function updateRobotPose(angles) {
  if (!robotArm || !robotArm.j1Rot) return;

  const degToRad = (deg) => (deg - 90) * (Math.PI / 180);

  // Joint 1: Base Pitch
  robotArm.j1Rot.rotation.x = degToRad(angles[0]);

  // Joint 2: Elbow Pitch
  robotArm.j2Rot.rotation.x = degToRad(angles[1]);

  // Joint 3: Wrist Pitch
  robotArm.j3Rot.rotation.x = degToRad(angles[2]);

  // Joint 4: Wrist Roll
  robotArm.j4Rot.rotation.y = (angles[3] - 90) * (Math.PI / 180);

  // Joint 5: Gripper
  const gripRad = (angles[4] / 90) * 0.45;
  if (robotArm.leftJawGroup) robotArm.leftJawGroup.rotation.z = -gripRad;
  if (robotArm.rightJawGroup) robotArm.rightJawGroup.rotation.z = gripRad;

  // Update End-Effector Telemetry HUD
  if (isBrowser && robotArm.endEffectorNode) {
    const el = document.getElementById('hudEndEffector');
    if (el) {
      const worldPos = new THREE.Vector3();
      robotArm.endEffectorNode.getWorldPosition(worldPos);
      el.textContent = `${worldPos.x.toFixed(1)}, ${worldPos.y.toFixed(1)}, ${worldPos.z.toFixed(1)}`;
    }
  }
}

// ====================================================================================
// MAIN ANIMATION / RENDER LOOP
// ====================================================================================
let lastFrameTime = getNow();
let frameCount = 0;
let lastFpsTime = getNow();

function animate() {
  if (!isBrowser) return;

  if (typeof requestAnimationFrame === 'function') {
    requestAnimationFrame(animate);
  }

  const now = getNow();
  const dt = (now - lastFrameTime) / 1000.0;
  lastFrameTime = now;

  // FPS Counter
  frameCount++;
  if (now - lastFpsTime >= 1000) {
    const fpsEl = document.getElementById('hudFps');
    if (fpsEl) fpsEl.textContent = frameCount;
    frameCount = 0;
    lastFpsTime = now;
  }

  // Smooth interpolation of currentAngles toward targetAngles (Simulation)
  const maxStep = motionSpeed * dt;

  for (let i = 0; i < NUM_JOINTS; i++) {
    const diff = targetAngles[i] - currentAngles[i];
    if (Math.abs(diff) > 0.05) {
      if (Math.abs(diff) <= maxStep) {
        currentAngles[i] = targetAngles[i];
      } else {
        currentAngles[i] += (diff > 0) ? maxStep : -maxStep;
      }
    }
  }

  // Render arm in current interpolated pose
  updateRobotPose(currentAngles);

  // Update Trajectory Trail
  updateMotionTrail();

  // Poll USB / Bluetooth Gamepad
  pollGamepad(dt);

  // Update OrbitControls
  if (controls) controls.update();

  // Render Three.js Scene
  if (renderer && scene && camera) {
    renderer.render(scene, camera);
  }
}

function updateMotionTrail() {
  if (!isBrowser || !robotArm.endEffectorNode || !trailLine) return;
  const worldPos = new THREE.Vector3();
  robotArm.endEffectorNode.getWorldPosition(worldPos);

  if (trailPoints.length === 0 || worldPos.distanceTo(trailPoints[trailPoints.length - 1]) > 0.4) {
    trailPoints.push(worldPos.clone());
    if (trailPoints.length > MAX_TRAIL_POINTS) {
      trailPoints.shift();
    }

    const posAttr = trailLine.geometry.attributes.position;
    for (let i = 0; i < trailPoints.length; i++) {
      posAttr.setXYZ(i, trailPoints[i].x, trailPoints[i].y, trailPoints[i].z);
    }
    trailLine.geometry.setDrawRange(0, trailPoints.length);
    posAttr.needsUpdate = true;
  }
}

function pollGamepad(dt) {
  if (!isBrowser || typeof navigator === 'undefined' || !navigator.getGamepads) return;
  const gamepads = navigator.getGamepads();
  if (!gamepads || !gamepads[0]) return;
  const gp = gamepads[0];
  const deadzone = 0.2;

  // Left Stick Y (Axes 1) -> Front / Back
  if (Math.abs(gp.axes[1]) > deadzone) {
    if (gp.axes[1] < 0) {
      jogDirection('front');
    } else {
      jogDirection('back');
    }
  }
  // Left Stick X (Axes 0) -> Left / Right
  if (Math.abs(gp.axes[0]) > deadzone) {
    if (gp.axes[0] < 0) {
      jogDirection('left');
    } else {
      jogDirection('right');
    }
  }
  // Right Stick Y (Axes 3) -> Up / Down
  if (Math.abs(gp.axes[3]) > deadzone) {
    if (gp.axes[3] < 0) {
      jogDirection('up');
    } else {
      jogDirection('down');
    }
  }
  // Right Trigger (7) / Left Trigger (6) -> Gripper Clamp / Open
  if (gp.buttons[7] && gp.buttons[7].pressed) {
    jogDirection('clamp');
  } else if (gp.buttons[6] && gp.buttons[6].pressed) {
    jogDirection('open');
  }
  // Button A (0) -> Home
  if (gp.buttons[0] && gp.buttons[0].pressed) {
    applyPreset('home');
  }
  // Button B (1) -> E-Stop
  if (gp.buttons[1] && gp.buttons[1].pressed) {
    triggerEStop();
  }
}

if (typeof window !== 'undefined') {
  window.addEventListener("gamepadconnected", (e) => {
    showToast(`🎮 Gamepad Detected: ${e.gamepad.id.slice(0, 20)}...`);
  });
}

// ====================================================================================
// FORWARD KINEMATICS & UI SLIDER CONTROLS
// ====================================================================================
function initUIEvents() {
  if (!isBrowser) return;

  // Tab Switching
  document.querySelectorAll('.panel-tab').forEach(tab => {
    tab.addEventListener('click', () => {
      document.querySelectorAll('.panel-tab').forEach(t => t.classList.remove('active'));
      document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
      tab.classList.add('active');
      const target = document.getElementById(tab.dataset.tab);
      if (target) target.classList.add('active');
    });
  });

  // Joint Sliders Listeners
  for (let i = 0; i < NUM_JOINTS; i++) {
    const slider = document.getElementById(`slider_j${i + 1}`);
    const inv = document.getElementById(`inv_j${i + 1}`);

    if (slider) {
      slider.addEventListener('input', (e) => {
        let val = parseFloat(e.target.value);
        if (isInverted[i]) val = 180 - val;
        targetAngles[i] = val;
        updateAnglesDisplay();
        transmitCommandThrottled();
      });
    }

    if (inv) {
      inv.addEventListener('change', (e) => {
        isInverted[i] = e.target.checked;
        transmitCommandThrottled();
      });
    }
  }

  // Speed Slider
  const speedSlider = document.getElementById('slider_speed');
  if (speedSlider) {
    speedSlider.addEventListener('input', (e) => {
      motionSpeed = parseFloat(e.target.value);
      const valSp = document.getElementById('val_speed');
      if (valSp) valSp.textContent = motionSpeed;
      sendJsonCommand({ cmd: "set_speed", speed: motionSpeed });
    });
  }

  // Home Button
  const btnHome = document.getElementById('btnHome');
  if (btnHome) btnHome.addEventListener('click', () => applyPreset('home'));

  // Emergency Stop Button
  const eStopBtn = document.getElementById('eStopBtn');
  if (eStopBtn) eStopBtn.addEventListener('click', triggerEStop);

  // Reset Camera View
  const resetViewBtn = document.getElementById('resetViewBtn');
  if (resetViewBtn) {
    resetViewBtn.addEventListener('click', () => {
      if (camera && controls) {
        camera.position.set(130, 90, 140);
        controls.target.set(0, 55, 0);
      }
    });
  }

  // Viewport Toggles
  const btnToggleGrid = document.getElementById('btnToggleGrid');
  if (btnToggleGrid) {
    btnToggleGrid.addEventListener('click', () => {
      if (gridHelper) gridHelper.visible = !gridHelper.visible;
    });
  }
  const btnToggleAxes = document.getElementById('btnToggleAxes');
  if (btnToggleAxes) {
    btnToggleAxes.addEventListener('click', () => {
      if (axesHelper) axesHelper.visible = !axesHelper.visible;
    });
  }

  // Connection Mode Tabs (Wi-Fi vs USB Serial)
  const tabWifiBtn = document.getElementById('tabWifiBtn');
  const tabSerialBtn = document.getElementById('tabSerialBtn');
  const wifiControls = document.getElementById('wifiControls');
  const serialControls = document.getElementById('serialControls');

  if (tabWifiBtn && tabSerialBtn && wifiControls && serialControls) {
    tabWifiBtn.addEventListener('click', () => {
      tabWifiBtn.classList.add('active');
      tabSerialBtn.classList.remove('active');
      wifiControls.style.display = 'flex';
      serialControls.style.display = 'none';
      connMode = 'wifi';
    });

    tabSerialBtn.addEventListener('click', () => {
      tabSerialBtn.classList.add('active');
      tabWifiBtn.classList.remove('active');
      wifiControls.style.display = 'none';
      serialControls.style.display = 'flex';
      connMode = 'serial';
    });
  }

  // Connect Buttons
  const connectWifiBtn = document.getElementById('connectWifiBtn');
  if (connectWifiBtn) connectWifiBtn.addEventListener('click', toggleWifiConnection);

  const connectSerialBtn = document.getElementById('connectSerialBtn');
  if (connectSerialBtn) connectSerialBtn.addEventListener('click', toggleSerialConnection);

  // IK Sliders & Compute
  const sx = document.getElementById('slider_ik_x');
  const sy = document.getElementById('slider_ik_y');
  const sz = document.getElementById('slider_ik_z');
  const sp = document.getElementById('slider_ik_pitch');
  if (sx) sx.addEventListener('input', updateIkFromSliders);
  if (sy) sy.addEventListener('input', updateIkFromSliders);
  if (sz) sz.addEventListener('input', updateIkFromSliders);
  if (sp) sp.addEventListener('input', updateIkFromSliders);

  const btnSolveIk = document.getElementById('btnSolveIk');
  if (btnSolveIk) btnSolveIk.addEventListener('click', applyIkToArm);

  const btnSyncIk = document.getElementById('btnSyncIk');
  if (btnSyncIk) btnSyncIk.addEventListener('click', syncIkSlidersFromWorld);

  // Sequencer Controls
  const btnAddKeyframe = document.getElementById('btnAddKeyframe');
  if (btnAddKeyframe) btnAddKeyframe.addEventListener('click', addCurrentPoseAsKeyframe);

  const btnPlaySeq = document.getElementById('btnPlaySeq');
  if (btnPlaySeq) btnPlaySeq.addEventListener('click', playSequence);

  const btnStopSeq = document.getElementById('btnStopSeq');
  if (btnStopSeq) btnStopSeq.addEventListener('click', stopSequence);

  const btnClearSeq = document.getElementById('btnClearSeq');
  if (btnClearSeq) {
    btnClearSeq.addEventListener('click', () => {
      keyframes = [];
      renderKeyframesList();
    });
  }

  const btnExportSeq = document.getElementById('btnExportSeq');
  if (btnExportSeq) btnExportSeq.addEventListener('click', exportSequenceJson);

  const btnImportSeq = document.getElementById('btnImportSeq');
  const seqFileInput = document.getElementById('seqFileInput');
  if (btnImportSeq && seqFileInput) {
    btnImportSeq.addEventListener('click', () => seqFileInput.click());
    seqFileInput.addEventListener('change', importSequenceJson);
  }

  // Load saved trims from safeStorage
  const saved = safeStorage.getItem('roboarm_trims');
  if (saved) {
    try {
      jointTrims = JSON.parse(saved);
      for (let i = 0; i < NUM_JOINTS; i++) {
        const slider = document.getElementById(`trim_j${i + 1}`);
        const val = document.getElementById(`val_trim_${i + 1}`);
        if (slider) slider.value = jointTrims[i];
        if (val) val.textContent = jointTrims[i];
      }
    } catch (e) {}
  }

  // Trim Sliders
  for (let i = 0; i < NUM_JOINTS; i++) {
    const slider = document.getElementById(`trim_j${i + 1}`);
    const val = document.getElementById(`val_trim_${i + 1}`);
    if (slider) {
      slider.addEventListener('input', (e) => {
        jointTrims[i] = parseInt(e.target.value) || 0;
        if (val) val.textContent = jointTrims[i];
        transmitCommandThrottled();
      });
    }
  }

  // Reset Trims
  const btnResetTrims = document.getElementById('btnResetTrims');
  if (btnResetTrims) {
    btnResetTrims.addEventListener('click', () => {
      jointTrims = [0, 0, 0, 0, 0];
      for (let i = 0; i < NUM_JOINTS; i++) {
        const slider = document.getElementById(`trim_j${i + 1}`);
        const val = document.getElementById(`val_trim_${i + 1}`);
        if (slider) slider.value = 0;
        if (val) val.textContent = 0;
      }
      safeStorage.removeItem('roboarm_trims');
      showToast("Trims reset to zero.");
      transmitCommandThrottled();
    });
  }

  // Save Config
  const btnSaveConfig = document.getElementById('btnSaveConfig');
  if (btnSaveConfig) {
    btnSaveConfig.addEventListener('click', () => {
      safeStorage.setItem('roboarm_trims', JSON.stringify(jointTrims));
      showToast("Calibration trims saved to browser memory!");
    });
  }

  // Bind On-Screen Jog Pendant Buttons
  const bindJog = (id, dir) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener('mousedown', (e) => { e.preventDefault(); startContinuousJog(dir); });
    el.addEventListener('mouseup', stopContinuousJog);
    el.addEventListener('mouseleave', stopContinuousJog);
    el.addEventListener('touchstart', (e) => { e.preventDefault(); startContinuousJog(dir); });
    el.addEventListener('touchend', stopContinuousJog);
  };

  bindJog('btnJogFront', 'front');
  bindJog('btnJogBack', 'back');
  bindJog('btnJogLeft', 'left');
  bindJog('btnJogRight', 'right');
  bindJog('btnJogUp', 'up');
  bindJog('btnJogDown', 'down');
  bindJog('btnJogGripClose', 'clamp');
  bindJog('btnJogGripOpen', 'open');

  const btnJogCenter = document.getElementById('btnJogCenter');
  if (btnJogCenter) {
    btnJogCenter.addEventListener('click', () => applyPreset('home'));
  }

  // Keyboard Shortcuts
  if (typeof window !== 'undefined') {
    window.addEventListener('keydown', (e) => {
      if (e.target && e.target.tagName === 'INPUT') return;
      const key = e.key ? e.key.toLowerCase() : '';

      // Arrow Keys Jog
      if (e.key === 'ArrowUp') {
        e.preventDefault();
        jogDirection('front');
      } else if (e.key === 'ArrowDown') {
        e.preventDefault();
        jogDirection('back');
      } else if (e.key === 'ArrowLeft') {
        e.preventDefault();
        jogDirection('left');
      } else if (e.key === 'ArrowRight') {
        e.preventDefault();
        jogDirection('right');
      } else if (e.key === 'PageUp' || key === 'u') {
        e.preventDefault();
        jogDirection('up');
      } else if (e.key === 'PageDown' || key === 'j') {
        e.preventDefault();
        jogDirection('down');
      }
      // Individual Joint Key Controls
      else if (key === 'q') stepJoint(0, 3);
      else if (key === 'a') stepJoint(0, -3);
      else if (key === 'w') stepJoint(1, 3);
      else if (key === 's') stepJoint(1, -3);
      else if (key === 'e') stepJoint(2, 3);
      else if (key === 'd') stepJoint(2, -3);
      else if (key === 'r') stepJoint(3, 3);
      else if (key === 'f') stepJoint(3, -3);
      else if (e.code === 'Space') {
        e.preventDefault();
        targetAngles[4] = (targetAngles[4] > 40) ? 0 : 80;
        updateAnglesDisplay();
        transmitCommandThrottled();
      } else if (key === 'h') {
        applyPreset('home');
      } else if (key === 'escape') {
        triggerEStop();
      }
    });
  }
}

// Step joint button handler (+1, -1, +5, -5)
function stepJoint(index, delta) {
  let val = targetAngles[index] + delta;
  const max = (index === 4) ? 90 : 180;
  val = Math.max(0, Math.min(max, val));
  targetAngles[index] = val;
  updateAnglesDisplay();
  transmitCommandThrottled();
}

function setGripper(angle) {
  targetAngles[4] = angle;
  updateAnglesDisplay();
  transmitCommandThrottled();
}

// ====================================================================================
// DIRECTIONAL JOG SYSTEM (FRONT, BACKWARD, LEFT, RIGHT, UP, DOWN)
// ====================================================================================
let jogInterval = null;

function jogDirection(dir) {
  const step = 2.5;

  if (dir === 'front') {
    targetAngles[0] = Math.min(180, targetAngles[0] + step);
    targetAngles[1] = Math.max(0, targetAngles[1] - (step * 0.7));
  } else if (dir === 'back') {
    targetAngles[0] = Math.max(0, targetAngles[0] - step);
    targetAngles[1] = Math.min(180, targetAngles[1] + (step * 0.7));
  } else if (dir === 'left') {
    targetAngles[3] = Math.max(0, targetAngles[3] - step * 1.5);
  } else if (dir === 'right') {
    targetAngles[3] = Math.min(180, targetAngles[3] + step * 1.5);
  } else if (dir === 'up') {
    targetAngles[0] = Math.max(0, targetAngles[0] - step);
    targetAngles[1] = Math.max(0, targetAngles[1] - step);
    targetAngles[2] = Math.min(180, targetAngles[2] + step);
  } else if (dir === 'down') {
    targetAngles[0] = Math.min(180, targetAngles[0] + step);
    targetAngles[1] = Math.min(180, targetAngles[1] + step);
    targetAngles[2] = Math.max(0, targetAngles[2] - step);
  } else if (dir === 'clamp') {
    targetAngles[4] = Math.max(0, targetAngles[4] - 5);
  } else if (dir === 'open') {
    targetAngles[4] = Math.min(90, targetAngles[4] + 5);
  }

  updateAnglesDisplay();
  transmitCommandThrottled();
}

function startContinuousJog(dir) {
  stopContinuousJog();
  jogDirection(dir);
  jogInterval = setInterval(() => {
    jogDirection(dir);
  }, 45);
}

function stopContinuousJog() {
  if (jogInterval) {
    clearInterval(jogInterval);
    jogInterval = null;
  }
}

// Update UI values and sliders
function updateAnglesDisplay() {
  if (!isBrowser) return;
  for (let i = 0; i < NUM_JOINTS; i++) {
    const valSpan = document.getElementById(`val_j${i + 1}`);
    const slider = document.getElementById(`slider_j${i + 1}`);
    if (valSpan) valSpan.textContent = Math.round(targetAngles[i]);
    if (slider) slider.value = Math.round(targetAngles[i]);
  }
  const gripText = document.getElementById('gripperStateText');
  if (gripText) {
    if (targetAngles[4] <= 10) gripText.textContent = "(Clamped)";
    else if (targetAngles[4] >= 75) gripText.textContent = "(Wide Open)";
    else gripText.textContent = "(Partial)";
  }
}

// Apply Preset Pose
function applyPreset(name) {
  if (PRESETS[name]) {
    targetAngles = [...PRESETS[name]];
    updateAnglesDisplay();
    transmitCommandThrottled();
    showToast(`Applied Preset: ${name.toUpperCase()}`);
  }
}

// Emergency Stop
function triggerEStop() {
  for (let i = 0; i < NUM_JOINTS; i++) {
    targetAngles[i] = currentAngles[i];
  }
  stopSequence();
  updateAnglesDisplay();
  sendJsonCommand({ cmd: "stop" });
  if (connMode === 'serial' && serialWriter) {
    try { serialWriter.write("STOP\n"); } catch (e) {}
  }
  showToast("EMERGENCY STOP ACTIVATED!", true);
}

// ====================================================================================
// INVERSE KINEMATICS (IK SOLVER)
// ====================================================================================
function updateIkFromSliders() {
  if (!isBrowser) return;
  const sx = document.getElementById('slider_ik_x');
  const sy = document.getElementById('slider_ik_y');
  const sz = document.getElementById('slider_ik_z');
  const sp = document.getElementById('slider_ik_pitch');

  const x = sx ? parseFloat(sx.value) : 35;
  const y = sy ? parseFloat(sy.value) : 60;
  const z = sz ? parseFloat(sz.value) : 0;
  const pitch = sp ? parseFloat(sp.value) : 0;

  const vx = document.getElementById('val_ik_x');
  const vy = document.getElementById('val_ik_y');
  const vz = document.getElementById('val_ik_z');
  const vp = document.getElementById('val_ik_pitch');
  if (vx) vx.textContent = x;
  if (vy) vy.textContent = y;
  if (vz) vz.textContent = z;
  if (vp) vp.textContent = pitch;

  // Move 3D Target Mesh
  if (ikTargetMesh) {
    ikTargetMesh.position.set(x, y, z);
  }

  solveAnalyticalIK(x, y, z, pitch, false);
}

function syncIkSlidersFromWorld() {
  if (!isBrowser || !robotArm.endEffectorNode) return;
  const worldPos = new THREE.Vector3();
  robotArm.endEffectorNode.getWorldPosition(worldPos);

  const sx = document.getElementById('slider_ik_x');
  const sy = document.getElementById('slider_ik_y');
  const sz = document.getElementById('slider_ik_z');
  if (sx) sx.value = Math.round(worldPos.x);
  if (sy) sy.value = Math.round(worldPos.y);
  if (sz) sz.value = Math.round(worldPos.z);
  updateIkFromSliders();
}

function solveAnalyticalIK(targetX, targetY, targetZ, wristPitchDeg, apply = false) {
  const L1 = DIM.link1Length; // 42 cm
  const L2 = DIM.link2Length; // 40 cm
  const baseH = DIM.baseHeight + 4.3;

  const dy = targetY - baseH;
  const r = Math.sqrt(targetX * targetX + targetZ * targetZ);

  const D = Math.sqrt(r * r + dy * dy);
  const notice = isBrowser ? document.getElementById('ikResultNotice') : null;

  // Reach check
  if (D > (L1 + L2) * 0.98 || D < Math.abs(L1 - L2) * 1.05) {
    if (notice) {
      notice.textContent = "Out of Reach Workspace";
      notice.style.color = "#f87171";
      notice.style.background = "rgba(239, 68, 68, 0.15)";
    }
    return false;
  } else {
    if (notice) {
      notice.textContent = "Target Reachable (Valid Solution)";
      notice.style.color = "#34d399";
      notice.style.background = "rgba(16, 185, 129, 0.15)";
    }
  }

  // Law of Cosines for elbow angle
  const cosElbow = (L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2);
  const theta2Rad = Math.PI - Math.acos(Math.max(-1, Math.min(1, cosElbow)));

  // Shoulder angle
  const alpha = Math.atan2(dy, r);
  const beta = Math.acos(Math.max(-1, Math.min(1, (L1 * L1 + D * D - L2 * L2) / (2 * L1 * D))));
  const theta1Rad = alpha + beta;

  // Convert to servo degrees
  let j1 = Math.round((Math.PI / 2 - theta1Rad) * (180 / Math.PI) + 90);
  let j2 = Math.round(180 - (theta2Rad * (180 / Math.PI)));
  let j3 = Math.round(90 + wristPitchDeg);
  let j4 = 90;

  j1 = Math.max(0, Math.min(180, j1));
  j2 = Math.max(0, Math.min(180, j2));
  j3 = Math.max(0, Math.min(180, j3));

  if (apply) {
    targetAngles[0] = j1;
    targetAngles[1] = j2;
    targetAngles[2] = j3;
    targetAngles[3] = j4;
    updateAnglesDisplay();
    transmitCommandThrottled();
    showToast(`Applied IK: [${j1}°, ${j2}°, ${j3}°]`);
  }

  return true;
}

function applyIkToArm() {
  if (!isBrowser) return;
  const sx = document.getElementById('slider_ik_x');
  const sy = document.getElementById('slider_ik_y');
  const sz = document.getElementById('slider_ik_z');
  const sp = document.getElementById('slider_ik_pitch');
  const x = sx ? parseFloat(sx.value) : 35;
  const y = sy ? parseFloat(sy.value) : 60;
  const z = sz ? parseFloat(sz.value) : 0;
  const pitch = sp ? parseFloat(sp.value) : 0;
  solveAnalyticalIK(x, y, z, pitch, true);
}

// ====================================================================================
// MOTION SEQUENCER & KEYFRAME RECORDER
// ====================================================================================
function renderKeyframesList() {
  if (!isBrowser) return;
  const container = document.getElementById('keyframeList');
  if (!container) return;
  container.innerHTML = '';

  keyframes.forEach((kf, idx) => {
    const item = document.createElement('div');
    item.className = `keyframe-item ${idx === seqIndex && isPlayingSeq ? 'active' : ''}`;
    item.innerHTML = `
      <div>
        <strong>${kf.name}</strong>
        <div style="color:var(--text-muted);font-size:0.7rem;">[${kf.angles.join(', ')}] • ${kf.duration}ms</div>
      </div>
      <div style="display:flex;gap:8px;align-items:center;">
        <button class="step-btn" onclick="testKeyframe(${idx})" title="Test Pose"><i class="fa-solid fa-arrow-right"></i></button>
        <i class="fa-solid fa-trash keyframe-del" onclick="deleteKeyframe(${idx})" title="Delete"></i>
      </div>
    `;
    container.appendChild(item);
  });
}

function addCurrentPoseAsKeyframe() {
  const name = `Pose #${keyframes.length + 1}`;
  keyframes.push({
    name: name,
    angles: [...targetAngles],
    duration: 1500
  });
  renderKeyframesList();
  showToast(`Recorded: ${name}`);
}

function deleteKeyframe(idx) {
  keyframes.splice(idx, 1);
  renderKeyframesList();
}

function testKeyframe(idx) {
  if (keyframes[idx]) {
    targetAngles = [...keyframes[idx].angles];
    updateAnglesDisplay();
    transmitCommandThrottled();
  }
}

function playSequence() {
  if (keyframes.length === 0) {
    showToast("No keyframes recorded!");
    return;
  }
  isPlayingSeq = true;
  seqIndex = 0;
  executeNextKeyframe();
}

function stopSequence() {
  isPlayingSeq = false;
  clearTimeout(seqTimer);
  renderKeyframesList();
}

function executeNextKeyframe() {
  if (!isPlayingSeq) return;

  if (seqIndex >= keyframes.length) {
    const loopEl = isBrowser ? document.getElementById('seqLoop') : null;
    const isLoop = loopEl ? loopEl.checked : false;
    if (isLoop) {
      seqIndex = 0;
    } else {
      stopSequence();
      showToast("Sequence Completed.");
      return;
    }
  }

  const kf = keyframes[seqIndex];
  targetAngles = [...kf.angles];
  updateAnglesDisplay();
  transmitCommandThrottled();
  renderKeyframesList();

  seqTimer = setTimeout(() => {
    seqIndex++;
    executeNextKeyframe();
  }, kf.duration);
}

function exportSequenceJson() {
  if (!isBrowser) return;
  const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(keyframes, null, 2));
  const dlAnchor = document.createElement('a');
  dlAnchor.setAttribute("href", dataStr);
  dlAnchor.setAttribute("download", "robot_arm_routine.json");
  dlAnchor.click();
}

function importSequenceJson(e) {
  if (!isBrowser || !e.target.files) return;
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (event) => {
    try {
      const parsed = JSON.parse(event.target.result);
      if (Array.isArray(parsed)) {
        keyframes = parsed;
        renderKeyframesList();
        showToast("Keyframe routine imported!");
      }
    } catch (err) {
      showToast("Invalid JSON file", true);
    }
  };
  reader.readAsText(file);
}

// ====================================================================================
// HARDWARE COMMUNICATION: WEBSOCKET & WEB SERIAL API
// ====================================================================================
function transmitCommandThrottled() {
  const now = getNow();
  if (now - lastSendTime < 35) return; // 30Hz limit
  lastSendTime = now;

  // Apply calibration trims and clamp
  const actualAngles = targetAngles.map((ang, i) => {
    let finalAng = ang + (jointTrims[i] || 0);
    const max = (i === 4) ? 90 : 180;
    return Math.max(0, Math.min(max, Math.round(finalAng)));
  });

  const payload = {
    cmd: "set_angles",
    angles: actualAngles,
    speed: motionSpeed
  };

  sendJsonCommand(payload);

  // If USB Serial mode is active, send fast compact string: S:90,90,90,90,20\n
  if (connMode === 'serial' && serialWriter) {
    const cmdStr = `S:${actualAngles.join(',')}\n`;
    try {
      serialWriter.write(cmdStr);
    } catch (e) {
      console.warn("Serial write error:", e);
    }
  }
}

function sendJsonCommand(obj) {
  if (typeof WebSocket !== 'undefined' && ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

// --- WebSocket Handler ---
function toggleWifiConnection() {
  if (!isBrowser) return;

  if (ws) {
    ws.close();
    ws = null;
    updateConnStatus(false);
    const btn = document.getElementById('connectWifiBtn');
    if (btn) btn.innerHTML = `<i class="fa-solid fa-link"></i> Connect`;
    return;
  }

  const ipEl = document.getElementById('espIpInput');
  const ip = ipEl ? ipEl.value.trim() : '';
  if (!ip) {
    showToast("Enter a valid IP Address", true);
    return;
  }

  const wsUrl = `ws://${ip}:81/`;
  showToast(`Connecting to ${wsUrl}...`);

  try {
    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      updateConnStatus(true, `Connected (WS @ ${ip})`);
      const btn = document.getElementById('connectWifiBtn');
      if (btn) btn.innerHTML = `<i class="fa-solid fa-unlink"></i> Disconnect`;
      showToast("ESP8266 Connected via Wi-Fi!");
      sendJsonCommand({ cmd: "get_status" });
      pingStartTime = getNow();
    };

    ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === "telemetry" && msg.current) {
          const latency = Math.round(getNow() - pingStartTime);
          const badge = document.getElementById('latencyBadge');
          if (badge) badge.textContent = `${Math.min(latency, 99)} ms`;
          pingStartTime = getNow();
        }
      } catch (err) {}
    };

    ws.onerror = (err) => {
      console.error("WebSocket Error:", err);
      updateConnStatus(false);
    };

    ws.onclose = () => {
      ws = null;
      updateConnStatus(false);
      const btn = document.getElementById('connectWifiBtn');
      if (btn) btn.innerHTML = `<i class="fa-solid fa-link"></i> Connect`;
      showToast("ESP8266 Disconnected");
    };

  } catch (err) {
    showToast(`Connection failed: ${err.message}`, true);
    updateConnStatus(false);
  }
}

// --- Web Serial API Handler (USB Cable Direct) ---
async function toggleSerialConnection() {
  if (!isBrowser) return;

  if (serialPort) {
    try {
      if (serialReader) await serialReader.cancel();
      if (serialWriter) await serialWriter.close();
      await serialPort.close();
    } catch (e) {}
    serialPort = null;
    serialWriter = null;
    serialReader = null;
    updateConnStatus(false);
    const btn = document.getElementById('connectSerialBtn');
    if (btn) btn.innerHTML = `<i class="fa-solid fa-bolt"></i> Connect USB Port`;
    showToast("USB Port Disconnected");
    return;
  }

  if (typeof navigator === 'undefined' || !("serial" in navigator)) {
    showToast("Web Serial is not supported in this browser. Use Chrome or Edge!", true);
    return;
  }

  try {
    serialPort = await navigator.serial.requestPort();
    await serialPort.open({ baudRate: 115200 });

    const textEncoder = new TextEncoderStream();
    textEncoder.readable.pipeTo(serialPort.writable);
    serialWriter = textEncoder.writable.getWriter();

    updateConnStatus(true, "Connected (USB Serial @ 115200)");
    const btn = document.getElementById('connectSerialBtn');
    if (btn) btn.innerHTML = `<i class="fa-solid fa-unlink"></i> Disconnect Port`;
    showToast("USB Serial Port Connected!");

    readSerialLoop();

  } catch (err) {
    showToast(`Serial error: ${err.message}`, true);
    updateConnStatus(false);
  }
}

async function readSerialLoop() {
  if (!isBrowser || !serialPort || !serialPort.readable) return;
  const textDecoder = new TextDecoderStream();
  serialPort.readable.pipeTo(textDecoder.writable);
  serialReader = textDecoder.readable.getReader();

  try {
    while (true) {
      const { value, done } = await serialReader.read();
      if (done) break;
    }
  } catch (error) {
    console.warn("Serial read loop closed:", error);
  } finally {
    if (serialReader) serialReader.releaseLock();
  }
}

function updateConnStatus(connected, text = "Disconnected (Sim Only)") {
  if (!isBrowser) return;
  isConnected = connected;
  const led = document.getElementById('statusLed');
  const txt = document.getElementById('statusText');
  const badge = document.getElementById('latencyBadge');

  if (connected) {
    if (led) led.className = "led connected";
    if (txt) txt.textContent = text;
    if (badge) badge.textContent = "Online";
  } else {
    if (led) led.className = "led disconnected";
    if (txt) txt.textContent = "Disconnected (Sim Only)";
    if (badge) badge.textContent = "-- ms";
  }
}

// Notification Toast Utility
function showToast(msg, isError = false) {
  if (!isBrowser) return;
  const toast = document.getElementById('toast');
  if (!toast) return;
  toast.textContent = msg;
  toast.style.borderColor = isError ? "#ef4444" : "#38bdf8";
  toast.classList.add('show');
  setTimeout(() => toast.classList.remove('show'), 2800);
}

// Expose public functions to global scope for HTML onclick bindings
globalScope.stepJoint = stepJoint;
globalScope.setGripper = setGripper;
globalScope.jogDirection = jogDirection;
globalScope.startContinuousJog = startContinuousJog;
globalScope.stopContinuousJog = stopContinuousJog;
globalScope.applyPreset = applyPreset;
globalScope.deleteKeyframe = deleteKeyframe;
globalScope.testKeyframe = testKeyframe;

// Node.js CommonJS Module Export
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    NUM_JOINTS,
    targetAngles,
    currentAngles,
    motionSpeed,
    PRESETS,
    DIM,
    stepJoint,
    setGripper,
    jogDirection,
    applyPreset,
    solveAnalyticalIK
  };
}
