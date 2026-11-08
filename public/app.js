/*
  ====================================================================================
  5-DOF ROBOTIC ARM - 3D SIMULATION & REAL-TIME CONTROLLER
  ====================================================================================
  Three.js Kinematics, WebSockets, Web Serial API, and Motion Choreographer.
*/

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
let scene, camera, renderer, controls;
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

// --- Initialize App ---
window.addEventListener('DOMContentLoaded', () => {
  initThree();
  initUIEvents();
  renderKeyframesList();
  updateAnglesDisplay();
  animate();
});

// Motion Trail Globals
const MAX_TRAIL_POINTS = 120;
let trailPoints = [];
let trailLine = null;

// ====================================================================================
// THREE.JS 3D ROBOT RIGGING & SCENE BUILDER
// ====================================================================================
function initThree() {
  const container = document.getElementById('canvas-container');
  const width = container.clientWidth;
  const height = container.clientHeight;

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
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  // OrbitControls
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.maxPolarAngle = Math.PI / 2 - 0.02; // Don't go below floor
  controls.target.set(0, 55, 0);

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
      controls.enabled = !event.value;
    });

    transformControls.addEventListener('change', () => {
      if (transformControls.dragging) {
        const pos = ikTargetMesh.position;
        document.getElementById('slider_ik_x').value = Math.round(pos.x);
        document.getElementById('slider_ik_y').value = Math.round(pos.y);
        document.getElementById('slider_ik_z').value = Math.round(pos.z);
        document.getElementById('val_ik_x').textContent = Math.round(pos.x);
        document.getElementById('val_ik_y').textContent = Math.round(pos.y);
        document.getElementById('val_ik_z').textContent = Math.round(pos.z);
        const pitch = parseFloat(document.getElementById('slider_ik_pitch').value);
        solveAnalyticalIK(pos.x, pos.y, pos.z, pitch, true);
      }
    });
  }

  // Build the 3D Robot Arm Assembly
  buildRobotHierarchy();

  // Resize handler
  window.addEventListener('resize', onWindowResize);
}

function onWindowResize() {
  const container = document.getElementById('canvas-container');
  if (!container) return;
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}

// ====================================================================================
// 3D PROCEDURAL ROBOT RIG (Authentic to User Photos)
// ====================================================================================
function buildRobotHierarchy() {
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

  // Mount bracket on top plate
  const j1Mount = new THREE.Mesh(new THREE.BoxGeometry(7.5, 4, 6), blackBracketMat);
  j1Mount.position.y = -1;
  j1Group.add(j1Mount);

  // Rotating Shoulder Axis
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

  // Black MG996R Servo Body
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

  // Gripper mount plate
  const palmPlate = new THREE.Mesh(new THREE.BoxGeometry(5.5, 0.8, 3.5), blackBracketMat);
  palmPlate.castShadow = true;
  gripperBase.add(palmPlate);

  // Left Claw Jaw
  const leftJawGroup = new THREE.Group();
  leftJawGroup.position.set(-1.8, 0.4, 0);
  gripperBase.add(leftJawGroup);

  const leftJawMesh = new THREE.Mesh(new THREE.BoxGeometry(0.8, 8, 2.2), blackBracketMat);
  leftJawMesh.position.set(0, 4, 0);
  leftJawMesh.castShadow = true;
  leftJawGroup.add(leftJawMesh);

  // Right Claw Jaw
  const rightJawGroup = new THREE.Group();
  rightJawGroup.position.set(1.8, 0.4, 0);
  gripperBase.add(rightJawGroup);

  const rightJawMesh = new THREE.Mesh(new THREE.BoxGeometry(0.8, 8, 2.2), blackBracketMat);
  rightJawMesh.position.set(0, 4, 0);
  rightJawMesh.castShadow = true;
  rightJawGroup.add(rightJawMesh);

  // Store pointers in robotArm object for easy joint rotation
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
  if (!robotArm.j1Rot) return;

  // Convert 0..180 deg to radians centered around 90 deg (0 = -PI/2, 90 = 0, 180 = +PI/2)
  const degToRad = (deg) => (deg - 90) * (Math.PI / 180);

  // Joint 1: Base Pitch (Rotates around X)
  robotArm.j1Rot.rotation.x = degToRad(angles[0]);

  // Joint 2: Elbow Pitch (Rotates around X)
  robotArm.j2Rot.rotation.x = degToRad(angles[1]);

  // Joint 3: Wrist Pitch (Rotates around X)
  robotArm.j3Rot.rotation.x = degToRad(angles[2]);

  // Joint 4: Wrist Roll / Yaw (Rotates around Y)
  robotArm.j4Rot.rotation.y = (angles[3] - 90) * (Math.PI / 180);

  // Joint 5: Gripper (0 = closed/touching, 90 = wide open)
  const gripRad = (angles[4] / 90) * 0.45; // Jaw pivot range
  robotArm.leftJawGroup.rotation.z = -gripRad;
  robotArm.rightJawGroup.rotation.z = gripRad;

  // Update End-Effector Telemetry HUD
  if (robotArm.endEffectorNode) {
    const worldPos = new THREE.Vector3();
    robotArm.endEffectorNode.getWorldPosition(worldPos);
    document.getElementById('hudEndEffector').textContent = 
      `${worldPos.x.toFixed(1)}, ${worldPos.y.toFixed(1)}, ${worldPos.z.toFixed(1)}`;
  }
}

// ====================================================================================
// MAIN ANIMATION / RENDER LOOP
// ====================================================================================
let lastFrameTime = performance.now();
let frameCount = 0;
let lastFpsTime = performance.now();

function animate() {
  requestAnimationFrame(animate);

  const now = performance.now();
  const dt = (now - lastFrameTime) / 1000.0;
  lastFrameTime = now;

  // FPS Counter
  frameCount++;
  if (now - lastFpsTime >= 1000) {
    document.getElementById('hudFps').textContent = frameCount;
    frameCount = 0;
    lastFpsTime = now;
  }

  // Smooth interpolation of currentAngles toward targetAngles (Simulation)
  const maxStep = motionSpeed * dt;
  let hasMoved = false;

  for (let i = 0; i < NUM_JOINTS; i++) {
    const diff = targetAngles[i] - currentAngles[i];
    if (Math.abs(diff) > 0.05) {
      if (Math.abs(diff) <= maxStep) {
        currentAngles[i] = targetAngles[i];
      } else {
        currentAngles[i] += (diff > 0) ? maxStep : -maxStep;
      }
      hasMoved = true;
    }
  }

  // Render arm in current interpolated pose
  updateRobotPose(currentAngles);

  // Update Trajectory Trail
  updateMotionTrail();

  // Poll USB / Bluetooth Gamepad
  pollGamepad(dt);

  // Update OrbitControls
  controls.update();

  // Render Three.js Scene
  renderer.render(scene, camera);
}

function updateMotionTrail() {
  if (!robotArm.endEffectorNode || !trailLine) return;
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
  const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
  if (!gamepads || !gamepads[0]) return;
  const gp = gamepads[0];
  const deadzone = 0.2;

  // Left Stick Y (Axes 1) -> Front / Back
  if (Math.abs(gp.axes[1]) > deadzone) {
    if (gp.axes[1] < 0) {
      window.jogDirection('front');
    } else {
      window.jogDirection('back');
    }
  }
  // Left Stick X (Axes 0) -> Left / Right
  if (Math.abs(gp.axes[0]) > deadzone) {
    if (gp.axes[0] < 0) {
      window.jogDirection('left');
    } else {
      window.jogDirection('right');
    }
  }
  // Right Stick Y (Axes 3) -> Up / Down
  if (Math.abs(gp.axes[3]) > deadzone) {
    if (gp.axes[3] < 0) {
      window.jogDirection('up');
    } else {
      window.jogDirection('down');
    }
  }
  // Right Trigger (7) / Left Trigger (6) -> Gripper Clamp / Open
  if (gp.buttons[7] && gp.buttons[7].pressed) {
    window.jogDirection('clamp');
  } else if (gp.buttons[6] && gp.buttons[6].pressed) {
    window.jogDirection('open');
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

window.addEventListener("gamepadconnected", (e) => {
  showToast(`🎮 Gamepad Detected: ${e.gamepad.id.slice(0, 20)}...`);
});

// ====================================================================================
// FORWARD KINEMATICS & UI SLIDER CONTROLS
// ====================================================================================
function initUIEvents() {
  // Tab Switching
  document.querySelectorAll('.panel-tab').forEach(tab => {
    tab.addEventListener('click', () => {
      document.querySelectorAll('.panel-tab').forEach(t => t.classList.remove('active'));
      document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
      tab.classList.add('active');
      document.getElementById(tab.dataset.tab).classList.add('active');
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
  speedSlider.addEventListener('input', (e) => {
    motionSpeed = parseFloat(e.target.value);
    document.getElementById('val_speed').textContent = motionSpeed;
    sendJsonCommand({ cmd: "set_speed", speed: motionSpeed });
  });

  // Home Button
  document.getElementById('btnHome').addEventListener('click', () => applyPreset('home'));

  // Emergency Stop Button
  document.getElementById('eStopBtn').addEventListener('click', triggerEStop);

  // Reset Camera View
  document.getElementById('resetViewBtn').addEventListener('click', () => {
    camera.position.set(130, 90, 140);
    controls.target.set(0, 55, 0);
  });

  // Viewport Toggles
  document.getElementById('btnToggleGrid').addEventListener('click', () => {
    gridHelper.visible = !gridHelper.visible;
  });
  document.getElementById('btnToggleAxes').addEventListener('click', () => {
    axesHelper.visible = !axesHelper.visible;
  });

  // Connection Mode Tabs (Wi-Fi vs USB Serial)
  const tabWifiBtn = document.getElementById('tabWifiBtn');
  const tabSerialBtn = document.getElementById('tabSerialBtn');
  const wifiControls = document.getElementById('wifiControls');
  const serialControls = document.getElementById('serialControls');

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

  // Connect Buttons
  document.getElementById('connectWifiBtn').addEventListener('click', toggleWifiConnection);
  document.getElementById('connectSerialBtn').addEventListener('click', toggleSerialConnection);

  // IK Sliders & Compute
  document.getElementById('slider_ik_x').addEventListener('input', updateIkFromSliders);
  document.getElementById('slider_ik_y').addEventListener('input', updateIkFromSliders);
  document.getElementById('slider_ik_z').addEventListener('input', updateIkFromSliders);
  document.getElementById('slider_ik_pitch').addEventListener('input', updateIkFromSliders);
  document.getElementById('btnSolveIk').addEventListener('click', applyIkToArm);
  document.getElementById('btnSyncIk').addEventListener('click', syncIkSlidersFromWorld);

  // Sequencer Controls
  document.getElementById('btnAddKeyframe').addEventListener('click', addCurrentPoseAsKeyframe);
  document.getElementById('btnPlaySeq').addEventListener('click', playSequence);
  document.getElementById('btnStopSeq').addEventListener('click', stopSequence);
  document.getElementById('btnClearSeq').addEventListener('click', () => {
    keyframes = [];
    renderKeyframesList();
  });
  document.getElementById('btnExportSeq').addEventListener('click', exportSequenceJson);
  document.getElementById('btnImportSeq').addEventListener('click', () => {
    document.getElementById('seqFileInput').click();
  });
  document.getElementById('seqFileInput').addEventListener('change', importSequenceJson);

  // Load saved trims from localStorage
  try {
    const saved = localStorage.getItem('roboarm_trims');
    if (saved) {
      jointTrims = JSON.parse(saved);
      for (let i = 0; i < NUM_JOINTS; i++) {
        const slider = document.getElementById(`trim_j${i + 1}`);
        const val = document.getElementById(`val_trim_${i + 1}`);
        if (slider) slider.value = jointTrims[i];
        if (val) val.textContent = jointTrims[i];
      }
    }
  } catch (e) {}

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
      localStorage.removeItem('roboarm_trims');
      showToast("Trims reset to zero.");
      transmitCommandThrottled();
    });
  }

  // Save Config
  const btnSaveConfig = document.getElementById('btnSaveConfig');
  if (btnSaveConfig) {
    btnSaveConfig.addEventListener('click', () => {
      try {
        localStorage.setItem('roboarm_trims', JSON.stringify(jointTrims));
        showToast("Calibration trims saved to browser memory!");
      } catch (e) {
        showToast("Error saving configuration", true);
      }
    });
  }

  // Bind On-Screen Jog Pendant Buttons
  const bindJog = (id, dir) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener('mousedown', (e) => { e.preventDefault(); window.startContinuousJog(dir); });
    el.addEventListener('mouseup', window.stopContinuousJog);
    el.addEventListener('mouseleave', window.stopContinuousJog);
    el.addEventListener('touchstart', (e) => { e.preventDefault(); window.startContinuousJog(dir); });
    el.addEventListener('touchend', window.stopContinuousJog);
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

  // Keyboard Shortcuts (Arrow Keys for Front/Back/Left/Right)
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT') return;
    const key = e.key.toLowerCase();

    // Arrow Keys Jog
    if (e.key === 'ArrowUp') {
      e.preventDefault();
      window.jogDirection('front');
    } else if (e.key === 'ArrowDown') {
      e.preventDefault();
      window.jogDirection('back');
    } else if (e.key === 'ArrowLeft') {
      e.preventDefault();
      window.jogDirection('left');
    } else if (e.key === 'ArrowRight') {
      e.preventDefault();
      window.jogDirection('right');
    } else if (e.key === 'PageUp' || key === 'u') {
      e.preventDefault();
      window.jogDirection('up');
    } else if (e.key === 'PageDown' || key === 'j') {
      e.preventDefault();
      window.jogDirection('down');
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

// Step joint button handler (+1, -1, +5, -5)
window.stepJoint = function(index, delta) {
  let val = targetAngles[index] + delta;
  const max = (index === 4) ? 90 : 180;
  val = Math.max(0, Math.min(max, val));
  targetAngles[index] = val;
  updateAnglesDisplay();
  transmitCommandThrottled();
};

window.setGripper = function(angle) {
  targetAngles[4] = angle;
  updateAnglesDisplay();
  transmitCommandThrottled();
};

// ====================================================================================
// DIRECTIONAL JOG SYSTEM (FRONT, BACKWARD, LEFT, RIGHT, UP, DOWN)
// ====================================================================================
let jogInterval = null;

window.jogDirection = function(dir) {
  const step = 2.5;

  if (dir === 'front') {
    // Move front / forward reach: Shoulder pitches forward, elbow extends
    targetAngles[0] = Math.min(180, targetAngles[0] + step);
    targetAngles[1] = Math.max(0, targetAngles[1] - (step * 0.7));
  } else if (dir === 'back') {
    // Move backward / pull back: Shoulder pitches back, elbow folds
    targetAngles[0] = Math.max(0, targetAngles[0] - step);
    targetAngles[1] = Math.min(180, targetAngles[1] + (step * 0.7));
  } else if (dir === 'left') {
    // Turn / Roll Left
    targetAngles[3] = Math.max(0, targetAngles[3] - step * 1.5);
  } else if (dir === 'right') {
    // Turn / Roll Right
    targetAngles[3] = Math.min(180, targetAngles[3] + step * 1.5);
  } else if (dir === 'up') {
    // Lift arm up
    targetAngles[0] = Math.max(0, targetAngles[0] - step);
    targetAngles[1] = Math.max(0, targetAngles[1] - step);
    targetAngles[2] = Math.min(180, targetAngles[2] + step);
  } else if (dir === 'down') {
    // Lower arm down
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
};

window.startContinuousJog = function(dir) {
  window.stopContinuousJog();
  window.jogDirection(dir);
  jogInterval = setInterval(() => {
    window.jogDirection(dir);
  }, 45);
};

window.stopContinuousJog = function() {
  if (jogInterval) {
    clearInterval(jogInterval);
    jogInterval = null;
  }
};

// Update UI values and sliders
function updateAnglesDisplay() {
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
window.applyPreset = function(name) {
  if (PRESETS[name]) {
    targetAngles = [...PRESETS[name]];
    updateAnglesDisplay();
    transmitCommandThrottled();
    showToast(`Applied Preset: ${name.toUpperCase()}`);
  }
};

// Emergency Stop
function triggerEStop() {
  // Halt all motion setpoints to current instantaneous position
  for (let i = 0; i < NUM_JOINTS; i++) {
    targetAngles[i] = currentAngles[i];
  }
  stopSequence();
  updateAnglesDisplay();
  sendJsonCommand({ cmd: "stop" });
  if (connMode === 'serial' && serialWriter) {
    serialWriter.write("STOP\n");
  }
  showToast("EMERGENCY STOP ACTIVATED!", true);
}

// ====================================================================================
// INVERSE KINEMATICS (IK SOLVER)
// ====================================================================================
function updateIkFromSliders() {
  const x = parseFloat(document.getElementById('slider_ik_x').value);
  const y = parseFloat(document.getElementById('slider_ik_y').value);
  const z = parseFloat(document.getElementById('slider_ik_z').value);
  const pitch = parseFloat(document.getElementById('slider_ik_pitch').value);

  document.getElementById('val_ik_x').textContent = x;
  document.getElementById('val_ik_y').textContent = y;
  document.getElementById('val_ik_z').textContent = z;
  document.getElementById('val_ik_pitch').textContent = pitch;

  // Move 3D Target Mesh
  if (ikTargetMesh) {
    ikTargetMesh.position.set(x, y, z);
  }

  solveAnalyticalIK(x, y, z, pitch, false);
}

function syncIkSlidersFromWorld() {
  if (!robotArm.endEffectorNode) return;
  const worldPos = new THREE.Vector3();
  robotArm.endEffectorNode.getWorldPosition(worldPos);

  document.getElementById('slider_ik_x').value = Math.round(worldPos.x);
  document.getElementById('slider_ik_y').value = Math.round(worldPos.y);
  document.getElementById('slider_ik_z').value = Math.round(worldPos.z);
  updateIkFromSliders();
}

function solveAnalyticalIK(targetX, targetY, targetZ, wristPitchDeg, apply = false) {
  const L1 = DIM.link1Length; // 42 cm
  const L2 = DIM.link2Length; // 40 cm
  const baseH = DIM.baseHeight + 4.3;

  // Height relative to shoulder joint
  const dy = targetY - baseH;
  const r = Math.sqrt(targetX * targetX + targetZ * targetZ);

  // 2-Link planar distance from shoulder to wrist
  const D = Math.sqrt(r * r + dy * dy);
  const notice = document.getElementById('ikResultNotice');

  // Reach check
  if (D > (L1 + L2) * 0.98 || D < Math.abs(L1 - L2) * 1.05) {
    notice.textContent = "Out of Reach Workspace";
    notice.style.color = "#f87171";
    notice.style.background = "rgba(239, 68, 68, 0.15)";
    return false;
  } else {
    notice.textContent = "Target Reachable (Valid Solution)";
    notice.style.color = "#34d399";
    notice.style.background = "rgba(16, 185, 129, 0.15)";
  }

  // Law of Cosines for elbow angle (theta2)
  const cosElbow = (L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2);
  const theta2Rad = Math.PI - Math.acos(Math.max(-1, Math.min(1, cosElbow)));

  // Shoulder angle (theta1)
  const alpha = Math.atan2(dy, r);
  const beta = Math.acos((L1 * L1 + D * D - L2 * L2) / (2 * L1 * D));
  const theta1Rad = alpha + beta;

  // Convert to servo degrees (90 is upright)
  let j1 = Math.round((Math.PI / 2 - theta1Rad) * (180 / Math.PI) + 90);
  let j2 = Math.round(180 - (theta2Rad * (180 / Math.PI)));
  let j3 = Math.round(90 + wristPitchDeg);
  let j4 = 90; // Centered roll

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
  const x = parseFloat(document.getElementById('slider_ik_x').value);
  const y = parseFloat(document.getElementById('slider_ik_y').value);
  const z = parseFloat(document.getElementById('slider_ik_z').value);
  const pitch = parseFloat(document.getElementById('slider_ik_pitch').value);
  solveAnalyticalIK(x, y, z, pitch, true);
}

// ====================================================================================
// MOTION SEQUENCER & KEYFRAME RECORDER
// ====================================================================================
function renderKeyframesList() {
  const container = document.getElementById('keyframeList');
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

window.deleteKeyframe = function(idx) {
  keyframes.splice(idx, 1);
  renderKeyframesList();
};

window.testKeyframe = function(idx) {
  if (keyframes[idx]) {
    targetAngles = [...keyframes[idx].angles];
    updateAnglesDisplay();
    transmitCommandThrottled();
  }
};

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
    const isLoop = document.getElementById('seqLoop').checked;
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
  const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(keyframes, null, 2));
  const dlAnchor = document.createElement('a');
  dlAnchor.setAttribute("href", dataStr);
  dlAnchor.setAttribute("download", "robot_arm_routine.json");
  dlAnchor.click();
}

function importSequenceJson(e) {
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

// Throttled sender to keep from congesting the serial or network pipeline
function transmitCommandThrottled() {
  const now = performance.now();
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
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

// --- WebSocket Handler ---
function toggleWifiConnection() {
  if (ws) {
    ws.close();
    ws = null;
    updateConnStatus(false);
    document.getElementById('connectWifiBtn').innerHTML = `<i class="fa-solid fa-link"></i> Connect`;
    return;
  }

  const ip = document.getElementById('espIpInput').value.trim();
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
      document.getElementById('connectWifiBtn').innerHTML = `<i class="fa-solid fa-unlink"></i> Disconnect`;
      showToast("ESP8266 Connected via Wi-Fi!");
      // Request initial status
      sendJsonCommand({ cmd: "get_status" });
      pingStartTime = performance.now();
    };

    ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === "telemetry" && msg.current) {
          // Calculate roundtrip latency
          const latency = Math.round(performance.now() - pingStartTime);
          document.getElementById('latencyBadge').textContent = `${Math.min(latency, 99)} ms`;
          pingStartTime = performance.now();
        }
      } catch (err) {
        // Raw message
      }
    };

    ws.onerror = (err) => {
      console.error("WebSocket Error:", err);
      updateConnStatus(false);
    };

    ws.onclose = () => {
      ws = null;
      updateConnStatus(false);
      document.getElementById('connectWifiBtn').innerHTML = `<i class="fa-solid fa-link"></i> Connect`;
      showToast("ESP8266 Disconnected");
    };

  } catch (err) {
    showToast(`Connection failed: ${err.message}`, true);
    updateConnStatus(false);
  }
}

// --- Web Serial API Handler (USB Cable Direct) ---
async function toggleSerialConnection() {
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
    document.getElementById('connectSerialBtn').innerHTML = `<i class="fa-solid fa-bolt"></i> Connect USB Port`;
    showToast("USB Port Disconnected");
    return;
  }

  if (!("serial" in navigator)) {
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
    document.getElementById('connectSerialBtn').innerHTML = `<i class="fa-solid fa-unlink"></i> Disconnect Port`;
    showToast("USB Serial Port Connected!");

    // Start reading serial
    readSerialLoop();

  } catch (err) {
    showToast(`Serial error: ${err.message}`, true);
    updateConnStatus(false);
  }
}

async function readSerialLoop() {
  while (serialPort && serialPort.readable) {
    const textDecoder = new TextDecoderStream();
    const readableStreamClosed = serialPort.readable.pipeTo(textDecoder.writable);
    serialReader = textDecoder.readable.getReader();

    try {
      while (true) {
        const { value, done } = await serialReader.read();
        if (done) break;
        if (value) {
          // incoming serial data (logs/telemetry)
        }
      }
    } catch (error) {
      console.warn("Serial read loop closed:", error);
    } finally {
      serialReader.releaseLock();
    }
  }
}

function updateConnStatus(connected, text = "Disconnected (Sim Only)") {
  isConnected = connected;
  const led = document.getElementById('statusLed');
  const txt = document.getElementById('statusText');
  const badge = document.getElementById('latencyBadge');

  if (connected) {
    led.className = "led connected";
    txt.textContent = text;
    badge.textContent = "Online";
  } else {
    led.className = "led disconnected";
    txt.textContent = "Disconnected (Sim Only)";
    badge.textContent = "-- ms";
  }
}

// Notification Toast Utility
function showToast(msg, isError = false) {
  const toast = document.getElementById('toast');
  toast.textContent = msg;
  toast.style.borderColor = isError ? "#ef4444" : "#38bdf8";
  toast.classList.add('show');
  setTimeout(() => toast.classList.remove('show'), 2800);
}

