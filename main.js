// MAC0623 / A2 — Desktop + WebXR docking (mappings 1–5)

import * as THREE from "three";
import { VRButton } from "three/addons/webxr/VRButton.js";

let scene, camera, renderer, cube, target, cubePickMesh;

let controller0, controller1;
let controllerGrip0, controllerGrip1;
let cameraRig;

const _controllerWorldPos = new THREE.Vector3();
const _grabDeltaPos = new THREE.Vector3();
const _controllerWorldQuat = new THREE.Quaternion();
const _deltaQuat = new THREE.Quaternion();
const _invStartControllerQuat = new THREE.Quaternion();
const _rayRotationMatrix = new THREE.Matrix4();
const _vrRaycaster = new THREE.Raycaster();

/** Desktop-only: pull the camera back for a monitor view. */
const DESKTOP_CAMERA_RIG_Z = 1.8;
/** VR: stand this far in +Z from the workspace center (meters, arm's reach). */
const VR_VIEWER_RIG_Z = 1.1;
/** Point of regard — mid-height of the target sampling volume (desktop camera). */
const WORKSPACE_LOOK_AT = new THREE.Vector3(0, 0.65, 0);
/** Visual ray length (meters); raycast range is separate and longer. */
const VR_RAY_VISUAL_LENGTH = 4;
const VR_RAYCAST_FAR = 20;

/**
 * Mapping 4 — VR trackball rotation gain. Scales controller twist delta before
 * applying it to the cube (1 = isomorphic twist; <1 = finer indirect control).
 * Justify this value in the A2 report.
 */
const VR_TRACKBALL_GAIN = 0.65;

const GIZMO_COLORS = { x: 0xe74c3c, y: 0x2ecc71, z: 0x3498db };
const GIZMO_TRANSLATE_LENGTH = 0.55;
const GIZMO_TRANSLATE_RADIUS = 0.028;
const GIZMO_RING_RADIUS = 0.38;
const GIZMO_RING_TUBE = 0.018;
const GIZMO_IDLE_RING_OPACITY = 0.55;
/** Subtle aim feedback on gizmo handles (mapping 5). */
const GIZMO_AIM_COLOR_LERP = 0.2;
const GIZMO_AIM_OPACITY_BOOST = 0.18;

let vrGizmo = null;
/** @type {THREE.Mesh[]} */
let vrGizmoPickables = [];

const _gizmoAxisLocal = new THREE.Vector3();
const _gizmoAxisWorld = new THREE.Vector3();
const _gizmoCenter = new THREE.Vector3();
const _gizmoVecA = new THREE.Vector3();
const _gizmoVecB = new THREE.Vector3();
const _gizmoDelta = new THREE.Vector3();
const _gizmoQuat = new THREE.Quaternion();
const _gizmoColorTemp = new THREE.Color();
const _gizmoColorWhite = new THREE.Color(0xffffff);

function buildScene() {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1a1a);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 1.2));
  const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
  dirLight.position.set(2, 4, 3);
  scene.add(dirLight);

  scene.add(new THREE.GridHelper(6, 24, 0x444444, 0x2a2a2a));
  scene.add(new THREE.AxesHelper(0.6));

  // Cube (student-controlled) and target (goal pose) share one geometry —
  // the target clones it so the two meshes can have independent materials
  // (opaque vs. translucent) without sharing a single Mesh instance.
  const cubeGeometry = new THREE.BoxGeometry(0.4, 0.4, 0.4);

  const cube = new THREE.Mesh(
    cubeGeometry,
    new THREE.MeshStandardMaterial({ color: 0x3d8bfd })
  );
  cube.position.set(0, 0.5, 0);

  const cubePickMesh = new THREE.Mesh(
    new THREE.BoxGeometry(0.55, 0.55, 0.55),
    new THREE.MeshBasicMaterial({ visible: false })
  );
  cubePickMesh.name = "cubePick";
  cube.add(cubePickMesh);

  scene.add(cube);

  const target = new THREE.Mesh(
    cubeGeometry.clone(),
    new THREE.MeshStandardMaterial({
      color: 0x2ecc71,
      transparent: true,
      opacity: 0.35,
      depthWrite: false,
    })
  );
  scene.add(target);

  return { scene, cube, target, cubePickMesh };
}

function main() {
  ({ scene, cube, target, cubePickMesh } = buildScene());

  camera = new THREE.PerspectiveCamera(
    60,
    window.innerWidth / window.innerHeight,
    0.05,
    100
  );

  // Camera rig positions the user at a comfortable distance in front of the target area
  cameraRig = new THREE.Group();
  cameraRig.position.set(0, 0, DESKTOP_CAMERA_RIG_Z);
  cameraRig.add(camera);
  scene.add(cameraRig);

  camera.position.set(0, 1.4, 0);
  camera.lookAt(WORKSPACE_LOOK_AT);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);

  // WebXR setup
  renderer.xr.enabled = true;
  renderer.xr.setReferenceSpaceType("local-floor");
  document.body.appendChild(VRButton.createButton(renderer));
  document.body.appendChild(renderer.domElement);

  renderer.xr.addEventListener("sessionstart", onXRSessionStart);
  renderer.xr.addEventListener("sessionend", onXRSessionEnd);

  // Same parent as the camera so XR hand poses match the headset offset.
  ({ controller0, controller1, controllerGrip0, controllerGrip1 } =
    buildControllers(renderer, cameraRig));

  [controller0, controller1].forEach((controller) => {
    controller.addEventListener("selectstart", onSelectStart);
    controller.addEventListener("selectend", onSelectEnd);
    controller.addEventListener("squeezestart", onSqueezeStart);
  });

  window.addEventListener("resize", handleWindowResize);

  buildVRGizmo(cube);
  setVRGizmoVisible(false);

  startTrial();
  renderer.setAnimationLoop(animate);
}

/**
 * handleWindowResize()
 *
 * Keeps the camera's aspect ratio and the renderer's output size in sync
 * with the browser window. Registered as the "resize" listener in main().
 */
function handleWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

// ---------------------------------------------------------------------------
// Target-pose generation — provided
//
// Uses Shoemake's algorithm for a uniformly-random unit quaternion (uniform
// over SO(3)), rather than converting random Euler angles, which would bias
// the sampled orientations. Position is uniform within a bounding box in
// front of the camera.
// ---------------------------------------------------------------------------

function randomQuaternionShoemake() {
  const u1 = Math.random();
  const u2 = Math.random();
  const u3 = Math.random();

  const sqrt1MinusU1 = Math.sqrt(1 - u1);
  const sqrtU1 = Math.sqrt(u1);

  const theta1 = 2 * Math.PI * u2;
  const theta2 = 2 * Math.PI * u3;

  return new THREE.Quaternion(
    sqrt1MinusU1 * Math.sin(theta1),
    sqrt1MinusU1 * Math.cos(theta1),
    sqrtU1 * Math.sin(theta2),
    sqrtU1 * Math.cos(theta2)
  );
}

const TARGET_BOUNDS = {
  x: [-1.0, 1.0],
  y: [0.2, 1.6],
  z: [-0.6, 0.6],
};

function randomInRange([min, max]) {
  return min + Math.random() * (max - min);
}

function generateTargetPose() {
  target.position.set(
    randomInRange(TARGET_BOUNDS.x),
    randomInRange(TARGET_BOUNDS.y),
    randomInRange(TARGET_BOUNDS.z)
  );
  target.quaternion.copy(randomQuaternionShoemake());
}

// ---------------------------------------------------------------------------
// Tolerance check — provided
//
// Position tolerance: 0.05 units (world units == meters, at this scene
// scale). Orientation tolerance: 10 degrees, measured via
// Quaternion.angleTo(), which is robust to double-cover (q and -q represent
// the same rotation) — do not compute orientation error from Euler angles.
// ---------------------------------------------------------------------------

const POSITION_TOLERANCE = 0.05;
const ORIENTATION_TOLERANCE_DEG = 10;

function checkTolerance() {
  const positionError = cube.position.distanceTo(target.position);
  const orientationErrorRad = cube.quaternion.angleTo(target.quaternion);
  const orientationErrorDeg = THREE.MathUtils.radToDeg(orientationErrorRad);

  const withinTolerance =
    positionError <= POSITION_TOLERANCE &&
    orientationErrorDeg <= ORIENTATION_TOLERANCE_DEG;

  return { positionError, orientationErrorDeg, withinTolerance };
}

// ---------------------------------------------------------------------------
// HUD references — provided
// ---------------------------------------------------------------------------

const participantIdInput = document.getElementById("participantId");
const mappingSelect = document.getElementById("mappingSelect");
const trialCountEl = document.getElementById("trialCount");
const confirmBtn = document.getElementById("confirmBtn");
const downloadBtn = document.getElementById("downloadBtn");
const statusEl = document.getElementById("status");

// ---------------------------------------------------------------------------
// Trial state machine — provided
//
// presentation_order counts trials within the *current* mapping selection
// since the page loaded — it does not reset when you switch mapping in the
// dropdown mid-session, since order-of-presentation across mappings is part
// of what you're counterbalancing across participants (see A1's ABBA
// counterbalancing note). trial_number is a simple running counter of every
// trial confirmed this session, regardless of mapping.
// ---------------------------------------------------------------------------

let trialNumber = 0;
let presentationOrderByMapping = { 1: 0, 2: 0, 3: 0, 4: 0, 5: 0 };
let trialStartTime = performance.now();
let pathLength = 0; // accumulated cube-position travel distance this trial
// Placeholder — cube doesn't exist yet at module-load time (main() creates
// it via buildScene()). startTrial() calls lastCubePosition.copy(cube.position)
// before this value is ever read, so the zero vector here is never used.
let lastCubePosition = new THREE.Vector3();

// ===== STUDENT TODO =====
// Increment this from your own mapping code every time the user switches
// input mode (e.g. toggling translate/rotate mode in the baseline mapping).
// It is read (and reset) when a trial is confirmed.
let modeSwitches = 0;
/** Mapping 4: last grab type for mode_switches when alternating translate/rotate. */
let lastTrackballGrabKind = null;
// ===== END STUDENT TODO =====

const rows = [];
const CSV_HEADER = [
  "participant_id",
  "mapping",
  "trial_number",
  "presentation_order",
  "completion_time_s",
  "final_position_error",
  "final_orientation_error_deg",
  "mode_switches",
  "path_length",
];

function currentMapping() {
  return mappingSelect ? mappingSelect.value : "1";
}

function startTrial() {
  trialStartTime = performance.now();
  pathLength = 0;
  if (cube) lastCubePosition.copy(cube.position);
  modeSwitches = 0;
  lastTrackballGrabKind = null;
  generateTargetPose();
  if (trialCountEl) trialCountEl.textContent = `Trial ${trialNumber + 1}`;
}

function confirmTrial() {
  const { positionError, orientationErrorDeg } = checkTolerance();
  const completionTimeS = (performance.now() - trialStartTime) / 1000;
  const mapping = currentMapping();

  trialNumber += 1;
  presentationOrderByMapping[mapping] = (presentationOrderByMapping[mapping] || 0) + 1;

  rows.push({
    participant_id: (participantIdInput && participantIdInput.value.trim()) || "UNKNOWN",
    mapping,
    trial_number: trialNumber,
    presentation_order: presentationOrderByMapping[mapping],
    completion_time_s: completionTimeS.toFixed(3),
    final_position_error: positionError.toFixed(4),
    final_orientation_error_deg: orientationErrorDeg.toFixed(2),
    mode_switches: modeSwitches,
    path_length: pathLength.toFixed(4),
  });

  startTrial();
}

if (confirmBtn) confirmBtn.addEventListener("click", confirmTrial);
window.addEventListener("keydown", handleKeydown);

/**
 * handleKeydown(e)
 *
 * Keyboard shortcut for Confirm: Enter does the same thing as clicking
 * #confirmBtn. Registered as the "keydown" listener above.
 */
function handleKeydown(e) {
  if (e.key === "Enter") confirmTrial();
}

// ---------------------------------------------------------------------------
// CSV download — provided
// ---------------------------------------------------------------------------

function buildCsv() {
  const lines = [CSV_HEADER.join(",")];
  for (const row of rows) {
    lines.push(
      CSV_HEADER.map(function (key) {
        return row[key];
      }).join(",")
    );
  }
  return lines.join("\n");
}

if (downloadBtn) downloadBtn.addEventListener("click", handleDownloadClick);

/**
 * handleDownloadClick()
 *
 * Builds the CSV from `rows` (via buildCsv()), then triggers a browser
 * download through a temporary Blob URL and an off-DOM `<a>` click.
 * Registered as the "click" listener on #downloadBtn above.
 */
function handleDownloadClick() {
  const csv = buildCsv();
  const blob = new Blob([csv], { type: "text/csv" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  const pid = (participantIdInput && participantIdInput.value.trim()) || "UNKNOWN";
  a.href = url;
  a.download = `a2_${pid}_${Date.now()}.csv`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

// ---------------------------------------------------------------------------
// Status indicator — provided
// ---------------------------------------------------------------------------

function updateStatus() {
  const { positionError, orientationErrorDeg, withinTolerance } = checkTolerance();
  if (!statusEl) return;

  if (renderer?.xr?.isPresenting && isVRPointerMapping()) {
    if (cube.userData.heldBy) {
      const kind = cube.userData.heldBy.userData.vrGrabKind || "grab";
      statusEl.textContent = `${kind} | dPos ${positionError.toFixed(3)} | dRot ${orientationErrorDeg.toFixed(1)}deg`;
    } else if (currentMapping() === "4") {
      const onCube = isControllerAimingAtCube();
      const nextAction = onCube ? "trigger: move cube" : "trigger: rotate";
      statusEl.textContent = `Trackball | ${nextAction} | grip: Confirm | dPos ${positionError.toFixed(3)} | dRot ${orientationErrorDeg.toFixed(1)}deg`;
    } else if (currentMapping() === "5") {
      const handle = getAimedGizmoHandle();
      const aim = handle
        ? `${handle.userData.gizmoMode} ${handle.userData.gizmoAxis}`
        : "—";
      statusEl.textContent = `Gizmo | aim ${aim} | grip: Confirm | dPos ${positionError.toFixed(3)} | dRot ${orientationErrorDeg.toFixed(1)}deg`;
    } else {
      const onTarget = isControllerAimingAtCube();
      statusEl.textContent = `Direct grab | aim ${onTarget ? "OK" : "—"} | grip: Confirm | dPos ${positionError.toFixed(3)} | dRot ${orientationErrorDeg.toFixed(1)}deg`;
    }
    statusEl.classList.toggle("in-tolerance", withinTolerance);
    statusEl.classList.toggle("in-vr", true);
    return;
  }

  statusEl.textContent = `dPos ${positionError.toFixed(3)} | dRot ${orientationErrorDeg.toFixed(1)}deg`;
  statusEl.classList.toggle("in-tolerance", withinTolerance);
}

function setStatus(text, inVR = false) {
  if (!statusEl) return;
  statusEl.textContent = text;
  statusEl.classList.toggle("in-vr", inVR);
}

function onXRSessionStart() {
  // Face the workspace from +Z. Eye height comes from XR tracking — do not add
  // VR_EYE_HEIGHT on the camera or you end up ~3 m above the floor.
  cameraRig.position.set(0, 0, VR_VIEWER_RIG_Z);
  camera.position.set(0, 0, 0);
  camera.rotation.set(0, 0, 0);
  cameraRig.updateMatrixWorld(true);
  setStatus("In VR", true);
}

function onXRSessionEnd() {
  cameraRig.position.set(0, 0, DESKTOP_CAMERA_RIG_Z);
  camera.position.set(0, 1.4, 0);
  camera.lookAt(WORKSPACE_LOOK_AT);
  camera.updateMatrixWorld(true);
  setStatus("Left VR");
}

// ---------------------------------------------------------------------------
// WebXR Controller Building & Raycasting
// ---------------------------------------------------------------------------

function buildControllers(renderer, parentGroup) {
  function buildControllerPair(index) {
    const controller = renderer.xr.getController(index);
    const grip = renderer.xr.getControllerGrip(index);

    const rayGeometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, -1),
    ]);
    const line = new THREE.Line(
      rayGeometry,
      new THREE.LineBasicMaterial({
        color: index === 0 ? 0xff6666 : 0x66aaff,
      })
    );

    line.name = "ray";
    line.scale.z = VR_RAY_VISUAL_LENGTH;
    controller.add(line);

    parentGroup.add(controller);
    parentGroup.add(grip);

    return { controller, grip };
  }

  const pair0 = buildControllerPair(0);
  const pair1 = buildControllerPair(1);

  return {
    controller0: pair0.controller,
    controller1: pair1.controller,
    controllerGrip0: pair0.grip,
    controllerGrip1: pair1.grip,
  };
}

function getControllerGrip(controller) {
  if (controller === controller0) return controllerGrip0;
  if (controller === controller1) return controllerGrip1;
  return null;
}

/** Grip pose for motion; target-ray pose for pointing. */
function readControllerMotionPosition(controller, target) {
  const grip = getControllerGrip(controller);
  if (grip) {
    grip.updateMatrixWorld(true);
    return target.setFromMatrixPosition(grip.matrixWorld);
  }
  controller.updateMatrixWorld(true);
  return target.setFromMatrixPosition(controller.matrixWorld);
}

function readControllerMotionQuaternion(controller, target) {
  const grip = getControllerGrip(controller);
  if (grip) {
    grip.updateMatrixWorld(true);
    return target.setFromRotationMatrix(grip.matrixWorld);
  }
  return controller.getWorldQuaternion(target);
}

function updateVRPointerMatrices() {
  if (cameraRig) cameraRig.updateMatrixWorld(true);
  if (cube) cube.updateMatrixWorld(true);
  if (controller0) controller0.updateMatrixWorld(true);
  if (controller1) controller1.updateMatrixWorld(true);
  if (controllerGrip0) controllerGrip0.updateMatrixWorld(true);
  if (controllerGrip1) controllerGrip1.updateMatrixWorld(true);
}

function getVRPickTargets() {
  return cubePickMesh ? [cubePickMesh] : [cube];
}

function getIntersections(controller, objects) {
  updateVRPointerMatrices();

  _rayRotationMatrix.identity().extractRotation(controller.matrixWorld);

  _vrRaycaster.far = VR_RAYCAST_FAR;
  _vrRaycaster.near = 0.01;
  _vrRaycaster.ray.origin.setFromMatrixPosition(controller.matrixWorld);
  _vrRaycaster.ray.direction.set(0, 0, -1).applyMatrix4(_rayRotationMatrix);

  return _vrRaycaster.intersectObjects(objects, true);
}

function isControllerAimingAtCube() {
  if (!cubePickMesh && !cube) return false;
  const targets = getVRPickTargets();
  for (const controller of [controller0, controller1]) {
    if (controller && getIntersections(controller, targets).length > 0) {
      return true;
    }
  }
  return false;
}

function setVRGizmoVisible(visible) {
  if (vrGizmo) vrGizmo.visible = visible;
  if (!visible) resetAllGizmoHandleVisuals();
}

function initGizmoHandleUserData(mesh, color, baseOpacity) {
  mesh.userData.gizmoBaseColor = color;
  mesh.userData.gizmoBaseOpacity = baseOpacity;
  mesh.userData.gizmoBaseScale = mesh.scale.x;
}

function resetGizmoHandleVisual(mesh) {
  const mat = mesh.material;
  mat.color.setHex(mesh.userData.gizmoBaseColor);
  mat.opacity = mesh.userData.gizmoBaseOpacity;
  mesh.scale.setScalar(mesh.userData.gizmoBaseScale);
}

function resetAllGizmoHandleVisuals() {
  for (const mesh of vrGizmoPickables) resetGizmoHandleVisual(mesh);
}

function updateGizmoAimHighlight() {
  if (!vrGizmoPickables.length) return;

  const gizmoActive =
    renderer?.xr?.isPresenting && currentMapping() === "5";
  if (!gizmoActive) {
    resetAllGizmoHandleVisuals();
    return;
  }

  const heldHandle = cube.userData.heldBy?.userData.gizmoHandle;
  const aimedHandle = heldHandle || getAimedGizmoHandle();

  for (const mesh of vrGizmoPickables) {
    if (mesh === aimedHandle) {
      const mat = mesh.material;
      _gizmoColorTemp.setHex(mesh.userData.gizmoBaseColor);
      _gizmoColorTemp.lerp(_gizmoColorWhite, GIZMO_AIM_COLOR_LERP);
      mat.color.copy(_gizmoColorTemp);
      mat.opacity = Math.min(
        1,
        mesh.userData.gizmoBaseOpacity + GIZMO_AIM_OPACITY_BOOST
      );
    } else {
      resetGizmoHandleVisual(mesh);
    }
  }
}

function buildVRGizmo(parentCube) {
  vrGizmo = new THREE.Group();
  vrGizmo.name = "vrGizmo";
  parentCube.add(vrGizmo);
  vrGizmoPickables = [];

  for (const axis of ["x", "y", "z"]) {
    const color = GIZMO_COLORS[axis];

    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(
        GIZMO_TRANSLATE_RADIUS,
        GIZMO_TRANSLATE_RADIUS,
        GIZMO_TRANSLATE_LENGTH,
        10
      ),
      new THREE.MeshBasicMaterial({ color })
    );
    shaft.position.y = GIZMO_TRANSLATE_LENGTH / 2;
    if (axis === "x") shaft.rotation.z = -Math.PI / 2;
    if (axis === "z") shaft.rotation.x = Math.PI / 2;
    shaft.userData.gizmoMode = "translate";
    shaft.userData.gizmoAxis = axis;
    initGizmoHandleUserData(shaft, color, 1);
    vrGizmo.add(shaft);
    vrGizmoPickables.push(shaft);

    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(GIZMO_RING_RADIUS, GIZMO_RING_TUBE, 10, 32),
      new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: GIZMO_IDLE_RING_OPACITY,
      })
    );
    if (axis === "x") ring.rotation.y = Math.PI / 2;
    if (axis === "y") ring.rotation.x = Math.PI / 2;
    ring.userData.gizmoMode = "rotate";
    ring.userData.gizmoAxis = axis;
    initGizmoHandleUserData(ring, color, GIZMO_IDLE_RING_OPACITY);
    vrGizmo.add(ring);
    vrGizmoPickables.push(ring);
  }
}

function updateVRGizmoVisibility() {
  setVRGizmoVisible(
    Boolean(renderer?.xr?.isPresenting && currentMapping() === "5")
  );
}

function getAimedGizmoHandle() {
  if (!vrGizmoPickables.length) return null;
  for (const controller of [controller0, controller1]) {
    const hits = getIntersections(controller, vrGizmoPickables);
    if (hits.length > 0) return hits[0].object;
  }
  return null;
}

function gizmoAxisWorldFromQuaternion(axisChar, quaternion) {
  _gizmoAxisLocal.set(
    axisChar === "x" ? 1 : 0,
    axisChar === "y" ? 1 : 0,
    axisChar === "z" ? 1 : 0
  );
  return _gizmoAxisWorld.copy(_gizmoAxisLocal).applyQuaternion(quaternion).normalize();
}

/** Swing–twist: rotation component about axisWorld only. */
function quatTwistAboutAxis(deltaQuat, axisWorld) {
  const proj = _gizmoVecA
    .set(deltaQuat.x, deltaQuat.y, deltaQuat.z)
    .dot(axisWorld);
  _gizmoDelta.set(axisWorld.x * proj, axisWorld.y * proj, axisWorld.z * proj);
  _gizmoQuat.set(_gizmoDelta.x, _gizmoDelta.y, _gizmoDelta.z, deltaQuat.w);
  return _gizmoQuat.normalize();
}

// ---------------------------------------------------------------------------
// VR direct grab (mapping 3): translation follows controller motion;
// rotation applies controller delta to the cube about its own center.
// ---------------------------------------------------------------------------

function clearGrabState(controller) {
  if (!controller) return;
  controller.userData.selected = null;
  controller.userData.gizmoHandle = null;
  controller.userData.vrGrabKind = null;
  controller.userData.gizmoAxis = null;
  controller.userData.grabStartControllerPos = null;
  controller.userData.grabStartCubePos = null;
  controller.userData.grabStartControllerQuat = null;
  controller.userData.grabStartCubeQuat = null;
}

function isVRPointerMapping() {
  const mapping = currentMapping();
  return mapping === "3" || mapping === "4" || mapping === "5";
}

function onGizmoSelectStart(event) {
  const controller = event.target;
  if (cube.userData.heldBy) return;

  const hits = getIntersections(controller, vrGizmoPickables);
  if (hits.length === 0) return;

  const handle = hits[0].object;
  const axis = handle.userData.gizmoAxis;
  const mode = handle.userData.gizmoMode;

  readControllerMotionPosition(controller, _controllerWorldPos);
  readControllerMotionQuaternion(controller, _controllerWorldQuat);
  controller.userData.selected = handle;
  controller.userData.gizmoHandle = handle;
  cube.userData.heldBy = controller;
  controller.userData.gizmoAxis = axis;
  controller.userData.vrGrabKind =
    mode === "translate" ? "gizmo-translate" : "gizmo-rotate";
  controller.userData.grabStartControllerPos = _controllerWorldPos.clone();
  controller.userData.grabStartControllerQuat = _controllerWorldQuat.clone();
  controller.userData.grabStartCubePos = cube.position.clone();
  controller.userData.grabStartCubeQuat = cube.quaternion.clone();

  const ctrlLabel = controller === controller0 ? 0 : 1;
  setStatus(`Gizmo ${mode} ${axis} (ctrl ${ctrlLabel})`, true);
}

function onSelectStart(event) {
  if (!renderer.xr.isPresenting) return;

  const mapping = currentMapping();
  const controller = event.target;
  if (cube.userData.heldBy) return;

  if (mapping === "5") {
    onGizmoSelectStart(event);
    return;
  }

  if (mapping === "3") {
    const hits = getIntersections(controller, getVRPickTargets());
    if (hits.length === 0) return;

    readControllerMotionQuaternion(controller, _controllerWorldQuat);
    readControllerMotionPosition(controller, _controllerWorldPos);
    controller.userData.selected = cube;
    cube.userData.heldBy = controller;
    controller.userData.grabStartControllerQuat = _controllerWorldQuat.clone();
    controller.userData.grabStartCubeQuat = cube.quaternion.clone();
    controller.userData.vrGrabKind = "direct";
    controller.userData.grabStartControllerPos = _controllerWorldPos.clone();
    controller.userData.grabStartCubePos = cube.position.clone();
    const ctrlLabel = controller === controller0 ? 0 : 1;
    setStatus(`Direct grab (ctrl ${ctrlLabel})`, true);
    return;
  }

  if (mapping === "4") {
    const hits = getIntersections(controller, getVRPickTargets());
    const onCube = hits.length > 0;
    const grabKind = onCube ? "trackball-translate" : "trackball-rotate";

    if (grabKind !== lastTrackballGrabKind) {
      modeSwitches++;
      lastTrackballGrabKind = grabKind;
    }

    readControllerMotionQuaternion(controller, _controllerWorldQuat);
    controller.userData.selected = cube;
    cube.userData.heldBy = controller;
    controller.userData.grabStartControllerQuat = _controllerWorldQuat.clone();
    controller.userData.grabStartCubeQuat = cube.quaternion.clone();
    controller.userData.vrGrabKind = grabKind;

    const ctrlLabel = controller === controller0 ? 0 : 1;
    if (onCube) {
      readControllerMotionPosition(controller, _controllerWorldPos);
      controller.userData.grabStartControllerPos = _controllerWorldPos.clone();
      controller.userData.grabStartCubePos = cube.position.clone();
      setStatus(`Trackball translate (ctrl ${ctrlLabel})`, true);
    } else {
      setStatus(
        `Trackball rotate (gain ${VR_TRACKBALL_GAIN}, ctrl ${ctrlLabel})`,
        true
      );
    }
  }
}

function onSelectEnd(event) {
  const controller = event.target;
  if (cube.userData.heldBy !== controller) return;

  cube.userData.heldBy = null;
  clearGrabState(controller);
  setStatus("Released", true);
}

/** VR mappings 3–5: grip confirms the trial (same as HUD Confirm). */
function onSqueezeStart() {
  if (!renderer?.xr?.isPresenting) return;
  const mapping = currentMapping();
  if (mapping !== "3" && mapping !== "4" && mapping !== "5") return;
  confirmTrial();
  setStatus("Trial confirmed", true);
}

function toggleTranslateRotateMode() {
  isRotationMode = !isRotationMode;
  modeSwitches++;
}

function updateVRGrabTranslation(controller) {
  const startControllerPos = controller.userData.grabStartControllerPos;
  const startCubePos = controller.userData.grabStartCubePos;
  if (!startControllerPos || !startCubePos) return;

  readControllerMotionPosition(controller, _controllerWorldPos);
  _grabDeltaPos.subVectors(_controllerWorldPos, startControllerPos);
  cube.position.copy(startCubePos).add(_grabDeltaPos);
}

function applyControllerRotationDeltaToCube(
  controller,
  startControllerQuat,
  startCubeQuat,
  gain
) {
  readControllerMotionQuaternion(controller, _controllerWorldQuat);
  _invStartControllerQuat.copy(startControllerQuat).invert();
  _deltaQuat.copy(_controllerWorldQuat).multiply(_invStartControllerQuat);

  if (gain !== 1) {
    const angle = 2 * Math.acos(Math.min(1, Math.abs(_deltaQuat.w)));
    if (angle > 1e-8) {
      const scaledAngle = angle * gain;
      const sinHalf = Math.sin(angle / 2);
      const scale = Math.sin(scaledAngle / 2) / sinHalf;
      _deltaQuat.x *= scale;
      _deltaQuat.y *= scale;
      _deltaQuat.z *= scale;
      _deltaQuat.w = Math.cos(scaledAngle / 2);
      _deltaQuat.normalize();
    }
  }

  cube.quaternion.copy(_deltaQuat).multiply(startCubeQuat);
}

function updateVRDirectGrab() {
  const controller = cube.userData.heldBy;
  if (!controller || controller.userData.vrGrabKind !== "direct") return;

  const startControllerQuat = controller.userData.grabStartControllerQuat;
  const startCubeQuat = controller.userData.grabStartCubeQuat;
  if (!startControllerQuat || !startCubeQuat) return;

  updateVRGrabTranslation(controller);
  applyControllerRotationDeltaToCube(
    controller,
    startControllerQuat,
    startCubeQuat,
    1
  );
}

function updateVRTrackball() {
  const controller = cube.userData.heldBy;
  if (!controller) return;

  const kind = controller.userData.vrGrabKind;
  const startControllerQuat = controller.userData.grabStartControllerQuat;
  const startCubeQuat = controller.userData.grabStartCubeQuat;
  if (!startControllerQuat || !startCubeQuat) return;

  if (kind === "trackball-translate") {
    updateVRGrabTranslation(controller);
    return;
  }

  if (kind === "trackball-rotate") {
    applyControllerRotationDeltaToCube(
      controller,
      startControllerQuat,
      startCubeQuat,
      VR_TRACKBALL_GAIN
    );
  }
}

function updateVRGizmoDrag() {
  const controller = cube.userData.heldBy;
  if (!controller) return;

  updateVRPointerMatrices();

  const kind = controller.userData.vrGrabKind;
  const axis = controller.userData.gizmoAxis;
  const startControllerPos = controller.userData.grabStartControllerPos;
  const startControllerQuat = controller.userData.grabStartControllerQuat;
  const startCubePos = controller.userData.grabStartCubePos;
  const startCubeQuat = controller.userData.grabStartCubeQuat;
  if (
    !axis ||
    !startControllerPos ||
    !startControllerQuat ||
    !startCubePos ||
    !startCubeQuat
  ) {
    return;
  }

  const axisWorld = gizmoAxisWorldFromQuaternion(axis, startCubeQuat);

  if (kind === "gizmo-translate") {
    readControllerMotionPosition(controller, _controllerWorldPos);
    _gizmoDelta.subVectors(_controllerWorldPos, startControllerPos);
    const along = _gizmoDelta.dot(axisWorld);
    cube.position.copy(startCubePos).addScaledVector(axisWorld, along);
    return;
  }

  if (kind === "gizmo-rotate") {
    readControllerMotionQuaternion(controller, _controllerWorldQuat);
    _invStartControllerQuat.copy(startControllerQuat).invert();
    _deltaQuat.copy(_controllerWorldQuat).multiply(_invStartControllerQuat);
    quatTwistAboutAxis(_deltaQuat, axisWorld);
    cube.quaternion.copy(_gizmoQuat).multiply(startCubeQuat);
    cube.position.copy(startCubePos);
  }
}

function updateVRMapping() {
  const mapping = currentMapping();
  if (mapping === "3") updateVRDirectGrab();
  else if (mapping === "4") updateVRTrackball();
  else if (mapping === "5") updateVRGizmoDrag();
}

// ---------------------------------------------------------------------------
// Control mapping — STUDENT TODO
// ---------------------------------------------------------------------------

let isRotationMode = false; // false = translation, true = rotation

let isMouseDown = false;
let mouseDx = 0;
let mouseDy = 0;
let mouseWheelDelta = 0;

const keysPressed = {};

window.addEventListener("keydown", (e) => {
  keysPressed[e.code] = true;

  if (e.code === "Space") {
    e.preventDefault();
    if (renderer && !renderer.xr.isPresenting) toggleTranslateRotateMode();
  }
});

window.addEventListener("keyup", (e) => {
  keysPressed[e.code] = false;
});

window.addEventListener("mousedown", (e) => {
  if (e.button === 0) isMouseDown = true;
});

window.addEventListener("mouseup", () => {
  isMouseDown = false;
});

window.addEventListener("mousemove", (e) => {
  if (isMouseDown) {
    mouseDx += e.movementX;
    mouseDy += e.movementY;
  }
});

window.addEventListener("wheel", (e) => {
  mouseWheelDelta += e.deltaY;
});

// Control update per frame
function updateControlMapping(delta) {
  if (renderer?.xr?.isPresenting) return;

  const mapping = currentMapping();
  const moveSpeed = 3.0 * delta;
  const rotSpeed = 2.5 * delta;

  if (mapping === "1") {
    // ==========================================
    // MAPPING 1: Mouse + wheel + space
    // ==========================================
    const mouseSensitivity = 0.003;
    const wheelSensitivity = 0.001;

    if (isRotationMode) {
      // ROTATION MODE
      rotateCube(0, 1, 0, mouseDx * mouseSensitivity);
      rotateCube(1, 0, 0, mouseDy * mouseSensitivity);
      if (mouseWheelDelta !== 0) {
        rotateCube(0, 0, 1, mouseWheelDelta * wheelSensitivity);
      }
    } else {
      // TRANSLATION MODE
      cube.position.x += mouseDx * mouseSensitivity;
      cube.position.y -= mouseDy * mouseSensitivity;
      cube.position.z -= mouseWheelDelta * wheelSensitivity;
    }
  } else {
    // ==========================================
    // MAPPING 2: keyboard + space
    // ==========================================
    if (isRotationMode) {
      // ROTATION MODE
      if (keysPressed["KeyW"]) rotateCube(1, 0, 0, -rotSpeed); 
      if (keysPressed["KeyS"]) rotateCube(1, 0, 0, rotSpeed);
      if (keysPressed["KeyA"]) rotateCube(0, 1, 0, -rotSpeed);
      if (keysPressed["KeyD"]) rotateCube(0, 1, 0, rotSpeed);
      if (keysPressed["KeyQ"]) rotateCube(0, 0, 1, rotSpeed);
      if (keysPressed["KeyE"]) rotateCube(0, 0, 1, -rotSpeed);
    } else {
      // TRANSLATION MODE
      if (keysPressed["KeyW"]) cube.position.y += moveSpeed;
      if (keysPressed["KeyS"]) cube.position.y -= moveSpeed;
      if (keysPressed["KeyA"]) cube.position.x -= moveSpeed;
      if (keysPressed["KeyD"]) cube.position.x += moveSpeed;
      if (keysPressed["KeyQ"]) cube.position.z += moveSpeed;
      if (keysPressed["KeyE"]) cube.position.z -= moveSpeed;
    }
  }

  mouseDx = 0;
  mouseDy = 0;
  mouseWheelDelta = 0;
}

function rotateCube(x, y, z, angle) {
  const axis = new THREE.Vector3(x, y, z).normalize();
  const q = new THREE.Quaternion().setFromAxisAngle(axis, angle);
  cube.quaternion.multiplyQuaternions(q, cube.quaternion);
}

// ===== END STUDENT TODO =====

// ---------------------------------------------------------------------------
// Render loop — provided
// ---------------------------------------------------------------------------

const clock = new THREE.Clock();

function animate() {
  const delta = clock.getDelta();

  updateControlMapping(delta);
  updateVRGizmoVisibility();
  updateGizmoAimHighlight();
  if (renderer?.xr?.isPresenting) updateVRMapping();

  // Generic path-length accumulation — measures how far the cube has
  // physically travelled this trial, regardless of mapping.
  if (cube && lastCubePosition) {
    pathLength += cube.position.distanceTo(lastCubePosition);
    lastCubePosition.copy(cube.position);
  }

  updateStatus();

  renderer.render(scene, camera);
}

main();