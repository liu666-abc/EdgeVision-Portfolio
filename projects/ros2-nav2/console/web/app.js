const $ = (id) => document.getElementById(id);

const els = {
  wsPill: $("wsPill"),
  rosPill: $("rosPill"),
  navPill: $("navPill"),
  frameLabel: $("frameLabel"),
  clockDate: $("clockDate"),
  clockTime: $("clockTime"),
  helpButton: $("helpButton"),
  inlineHelpButton: $("inlineHelpButton"),
  closeHelpButton: $("closeHelpButton"),
  helpDialog: $("helpDialog"),
  operationBanner: $("operationBanner"),
  fullscreenButton: $("fullscreenButton"),
  mapStage: $("mapStage"),
  mapCanvas: $("mapCanvas"),
  mapEmpty: $("mapEmpty"),
  poseAge: $("poseAge"),
  scanAge: $("scanAge"),
  mapAge: $("mapAge"),
  cursorReadout: $("cursorReadout"),
  zoomReadout: $("zoomReadout"),
  layerMap: $("layerMap"),
  layerScan: $("layerScan"),
  layerPaths: $("layerPaths"),
  layerFootprint: $("layerFootprint"),
  rawScanPanel: $("rawScanPanel"),
  rawScanCanvas: $("rawScanCanvas"),
  rawScanFrame: $("rawScanFrame"),
  panTool: $("panTool"),
  goalTool: $("goalTool"),
  initialTool: $("initialTool"),
  fitTool: $("fitTool"),
  centerTool: $("centerTool"),
  cancelGoalTool: $("cancelGoalTool"),
  missionMode: $("missionMode"),
  quickModeState: $("quickModeState"),
  quickMappingButton: $("quickMappingButton"),
  quickNavigationButton: $("quickNavigationButton"),
  quickStopButton: $("quickStopButton"),
  quickModeHelp: $("quickModeHelp"),
  manualDriveSection: $("manualDriveSection"),
  manualDriveState: $("manualDriveState"),
  driveSpeed: $("driveSpeed"),
  driveSpeedValue: $("driveSpeedValue"),
  targetX: $("targetX"),
  targetY: $("targetY"),
  targetYaw: $("targetYaw"),
  sendGoalButton: $("sendGoalButton"),
  sendInitialButton: $("sendInitialButton"),
  safetyConfirm: $("safetyConfirm"),
  localizationQuality: $("localizationQuality"),
  originPresetButton: $("originPresetButton"),
  readPoseButton: $("readPoseButton"),
  coarseInitial: $("coarseInitial"),
  globalLocalizeButton: $("globalLocalizeButton"),
  nomotionUpdateButton: $("nomotionUpdateButton"),
  goalResult: $("goalResult"),
  goalProgress: $("goalProgress"),
  robotPose: $("robotPose"),
  distanceRemaining: $("distanceRemaining"),
  etaValue: $("etaValue"),
  recoveries: $("recoveries"),
  actionServerState: $("actionServerState"),
  velocityValue: $("velocityValue"),
  cancelGoalButton: $("cancelGoalButton"),
  stackMode: $("stackMode"),
  mappingStartButton: $("mappingStartButton"),
  mappingStopButton: $("mappingStopButton"),
  navigationStartButton: $("navigationStartButton"),
  navigationStopButton: $("navigationStopButton"),
  mappingStartState: $("mappingStartState"),
  mappingStopState: $("mappingStopState"),
  navigationStartState: $("navigationStartState"),
  navigationStopState: $("navigationStopState"),
  mapName: $("mapName"),
  saveMapButton: $("saveMapButton"),
  cameraState: $("cameraState"),
  cameraFeed: $("cameraFeed"),
  cameraEmpty: $("cameraEmpty"),
  cameraHint: $("cameraHint"),
  telemetryRate: $("telemetryRate"),
  mapCells: $("mapCells"),
  mapResolution: $("mapResolution"),
  scanPoints: $("scanPoints"),
  scanLatency: $("scanLatency"),
  wsRate: $("wsRate"),
  serverUptime: $("serverUptime"),
  mapTopic: $("mapTopic"),
  scanTopic: $("scanTopic"),
  poseTopic: $("poseTopic"),
  eventList: $("eventList"),
  footerMessage: $("footerMessage"),
  footerLatency: $("footerLatency"),
  toastStack: $("toastStack"),
};

const ui = {
  config: null,
  telemetry: null,
  scene: null,
  status: null,
  map: null,
  tool: "pan",
  pendingGoal: null,
  pendingInitial: null,
  ws: null,
  reconnectDelay: 1000,
  wsMessageTimes: [],
  wsLatencyMs: null,
  lastDirtyAt: 0,
  renderDirty: true,
  dragging: false,
  dragStart: null,
  dragLast: null,
  pointerMoved: false,
  mappingRunning: false,
  navigationRunning: false,
  driveTimer: null,
  driveKind: "",
};

const view = {
  ready: false,
  cx: 0,
  cy: 0,
  scale: 50,
};

const canvas = els.mapCanvas;
const ctx = canvas.getContext("2d");
let dpr = window.devicePixelRatio || 1;

function setPill(element, state, text) {
  element.classList.toggle("is-online", state === "online");
  element.classList.toggle("is-warn", state === "warn");
  element.querySelector("em").textContent = text;
}

function toast(message, level = "info") {
  const item = document.createElement("div");
  item.className = `toast${level === "error" ? " is-error" : level === "warn" ? " is-warn" : ""}`;
  item.textContent = message;
  els.toastStack.appendChild(item);
  while (els.toastStack.children.length > 4) {
    els.toastStack.firstElementChild.remove();
  }
  window.setTimeout(() => item.remove(), 3800);
}

function formatNumber(value, digits = 2) {
  return value === null || value === undefined || Number.isNaN(Number(value))
    ? "--"
    : Number(value).toFixed(digits);
}

function formatAge(value) {
  if (value === null || value === undefined) return "--";
  return `${Math.round(Number(value))} ms`;
}

function formatDuration(seconds) {
  if (seconds === null || seconds === undefined) return "-- s";
  const value = Math.max(0, Number(seconds));
  if (value < 60) return `${value.toFixed(0)} s`;
  const minutes = Math.floor(value / 60);
  const remainder = Math.floor(value % 60);
  return `${minutes}m ${remainder}s`;
}

function normalizeAngle(angle) {
  let value = angle;
  while (value > Math.PI) value -= Math.PI * 2;
  while (value < -Math.PI) value += Math.PI * 2;
  return value;
}

function resizeCanvas() {
  dpr = window.devicePixelRatio || 1;
  const rect = els.mapStage.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ui.renderDirty = true;
}

function cssSize() {
  const rect = canvas.getBoundingClientRect();
  return {
    width: Math.max(1, rect.width),
    height: Math.max(1, rect.height),
  };
}

function worldToScreen(x, y) {
  const { width, height } = cssSize();
  return {
    x: width * 0.5 + (x - view.cx) * view.scale,
    y: height * 0.5 - (y - view.cy) * view.scale,
  };
}

function screenToWorld(x, y) {
  const { width, height } = cssSize();
  return {
    x: view.cx + (x - width * 0.5) / view.scale,
    y: view.cy - (y - height * 0.5) / view.scale,
  };
}

function fitMap() {
  const map = ui.map;
  if (!map) return;
  const widthM = map.width * map.resolution;
  const heightM = map.height * map.resolution;
  const { width, height } = cssSize();
  view.cx = map.originX + widthM * 0.5;
  view.cy = map.originY + heightM * 0.5;
  view.scale = Math.max(4, Math.min((width - 70) / widthM, (height - 70) / heightM));
  view.ready = true;
  ui.renderDirty = true;
}

function centerRobot() {
  const pose = ui.telemetry?.pose;
  if (!pose || pose.age_ms === null || pose.age_ms === undefined) return;
  view.cx = Number(pose.x);
  view.cy = Number(pose.y);
  ui.renderDirty = true;
}

function buildMapBitmap(width, height, cells) {
  const image = new ImageData(width, height);
  const data = image.data;
  for (let index = 0; index < cells.length; index += 1) {
    const value = cells[index];
    const offset = index * 4;
    if (value < 0) {
      data[offset] = 14;
      data[offset + 1] = 18;
      data[offset + 2] = 18;
      data[offset + 3] = 0;
    } else if (value === 0) {
      data[offset] = 39;
      data[offset + 1] = 55;
      data[offset + 2] = 51;
      data[offset + 3] = 255;
    } else {
      const t = Math.min(1, value / 100);
      data[offset] = Math.round(65 + 145 * t);
      data[offset + 1] = Math.round(80 + 135 * t);
      data[offset + 2] = Math.round(74 + 135 * t);
      data[offset + 3] = 255;
    }
  }
  const bitmap = document.createElement("canvas");
  bitmap.width = width;
  bitmap.height = height;
  bitmap.getContext("2d").putImageData(image, 0, 0);
  return bitmap;
}

function decodeMap(buffer) {
  const bytes = new Uint8Array(buffer);
  if (bytes.length < 38) return null;
  const magic = String.fromCharCode(...bytes.slice(0, 4));
  if (magic !== "EVM1") return null;
  const dataView = new DataView(buffer);
  const headerSize = dataView.getUint8(5);
  const width = dataView.getUint32(6, true);
  const height = dataView.getUint32(10, true);
  const revision = dataView.getUint32(14, true);
  const resolution = dataView.getFloat32(18, true);
  const originX = dataView.getFloat32(22, true);
  const originY = dataView.getFloat32(26, true);
  const originYaw = dataView.getFloat32(30, true);
  const dataLength = dataView.getUint32(34, true);
  if (width <= 0 || height <= 0 || dataLength !== width * height) return null;
  if (headerSize + dataLength > bytes.length) return null;
  const cells = new Int8Array(buffer, headerSize, dataLength);
  return {
    width,
    height,
    revision,
    resolution,
    originX,
    originY,
    originYaw,
    bitmap: buildMapBitmap(width, height, cells),
  };
}

function drawGrid() {
  const { width, height } = cssSize();
  const steps = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50];
  const step = steps.find((value) => value * view.scale >= 54) || 100;
  const min = screenToWorld(0, height);
  const max = screenToWorld(width, 0);
  ctx.save();
  ctx.strokeStyle = "rgba(255,255,255,0.035)";
  ctx.lineWidth = 1;
  for (let x = Math.ceil(min.x / step) * step; x <= max.x; x += step) {
    const p = worldToScreen(x, 0);
    ctx.beginPath();
    ctx.moveTo(p.x, 0);
    ctx.lineTo(p.x, height);
    ctx.stroke();
  }
  for (let y = Math.ceil(min.y / step) * step; y <= max.y; y += step) {
    const p = worldToScreen(0, y);
    ctx.beginPath();
    ctx.moveTo(0, p.y);
    ctx.lineTo(width, p.y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawMapLayer() {
  const map = ui.map;
  if (!map || !els.layerMap.checked) return;
  const origin = worldToScreen(map.originX, map.originY);
  ctx.save();
  ctx.translate(origin.x, origin.y);
  ctx.rotate(-map.originYaw);
  ctx.scale(view.scale * map.resolution, -view.scale * map.resolution);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(map.bitmap, 0, 0);
  ctx.restore();
}

function drawPolyline(points, color, width, dash = []) {
  if (!points || points.length < 2) return;
  ctx.save();
  ctx.beginPath();
  ctx.lineWidth = width;
  ctx.strokeStyle = color;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.setLineDash(dash);
  points.forEach((point, index) => {
    const screen = worldToScreen(point[0], point[1]);
    if (index === 0) ctx.moveTo(screen.x, screen.y);
    else ctx.lineTo(screen.x, screen.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawScan(points) {
  if (!points?.length || !els.layerScan.checked) return;
  const { width, height } = cssSize();
  ctx.save();
  ctx.fillStyle = "rgba(159,232,112,0.72)";
  for (const point of points) {
    const p = worldToScreen(point[0], point[1]);
    if (p.x < -4 || p.y < -4 || p.x > width + 4 || p.y > height + 4) continue;
    ctx.fillRect(p.x - 1, p.y - 1, 2, 2);
  }
  ctx.restore();
}

function drawPoseMarker(pose, pending = false) {
  if (!pose) return;
  const p = worldToScreen(Number(pose.x), Number(pose.y));
  ctx.save();
  ctx.translate(p.x, p.y);
  ctx.rotate(-Number(pose.yaw || 0));
  ctx.strokeStyle = pending ? "var(--amber)" : "#eef7f2";
  ctx.fillStyle = pending ? "rgba(255,180,84,0.18)" : "rgba(70,215,209,0.18)";
  ctx.lineWidth = pending ? 1.5 : 2;
  ctx.setLineDash(pending ? [4, 3] : []);
  ctx.beginPath();
  ctx.moveTo(14, 0);
  ctx.lineTo(-9, -8);
  ctx.lineTo(-5, 0);
  ctx.lineTo(-9, 8);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawRobot() {
  const pose = ui.telemetry?.pose;
  if (
    !pose
    || pose.age_ms === null
    || pose.age_ms === undefined
    || !els.layerFootprint.checked
  ) return;
  const localFootprint = ui.scene?.footprint?.length
    ? ui.scene.footprint
    : [[-0.14, -0.11], [0.14, -0.11], [0.14, 0.11], [-0.14, 0.11]];
  const cos = Math.cos(Number(pose.yaw || 0));
  const sin = Math.sin(Number(pose.yaw || 0));
  const points = localFootprint.map(([lx, ly]) => worldToScreen(
    Number(pose.x) + cos * lx - sin * ly,
    Number(pose.y) + sin * lx + cos * ly,
  ));
  ctx.save();
  ctx.beginPath();
  points.forEach((point, index) => {
    if (index === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.closePath();
  ctx.fillStyle = "rgba(70,215,209,0.15)";
  ctx.strokeStyle = "rgba(70,215,209,0.82)";
  ctx.lineWidth = 1.5;
  ctx.fill();
  ctx.stroke();
  ctx.restore();
  drawPoseMarker({ x: pose.x, y: pose.y, yaw: pose.yaw }, false);
}

function drawMarkers() {
  if (ui.pendingGoal) drawPoseMarker(ui.pendingGoal, true);
  if (ui.scene?.goal) drawPoseMarker(ui.scene.goal, false);
  if (ui.pendingInitial) {
    const p = worldToScreen(ui.pendingInitial.x, ui.pendingInitial.y);
    ctx.save();
    ctx.strokeStyle = "var(--amber)";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.arc(p.x, p.y, 13, 0, Math.PI * 2);
    ctx.moveTo(p.x - 19, p.y);
    ctx.lineTo(p.x + 19, p.y);
    ctx.moveTo(p.x, p.y - 19);
    ctx.lineTo(p.x, p.y + 19);
    ctx.stroke();
    ctx.restore();
  }
  if (ui.scene?.initial_pose) {
    const p = worldToScreen(ui.scene.initial_pose.x, ui.scene.initial_pose.y);
    ctx.save();
    ctx.strokeStyle = "rgba(255,180,84,0.65)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
  }
}

function drawScaleBar() {
  const { width, height } = cssSize();
  const candidates = [0.1, 0.2, 0.5, 1, 2, 5, 10];
  const meters = candidates.find((value) => value * view.scale >= 72) || 20;
  const pixels = meters * view.scale;
  const x = width - pixels - 18;
  const y = height - (width < 520 ? 48 : 20);
  ctx.save();
  ctx.fillStyle = "rgba(12,15,16,0.78)";
  ctx.fillRect(x - 6, y - 16, pixels + 12, 22);
  ctx.strokeStyle = "rgba(238,243,239,0.7)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + pixels, y);
  ctx.moveTo(x, y - 4);
  ctx.lineTo(x, y + 4);
  ctx.moveTo(x + pixels, y - 4);
  ctx.lineTo(x + pixels, y + 4);
  ctx.stroke();
  ctx.fillStyle = "rgba(238,243,239,0.78)";
  ctx.font = '9px "Cascadia Code", Consolas, monospace';
  ctx.fillText(`${meters} m`, x, y - 7);
  ctx.restore();
}

function drawRawScan() {
  const points = ui.scene?.scan_polar || [];
  const show = ui.scene?.scan_transformed === false && points.length > 0;
  els.rawScanPanel.classList.toggle("is-hidden", !show);
  if (!show) return;
  const scanCanvas = els.rawScanCanvas;
  const scanCtx = scanCanvas.getContext("2d");
  const rect = scanCanvas.getBoundingClientRect();
  const width = Math.max(1, rect.width || 142);
  const height = Math.max(1, rect.height || 142);
  if (scanCanvas.width !== Math.round(width * dpr) || scanCanvas.height !== Math.round(height * dpr)) {
    scanCanvas.width = Math.round(width * dpr);
    scanCanvas.height = Math.round(height * dpr);
  }
  scanCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  scanCtx.clearRect(0, 0, width, height);
  const cx = width / 2;
  const cy = height / 2;
  const radius = Math.min(width, height) * 0.43;
  const ranges = points.map((point) => Number(point[1])).filter(Number.isFinite);
  const maxRange = Math.min(12, Math.max(2, ranges.length ? Math.max(...ranges) : 6));
  scanCtx.save();
  scanCtx.strokeStyle = "rgba(255,255,255,0.1)";
  scanCtx.lineWidth = 1;
  for (let i = 1; i <= 4; i += 1) {
    scanCtx.beginPath();
    scanCtx.arc(cx, cy, radius * i / 4, 0, Math.PI * 2);
    scanCtx.stroke();
  }
  scanCtx.beginPath();
  scanCtx.moveTo(cx, cy - radius);
  scanCtx.lineTo(cx, cy + radius);
  scanCtx.moveTo(cx - radius, cy);
  scanCtx.lineTo(cx + radius, cy);
  scanCtx.stroke();
  scanCtx.fillStyle = "rgba(159,232,112,0.8)";
  for (const [angleDeg, range] of points) {
    const angle = Number(angleDeg) * Math.PI / 180;
    const scaled = Math.min(1, Number(range) / maxRange) * radius;
    const x = cx + Math.sin(angle) * scaled;
    const y = cy - Math.cos(angle) * scaled;
    scanCtx.fillRect(x - 0.8, y - 0.8, 1.6, 1.6);
  }
  scanCtx.fillStyle = "rgba(238,243,239,0.62)";
  scanCtx.font = '7px "Cascadia Code", Consolas, monospace';
  scanCtx.textAlign = "center";
  scanCtx.fillText("F", cx, 8);
  scanCtx.fillText(`${maxRange.toFixed(1)}m`, cx, height - 3);
  scanCtx.restore();
  els.rawScanFrame.textContent = ui.scene?.scan_frame || "laser";
}

function render() {
  const { width, height } = cssSize();
  ctx.clearRect(0, 0, width, height);
  drawGrid();
  drawMapLayer();
  if (els.layerPaths.checked) {
    drawPolyline(ui.scene?.global_path, "rgba(70,215,209,0.9)", 2.2);
    drawPolyline(ui.scene?.local_path, "rgba(255,180,84,0.9)", 1.8, [5, 4]);
  }
  drawScan(ui.scene?.scan);
  drawMarkers();
  drawRobot();
  drawScaleBar();
  drawRawScan();
  els.zoomReadout.textContent = `${view.scale.toFixed(0)} px/m`;
}

function animationLoop(now) {
  if (ui.renderDirty || ui.dragging || now - ui.lastDirtyAt > 120) {
    render();
    ui.renderDirty = false;
    ui.lastDirtyAt = now;
  }
  requestAnimationFrame(animationLoop);
}

function updateMapPacket(buffer) {
  const map = decodeMap(buffer);
  if (!map) {
    toast("地图数据格式无效", "error");
    return;
  }
  const firstMap = !ui.map || ui.map.revision !== map.revision;
  ui.map = map;
  els.mapEmpty.classList.add("is-hidden");
  els.mapCells.textContent = (map.width * map.height).toLocaleString("en-US");
  els.mapResolution.textContent = `${map.resolution.toFixed(3)} m/cell`;
  if (firstMap && !view.ready) fitMap();
  ui.renderDirty = true;
}

function renderEvents(events) {
  els.eventList.replaceChildren();
  if (!events?.length) {
    const empty = document.createElement("div");
    empty.className = "event-empty";
    empty.textContent = "等待事件";
    els.eventList.appendChild(empty);
    return;
  }
  for (const event of events.slice(0, 14)) {
    const row = document.createElement("div");
    row.className = `event-row${event.level === "warn" ? " is-warn" : ""}`;
    const time = document.createElement("time");
    time.textContent = event.time;
    const dot = document.createElement("i");
    const message = document.createElement("span");
    message.textContent = event.message;
    row.append(time, dot, message);
    els.eventList.appendChild(row);
  }
}

function renderStatus() {
  const status = ui.status;
  if (!status) return;
  const processes = status.processes || {};
  const mapping = processes.mapping_start || {};
  const navigation = processes.navigation_start || {};
  const mappingStop = processes.mapping_stop || {};
  const navigationStop = processes.navigation_stop || {};
  ui.mappingRunning = Boolean(mapping.running);
  ui.navigationRunning = Boolean(navigation.running);
  els.mappingStartState.textContent = mapping.running ? "RUNNING" : (mapping.last_result || "READY").toUpperCase();
  els.mappingStopState.textContent = mappingStop.running ? "RUNNING" : (mappingStop.last_result || "READY").toUpperCase();
  els.navigationStartState.textContent = navigation.running ? "RUNNING" : (navigation.last_result || "READY").toUpperCase();
  els.navigationStopState.textContent = navigationStop.running ? "RUNNING" : (navigationStop.last_result || "READY").toUpperCase();
  els.mappingStartButton.classList.toggle("is-running", Boolean(mapping.running));
  els.navigationStartButton.classList.toggle("is-running", Boolean(navigation.running));
  els.stackMode.textContent = navigation.running ? "NAV ACTIVE" : mapping.running ? "MAP ACTIVE" : "STANDBY";
  els.quickModeState.textContent = navigation.running ? "NAVIGATION" : mapping.running ? "MAPPING" : "STANDBY";
  els.quickMappingButton.classList.toggle("is-active", Boolean(mapping.running));
  els.quickNavigationButton.classList.toggle("is-active", Boolean(navigation.running));
  els.quickModeHelp.textContent = navigation.running
    ? "导航栈运行中：先设置初始位姿，再点击地图目标。"
    : mapping.running
      ? "建图栈运行中：勾选安全确认后，按住下方方向键低速建图。"
      : "先勾选安全确认，再选择建图模式或导航模式。";
  updateManualDriveState();
  renderEvents(status.events);
}

function renderTelemetry() {
  const telemetry = ui.telemetry;
  if (!telemetry) return;
  const pose = telemetry.pose || {};
  const nav = telemetry.nav || {};
  const map = telemetry.map || {};
  const speed = telemetry.speed || {};
  const frame = ui.config?.map_frame || telemetry.frame || "map";

  els.frameLabel.textContent = frame;
  setPill(els.rosPill, telemetry.ros_connected ? "online" : "warn", telemetry.ros_connected ? "ONLINE" : "OFFLINE");
  setPill(els.navPill, nav.mode === "NAVIGATING" ? "online" : nav.mode?.includes("FAIL") || nav.mode === "ABORTED" ? "warn" : "", nav.mode || "IDLE");

  els.poseAge.textContent = formatAge(pose.age_ms);
  els.scanAge.textContent = formatAge(telemetry.sensor_age_ms?.scan);
  els.mapAge.textContent = formatAge(map.age_ms);
  els.missionMode.textContent = nav.mode || "IDLE";
  els.missionMode.style.color = nav.mode === "NAVIGATING" ? "var(--cyan)" : nav.mode === "SUCCEEDED" ? "var(--green)" : nav.mode === "ABORTED" || nav.mode === "FAILED" ? "var(--red)" : "var(--muted)";
  els.goalResult.textContent = nav.last_result || "--";
  const poseValid = pose.age_ms !== null && pose.age_ms !== undefined;
  els.robotPose.textContent = poseValid
    ? `${formatNumber(pose.x, 2)} / ${formatNumber(pose.y, 2)} / ${formatNumber((Number(pose.yaw) * 180) / Math.PI, 1)}°`
    : "-- / -- / --°";
  els.distanceRemaining.textContent = nav.distance_remaining === null || nav.distance_remaining === undefined ? "-- m" : `${formatNumber(nav.distance_remaining, 2)} m`;
  els.etaValue.textContent = formatDuration(nav.eta_s);
  els.recoveries.textContent = String(nav.recoveries ?? 0);
  els.actionServerState.textContent = nav.server_ready ? "READY" : "WAITING";
  els.actionServerState.style.color = nav.server_ready ? "var(--green)" : "var(--amber)";
  const scanAge = telemetry.sensor_age_ms?.scan;
  const amclAge = telemetry.sensor_age_ms?.amcl;
  const localizationGood = poseValid
    && scanAge !== null
    && scanAge !== undefined
    && scanAge < 600
    && amclAge !== null
    && amclAge !== undefined
    && amclAge < 1500;
  els.localizationQuality.textContent = localizationGood ? "GOOD" : poseValid ? "COARSE" : "WAITING";
  els.localizationQuality.style.color = localizationGood
    ? "var(--green)"
    : poseValid
      ? "var(--amber)"
      : "var(--muted)";
  els.velocityValue.textContent = `${formatNumber(speed.linear_x, 3)} / ${formatNumber(speed.angular_z, 3)}`;
  els.serverUptime.textContent = formatDuration(telemetry.uptime_s);
  els.scanPoints.textContent = String(
    ui.scene?.scan?.length || ui.scene?.scan_polar?.length || 0,
  );
  els.scanLatency.textContent = `age ${formatAge(telemetry.sensor_age_ms?.scan)}`;
  els.telemetryRate.textContent = `${Number(ui.config?.ws_hz || 20).toFixed(0)} HZ`;

  const totalDistance = Number(nav.goal_total_distance || 0);
  const remaining = Number(nav.distance_remaining);
  const progress = totalDistance > 0 && Number.isFinite(remaining)
    ? Math.max(0, Math.min(100, (1 - remaining / totalDistance) * 100))
    : nav.mode === "SUCCEEDED" ? 100 : 0;
  els.goalProgress.style.width = `${progress}%`;
  els.footerMessage.textContent = `NAV CONSOLE / ${nav.mode || "IDLE"} / ${frame}`;
  renderOperationHint();
  updateSafetyButton();
  ui.renderDirty = true;
}

function renderOperationHint() {
  const telemetry = ui.telemetry;
  if (!telemetry) return;
  const nav = telemetry.nav || {};
  const map = telemetry.map || {};
  const pose = telemetry.pose || {};
  const poseValid = pose.age_ms !== null && pose.age_ms !== undefined;
  let label = "当前模式";
  let text = "";
  let state = "";
  if (!telemetry.ros_connected) {
    label = "ROS OFFLINE";
    text = "检查控制台、ROS_DOMAIN_ID 和 DDS";
    state = "is-offline";
  } else if (!map.width) {
    label = "WAITING MAP";
    text = "没有收到 /map，请确认地图服务";
    state = "is-warn";
  } else if (!nav.server_ready) {
    label = "PREVIEW";
    text = "地图和雷达可查看；Nav2 尚未就绪，导航目标按钮已禁用";
    state = "is-warn";
  } else if (!poseValid) {
    label = "LOCALIZE";
    text = "Nav2 已就绪，请设置初始位姿";
    state = "is-warn";
  } else {
    label = "NAV READY";
    text = "定位有效，可以设置目标；发送前必须确认场地安全";
  }
  els.operationBanner.classList.toggle("is-warn", state === "is-warn");
  els.operationBanner.classList.toggle("is-offline", state === "is-offline");
  const first = els.operationBanner.firstElementChild;
  first.replaceChildren();
  const strong = document.createElement("b");
  strong.textContent = label;
  first.append(strong, document.createTextNode(text));
}

function renderScene() {
  if (!ui.scene) return;
  els.scanPoints.textContent = String(
    ui.scene.scan?.length || ui.scene.scan_polar?.length || 0,
  );
  ui.renderDirty = true;
}

function renderConfig() {
  if (!ui.config) return;
  document.title = ui.config.title || "EVI Nav2 Mission Console";
  els.mapTopic.textContent = ui.config.topics?.map || "/map";
  els.scanTopic.textContent = ui.config.topics?.scan || "/scan";
  els.poseTopic.textContent = ui.config.topics?.amcl_pose || "/amcl_pose";
  startCamera();
}

function startCamera() {
  const url = ui.config?.camera_stream_url;
  if (!url) {
    els.cameraState.textContent = "NOT CONFIGURED";
    return;
  }
  els.cameraHint.textContent = url;
  els.cameraFeed.onload = () => {
    els.cameraState.textContent = "LIVE";
    els.cameraEmpty.classList.add("is-hidden");
  };
  els.cameraFeed.onerror = () => {
    els.cameraState.textContent = "WAITING";
    els.cameraEmpty.classList.remove("is-hidden");
  };
  els.cameraFeed.src = `${url}${url.includes("?") ? "&" : "?"}t=${Date.now()}`;
}

function setTool(tool) {
  ui.tool = tool;
  for (const button of [els.panTool, els.goalTool, els.initialTool]) {
    const active = button.dataset.tool === tool;
    button.classList.toggle("is-active", active);
    button.setAttribute("aria-pressed", String(active));
  }
  canvas.classList.toggle("tool-goal", tool === "goal");
  canvas.classList.toggle("tool-initial", tool === "initial");
}

function setCoordinateInputs(x, y) {
  els.targetX.value = Number(x).toFixed(2);
  els.targetY.value = Number(y).toFixed(2);
}

function pointFromEvent(event) {
  const rect = canvas.getBoundingClientRect();
  return {
    sx: event.clientX - rect.left,
    sy: event.clientY - rect.top,
  };
}

function updateSafetyButton() {
  const actionReady = Boolean(ui.telemetry?.nav?.server_ready);
  els.sendGoalButton.disabled = !els.safetyConfirm.checked || !actionReady;
  updateManualDriveState();
}

function updateManualDriveState() {
  const enabled = ui.mappingRunning && els.safetyConfirm.checked;
  els.manualDriveSection.classList.toggle("is-disabled", !enabled);
  els.manualDriveState.textContent = enabled ? "ENABLED" : "LOCKED";
  els.manualDriveState.style.color = enabled ? "var(--green)" : "var(--muted)";
  if (!enabled) stopManualDrive(false);
}

async function sendCommand(payload) {
  const response = await fetch("/api/command", {
    method: "POST",
    cache: "no-store",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const body = await response.json().catch(() => ({}));
  if (!response.ok || body.ok === false) {
    throw new Error(body.error || `HTTP ${response.status}`);
  }
  return body;
}

async function sendGoal() {
  if (!els.safetyConfirm.checked) {
    toast("请先确认场地清空且手在电机开关旁", "warn");
    return;
  }
  const x = Number(els.targetX.value);
  const y = Number(els.targetY.value);
  const yaw = Number(els.targetYaw.value) * Math.PI / 180;
  if (![x, y, yaw].every(Number.isFinite)) {
    toast("目标坐标无效", "error");
    return;
  }
  if (!window.confirm(`确认发送导航目标 X=${x.toFixed(2)} m, Y=${y.toFixed(2)} m？\n场地已清空，手在电机开关旁。`)) return;
  try {
    await sendCommand({ action: "goal", x, y, yaw, safety_confirmed: true });
    ui.pendingGoal = { x, y, yaw };
    els.safetyConfirm.checked = false;
    updateSafetyButton();
    toast("导航目标已发送");
  } catch (error) {
    toast(`发送失败：${error.message}`, "error");
  }
}

async function sendInitialPose() {
  const x = Number(els.targetX.value);
  const y = Number(els.targetY.value);
  const yaw = Number(els.targetYaw.value) * Math.PI / 180;
  if (![x, y, yaw].every(Number.isFinite)) {
    toast("初始位姿无效", "error");
    return;
  }
  try {
    await sendCommand({
      action: "initial_pose",
      x,
      y,
      yaw,
      coarse: els.coarseInitial.checked,
    });
    ui.pendingInitial = { x, y, yaw };
    toast("初始位姿已发送");
  } catch (error) {
    toast(`设置失败：${error.message}`, "error");
  }
}

function useOriginPreset() {
  els.targetX.value = "0.00";
  els.targetY.value = "0.00";
  els.targetYaw.value = "0";
  toast("已填入 A4 建图起点 (0,0,0)；确认车在原点后再设置");
}

function readCurrentPose() {
  const pose = ui.telemetry?.pose;
  if (!pose || pose.age_ms === null || pose.age_ms === undefined) {
    toast("当前没有有效 AMCL/TF 位姿", "warn");
    return;
  }
  els.targetX.value = Number(pose.x).toFixed(2);
  els.targetY.value = Number(pose.y).toFixed(2);
  els.targetYaw.value = (Number(pose.yaw) * 180 / Math.PI).toFixed(1);
  toast("已读取当前 AMCL 位姿");
}

function nudgePose(kind) {
  const step = 0.05;
  const yawStep = 5;
  if (kind === "x-") els.targetX.value = (Number(els.targetX.value || 0) - step).toFixed(2);
  if (kind === "x+") els.targetX.value = (Number(els.targetX.value || 0) + step).toFixed(2);
  if (kind === "y-") els.targetY.value = (Number(els.targetY.value || 0) - step).toFixed(2);
  if (kind === "y+") els.targetY.value = (Number(els.targetY.value || 0) + step).toFixed(2);
  if (kind === "yaw-") els.targetYaw.value = (Number(els.targetYaw.value || 0) - yawStep).toFixed(1);
  if (kind === "yaw+") els.targetYaw.value = (Number(els.targetYaw.value || 0) + yawStep).toFixed(1);
}

async function requestLocalizationUpdate(action, label) {
  try {
    await sendCommand({ action });
    toast(`${label}请求已发送`);
  } catch (error) {
    toast(`${label}失败：${error.message}`, "error");
  }
}

async function cancelGoal() {
  try {
    await sendCommand({ action: "cancel" });
    ui.pendingGoal = null;
    toast("已请求取消导航目标", "warn");
  } catch (error) {
    toast(`取消失败：${error.message}`, "error");
  }
}

async function runOperation(action, label, confirmation) {
  if (confirmation && !window.confirm(confirmation)) return;
  try {
    const result = await sendCommand({ action });
    toast(`${label}：${result.ok === false ? result.error : "已提交"}`, result.ok === false ? "warn" : "info");
  } catch (error) {
    toast(`${label}失败：${error.message}`, "error");
  }
}

function driveVector(kind) {
  const speed = Number(els.driveSpeed.value || 0.08);
  if (kind === "left") return { linear_x: speed * 0.75, angular_z: 0.22 };
  if (kind === "right") return { linear_x: speed * 0.75, angular_z: -0.22 };
  if (kind === "forward") return { linear_x: speed, angular_z: 0.0 };
  return { linear_x: 0.0, angular_z: 0.0 };
}

async function sendTeleop(kind) {
  const vector = driveVector(kind);
  try {
    await sendCommand({
      action: "teleop",
      linear_x: vector.linear_x,
      angular_z: vector.angular_z,
      safety_confirmed: true,
    });
  } catch (error) {
    stopManualDrive(false);
    toast(`遥控失败：${error.message}`, "error");
  }
}

function startManualDrive(kind, button) {
  if (!ui.mappingRunning) {
    toast("请先进入建图模式", "warn");
    return;
  }
  if (!els.safetyConfirm.checked) {
    toast("请先确认场地清空且手在电机开关旁", "warn");
    return;
  }
  stopManualDrive(false);
  ui.driveKind = kind;
  button?.classList.add("is-held");
  sendTeleop(kind);
  ui.driveTimer = window.setInterval(() => sendTeleop(kind), 100);
}

function stopManualDrive(sendZero = true) {
  if (ui.driveTimer) {
    window.clearInterval(ui.driveTimer);
    ui.driveTimer = null;
  }
  ui.driveKind = "";
  for (const button of document.querySelectorAll(".drive-key")) {
    button.classList.remove("is-held");
  }
  if (sendZero && ui.mappingRunning) sendTeleop("stop");
}

async function saveMap() {
  const name = els.mapName.value.trim();
  if (!/^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$/.test(name)) {
    toast("地图名只能包含字母、数字、下划线和短横线", "error");
    return;
  }
  if (!window.confirm(`保存地图为 ${name}？`)) return;
  try {
    await sendCommand({ action: "save_map", name });
    toast(`正在保存地图 ${name}`);
  } catch (error) {
    toast(`保存失败：${error.message}`, "error");
  }
}

function setupInteractions() {
  let resizeQueued = false;
  const observer = new ResizeObserver(() => {
    if (resizeQueued) return;
    resizeQueued = true;
    requestAnimationFrame(() => {
      resizeQueued = false;
      resizeCanvas();
    });
  });
  observer.observe(els.mapStage);

  els.panTool.addEventListener("click", () => setTool("pan"));
  els.goalTool.addEventListener("click", () => setTool("goal"));
  els.initialTool.addEventListener("click", () => setTool("initial"));
  els.fitTool.addEventListener("click", fitMap);
  els.centerTool.addEventListener("click", centerRobot);
  els.cancelGoalTool.addEventListener("click", cancelGoal);
  els.cancelGoalButton.addEventListener("click", cancelGoal);
  els.sendGoalButton.addEventListener("click", sendGoal);
  els.sendInitialButton.addEventListener("click", sendInitialPose);
  els.originPresetButton.addEventListener("click", useOriginPreset);
  els.readPoseButton.addEventListener("click", readCurrentPose);
  els.globalLocalizeButton.addEventListener("click", () => {
    if (window.confirm("全局重定位会让 AMCL 重置粒子分布。确认当前场地与地图一致吗？")) {
      requestLocalizationUpdate("global_localize", "全局重定位");
    }
  });
  els.nomotionUpdateButton.addEventListener("click", () => {
    requestLocalizationUpdate("nomotion_update", "静止更新");
  });
  for (const button of document.querySelectorAll("[data-nudge]")) {
    button.addEventListener("click", () => nudgePose(button.dataset.nudge));
  }
  els.safetyConfirm.addEventListener("change", updateSafetyButton);
  els.quickMappingButton.addEventListener("click", () => runOperation("mapping_start", "进入建图模式"));
  els.quickNavigationButton.addEventListener("click", () => runOperation("navigation_start", "进入导航模式"));
  els.quickStopButton.addEventListener("click", () => runOperation("safe_stop", "安全停止", "确认取消目标、发送零速并停止当前机器人栈？"));
  els.driveSpeed.addEventListener("input", () => {
    els.driveSpeedValue.textContent = `${Number(els.driveSpeed.value).toFixed(2)} m/s`;
  });
  for (const button of document.querySelectorAll("[data-drive]")) {
    const kind = button.dataset.drive;
    if (kind === "stop") {
      button.addEventListener("click", () => stopManualDrive(true));
      continue;
    }
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      button.setPointerCapture?.(event.pointerId);
      startManualDrive(kind, button);
    });
    button.addEventListener("pointerup", () => stopManualDrive(true));
    button.addEventListener("pointercancel", () => stopManualDrive(true));
    button.addEventListener("lostpointercapture", () => stopManualDrive(true));
  }
  window.addEventListener("blur", () => stopManualDrive(true));
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stopManualDrive(true);
  });
  els.mappingStartButton.addEventListener("click", () => runOperation("mapping_start", "开始建图", "确认启动建图栈？车辆不会自动移动，但会启动底盘与传感器链路。"));
  els.mappingStopButton.addEventListener("click", () => runOperation("mapping_stop", "停止建图", "确认停止建图栈？"));
  els.navigationStartButton.addEventListener("click", () => runOperation("navigation_start", "启动导航", "确认启动 Nav2？车辆不会自动移动，仍需手动设置初始位姿和发送目标。"));
  els.navigationStopButton.addEventListener("click", () => runOperation("navigation_stop", "停止导航", "确认取消当前目标并停止 Nav2 进程？"));
  els.saveMapButton.addEventListener("click", saveMap);
  els.fullscreenButton.addEventListener("click", async () => {
    if (document.fullscreenElement) await document.exitFullscreen();
    else await document.documentElement.requestFullscreen();
  });
  els.helpButton.addEventListener("click", () => els.helpDialog.showModal());
  els.inlineHelpButton.addEventListener("click", () => els.helpDialog.showModal());
  els.closeHelpButton.addEventListener("click", () => els.helpDialog.close());
  els.helpDialog.addEventListener("click", (event) => {
    if (event.target === els.helpDialog) els.helpDialog.close();
  });
  for (const element of [els.layerMap, els.layerScan, els.layerPaths, els.layerFootprint]) {
    element.addEventListener("change", () => { ui.renderDirty = true; });
  }

  canvas.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    canvas.setPointerCapture(event.pointerId);
    const point = pointFromEvent(event);
    ui.dragging = true;
    ui.pointerMoved = false;
    ui.dragStart = point;
    ui.dragLast = point;
    canvas.classList.add("is-dragging");
  });

  canvas.addEventListener("pointermove", (event) => {
    const point = pointFromEvent(event);
    const world = screenToWorld(point.sx, point.sy);
    els.cursorReadout.textContent = `X ${world.x.toFixed(2)} / Y ${world.y.toFixed(2)}`;
    if (!ui.dragging) return;
    const dx = point.sx - ui.dragLast.sx;
    const dy = point.sy - ui.dragLast.sy;
    if (Math.abs(point.sx - ui.dragStart.sx) + Math.abs(point.sy - ui.dragStart.sy) > 4) {
      ui.pointerMoved = true;
    }
    if (!ui.pointerMoved) return;
    view.cx -= dx / view.scale;
    view.cy += dy / view.scale;
    view.ready = true;
    ui.dragLast = point;
    ui.renderDirty = true;
  });

  canvas.addEventListener("pointerup", (event) => {
    const point = pointFromEvent(event);
    const world = screenToWorld(point.sx, point.sy);
    ui.dragging = false;
    canvas.classList.remove("is-dragging");
    if (ui.pointerMoved) return;
    setCoordinateInputs(world.x, world.y);
    if (ui.tool === "goal") {
      ui.pendingGoal = { x: world.x, y: world.y, yaw: Number(els.targetYaw.value) * Math.PI / 180 };
      toast("目标点已选择，确认坐标后发送");
    } else if (ui.tool === "initial") {
      ui.pendingInitial = { x: world.x, y: world.y, yaw: Number(els.targetYaw.value) * Math.PI / 180 };
      toast("初始位姿点已选择，确认后发送");
    }
    ui.renderDirty = true;
  });

  canvas.addEventListener("pointercancel", () => {
    ui.dragging = false;
    canvas.classList.remove("is-dragging");
  });

  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    const point = pointFromEvent(event);
    const before = screenToWorld(point.sx, point.sy);
    const factor = Math.exp(-event.deltaY * 0.0012);
    view.scale = Math.max(3, Math.min(600, view.scale * factor));
    const rect = canvas.getBoundingClientRect();
    const cx = rect.width * 0.5;
    const cy = rect.height * 0.5;
    view.cx = before.x - (point.sx - cx) / view.scale;
    view.cy = before.y + (point.sy - cy) / view.scale;
    view.ready = true;
    ui.renderDirty = true;
  }, { passive: false });
}

function connectWebSocket() {
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  const url = `${protocol}//${window.location.host}/ws`;
  const socket = new WebSocket(url);
  socket.binaryType = "arraybuffer";
  ui.ws = socket;
  setPill(els.wsPill, "", "CONNECTING");

  socket.addEventListener("open", () => {
    ui.reconnectDelay = 1000;
    setPill(els.wsPill, "online", "ONLINE");
    els.footerLatency.textContent = "WS LIVE";
  });

  socket.addEventListener("message", (event) => {
    ui.wsMessageTimes.push(performance.now());
    ui.wsMessageTimes = ui.wsMessageTimes.filter((timestamp) => performance.now() - timestamp < 1000);
    if (typeof event.data === "string") {
      if (event.data.startsWith("pong:")) {
        const sentAt = Number(event.data.slice(5));
        ui.wsLatencyMs = Math.max(0, performance.now() - sentAt);
        els.footerLatency.textContent = `WS ${ui.wsLatencyMs.toFixed(0)} ms`;
        return;
      }
      let message;
      try {
        message = JSON.parse(event.data);
      } catch {
        return;
      }
      if (message.type === "config") {
        ui.config = message;
        renderConfig();
      } else if (message.type === "telemetry") {
        ui.telemetry = message;
        renderTelemetry();
      } else if (message.type === "scene") {
        ui.scene = message;
        renderScene();
      } else if (message.type === "status") {
        ui.status = message;
        renderStatus();
      }
    } else if (event.data instanceof ArrayBuffer) {
      updateMapPacket(event.data);
    }
  });

  socket.addEventListener("close", () => {
    setPill(els.wsPill, "warn", "RECONNECT");
    window.setTimeout(connectWebSocket, ui.reconnectDelay);
    ui.reconnectDelay = Math.min(10000, ui.reconnectDelay * 1.7);
  });

  socket.addEventListener("error", () => {
    setPill(els.wsPill, "warn", "ERROR");
    socket.close();
  });
}

function startHeartbeat() {
  window.setInterval(() => {
    if (ui.ws?.readyState === WebSocket.OPEN) {
      ui.ws.send(`ping:${performance.now()}`);
    }
    els.wsRate.textContent = String(ui.wsMessageTimes.length);
    const seconds = Number(ui.telemetry?.uptime_s || 0);
    els.serverUptime.textContent = formatDuration(seconds);
  }, 1000);
}

function updateClock() {
  const now = new Date();
  els.clockTime.textContent = now.toLocaleTimeString("zh-CN", { hour12: false });
  els.clockDate.textContent = now.toLocaleDateString("zh-CN", { month: "2-digit", day: "2-digit" });
}

async function bootstrap() {
  setupInteractions();
  resizeCanvas();
  updateSafetyButton();
  updateClock();
  window.setInterval(updateClock, 1000);
  requestAnimationFrame(animationLoop);
  startHeartbeat();
  connectWebSocket();
}

bootstrap();

