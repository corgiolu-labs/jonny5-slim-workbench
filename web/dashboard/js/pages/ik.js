/**
 * ik.js — Pagina cinematica FK → IK per JONNY5 Dashboard
 *
 * Solo modello POE: FK `compute_fk_poe`, IK `IK_SOLVE` con solver POE (ottimizzazione numerica sul POE del controller).
 */

import {
  connectJ5Dashboard,
  registerTelemetryHandler,
  registerIkResultHandler,
  registerFkPoeResultHandler,
  registerSettingsHandler,
  registerPoeParamsHandler,
  registerUartResponseHandler,
  registerSetposeDoneHandler,
  sendCommand,
  addLog,
  quatToEuler,
  loadRoutingConfig,
} from "../../../shared/js/j5_common.js";

const POE_STORAGE_KEY = "j5_poe_params";
const POE_MIGRATE_FLAG = "j5_poe_migrate_done";
const IK_TARGET_STORAGE_KEY = "j5_ik_target";
const JOINT_NAMES = ["B", "S", "G", "Y", "P", "R"];
/** Etichette risultato IK: stesso ordine B S G Y P R → nomi estesi. */
const JOINT_LABELS_IT = ["Base", "Spalla", "Gomito", "Yaw", "Pitch", "Roll"];
/** Ordine limiti = backend `routing_config.json` / `ik_solver._JOINT_LIMITS_ORDER`. */
const JOINT_LIMIT_KEYS = ["base", "spalla", "gomito", "yaw", "pitch", "roll"];
const FK_INPUT_IDS = ["fk-base", "fk-spalla", "fk-gomito", "fk-yaw", "fk-pitch", "fk-roll"];
const IK_COMPARE_FIELDS = [
  { key: "x", label: "X", unit: "mm", kind: "pos" },
  { key: "y", label: "Y", unit: "mm", kind: "pos" },
  { key: "z", label: "Z", unit: "mm", kind: "pos" },
  { key: "yaw", label: "Yaw", unit: "°", kind: "rot" },
  { key: "pitch", label: "Pitch", unit: "°", kind: "rot" },
  { key: "roll", label: "Roll", unit: "°", kind: "rot" },
];
const IK_COMPARE_CARD_META = {
  target: {
    className: "target",
    title: "Target inviato",
    note: "Ultima posa cartesiana inviata alla IK",
  },
  fk: {
    className: "fk",
    title: "FK live",
    note: "Ricostruita dai giunti correnti del robot",
  },
  imu: {
    className: "imu",
    title: "IMU estimate",
    note: "Stima short-term ancorata al wrist-center",
  },
  delta: {
    className: "delta",
    title: "Errore target - IMU",
    note: "Tende a zero quando l'IMU si avvicina al target",
  },
};
const IMU_GRAVITY_MPS2 = 9.80665;
const IMU_ACCEL_DEADBAND_MPS2 = 0.18;
const IMU_STILL_GYRO_RAD_S = 0.12;
const IMU_MAX_DT_S = 0.08;
const IMU_ACTIVE_DAMP = 0.985;
const IMU_STILL_DAMP = 0.42;
const IMU_MAX_SPEED_MPS = 0.45;
const TOOL_OFFSET_M = [0.06, 0.0, 0.0];

/** Finestra stabilizzazione dopo SETPOSE_DONE (o fallback timeout) prima del prime. */
const IK_COMPARE_SETTLE_MS = 300;
/** Se SETPOSE_DONE non arriva, si passa comunque a stabilizzazione (evita stallo). */
const IK_COMPARE_MOVE_TIMEOUT_MS = 15000;

/** Macchina a stati confronto IK / stima IMU (solo timing del prime, non il modello). */
const IK_COMPARE_PHASE = {
  IDLE: "IDLE",
  WAIT_MOVE_DONE: "WAIT_MOVE_DONE",
  WAIT_SETTLE: "WAIT_SETTLE",
  READY_TO_PRIME: "READY_TO_PRIME",
  TRACKING: "TRACKING",
};

/** Timer wall-clock per uscire da WAIT_SETTLE senza dipendere dalla frequenza telemetry. */
let _ikCompareSettleTimerId = null;

const DEFAULT_JOINT_LIMITS_VIRTUAL = {
  base: { min: 10, max: 170 },
  spalla: { min: 10, max: 170 },
  gomito: { min: 10, max: 170 },
  yaw: { min: 10, max: 170 },
  pitch: { min: 60, max: 120 },
  roll: { min: 60, max: 120 },
};

const POE_DEFAULT = {
  S: [
    [0, 0, 1, 0, 0, 0],
    [0, 1, 0, -0.094, 0, 0],
    [0, 1, 0, -0.154, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 1, 0, -0.311, 0, 0],
    [1, 0, 0, 0, 0.311, 0],
  ],
  M: [
    [1, 0, 0, 0.060],
    [0, 1, 0, 0.000],
    [0, 0, 1, 0.311],
    [0, 0, 0, 1.000],
  ],
};
const MAX_POS_ERROR_DEFAULT = 10.0;
const MAX_ORI_ERROR_DEFAULT = 10.0;

// Settings ricevuti dal server (vel_max e profilo per SETPOSE)
let _ikVel     = 40;
let _ikProfile = "RTR5";
let _ikCompare = {
  activeTarget: null,
  targetSentAtMs: 0,
  comparePhase: IK_COMPARE_PHASE.IDLE,
  moveWaitStartMs: 0,
  settleStartMs: null,
  fkLivePose: null,
  imuEstimatePose: null,
  estimatorPrimed: false,
  estWcPosM: null,
  estVelMps: [0, 0, 0],
  lastTelemetryTsMs: null,
  lastImuSampleCounter: null,
  lastImuRateHz: null,
  lastImuValid: false,
};

/** Ultimo POE ricevuto dal backend (source of truth). */
let _poeFromServer = null;

function _poeCfgEqualsIk(a, b) {
  if (!a?.S || !b?.S || !a?.M || !b?.M) return false;
  try {
    return JSON.stringify(a.S) === JSON.stringify(b.S) && JSON.stringify(a.M) === JSON.stringify(b.M);
  } catch (_) {
    return false;
  }
}

function loadPoeFromLocalOnly() {
  try {
    const raw = localStorage.getItem(POE_STORAGE_KEY);
    if (!raw) return JSON.parse(JSON.stringify(POE_DEFAULT));
    const parsed = JSON.parse(raw);
    if (Array.isArray(parsed?.S) && parsed.S.length === 6 && Array.isArray(parsed?.M) && parsed.M.length === 4) {
      return parsed;
    }
  } catch (_) {}
  return JSON.parse(JSON.stringify(POE_DEFAULT));
}

function resolvePoeCfgFromServerMessageIk(msg) {
  let cfg = { S: msg.S, M: msg.M };
  if (msg.persisted === false && sessionStorage.getItem(POE_MIGRATE_FLAG) !== "1") {
    const ls = loadPoeFromLocalOnly();
    const def = JSON.parse(JSON.stringify(POE_DEFAULT));
    if (!_poeCfgEqualsIk(ls, def)) {
      sendCommand("set_poe_params", { S: ls.S, M: ls.M });
      sessionStorage.setItem(POE_MIGRATE_FLAG, "1");
      addLog("POE: migrazione browser → Raspberry");
      cfg = { S: ls.S, M: ls.M };
    } else {
      sessionStorage.setItem(POE_MIGRATE_FLAG, "1");
    }
  }
  return cfg;
}

function savePoeLocalMirror(cfg) {
  localStorage.setItem(POE_STORAGE_KEY, JSON.stringify({ S: cfg.S, M: cfg.M }));
}

function _copyLimits(src) {
  const o = {};
  for (const k of JOINT_LIMIT_KEYS) {
    o[k] = { min: Number(src[k].min), max: Number(src[k].max) };
  }
  return o;
}

/** Copia mutabile; allineata a `routing_config.json` dopo `loadJointLimitsFromBackend()`. */
let _jointLimitsVirtual = _copyLimits(DEFAULT_JOINT_LIMITS_VIRTUAL);

function clampVirtualJoint(jointKey, deg) {
  const L = _jointLimitsVirtual[jointKey] || { min: 0, max: 180 };
  const v = Number(deg);
  if (!Number.isFinite(v)) return L.min;
  return Math.max(L.min, Math.min(L.max, v));
}

/** 6 angoli virtuali in ordine B S G Y P R. */
function clampAnglesBsgYpr(angles) {
  if (!Array.isArray(angles) || angles.length !== 6) return angles;
  return JOINT_LIMIT_KEYS.map((k, i) => clampVirtualJoint(k, angles[i]));
}

function anglesClampedFromRaw(raw) {
  const c = clampAnglesBsgYpr(raw);
  let changed = false;
  for (let i = 0; i < 6; i++) {
    if (Math.abs(c[i] - Number(raw[i])) > 0.05) {
      changed = true;
      break;
    }
  }
  return { clamped: c, changed };
}

function refreshJointLimitsSummaryText() {
  const el = document.getElementById("ik-limits-summary-fk");
  if (!el) return;
  const parts = JOINT_LIMIT_KEYS.map((k) => {
    const L = _jointLimitsVirtual[k];
    const label = JOINT_LABELS_IT[JOINT_LIMIT_KEYS.indexOf(k)] || k;
    return `${label} [${L.min}–${L.max}°]`;
  });
  el.textContent = `Limiti giunto virtuali (routing_config): ${parts.join(" · ")}`;
}

function applyFkInputAttrLimits() {
  JOINT_LIMIT_KEYS.forEach((k, i) => {
    const inp = document.getElementById(FK_INPUT_IDS[i]);
    if (!inp) return;
    const L = _jointLimitsVirtual[k];
    inp.min = L.min;
    inp.max = L.max;
  });
}

/**
 * Stessi limiti usati da Settings (`/api/routing-config`) e dal clamp UART su Raspberry.
 */
async function loadJointLimitsFromBackend() {
  try {
    const cfg = await loadRoutingConfig();
    if (cfg?.limits && typeof cfg.limits === "object") {
      const next = _copyLimits(_jointLimitsVirtual);
      for (const k of JOINT_LIMIT_KEYS) {
        const row = cfg.limits[k];
        if (!row || typeof row !== "object") continue;
        const mn = Number(row.min);
        const mx = Number(row.max);
        if (Number.isFinite(mn) && Number.isFinite(mx) && mn >= 0 && mx <= 180 && mn < mx) {
          next[k] = { min: mn, max: mx };
        }
      }
      _jointLimitsVirtual = next;
    }
  } catch (_) {
    _jointLimitsVirtual = _copyLimits(DEFAULT_JOINT_LIMITS_VIRTUAL);
  }
  refreshJointLimitsSummaryText();
  applyFkInputAttrLimits();
}

// ---------------------------------------------------------------------------
// Persistenza target cartesiano (X,Y,Z,Roll,Pitch,Yaw)
// ---------------------------------------------------------------------------
function collectTarget() {
  const getNum = (id) => parseFloat(document.getElementById(id)?.value ?? "0") || 0;
  return {
    x: getNum("ik-x"),
    y: getNum("ik-y"),
    z: getNum("ik-z"),
    roll: getNum("ik-roll"),
    pitch: getNum("ik-pitch"),
    yaw: getNum("ik-yaw"),
  };
}

function applyTarget(t) {
  if (!t || typeof t !== "object") return;
  const setVal = (id, val) => {
    const el = document.getElementById(id);
    if (el && Number.isFinite(val)) el.value = val;
  };
  setVal("ik-x", t.x);
  setVal("ik-y", t.y);
  setVal("ik-z", t.z);
  setVal("ik-roll", t.roll);
  setVal("ik-pitch", t.pitch);
  setVal("ik-yaw", t.yaw);
}

function loadTarget() {
  try {
    const raw = localStorage.getItem(IK_TARGET_STORAGE_KEY);
    if (!raw) return null;
    const parsed = JSON.parse(raw);
    const ok = ["x", "y", "z", "roll", "pitch", "yaw"].every(
      (k) => Number.isFinite(Number(parsed?.[k]))
    );
    return ok ? parsed : null;
  } catch (_) {
    return null;
  }
}

function saveTarget(target) {
  localStorage.setItem(IK_TARGET_STORAGE_KEY, JSON.stringify(target));
}

// ---------------------------------------------------------------------------
// Costruisce la griglia risultato IK (6 celle placeholder)
// ---------------------------------------------------------------------------
function buildResultGrid() {
  const grid = document.getElementById("ik-result-grid");
  if (!grid) return;
  grid.innerHTML = "";
  JOINT_NAMES.forEach((name, i) => {
    const cell = document.createElement("div");
    cell.className = "ik-value-cell ik-result-cell";
    cell.id = `ik-res-${name.toLowerCase()}`;
    const lab = JOINT_LABELS_IT[i] || name;
    cell.innerHTML = `
      <div class="cell-label">${lab} <span class="short">(${name})</span></div>
      <div class="cell-value" id="ik-res-val-${name.toLowerCase()}">—</div>
    `;
    grid.appendChild(cell);
  });
}

function _num(v) {
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

function _normAngleDeg(v) {
  let out = Number(v) || 0;
  while (out > 180) out -= 360;
  while (out < -180) out += 360;
  return out;
}

function _fmtPoseValue(key, value) {
  const n = _num(value);
  if (n === null) return "—";
  const unit = key === "x" || key === "y" || key === "z" ? " mm" : "°";
  return `${n.toFixed(1)}${unit}`;
}

function _isFiniteComparePose(pose) {
  return !!pose && IK_COMPARE_FIELDS.every((field) => Number.isFinite(Number(pose[field.key])));
}

function _vecAdd(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function _vecScale(v, s) {
  return [v[0] * s, v[1] * s, v[2] * s];
}

function _vecNorm(v) {
  return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

function _quatWxyzToMatrix3(w, x, y, z) {
  const n = Math.hypot(w, x, y, z) || 1;
  const qw = w / n;
  const qx = x / n;
  const qy = y / n;
  const qz = z / n;
  const xx = qx * qx;
  const yy = qy * qy;
  const zz = qz * qz;
  const xy = qx * qy;
  const xz = qx * qz;
  const yz = qy * qz;
  const wx = qw * qx;
  const wy = qw * qy;
  const wz = qw * qz;
  return [
    [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
    [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
    [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
  ];
}

function _matVecMul3(m, v) {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

function _buildImuEstimatePose(wcPosM, quat) {
  if (!Array.isArray(wcPosM) || wcPosM.length !== 3 || !quat) return null;
  const rot = _quatWxyzToMatrix3(quat.w, quat.x, quat.y, quat.z);
  const toolPosM = _vecAdd(wcPosM, _matVecMul3(rot, TOOL_OFFSET_M));
  const euler = quatToEuler(quat.w, quat.x, quat.y, quat.z);
  const pose = {
    x: toolPosM[0] * 1000.0,
    y: toolPosM[1] * 1000.0,
    z: toolPosM[2] * 1000.0,
    yaw: _normAngleDeg(euler.yaw),
    pitch: _normAngleDeg(euler.pitch),
    roll: _normAngleDeg(euler.roll),
  };
  return _isFiniteComparePose(pose) ? pose : null;
}

function _extractFkLivePose(msg) {
  if (!msg || msg.fk_live_valid !== true) return null;
  const pose = {};
  for (const field of IK_COMPARE_FIELDS) {
    const key = field.key;
    // Backend (ws_handlers_imu) usa fk_live_*_mm per X/Y/Z, non fk_live_x/y/z.
    const rawKey =
      key === "x"
        ? "fk_live_x_mm"
        : key === "y"
          ? "fk_live_y_mm"
          : key === "z"
            ? "fk_live_z_mm"
            : `fk_live_${key}`;
    const val = _num(msg[rawKey]);
    if (val === null) return null;
    pose[key] = field.kind === "rot" ? _normAngleDeg(val) : val;
  }
  pose.wcMm = [
    _num(msg.fk_live_wc_x_mm),
    _num(msg.fk_live_wc_y_mm),
    _num(msg.fk_live_wc_z_mm),
  ];
  if (pose.wcMm.some((v) => v === null)) return null;
  pose.quat = {
    w: _num(msg.fk_live_quat_w),
    x: _num(msg.fk_live_quat_x),
    y: _num(msg.fk_live_quat_y),
    z: _num(msg.fk_live_quat_z),
  };
  if (Object.values(pose.quat).some((v) => v === null)) return null;
  return pose;
}

function _extractImuQuat(msg) {
  const w = _num(msg?.imu_q_w);
  const x = _num(msg?.imu_q_x);
  const y = _num(msg?.imu_q_y);
  const z = _num(msg?.imu_q_z);
  if ([w, x, y, z].some((v) => v === null)) return null;
  return { w, x, y, z };
}

function _statusClassForCompare() {
  if (!_ikCompare.activeTarget) return "idle";
  if (_ikCompare.comparePhase === IK_COMPARE_PHASE.TRACKING && _ikCompare.estimatorPrimed) return "ok";
  if (
    _ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_MOVE_DONE ||
    _ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_SETTLE ||
    _ikCompare.comparePhase === IK_COMPARE_PHASE.READY_TO_PRIME
  ) {
    return "computing";
  }
  if (!_ikCompare.lastImuValid) return "error";
  return "computing";
}

function buildCompareGrid() {
  const grid = document.getElementById("ik-compare-grid");
  if (!grid) return;
  grid.innerHTML = "";
  for (const [cardKey, meta] of Object.entries(IK_COMPARE_CARD_META)) {
    const card = document.createElement("div");
    card.className = `ik-compare-card ${meta.className}`;
    card.innerHTML = `
      <div class="ik-compare-card-title">
        <h3>${meta.title}</h3>
        <span id="cmp-meta-${cardKey}">${meta.note}</span>
      </div>
      <div class="ik-num-grid" id="cmp-grid-${cardKey}"></div>
    `;
    const inner = card.querySelector(`#cmp-grid-${cardKey}`);
    IK_COMPARE_FIELDS.forEach((field) => {
      const cell = document.createElement("div");
      cell.className = "ik-value-cell out-pose";
      cell.innerHTML = `
        <div class="cell-label">${field.label} <span class="short">${field.unit}</span></div>
        <div class="cell-value" id="cmp-${cardKey}-${field.key}">—</div>
      `;
      inner.appendChild(cell);
    });
    grid.appendChild(card);
  }
}

function _setCompareMeta(cardKey, text) {
  const el = document.getElementById(`cmp-meta-${cardKey}`);
  if (el) el.textContent = text;
}

function _renderComparePose(cardKey, pose) {
  IK_COMPARE_FIELDS.forEach((field) => {
    const el = document.getElementById(`cmp-${cardKey}-${field.key}`);
    if (!el) return;
    el.textContent = pose ? _fmtPoseValue(field.key, pose[field.key]) : "—";
  });
}

function _currentTargetPose() {
  return _ikCompare.activeTarget || collectTarget();
}

function _currentTargetLabel() {
  if (_ikCompare.activeTarget && _ikCompare.targetSentAtMs > 0) {
    return `Ultimo invio: ${new Date(_ikCompare.targetSentAtMs).toLocaleTimeString()}`;
  }
  return "Campi IK correnti (non ancora inviati)";
}

function _computeTargetMinusPose(target, pose) {
  if (!target || !pose) return null;
  const out = {};
  IK_COMPARE_FIELDS.forEach((field) => {
    const t = _num(target[field.key]);
    const p = _num(pose[field.key]);
    if (t === null || p === null) {
      out[field.key] = null;
      return;
    }
    out[field.key] = field.kind === "rot" ? _normAngleDeg(t - p) : (t - p);
  });
  return out;
}

function _setCompareStatus(text) {
  const badge = document.getElementById("ik-compare-status");
  if (!badge) return;
  const cls = _statusClassForCompare();
  const dot = cls === "ok" ? "●" : cls === "error" ? "✕" : cls === "computing" ? "…" : "○";
  badge.className = `ik-status ${cls}`;
  badge.innerHTML = `<span>${dot}</span><span>${text}</span>`;
}

function renderIkCompare() {
  const target = _currentTargetPose();
  const delta = _computeTargetMinusPose(_ikCompare.activeTarget, _ikCompare.imuEstimatePose);
  _renderComparePose("target", target);
  _renderComparePose("fk", _ikCompare.fkLivePose);
  _renderComparePose("imu", _ikCompare.imuEstimatePose);
  _renderComparePose("delta", delta);

  _setCompareMeta("target", _currentTargetLabel());
  _setCompareMeta("fk", _ikCompare.fkLivePose ? "Stato live dai servo_deg_* correnti" : "In attesa di telemetria servo");
  _setCompareMeta(
    "imu",
    _ikCompare.estimatorPrimed
      ? "Stima attiva da wrist-center + IMU"
      : "Prime dopo SETPOSE_DONE + stabilizzazione (vedi stato sopra)",
  );
  _setCompareMeta("delta", _ikCompare.activeTarget ? "Differenza signed: target - stima IMU" : "Diventa utile dopo il primo invio IK");

  if (!_ikCompare.activeTarget) {
    _setCompareStatus("Invia una posa IK per armare il confronto realtime");
  } else if (_ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_MOVE_DONE) {
    _setCompareStatus("In attesa completamento movimento (SETPOSE_DONE)…");
  } else if (_ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_SETTLE) {
    _setCompareStatus("Movimento completato, attesa stabilizzazione…");
  } else if (_ikCompare.comparePhase === IK_COMPARE_PHASE.READY_TO_PRIME) {
    if (!_ikCompare.lastImuValid) {
      _setCompareStatus("Pronto al prime: in attesa IMU valida…");
    } else {
      _setCompareStatus("Pronto al prime della stima (primo campione utile)…");
    }
  } else if (_ikCompare.comparePhase === IK_COMPARE_PHASE.TRACKING && _ikCompare.estimatorPrimed) {
    _setCompareStatus("Tracking IMU attivo: confronto realtime in corso");
  } else if (!_ikCompare.lastImuValid) {
    _setCompareStatus("Target armato ma IMU non valida: tracking in attesa");
  } else {
    _setCompareStatus("Target armato: fase compare in transizione…");
  }

  const meta = document.getElementById("ik-compare-meta");
  if (meta) {
    const parts = [
      `Target attivo: ${_ikCompare.activeTarget ? "sì" : "no"}`,
      `Fase: ${_ikCompare.activeTarget ? _ikCompare.comparePhase : IK_COMPARE_PHASE.IDLE}`,
      `FK live: ${_ikCompare.fkLivePose ? "ok" : "—"}`,
      `IMU: ${_ikCompare.lastImuValid ? "valida" : "non valida"}`,
      `Sample: ${_ikCompare.lastImuSampleCounter ?? "-"}`,
    ];
    if (_ikCompare.lastImuRateHz !== null && Number.isFinite(_ikCompare.lastImuRateHz)) {
      parts.push(`Rate: ${_ikCompare.lastImuRateHz.toFixed(1)} Hz`);
    }
    meta.textContent = parts.join(" · ");
  }
}

function _clearIkCompareSettleTimer() {
  if (_ikCompareSettleTimerId != null) {
    clearTimeout(_ikCompareSettleTimerId);
    _ikCompareSettleTimerId = null;
  }
}

/** Dopo SETPOSE_DONE (o fallback timeout movimento): finestra IK_COMPARE_SETTLE_MS poi READY_TO_PRIME. */
function _scheduleIkCompareSettleEnd() {
  _clearIkCompareSettleTimer();
  _ikCompare.settleStartMs = performance.now();
  _ikCompareSettleTimerId = setTimeout(() => {
    _ikCompareSettleTimerId = null;
    if (!_ikCompare.activeTarget || _ikCompare.comparePhase !== IK_COMPARE_PHASE.WAIT_SETTLE) return;
    _ikCompare.comparePhase = IK_COMPARE_PHASE.READY_TO_PRIME;
    _ikCompare.settleStartMs = null;
    renderIkCompare();
  }, IK_COMPARE_SETTLE_MS);
}

function armIkCompareTarget(targetPose) {
  _clearIkCompareSettleTimer();
  _ikCompare = {
    activeTarget: { ...targetPose },
    targetSentAtMs: Date.now(),
    comparePhase: IK_COMPARE_PHASE.WAIT_MOVE_DONE,
    moveWaitStartMs: performance.now(),
    settleStartMs: null,
    // FK live è telemetry-driven: non invalidare la cache JS (evita buco UI); reset solo stato integratore.
    fkLivePose: _ikCompare.fkLivePose,
    imuEstimatePose: null,
    estimatorPrimed: false,
    estWcPosM: null,
    estVelMps: [0, 0, 0],
    lastTelemetryTsMs: null,
    lastImuSampleCounter: null,
    lastImuRateHz: _ikCompare.lastImuRateHz,
    lastImuValid: _ikCompare.lastImuValid,
  };
  renderIkCompare();
}

function _advanceIkComparePhases() {
  if (!_ikCompare.activeTarget) return;
  const now = performance.now();
  if (_ikCompare.comparePhase === IK_COMPARE_PHASE.WAIT_MOVE_DONE) {
    if (now - _ikCompare.moveWaitStartMs >= IK_COMPARE_MOVE_TIMEOUT_MS) {
      addLog(
        `⚠ IK compare: nessun SETPOSE_DONE entro ${IK_COMPARE_MOVE_TIMEOUT_MS} ms — fallback a stabilizzazione (${IK_COMPARE_SETTLE_MS} ms) prima del prime.`,
      );
      _ikCompare.comparePhase = IK_COMPARE_PHASE.WAIT_SETTLE;
      _scheduleIkCompareSettleEnd();
    }
  }
}

function initIkCompareSetposeDone() {
  registerSetposeDoneHandler((msg) => {
    if (!msg || msg.type !== "setpose_done") return;
    if (_ikCompare.comparePhase !== IK_COMPARE_PHASE.WAIT_MOVE_DONE || !_ikCompare.activeTarget) return;
    _ikCompare.comparePhase = IK_COMPARE_PHASE.WAIT_SETTLE;
    _scheduleIkCompareSettleEnd();
    addLog("IK compare: SETPOSE_DONE ricevuto — avvio finestra stabilizzazione prima del prime.");
    renderIkCompare();
  });
}

function _primeImuEstimatorFromTelemetry(msg) {
  if (!_ikCompare.activeTarget || _ikCompare.estimatorPrimed) return;
  const fkPose = _ikCompare.fkLivePose || _extractFkLivePose(msg);
  const quat = _extractImuQuat(msg);
  if (!fkPose || !quat) return;
  const estWcPosM = fkPose.wcMm.map((v) => Number(v) / 1000.0);
  const pose = _buildImuEstimatePose(estWcPosM, quat);
  if (!pose) return;
  _ikCompare.fkLivePose = fkPose;
  _ikCompare.estWcPosM = estWcPosM;
  _ikCompare.estVelMps = [0, 0, 0];
  _ikCompare.lastTelemetryTsMs = performance.now();
  _ikCompare.lastImuSampleCounter = _num(msg.imu_sample_counter);
  _ikCompare.imuEstimatePose = pose;
  _ikCompare.estimatorPrimed = true;
  renderIkCompare();
}

function _updateImuEstimator(msg) {
  _ikCompare.lastImuValid = msg?.imu_valid === true;
  _ikCompare.lastImuRateHz = Number.isFinite(Number(msg?.imu_rate_hz_est)) ? Number(msg.imu_rate_hz_est) : _ikCompare.lastImuRateHz;

  if (_ikCompare.activeTarget) {
    _advanceIkComparePhases();
  }

  if (!_ikCompare.activeTarget) {
    renderIkCompare();
    return;
  }

  const phase = _ikCompare.comparePhase;
  if (phase === IK_COMPARE_PHASE.WAIT_MOVE_DONE || phase === IK_COMPARE_PHASE.WAIT_SETTLE) {
    renderIkCompare();
    return;
  }

  if (!_ikCompare.lastImuValid) {
    renderIkCompare();
    return;
  }

  if (phase === IK_COMPARE_PHASE.READY_TO_PRIME) {
    _primeImuEstimatorFromTelemetry(msg);
    if (_ikCompare.estimatorPrimed) {
      _ikCompare.comparePhase = IK_COMPARE_PHASE.TRACKING;
    }
  }

  const canIntegrate =
    _ikCompare.comparePhase === IK_COMPARE_PHASE.TRACKING &&
    _ikCompare.estimatorPrimed &&
    _ikCompare.estWcPosM;

  if (!canIntegrate) {
    renderIkCompare();
    return;
  }

  const quat = _extractImuQuat(msg);
  const accBody = [_num(msg?.imu_accel_x), _num(msg?.imu_accel_y), _num(msg?.imu_accel_z)];
  const gyro = [_num(msg?.imu_gyro_x), _num(msg?.imu_gyro_y), _num(msg?.imu_gyro_z)];
  if (!quat || accBody.some((v) => v === null) || gyro.some((v) => v === null)) {
    renderIkCompare();
    return;
  }

  const sampleCounter = _num(msg?.imu_sample_counter);
  const nowMs = performance.now();
  if (
    sampleCounter !== null &&
    _ikCompare.lastImuSampleCounter !== null &&
    sampleCounter === _ikCompare.lastImuSampleCounter
  ) {
    renderIkCompare();
    return;
  }

  let dt = IMU_MAX_DT_S;
  if (_ikCompare.lastTelemetryTsMs !== null) {
    dt = Math.max(0.001, Math.min(IMU_MAX_DT_S, (nowMs - _ikCompare.lastTelemetryTsMs) / 1000.0));
  }
  _ikCompare.lastTelemetryTsMs = nowMs;
  _ikCompare.lastImuSampleCounter = sampleCounter;

  const rot = _quatWxyzToMatrix3(quat.w, quat.x, quat.y, quat.z);
  const accWorld = _matVecMul3(rot, accBody);
  let linAccWorld = [accWorld[0], accWorld[1], accWorld[2] - IMU_GRAVITY_MPS2];
  const accMag = _vecNorm(linAccWorld);
  const gyroMag = _vecNorm(gyro);
  if (accMag < IMU_ACCEL_DEADBAND_MPS2) {
    linAccWorld = [0, 0, 0];
  }

  const damp = (linAccWorld[0] === 0 && linAccWorld[1] === 0 && linAccWorld[2] === 0 && gyroMag < IMU_STILL_GYRO_RAD_S)
    ? IMU_STILL_DAMP
    : IMU_ACTIVE_DAMP;
  let nextVel = _vecAdd(_vecScale(_ikCompare.estVelMps, damp), _vecScale(linAccWorld, dt));
  const speed = _vecNorm(nextVel);
  if (speed > IMU_MAX_SPEED_MPS) {
    nextVel = _vecScale(nextVel, IMU_MAX_SPEED_MPS / Math.max(speed, 1e-6));
  }
  if (gyroMag < IMU_STILL_GYRO_RAD_S * 0.7 && _vecNorm(linAccWorld) < IMU_ACCEL_DEADBAND_MPS2 * 0.5) {
    nextVel = [0, 0, 0];
  }
  _ikCompare.estVelMps = nextVel;
  const nextWcPosM = _vecAdd(_ikCompare.estWcPosM, _vecScale(nextVel, dt));
  const pose = _buildImuEstimatePose(nextWcPosM, quat);
  if (!pose) {
    renderIkCompare();
    return;
  }
  _ikCompare.estWcPosM = nextWcPosM;
  _ikCompare.imuEstimatePose = pose;
  renderIkCompare();
}

function initCompareTelemetry() {
  registerTelemetryHandler((msg) => {
    const fkPose = _extractFkLivePose(msg);
    if (fkPose) _ikCompare.fkLivePose = fkPose;
    _updateImuEstimator(msg);
  });
}

// ---------------------------------------------------------------------------
// Aggiorna lo status badge
// ---------------------------------------------------------------------------
function setIKStatus(state, text) {
  const badge = document.getElementById("ik-status-badge");
  const dot   = document.getElementById("ik-status-dot");
  const label = document.getElementById("ik-status-text");
  if (!badge) return;
  badge.className = `ik-status ${state}`;
  dot.textContent   = state === "ok" ? "●" : state === "error" ? "✕" : state === "computing" ? "…" : "○";
  label.textContent = text;
}

// ---------------------------------------------------------------------------
// Aggiorna le celle risultato IK
// ---------------------------------------------------------------------------
function setIKResult(angles, reachable) {
  let display = angles;
  if (reachable && Array.isArray(angles) && angles.length === 6) {
    const { clamped, changed } = anglesClampedFromRaw(angles);
    display = clamped;
    if (changed) {
      addLog("IK: angoli mostrati riportati entro limiti globali (per giunto).");
    }
  }
  JOINT_NAMES.forEach((name, i) => {
    const cell = document.getElementById(`ik-res-${name.toLowerCase()}`);
    const val  = document.getElementById(`ik-res-val-${name.toLowerCase()}`);
    if (!cell || !val) return;
    if (reachable) {
      cell.className = "ik-value-cell ik-result-cell reachable";
      val.className  = "cell-value";
      val.textContent = `${display[i].toFixed(1)}°`;
    } else {
      cell.className = "ik-value-cell ik-result-cell unreachable";
      val.className  = "cell-value unreachable";
      val.textContent = "—";
    }
  });

  const btnSend = document.getElementById("btn-send-ik");
  if (btnSend) btnSend.disabled = !reachable;
}

// ---------------------------------------------------------------------------
// Sezioni collassabili (stesso pattern di settings.js)
// ---------------------------------------------------------------------------
function initCollapsibles() {
  document.querySelectorAll(".ik-section-header").forEach(header => {
    const targetId = header.dataset.target;
    const body     = document.getElementById(targetId);
    if (!body) return;
    header.addEventListener("click", () => {
      const isCollapsed = header.classList.toggle("collapsed");
      body.style.display = isCollapsed ? "none" : "";
    });
  });
}

// ---------------------------------------------------------------------------
// Pulsante Salva target cartesiano
// ---------------------------------------------------------------------------
function initSaveTarget() {
  const btn = document.getElementById("btn-save-target");
  const fb = document.getElementById("fb-target");
  if (!btn) return;

  btn.addEventListener("click", () => {
    const t = collectTarget();
    saveTarget(t);
    if (fb) {
      fb.textContent = "Target salvato ✓";
      fb.className = "save-feedback ok visible";
      setTimeout(() => { fb.className = "save-feedback"; }, 2500);
    }
    addLog(`Target IK salvato: (${t.x}, ${t.y}, ${t.z}) YPR=(${t.yaw}, ${t.pitch}, ${t.roll})`);
  });
}

// ---------------------------------------------------------------------------
// Payload base64 per IK_SOLVE: solo solver POE + tolleranze (il modello S/M è sul controller)
// ---------------------------------------------------------------------------
function buildIkSolvePayloadB64() {
  const resetSolverCb = document.getElementById("ik-reset-solver");
  const resetSolver = resetSolverCb ? !!resetSolverCb.checked : true;
  const maxPosEl = document.getElementById("ik-max-pos-error");
  const maxOriEl = document.getElementById("ik-max-ori-error");
  const maxPos = Math.max(0.1, parseFloat(maxPosEl?.value || `${MAX_POS_ERROR_DEFAULT}`) || MAX_POS_ERROR_DEFAULT);
  const maxOri = Math.max(0.1, parseFloat(maxOriEl?.value || `${MAX_ORI_ERROR_DEFAULT}`) || MAX_ORI_ERROR_DEFAULT);
  try {
    const json = JSON.stringify({
      solver: "POE",
      fallback_numeric: false,
      reset_solver: resetSolver,
      max_pos_error_mm: maxPos,
      max_ori_error_deg: maxOri,
      preferred_angles_deg: [90, 90, 90, 90, 90, 90],
    });
    const bytes = new TextEncoder().encode(json);
    let bin = "";
    for (const b of bytes) bin += String.fromCharCode(b);
    return btoa(bin);
  } catch (_) {
    return "";
  }
}

// ---------------------------------------------------------------------------
// Pulsante Calcola IK — invia IK_SOLVE al backend Python
// ---------------------------------------------------------------------------
function initCalcIK() {
  const btn = document.getElementById("btn-calc-ik");
  if (!btn) return;

  // Abilita il bottone (il solver è ora attivo)
  btn.disabled = false;
  btn.title = "";

  // Rimuove eventuale nota "disponibile con solver attivo"
  const note = btn.nextElementSibling;
  if (note && note.tagName === "SPAN") note.style.display = "none";

  btn.addEventListener("click", () => {
    const x     = parseFloat(document.getElementById("ik-x")?.value)     || 0;
    const y     = parseFloat(document.getElementById("ik-y")?.value)     || 0;
    const z     = parseFloat(document.getElementById("ik-z")?.value)     || 0;
    const roll  = parseFloat(document.getElementById("ik-roll")?.value)  || 0;
    const pitch = parseFloat(document.getElementById("ik-pitch")?.value) || 0;
    const yaw   = parseFloat(document.getElementById("ik-yaw")?.value)   || 0;

    setIKStatus("computing", "Calcolo in corso…");
    btn.disabled = true;

    const payloadB64 = buildIkSolvePayloadB64();
    const cmd   = `IK_SOLVE ${x} ${y} ${z} ${roll} ${pitch} ${yaw}${payloadB64 ? " " + payloadB64 : ""}`;
    sendCommand("uart", { cmd });
    addLog(`IK_SOLVE[POE] → (${x}, ${y}, ${z}) YPR=(${yaw}, ${pitch}, ${roll})`);
  });
}

// ---------------------------------------------------------------------------
// Handler risposta ik_result dal server
// ---------------------------------------------------------------------------
function initIkResultHandler() {
  registerIkResultHandler((msg) => {
    const btn = document.getElementById("btn-calc-ik");
    if (btn) btn.disabled = false;

    const solverUsedEl = document.getElementById("ik-solver-used");
    const elapsedEl = document.getElementById("ik-elapsed-ms");
    if (solverUsedEl) solverUsedEl.textContent = msg.solver_used || "POE";
    if (elapsedEl) elapsedEl.textContent = Number.isFinite(Number(msg.elapsed_ms)) ? Number(msg.elapsed_ms).toFixed(2) : "-";

    if (msg.reachable) {
      const su = msg.solver_used || "POE";
      setIKStatus("ok", `Soluzione trovata — ${su} · err_pos ${msg.error_pos} mm, err_ori ${msg.error_ori}°, iter ${msg.iterations}`);
      setIKResult(msg.angles_deg, true);
      const shown = clampAnglesBsgYpr(msg.angles_deg);
      addLog(`IK ${su} OK — angoli: [${shown.map(v => v.toFixed(1)).join(", ")}] (${elapsedEl?.textContent || "-"} ms)`);
    } else {
      const su = msg.solver_used || "POE";
      setIKStatus("error", msg.message || "Target fuori workspace");
      setIKResult([], false);
      addLog(`IK ${su} FAIL — ${msg.message || "fuori workspace"}`);
    }

    // Aggiorna il pill solver
    const solver = document.getElementById("diag-solver");
    if (solver) {
      solver.textContent = msg.reachable ? "OK" : "FAIL";
      solver.className   = `diag-value state-pill ${msg.reachable ? "state-on" : "state-warn"}`;
    }
  });
}

// ---------------------------------------------------------------------------
// Pulsante Invia SETPOSE
// ---------------------------------------------------------------------------
function initSendIK() {
  const btn = document.getElementById("btn-send-ik");
  if (!btn) return;
  btn.addEventListener("click", () => {
    const targetPose = collectTarget();
    const vals = JOINT_NAMES.map(name => {
      const el = document.getElementById(`ik-res-val-${name.toLowerCase()}`);
      return el ? parseFloat(el.textContent) : NaN;
    });
    const [b, s, g, y, p, r] = vals.map((v, i) => {
      const key = JOINT_LIMIT_KEYS[i];
      const base = Number.isFinite(v) ? v : 90;
      return Math.round(clampVirtualJoint(key, base));
    });

    // Usa vel e profilo ricevuti dal server via get_settings (aggiornati in _ikVel/_ikProfile).
    // ws_server si aspetta: SETPOSE B S G Y P R vel_deg_s PLANNER
    const cmd = `SETPOSE ${b} ${s} ${g} ${y} ${p} ${r} ${_ikVel} ${_ikProfile}`;
    if (!sendCommand("uart", { cmd })) {
      addLog("✗ SETPOSE IK non inviato (WS non connesso)");
      return;
    }
    addLog(`… SETPOSE IK inviato (pending conferma): ${cmd}`);
    addLog("… Stima IMU reset: attesa primo ancoraggio FK/IMU post-SETPOSE");
    armIkCompareTarget(targetPose);
  });
}

// ---------------------------------------------------------------------------
// Pulsante HOME (pose HOME via comando UART)
// ---------------------------------------------------------------------------
function initHomeIK() {
  const btn = document.getElementById("btn-home-ik");
  if (!btn) return;
  btn.addEventListener("click", () => {
    // Richiama la pose HOME gestita dal controller (usa offsets/settings correnti).
    if (!sendCommand("uart", { cmd: "HOME" })) {
      addLog("✗ IK HOME non inviato (WS non connesso)");
      return;
    }
    addLog("… IK HOME inviato (pending conferma)");
  });
}

// ---------------------------------------------------------------------------
// Forward kinematics (POE) — compute_fk_poe
// ---------------------------------------------------------------------------
function initCalcFk() {
  const btn = document.getElementById("btn-calc-fk");
  if (!btn) return;
  btn.addEventListener("click", () => {
    const gv = (id) => parseFloat(document.getElementById(id)?.value);
    const angles = JOINT_LIMIT_KEYS.map((k, i) => {
      const id = FK_INPUT_IDS[i];
      const raw = gv(id);
      return clampVirtualJoint(k, Number.isFinite(raw) ? raw : 90);
    });
    const st = document.getElementById("fk-status");
    if (st) st.textContent = "Calcolo in corso…";
    if (!sendCommand("compute_fk_poe", { angles_deg: angles })) {
      if (st) st.textContent = "WebSocket non connesso";
      addLog("FK: WS non pronto");
    }
  });
}

function initFkResultHandler() {
  registerFkPoeResultHandler((msg) => {
    const st = document.getElementById("fk-status");
    const setOut = (id, v) => {
      const el = document.getElementById(id);
      if (el) el.textContent = v;
    };
    if (!msg.ok) {
      if (st) st.textContent = msg.error || "FK fallita";
      setOut("fk-out-x", "—");
      setOut("fk-out-y", "—");
      setOut("fk-out-z", "—");
      setOut("fk-out-roll", "—");
      setOut("fk-out-pitch", "—");
      setOut("fk-out-yaw", "—");
      const qel = document.getElementById("fk-out-quat");
      if (qel) qel.textContent = "Quaternione (xyzw): —";
      return;
    }
    if (st) st.textContent = "OK";
    const x = Number(msg.x_mm);
    const y = Number(msg.y_mm);
    const z = Number(msg.z_mm);
    const roll = Number(msg.roll_deg);
    const pitch = Number(msg.pitch_deg);
    const yaw = Number(msg.yaw_deg);
    setOut("fk-out-x", String(msg.x_mm));
    setOut("fk-out-y", String(msg.y_mm));
    setOut("fk-out-z", String(msg.z_mm));
    setOut("fk-out-roll", String(msg.roll_deg));
    setOut("fk-out-pitch", String(msg.pitch_deg));
    setOut("fk-out-yaw", String(msg.yaw_deg));
    if ([x, y, z, roll, pitch, yaw].every((v) => Number.isFinite(v))) {
      applyTarget({ x, y, z, roll, pitch, yaw });
      addLog("FK: posa copiata nei campi target IK");
    }
    const q = msg.quat_xyzw;
    const qel = document.getElementById("fk-out-quat");
    if (qel && Array.isArray(q) && q.length === 4) {
      qel.textContent = `Quaternione (xyzw): ${q.map((n) => Number(n).toFixed(4)).join(", ")}`;
    }
  });
}

// ---------------------------------------------------------------------------
// Handler settings dal server — aggiorna vel e profilo per SETPOSE
// ---------------------------------------------------------------------------
function onSettings(data) {
  if (data.vel_max !== undefined) _ikVel     = Number(data.vel_max);
  if (data.profile !== undefined) _ikProfile = String(data.profile);
}

function initPoeParamsHandler() {
  registerPoeParamsHandler((msg) => {
    if (msg.type === "poe_params" && Array.isArray(msg.S) && msg.S.length === 6 && Array.isArray(msg.M) && msg.M.length === 4) {
      const cfg = resolvePoeCfgFromServerMessageIk(msg);
      _poeFromServer = cfg;
      savePoeLocalMirror(cfg);
      addLog("POE sincronizzato dal Raspberry (FK/IK)");
    } else if (msg.type === "poe_params_saved") {
      if (msg.ok) sendCommand("get_poe_params", {});
    }
  });
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
function init() {
  const savedTarget = loadTarget();
  if (savedTarget) {
    applyTarget(savedTarget);
  }
  buildResultGrid();
  buildCompareGrid();
  loadJointLimitsFromBackend();
  initCollapsibles();
  initSaveTarget();
  initCalcIK();
  initIkResultHandler();
  initCalcFk();
  initFkResultHandler();
  initSendIK();
  initHomeIK();
  initPoeParamsHandler();
  initCompareTelemetry();
  initIkCompareSetposeDone();
  registerSettingsHandler(onSettings);
  registerUartResponseHandler((msg) => {
    if (msg?.type === "uart_response" && msg?.warning) {
      addLog(`⚠ ${msg.warning}`);
      alert(msg.warning);
    }
  });
  connectJ5Dashboard();
  sendCommand("get_settings", {});
  sendCommand("get_poe_params", {});
  renderIkCompare();
  addLog("Pagina FK / IK caricata");
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init);
} else {
  init();
}
