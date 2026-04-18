/**
 * JONNY5 Dashboard – funzioni comuni e inizializzazione
 *
 * NOTE [RPI-SAFE-REFACTOR-PHASE2]: modulo analizzato, nessuna modifica funzionale.
 * CORE: binding navbar + pulsanti SAFE/ENABLE/STOP/IMU/HOME/PARK/DEMO via window.wsSend.
 * LEGACY/UTILITY: fallback window.addLog quando j5_common.js non è caricato.
 */
import {
  addLog,
  sendCommand,
  connectJ5Dashboard,
  registerUartResponseHandler,
  registerSelfTestStatusHandler,
  registerSelfTestResultHandler,
} from "../../shared/js/j5_common.js";
/** Inizializzazione base: attiva il link navbar della pagina corrente */
function initNavbarActive() {
  const nav = document.getElementById("navbar");
  if (!nav) return;

  const path = window.location.pathname;
  const page = path.split("/").pop() || "index.html";

  nav.querySelectorAll(".navbar a").forEach((a) => {
    const href = a.getAttribute("href") || "";
    if (href === page || (page === "" && href === "index.html")) {
      a.classList.add("active");
    } else {
      a.classList.remove("active");
    }
  });
}

/** Init eseguito al caricamento */
function init() {
  initNavbarActive();
  // Garantisce WS pronto anche se ws_dashboard.js non è ancora inizializzato.
  connectJ5Dashboard();
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init);
} else {
  init();
}

document.addEventListener("navbar-loaded", initNavbarActive);

/**************************************************
 * HOME PAGE INIT (EVENTI NAVBAR + BOTTONI)
 **************************************************/
if (document.location.pathname.endsWith("index.html") || document.location.pathname.endsWith("/") || document.location.pathname.match(/\/dashboard\/?$/)) {
  const runWhenReady = () => {
    let demoActive = false;
    let demoPending = false;
    const btnDemo = document.getElementById("btn-demo");
    const setDemoBtn = (active, pending = false) => {
      if (!btnDemo) return;
      demoActive = active;
      demoPending = pending;
      if (pending) {
        btnDemo.textContent = active ? "Demo: arresto..." : "Demo: avvio...";
      } else {
        btnDemo.textContent = active ? "Ferma Demo" : "Avvia Demo";
      }
      btnDemo.classList.toggle("active", active);
      btnDemo.disabled = pending;
    };
    setDemoBtn(false, false);

    const btnEnable = document.getElementById("btn-enable");
    const btnStop = document.getElementById("btn-stop");
    if (btnEnable) btnEnable.addEventListener("click", () => {
      // Sequenza industrial-grade: da STOPPED serve SAFE poi ENABLE. Da SAFE/IDLE SAFE è idempotente.
      sendCommand("uart", { cmd: "SAFE" });
      sendCommand("uart", { cmd: "ENABLE" });
      addLog("SAFE + ENABLE inviati");
    });
    if (btnStop) btnStop.addEventListener("click", () => {
      sendCommand("uart", { cmd: "STOP" });
      addLog("STOP inviato");
    });
    const btnImuToggle = document.getElementById("btn-imu-toggle");
    if (btnImuToggle) {
      let imuActive = false;
      const updateImuBtn = (active) => {
        imuActive = active;
        btnImuToggle.textContent = active ? "IMU ON" : "IMU OFF";
        btnImuToggle.classList.toggle("active", active);
      };
      updateImuBtn(true); /* IMU ON di default al boot */
      btnImuToggle.addEventListener("click", () => {
        const next = !imuActive;
        sendCommand("set_imu", { enabled: next });
        updateImuBtn(next);
        addLog(next ? "IMU ON inviato" : "IMU OFF inviato");
      });
      /* Sincronizza con l'ack del server (imu_enabled) dopo set_imu.
         NON usare imu_valid (stato hardware IMU) che è indipendente dall'abilitazione logica. */
      window._syncImuToggle = (enabled) => {
        if (enabled !== imuActive) updateImuBtn(enabled);
      };
    }

    const btnVrPose = document.getElementById("btn-vrpose");
    const btnHome   = document.getElementById("btn-home");
    const btnPark   = document.getElementById("btn-park");
    const btnSelfTest = document.getElementById("btn-self-test");

    if (btnVrPose) btnVrPose.addEventListener("click", () => {
      sendCommand("uart", { cmd: "TELEOPPOSE" });
      addLog("VR Pose inviato");
    });
    if (btnHome) btnHome.addEventListener("click", () => {
      if (typeof window.j5NotifyHomeRequested === "function") window.j5NotifyHomeRequested();
      sendCommand("uart", { cmd: "HOME" });
      addLog("HOME inviato");
    });
    if (btnPark) btnPark.addEventListener("click", () => {
      sendCommand("uart", { cmd: "PARK" });
      addLog("PARK inviato");
    });
    if (btnSelfTest) btnSelfTest.addEventListener("click", () => {
      if (btnSelfTest.disabled) return;
      const sent = sendCommand("self_test", { action: "run" });
      if (!sent) {
        addLog("SELF TEST non inviato (WS non pronto)");
        return;
      }
      btnSelfTest.disabled = true;
      addLog("SELF TEST avviato");
    });
    if (btnDemo) btnDemo.addEventListener("click", () => {
      if (demoPending) return;
      const sent = sendCommand("uart", { cmd: "DEMO" });
      if (!sent) {
        addLog("DEMO non inviata (WS non pronto)");
        return;
      }
      setDemoBtn(demoActive, true);
      addLog(demoActive ? "Richiesta stop DEMO inviata" : "Richiesta avvio DEMO inviata");
    });

    registerUartResponseHandler((msg) => {
      const cmd = String(msg?.cmd || "").toUpperCase();
      if (cmd === "HOME" && msg.ok === false && typeof window.j5CancelHomeImuCapture === "function") {
        window.j5CancelHomeImuCapture();
      }
      if (cmd === "STOP" && msg.ok) {
        setDemoBtn(false, false);
        return;
      }
      if (cmd !== "DEMO") return;
      if (msg.ok && String(msg.response || "").includes("STARTED")) {
        setDemoBtn(true, false);
        addLog("DEMO avviata");
        return;
      }
      if (msg.ok && String(msg.response || "").includes("STOPPED")) {
        setDemoBtn(false, false);
        addLog("DEMO fermata senza STOP");
        return;
      }
      setDemoBtn(demoActive, false);
      addLog(`DEMO fallita: ${msg.response || "errore sconosciuto"}`);
    });

    registerSelfTestStatusHandler((msg) => {
      if (!btnSelfTest) return;
      btnSelfTest.disabled = Boolean(msg?.running);
      if (msg?.message) addLog(`SELF TEST: ${msg.message}`);
    });

    registerSelfTestResultHandler((msg) => {
      if (btnSelfTest) btnSelfTest.disabled = false;
      addLog(`SELF TEST RESULT: ${msg?.result || "UNKNOWN"}`);
    });
  };
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", runWhenReady);
  else runWhenReady();
}

// LEGACY/UTILITY: helper addLog ora centralizzato in j5_common.js.
// Il fallback locale precedente è stato rimosso in RPI-3 per ridurre duplicazione.
