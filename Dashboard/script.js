/* CONFIG */
var MQTT_URL = "wss://broker.emqx.io:8084/mqtt";
var TOPIC_JSON = "smartank/estado";
var TOPIC_CONTROL = "smartank/control";

/* RANGO FISICO DEL TANQUE */
const MAX_ALTURA = 12.0; // cm (0..12)

/* Estado local */
var state = {
  distancia: null,
  altura: null,
  porcentaje: null,
  ph: null,
  tds: null,
  bomba_llenado: null,
  bomba_drenaje: null,
  modo_manual: null,
  led_estado: null
};
var mqttClient = null, mqttConnected = false;

/* UTILIDADES */
function fmt(n, d) {
  if (n == null || Number.isNaN(n)) return "--";
  return Number(n).toFixed(d);
}
function clamp(n, min, max) {
  return Math.max(min, Math.min(max, n));
}
function setFill(el, p) {
  if (!el) return;
  el.style.transform = "scaleX(" + clamp(p, 0, 1) + ")";
}
function setRing(deg) {
  document.getElementById("ring").style.background =
    "conic-gradient(var(--accent) 0deg, var(--accent) " + deg + "deg, #041320 " + deg + "deg)";
}
function setGauge(deg) {
  document.getElementById("tdsGauge").style.background =
    "conic-gradient(var(--accent) 0deg, var(--accent) " + deg + "deg, #041320 " + deg + "deg)";
}
function phToState(v) {
  if (v == null) return ["--", "--"];
  if (v < 6.5) return ["Ácido", "bad"];
  if (v <= 7.5) return ["Neutro", "ok"];
  return ["Básico", "warn"];
}
function boolText(v) { if (v == null) return "--"; return v ? "Encendida" : "Apagada"; }
function boolMode(v) { if (v == null) return "--"; return v ? "Manual" : "Automático"; }

/* Calidad TDS (mismos rangos que tu código) */
function tdsToQuality(tds) {
  if (tds == null || Number.isNaN(tds)) return { label: '--', desc: 'Sin medición', color: '#888', class: '' };
  var v = Number(tds);
  if (v <= 150) return { label: 'Muy bueno', desc: 'TDS 0–150 ppm', color: 'var(--ok)', class: 'ok' };
  if (v <= 200) return { label: 'Bueno', desc: 'TDS 151–200 ppm', color: 'var(--accent)', class: 'ok' };
  if (v <= 249) return { label: 'Regular', desc: 'TDS 201–249 ppm', color: 'var(--warn)', class: 'warn' };
  if (v <= 299) return { label: 'Aceptable', desc: 'TDS 250–299 ppm', color: 'var(--warn)', class: 'warn' };
  if (v <= 399) return { label: 'Malo', desc: 'TDS 300–399 ppm (drenaje recomendado en automático)', color: 'var(--bad)', class: 'bad' };
  return { label: 'Muy malo', desc: 'TDS ≥ 400 ppm (drenaje urgente)', color: 'var(--bad)', class: 'bad' };
}

/* Actualiza la UI con un objeto parcial s */
function updateUI(s) {
  // DISTANCIA: espacio libre (cm)
  if (typeof s.distancia === "number") {
    state.distancia = s.distancia;
    document.getElementById("distancia").textContent = fmt(state.distancia, 1);
    // espacio libre en cm = MAX_ALTURA - distancia
    var espacio = MAX_ALTURA - clamp(state.distancia, 0, MAX_ALTURA);
    setFill(document.getElementById("distBar"), espacio / MAX_ALTURA);
  }

  // ALTURA del agua (cm)
  if (typeof s.altura === "number") {
    state.altura = s.altura;
    document.getElementById("altura").textContent = fmt(state.altura, 1);
    setFill(document.getElementById("altBar"), clamp(state.altura, 0, MAX_ALTURA) / MAX_ALTURA);
  }

  // PORCENTAJE (el que viene publicado para la UI — ya está ajustado por el ESP)
  if (typeof s.porcentaje === "number") {
    state.porcentaje = s.porcentaje;
    document.getElementById("porcentaje").textContent = fmt(state.porcentaje, 0);
    setRing(clamp(state.porcentaje, 0, 100) * 3.6);
  }

  // Si el JSON trae porcentaje_medido (opcional) lo podemos usar para debug — aquí se ignora para visual
  if (s.hasOwnProperty("porcentaje_medido")) {
    // no hace falta asignarlo al UI principal porque usamos "porcentaje" para mostrar,
    // pero lo guardamos por si quieres mostrarlo más adelante.
    state.porcentaje_medido = Number(s.porcentaje_medido);
  }

  // PH
  if (typeof s.ph === "number") {
    state.ph = s.ph;
    document.getElementById("ph").textContent = fmt(state.ph, 2);
    var st = phToState(state.ph);
    var c = document.getElementById("phState");
    c.classList.remove("ok", "warn", "bad");
    if (st[1] !== "--") c.classList.add(st[1]);
    document.getElementById("phLabel").textContent = st[0];
  }

  // TDS
  if (typeof s.tds === "number") {
    state.tds = s.tds;
    document.getElementById("tds").textContent = fmt(state.tds, 0);
    var max = 2000;
    var deg = clamp(state.tds / max, 0, 1) * 360;
    setGauge(deg);
    var q = tdsToQuality(state.tds);
    document.getElementById('qualityLabel').textContent = q.label;
    document.getElementById('qualityDesc').textContent = q.desc;
    var qdot = document.getElementById('qualityDot'); qdot.style.background = q.color;
    var drenState = document.getElementById('drenajeState'); drenState.classList.remove('ok', 'warn', 'bad'); if (q.class) drenState.classList.add(q.class);
  }

  // Bomba llenado
  if (s.hasOwnProperty("bomba_llenado")) {
    if (typeof s.bomba_llenado === "boolean") state.bomba_llenado = s.bomba_llenado;
    else state.bomba_llenado = String(s.bomba_llenado).toLowerCase() === "true";
    document.getElementById("llenadoText").textContent = boolText(state.bomba_llenado);
    var st2 = document.getElementById("llenadoState");
    st2.classList.remove("ok", "bad");
    st2.classList.add(state.bomba_llenado ? "ok" : "bad");
    document.getElementById("llenadoSwitch").checked = !!state.bomba_llenado;
  }

  // Bomba drenaje
  if (s.hasOwnProperty("bomba_drenaje")) {
    if (typeof s.bomba_drenaje === "boolean") state.bomba_drenaje = s.bomba_drenaje;
    else state.bomba_drenaje = String(s.bomba_drenaje).toLowerCase() === "true";
    document.getElementById('drenajeText').textContent = boolText(state.bomba_drenaje);
    var ds = document.getElementById('drenajeState'); ds.classList.remove('ok', 'warn', 'bad'); ds.classList.add(state.bomba_drenaje ? 'ok' : 'bad');
    var drenSwitch = document.getElementById('drenajeSwitch'); if (drenSwitch) { drenSwitch.checked = !!state.bomba_drenaje; drenSwitch.disabled = !(state.modo_manual && mqttConnected); }
  }

  // Modo manual/automático
  if (typeof s.modo_manual === "boolean") {
    state.modo_manual = s.modo_manual;
    document.getElementById("modoText").textContent = boolMode(state.modo_manual);
    document.getElementById("modoSwitch").checked = !!state.modo_manual;
    document.getElementById("llenadoSwitch").disabled = !(state.modo_manual && mqttConnected);
    var drenSwitch = document.getElementById('drenajeSwitch'); if (drenSwitch) drenSwitch.disabled = !(state.modo_manual && mqttConnected);
  }

  // Si no hay porcentaje publicado pero sí altura, calcular porcentaje desde altura respecto a MAX_ALTURA
  if ((state.porcentaje == null || Number.isNaN(state.porcentaje)) && (state.altura != null && !Number.isNaN(state.altura))) {
    var pct = clamp((state.altura / MAX_ALTURA) * 100, 0, 100);
    updateUI({ porcentaje: pct });
  }
}

/* EVENTOS DE UI: switches -> publican al TOPIC_CONTROL */
document.getElementById("modoSwitch").addEventListener("change", function () {
  if (!mqttClient || !mqttConnected) { this.checked = !this.checked; return; }
  const modoManual = this.checked ? "MODE:ON" : "MODE:OFF";
  mqttClient.publish(TOPIC_CONTROL, modoManual);
});

document.getElementById("llenadoSwitch").addEventListener("change", function () {
  if (!mqttClient || !mqttConnected) { this.checked = !this.checked; return; }
  if (!state.modo_manual) mqttClient.publish(TOPIC_CONTROL, "MODE:ON");
  const cmd = this.checked ? "PUMP_ON" : "PUMP_OFF";
  mqttClient.publish(TOPIC_CONTROL, cmd);
});

document.getElementById("drenajeSwitch").addEventListener("change", function () {
  if (!mqttClient || !mqttConnected) { this.checked = !this.checked; return; }
  if (!state.modo_manual) mqttClient.publish(TOPIC_CONTROL, "MODE:ON");
  const cmd = this.checked ? "DRENAR_ON" : "DRENAR_OFF";
  mqttClient.publish(TOPIC_CONTROL, cmd);
});

/* MQTT connect / subscribe */
function connectMQTT() {
  if (mqttConnected) {
    try { mqttClient.end(true); } catch (e) { }
    mqttClient = null; mqttConnected = false;
    document.getElementById("btnMqtt").textContent = "Conectar";
    document.getElementById("llenadoSwitch").disabled = true;
    document.getElementById("drenajeSwitch").disabled = true;
    return;
  }

  var user = document.getElementById("mqttUser").value || undefined;
  var pass = document.getElementById("mqttPass").value || undefined;
  var clientId = "smartank-web-" + Math.random().toString(16).slice(2);
  mqttClient = mqtt.connect(MQTT_URL, { clientId: clientId, username: user, password: pass, clean: true, connectTimeout: 6000, reconnectPeriod: 5000 });

  mqttClient.on("connect", function () {
    mqttConnected = true;
    document.getElementById("btnMqtt").textContent = "Desconectar";
    if (TOPIC_JSON) mqttClient.subscribe(TOPIC_JSON);
    if (TOPIC_CONTROL) mqttClient.subscribe(TOPIC_CONTROL);
    document.getElementById("llenadoSwitch").disabled = !(state.modo_manual && mqttConnected);
    document.getElementById("drenajeSwitch").disabled = !(state.modo_manual && mqttConnected);
  });

  mqttClient.on("message", function (topic, payload) {
    try {
      var txt = new TextDecoder().decode(payload);
      if (topic === TOPIC_JSON) {
        try {
          var obj = JSON.parse(txt);
          if (obj.hasOwnProperty("porcentaje")) obj.porcentaje = Number(obj.porcentaje);
          if (obj.hasOwnProperty("porcentaje_medido")) obj.porcentaje_medido = Number(obj.porcentaje_medido);
          if (obj.hasOwnProperty("distancia")) obj.distancia = Number(obj.distancia);
          if (obj.hasOwnProperty("altura")) obj.altura = Number(obj.altura);
          if (obj.hasOwnProperty("ph")) obj.ph = Number(obj.ph);
          if (obj.hasOwnProperty("tds")) obj.tds = Number(obj.tds);
          if (obj.hasOwnProperty("bomba_llenado")) obj.bomba_llenado = (obj.bomba_llenado === true || String(obj.bomba_llenado).toLowerCase() === "true");
          if (obj.hasOwnProperty("bomba_drenaje")) obj.bomba_drenaje = (obj.bomba_drenaje === true || String(obj.bomba_drenaje).toLowerCase() === "true");
          if (obj.hasOwnProperty("modo_manual")) obj.modo_manual = (obj.modo_manual === true || String(obj.modo_manual).toLowerCase() === "true");
          updateUI(obj);
        } catch (e) { console.warn("JSON inválido:", txt); }
      }
    } catch (e) { }
  });

  mqttClient.on("close", function () {
    mqttConnected = false;
    document.getElementById("btnMqtt").textContent = "Conectar";
    document.getElementById("llenadoSwitch").disabled = true;
    document.getElementById("drenajeSwitch").disabled = true;
  });

  mqttClient.on("error", function (err) { console.error("MQTT error:", err); });
}

document.getElementById("btnMqtt").addEventListener("click", connectMQTT);

/* Inicializar UI vacía */
updateUI({});