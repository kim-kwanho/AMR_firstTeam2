'use strict';

/* ---- map calibration: pgm(1206_sim_1) <-> topview(1206_top.png) ----
 * map.yaml: resolution 0.05 m/px, origin [-5, -3.8], pgm 200x150 px
 * topview image: 858x645 px (same area, scaled)
 */
let MAP = {
  res: 0.05,
  ox: -5.0,
  oy: -3.8,
  pgmW: 200,
  pgmH: 150,
  imgW: 858,
  imgH: 645,
};

const floorImg = document.getElementById('floor');
const canvas = document.getElementById('overlay');
const ctx = canvas.getContext('2d');
const poseWaiting = document.getElementById('pose-waiting');
const connBadge = document.getElementById('conn-badge');
const navBadge = document.getElementById('nav-badge');
const wanderBadge = document.getElementById('wander-badge');
const cameraWrap = document.getElementById('camera-wrap');
const peopleBadge = document.getElementById('people-badge');
const hideBadge = document.getElementById('hide-badge');
const eventLog = document.getElementById('event-log');
const chatLog = document.getElementById('chat-log');
const chatForm = document.getElementById('chat-form');
const chatInput = document.getElementById('chat-input');
const chatSend = document.getElementById('chat-send');

let state = { pose: null, goal: null, navigating: false };

async function loadMap() {
  try {
    const res = await fetch('/api/map-info');
    if (!res.ok) throw new Error(`map-info ${res.status}`);
    const info = await res.json();
    MAP = {
      res: info.resolution,
      ox: info.origin[0],
      oy: info.origin[1],
      pgmW: info.width,
      pgmH: info.height,
      imgW: info.width,
      imgH: info.height,
    };
    floorImg.src = `/api/map-image?t=${Date.now()}`;
  } catch (err) {
    console.warn('map metadata load failed, using bundled calibration', err);
  }
}

/* ---------- coordinate transform ---------- */
function worldToCanvas(x, y) {
  const px = (x - MAP.ox) / MAP.res * (MAP.imgW / MAP.pgmW);
  const py = MAP.imgH - (y - MAP.oy) / MAP.res * (MAP.imgH / MAP.pgmH);
  const sx = canvas.width / MAP.imgW;
  const sy = canvas.height / MAP.imgH;
  return [px * sx, py * sy];
}

/* ---------- drawing ---------- */
function syncCanvasSize() {
  const w = floorImg.clientWidth;
  const h = floorImg.clientHeight;
  if (w === 0 || h === 0) return;
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr;
    canvas.height = h * dpr;
  }
  draw();
}

function drawGoal(x, y) {
  const [cx, cy] = worldToCanvas(x, y);
  const s = canvas.width / 858;
  ctx.save();
  ctx.translate(cx, cy);
  // pin: pole + flag
  ctx.strokeStyle = '#b85c2e';
  ctx.lineWidth = 3 * s;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(0, -26 * s);
  ctx.stroke();
  ctx.fillStyle = '#d97742';
  ctx.beginPath();
  ctx.moveTo(0, -26 * s);
  ctx.lineTo(18 * s, -20 * s);
  ctx.lineTo(0, -14 * s);
  ctx.closePath();
  ctx.fill();
  ctx.fillStyle = 'rgba(184, 92, 46, 0.35)';
  ctx.beginPath();
  ctx.ellipse(0, 0, 7 * s, 4 * s, 0, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawRobot(x, y, yaw) {
  const [cx, cy] = worldToCanvas(x, y);
  const s = canvas.width / 858;
  const r = 13 * s;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-yaw); // canvas y axis points down
  // halo
  ctx.fillStyle = 'rgba(78, 154, 81, 0.25)';
  ctx.beginPath();
  ctx.arc(0, 0, r * 1.7, 0, Math.PI * 2);
  ctx.fill();
  // body
  ctx.fillStyle = '#4e9a51';
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2.5 * s;
  ctx.beginPath();
  ctx.arc(0, 0, r, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  // heading arrow
  ctx.fillStyle = '#fff';
  ctx.beginPath();
  ctx.moveTo(r * 0.85, 0);
  ctx.lineTo(-r * 0.35, -r * 0.5);
  ctx.lineTo(-r * 0.35, r * 0.5);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (state.goal) drawGoal(state.goal.x, state.goal.y);
  if (state.pose) drawRobot(state.pose.x, state.pose.y, state.pose.yaw);
  poseWaiting.classList.toggle('hidden', !!state.pose);
}

/* ---------- badges ---------- */
function setConn(ok) {
  connBadge.className = 'badge ' + (ok ? 'on' : 'off');
  connBadge.innerHTML = '<span class="dot"></span>' + (ok ? '서버 연결됨' : '서버 연결 끊김');
}

function setNav(navigating) {
  navBadge.className = 'badge ' + (navigating ? 'moving' : 'idle');
  navBadge.innerHTML = '<span class="dot"></span>' + (navigating ? '이동 중' : '대기 중');
}

function setPeople(count) {
  if (count === null || count === undefined) {
    peopleBadge.className = 'badge idle';
    peopleBadge.textContent = '👤 감지 꺼짐';
  } else if (count > 0) {
    peopleBadge.className = 'badge alert';
    peopleBadge.textContent = `👤 사람 ${count}명 감지`;
  } else {
    peopleBadge.className = 'badge idle';
    peopleBadge.textContent = '👤 사람 없음';
  }
}

function setWander(enabled) {
  wanderBadge.className = 'badge ' + (enabled ? 'on' : 'idle');
  wanderBadge.innerHTML = '<span class="dot"></span>' + (enabled ? '랜덤 이동 중' : '랜덤 꺼짐');
}

function setHideState(hideState) {
  if (!hideBadge) return;
  const isDocent = (hideState === 'WAKE' || hideState === 'GUIDE');
  
  if (hideState === 'FREEZE') {
    hideBadge.className = 'badge idle';
    hideBadge.innerHTML = '<span class="dot"></span>위장 대기 중';
  } else if (isDocent) {
    hideBadge.className = 'badge alert';
    hideBadge.innerHTML = '<span class="dot"></span>도슨트 활성';
  } else if (hideState === 'RETURN' || hideState === 'DOCK') {
    hideBadge.className = 'badge moving';
    hideBadge.innerHTML = '<span class="dot"></span>복귀 대기 중';
  } else {
    hideBadge.className = 'badge idle';
    hideBadge.innerHTML = '<span class="dot"></span>' + hideState;
  }
  
  document.body.classList.toggle('docent-mode', isDocent);
  const frame = document.querySelector('.frame');
  if (frame) frame.classList.toggle('docent-mode', isDocent);
}

/* ---------- event log ---------- */
function fmtTime(epoch) {
  const d = new Date(epoch * 1000);
  return d.toTimeString().slice(0, 8);
}

function appendEvents(events) {
  const empty = eventLog.querySelector('.event.empty');
  if (empty) empty.remove();
  for (const ev of events) {
    const row = document.createElement('div');
    row.className = 'event';
    const time = document.createElement('span');
    time.className = 'event-time';
    time.textContent = fmtTime(ev.t);
    const text = document.createElement('span');
    text.className = 'event-text';
    text.textContent = ev.text;
    row.append(time, text);
    eventLog.prepend(row); // newest on top
  }
  while (eventLog.children.length > 100) eventLog.lastChild.remove();
}

/* ---------- SSE stream ---------- */
function connectStream() {
  const es = new EventSource('/api/stream');
  es.onopen = () => setConn(true);
  es.onerror = () => setConn(false); // EventSource auto-reconnects
  es.onmessage = (ev) => {
    try {
      state = JSON.parse(ev.data);
    } catch (e) {
      return;
    }
    setNav(state.navigating);
    setWander(state.wander);
    setPeople(state.people);
    setHideState(state.hide_state);
    cameraWrap.classList.toggle('no-signal', !state.camera);
    if (state.events && state.events.length) appendEvents(state.events);
    draw();
  };
}

/* ---------- chat ---------- */
function appendMsg(role, text) {
  const msg = document.createElement('div');
  msg.className = 'msg ' + role;
  const bubble = document.createElement('div');
  bubble.className = 'bubble';
  bubble.textContent = text;
  msg.appendChild(bubble);
  chatLog.appendChild(msg);
  chatLog.scrollTop = chatLog.scrollHeight;
  return msg;
}

chatForm.addEventListener('submit', async (ev) => {
  ev.preventDefault();
  const question = chatInput.value.trim();
  if (!question) return;

  appendMsg('user', question);
  chatInput.value = '';
  chatInput.disabled = true;
  chatSend.disabled = true;
  const typing = appendMsg('bot typing', '응답을 기다리는 중');

  try {
    const res = await fetch('/api/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ question }),
    });
    const data = await res.json();
    typing.querySelector('.bubble').textContent = data.answer;
  } catch (e) {
    typing.querySelector('.bubble').textContent =
      '서버와 통신할 수 없습니다. 대시보드 노드가 실행 중인지 확인해 주세요.';
  } finally {
    typing.classList.remove('typing');
    chatInput.disabled = false;
    chatSend.disabled = false;
    chatInput.focus();
    chatLog.scrollTop = chatLog.scrollHeight;
  }
});

/* ---------- init ---------- */
new ResizeObserver(syncCanvasSize).observe(floorImg);
floorImg.addEventListener('load', syncCanvasSize);
loadMap();
syncCanvasSize();
connectStream();

const serviceDoneBtn = document.getElementById('btn-service-done');
if (serviceDoneBtn) {
  serviceDoneBtn.addEventListener('click', async () => {
    appendMsg('user', '서비스 완료 (메뉴판 수령 완료)');
    const typing = appendMsg('bot typing', '응답을 기다리는 중');
    try {
      const res = await fetch('/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ question: '서비스 완료' }),
      });
      const data = await res.json();
      typing.querySelector('.bubble').textContent = data.answer;
    } catch (e) {
      typing.querySelector('.bubble').textContent = '서버와 통신할 수 없습니다.';
    } finally {
      typing.classList.remove('typing');
    }
  });
}
