/**
 * 舵机控制面板 - 纯浏览器端逻辑
 * 使用 Web Serial API 直接在浏览器中访问串口
 * 包含完整的 LX 直连协议 + 控制器板协议实现
 */

// ============================================================
// 协议常量
// ============================================================
const CMD_SERVO_MOVE = 3;
const CMD_ACTION_GROUP_RUN = 6;
const CMD_ACTION_GROUP_STOP = 7;
const CMD_ACTION_GROUP_SPEED = 11;
const CMD_GET_BATTERY_VOLTAGE = 15;
const CMD_MULT_SERVO_UNLOAD = 20;
const CMD_MULT_SERVO_POS_READ = 21;

const LX_CMD_SERVO_MOVE_TIME_WRITE = 1;
const LX_CMD_ID_WRITE = 13;
const LX_CMD_ID_READ = 14;
const LX_CMD_POS_READ = 28;
const LX_CMD_VIN_READ = 27;
const LX_CMD_TEMP_READ = 26;
const LX_BROADCAST_ID = 254;

// ============================================================
// 全局状态
// ============================================================
const APP = {
    port: null,              // Web Serial API port
    reader: null,
    writer: null,
    readableStreamClosed: null,
    writableStreamClosed: null,
    connected: false,
    baudRate: 115200,
    servos: {},              // {id: {position, voltage, temperature}}
    charts: {},              // {id: Chart instance}
    chartData: {},           // {id: {pos:[], vin:[], temp:[], labels:[]}}
    maxDataPoints: 60,
    pollingTimer: null,
    serialLock: Promise.resolve(),  // 串口互斥锁
    rxBuffer: new Uint8Array(0),    // 接收缓冲区
};

// ============================================================
// Toast 提示
// ============================================================
function showToast(msg, type = 'info') {
    const container = document.getElementById('toast-container');
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.textContent = msg;
    container.appendChild(toast);
    setTimeout(() => toast.remove(), 3500);
}

function formatTime(ts) {
    const d = new Date(ts * 1000);
    return d.toLocaleTimeString('zh-CN', { hour12: false });
}

// ============================================================
// 串口互斥锁 — 确保命令按顺序执行
// ============================================================
function withSerialLock(fn) {
    const prev = APP.serialLock;
    let resolve;
    APP.serialLock = new Promise(r => resolve = r);
    return prev.then(() => fn().finally(resolve));
}

// ============================================================
// Web Serial API: 连接 / 断开
// ============================================================
async function connectSerial() {
    if (!('serial' in navigator)) {
        showToast('你的浏览器不支持 Web Serial API，请使用 Chrome / Edge', 'error');
        return;
    }

    const baudRate = parseInt(document.getElementById('baud-select').value);

    try {
        // 弹出浏览器原生的串口选择对话框
        const port = await navigator.serial.requestPort();
        await port.open({ baudRate });

        APP.port = port;
        APP.baudRate = baudRate;
        APP.connected = true;

        // 开启持续读 — 后台收集数据到 rxBuffer
        startReading();

        updateUI();
        showToast(`已连接 (${baudRate} baud)`, 'success');
    } catch (e) {
        if (e.name === 'NotFoundError') {
            showToast('已取消选择', 'info');
        } else {
            showToast(`连接失败: ${e.message}`, 'error');
        }
    }
}

async function disconnectSerial() {
    stopPolling();
    APP.connected = false;

    try {
        if (APP.readerController) {
            APP.readerController.cancel();
            APP.readerController = null;
        }
        // 等待 reader 关闭
        if (APP.readingPromise) {
            await APP.readingPromise.catch(() => { });
        }
        if (APP.port) {
            await APP.port.close();
        }
    } catch (e) {
        console.warn('disconnect error:', e);
    }

    APP.port = null;
    APP.servos = {};
    APP.charts = {};
    APP.chartData = {};
    APP.rxBuffer = new Uint8Array(0);
    clearServoCards();
    updateUI();
    showToast('已断开连接', 'info');
}

// ============================================================
// 串口读写
// ============================================================
function startReading() {
    const reader = APP.port.readable.getReader();
    APP.readerController = reader;

    APP.readingPromise = (async () => {
        try {
            while (true) {
                const { value, done } = await reader.read();
                if (done) break;
                if (value) {
                    // 追加到 rxBuffer
                    const merged = new Uint8Array(APP.rxBuffer.length + value.length);
                    merged.set(APP.rxBuffer);
                    merged.set(value, APP.rxBuffer.length);
                    APP.rxBuffer = merged;
                }
            }
        } catch (e) {
            if (e.name !== 'NetworkError' && e.message !== 'The device has been lost.') {
                console.warn('read error:', e);
            }
        } finally {
            reader.releaseLock();
        }
    })();
}

async function serialWrite(data) {
    if (!APP.port || !APP.port.writable) throw new Error('未连接');
    const writer = APP.port.writable.getWriter();
    try {
        await writer.write(data instanceof Uint8Array ? data : new Uint8Array(data));
    } finally {
        writer.releaseLock();
    }
}

function clearRxBuffer() {
    APP.rxBuffer = new Uint8Array(0);
}

function waitForResponse(timeoutMs = 150) {
    return new Promise((resolve) => {
        const start = performance.now();

        function check() {
            // 在 rxBuffer 中查找 0x55 0x55
            const buf = APP.rxBuffer;
            for (let i = 0; i < buf.length - 1; i++) {
                if (buf[i] === 0x55 && buf[i + 1] === 0x55) {
                    if (i + 4 > buf.length) break; // 还没收完
                    const pktLen = buf[i + 3];
                    const totalLen = 3 + pktLen;
                    if (i + totalLen > buf.length) break; // 还没收完
                    const packet = buf.slice(i, i + totalLen);
                    // 从 buffer 中移除已消费的数据
                    APP.rxBuffer = buf.slice(i + totalLen);
                    return resolve(packet);
                }
            }

            if (performance.now() - start < timeoutMs) {
                setTimeout(check, 10);
            } else {
                resolve(null);
            }
        }

        check();
    });
}

// ============================================================
// LX 协议
// ============================================================
function lxChecksum(id, len, cmd, params = []) {
    let s = id + len + cmd;
    for (const b of params) s += b;
    return (~s) & 0xFF;
}

function lxBuildCmd(servoId, cmd, params = []) {
    const len = params.length + 3;
    const cs = lxChecksum(servoId, len, cmd, params);
    return new Uint8Array([0x55, 0x55, servoId, len, cmd, ...params, cs]);
}

async function lxReadId(targetId = LX_BROADCAST_ID) {
    return withSerialLock(async () => {
        try {
            clearRxBuffer();
            await serialWrite(lxBuildCmd(targetId, LX_CMD_ID_READ));
            const resp = await waitForResponse(200);
            if (resp && resp.length >= 7) return resp[5];
            return null;
        } catch { return null; }
    });
}

async function lxWriteId(oldId, newId) {
    return withSerialLock(async () => {
        try {
            clearRxBuffer();
            newId = Math.max(0, Math.min(253, newId));
            await serialWrite(lxBuildCmd(oldId, LX_CMD_ID_WRITE, [newId]));
            await new Promise(r => setTimeout(r, 100));
            return true;
        } catch { return false; }
    });
}

async function lxReadPosition(servoId) {
    return withSerialLock(async () => {
        try {
            clearRxBuffer();
            await serialWrite(lxBuildCmd(servoId, LX_CMD_POS_READ));
            const resp = await waitForResponse(200);
            if (resp && resp.length >= 8) {
                let pos = resp[5] | (resp[6] << 8);
                if (pos > 32767) pos -= 65536;
                return pos;
            }
            return null;
        } catch { return null; }
    });
}

async function lxReadVin(servoId) {
    return withSerialLock(async () => {
        try {
            clearRxBuffer();
            await serialWrite(lxBuildCmd(servoId, LX_CMD_VIN_READ));
            const resp = await waitForResponse(200);
            if (resp && resp.length >= 8) {
                let vin = resp[5] | (resp[6] << 8);
                if (vin > 32767) vin -= 65536;
                return vin;
            }
            return null;
        } catch { return null; }
    });
}

async function lxReadTemp(servoId) {
    return withSerialLock(async () => {
        try {
            clearRxBuffer();
            await serialWrite(lxBuildCmd(servoId, LX_CMD_TEMP_READ));
            const resp = await waitForResponse(200);
            if (resp && resp.length >= 7) return resp[5];
            return null;
        } catch { return null; }
    });
}

async function lxMoveServo(servoId, position, moveTime = 1000) {
    // LX 直连协议 SERVO_MOVE_TIME_WRITE (与 Python lx_move_servo 完全一致)
    // 参数顺序: position_L, position_H, time_L, time_H  (位置在前,时间在后!)
    return withSerialLock(async () => {
        try {
            position = Math.max(0, Math.min(1000, position));
            moveTime = Math.max(0, Math.min(30000, moveTime));

            const params = [
                position & 0xFF, (position >> 8) & 0xFF,    // 位置 低/高 (先!)
                moveTime & 0xFF, (moveTime >> 8) & 0xFF,    // 时间 低/高 (后!)
            ];
            const cmd = lxBuildCmd(servoId, LX_CMD_SERVO_MOVE_TIME_WRITE, params);

            console.log(`[MOVE] servo=${servoId} pos=${position} time=${moveTime} bytes=[${Array.from(cmd).map(b => '0x' + b.toString(16).padStart(2, '0')).join(',')}]`);
            await serialWrite(cmd);
            return true;
        } catch (e) { console.error('[MOVE] error:', e); return false; }
    });
}

// ============================================================
// 控制器板协议
// ============================================================
async function ctrlUnloadServos(servoIds) {
    return withSerialLock(async () => {
        const count = servoIds.length;
        if (count === 0) return;
        const buf = new Uint8Array([0x55, 0x55, count + 3, CMD_MULT_SERVO_UNLOAD, count, ...servoIds]);
        await serialWrite(buf);
    });
}

async function ctrlRunActionGroup(groupId, times = 1) {
    return withSerialLock(async () => {
        await serialWrite(new Uint8Array([
            0x55, 0x55, 0x05, CMD_ACTION_GROUP_RUN,
            groupId & 0xFF, times & 0xFF, (times >> 8) & 0xFF
        ]));
    });
}

async function ctrlStopActionGroup() {
    return withSerialLock(async () => {
        await serialWrite(new Uint8Array([0x55, 0x55, 0x02, CMD_ACTION_GROUP_STOP]));
    });
}

async function ctrlSetActionGroupSpeed(groupId, speedPercent) {
    return withSerialLock(async () => {
        await serialWrite(new Uint8Array([
            0x55, 0x55, 0x05, CMD_ACTION_GROUP_SPEED,
            groupId & 0xFF, speedPercent & 0xFF, (speedPercent >> 8) & 0xFF
        ]));
    });
}

// ============================================================
// 舵机扫描
// ============================================================
async function scanServos() {
    if (!APP.connected) { showToast('请先连接串口', 'error'); return; }

    document.getElementById('btn-scan').disabled = true;
    document.getElementById('btn-scan').textContent = '⏳ 扫描中...';
    showToast('正在扫描舵机...', 'info');

    stopPolling();
    APP.servos = {};
    clearServoCards();
    await new Promise(r => setTimeout(r, 200));

    const found = [];

    // 广播读 ID
    const broadcastId = await lxReadId(LX_BROADCAST_ID);

    // 逐个扫描 1~20
    for (let sid = 1; sid <= 20; sid++) {
        const pos = await lxReadPosition(sid);
        if (pos !== null) {
            const vin = await lxReadVin(sid);
            const temp = await lxReadTemp(sid);
            APP.servos[sid] = { position: pos, voltage: vin, temperature: temp };
            found.push({ id: sid, position: pos, voltage: vin, temperature: temp });
        }
    }

    // 广播到的 ID 如果不在 1~20
    if (broadcastId !== null && !APP.servos[broadcastId]) {
        const pos = await lxReadPosition(broadcastId);
        const vin = await lxReadVin(broadcastId);
        const temp = await lxReadTemp(broadcastId);
        APP.servos[broadcastId] = { position: pos, voltage: vin, temperature: temp };
        found.push({ id: broadcastId, position: pos, voltage: vin, temperature: temp });
    }

    // 创建卡片
    found.forEach(s => createServoCard(s.id));

    document.getElementById('btn-scan').disabled = false;
    document.getElementById('btn-scan').textContent = '🔍 扫描舵机';
    document.getElementById('info-servo-count').textContent = found.length;

    if (found.length > 0) {
        showToast(`发现 ${found.length} 个舵机`, 'success');
        startPolling();
    } else {
        showToast('未发现舵机', 'error');
    }
}

// ============================================================
// 后台轮询
// ============================================================
function startPolling() {
    if (APP.pollingTimer) return;
    document.getElementById('info-poll-status').textContent = '运行中';

    async function poll() {
        if (!APP.connected) return;
        const ids = Object.keys(APP.servos).map(Number);

        for (const sid of ids) {
            if (!APP.connected) break;
            try {
                const pos = await lxReadPosition(sid);
                const vin = await lxReadVin(sid);
                const temp = await lxReadTemp(sid);
                const now = Date.now() / 1000;

                APP.servos[sid] = { position: pos, voltage: vin, temperature: temp };
                updateServoStats(sid);
                appendChartData(sid, { position: pos, voltage: vin, temperature: temp, timestamp: now });
            } catch { }
        }

        if (APP.connected) APP.pollingTimer = setTimeout(poll, 500);
    }

    poll();
}

function stopPolling() {
    if (APP.pollingTimer) {
        clearTimeout(APP.pollingTimer);
        APP.pollingTimer = null;
    }
    const el = document.getElementById('info-poll-status');
    if (el) el.textContent = '停止';
}

// ============================================================
// 舵机卡片 UI
// ============================================================
function clearServoCards() {
    const container = document.getElementById('servo-cards');
    container.innerHTML = `
        <div class="empty-state" id="empty-state">
            <div class="empty-icon">🤖</div>
            <h2>未发现舵机</h2>
            <p>请先连接串口，然后点击"扫描舵机"按钮</p>
        </div>`;
    APP.charts = {};
    APP.chartData = {};
}

function createServoCard(id) {
    const container = document.getElementById('servo-cards');
    const empty = document.getElementById('empty-state');
    if (empty) empty.remove();

    const data = APP.servos[id] || {};

    const card = document.createElement('div');
    card.className = 'servo-card';
    card.id = `servo-card-${id}`;

    card.innerHTML = `
        <div class="card-header">
            <span class="card-id">舵机 #${id}</span>
            <div class="card-actions">
                <button class="btn btn-sm" onclick="changeIdModal(${id})" title="修改 ID">✏️ ID</button>
                <button class="btn btn-sm btn-warning" onclick="unloadServo(${id})" title="卸载">🔓</button>
            </div>
        </div>

        <div class="card-stats">
            <div class="stat-item stat-pos">
                <div class="stat-label">位置</div>
                <div class="stat-value" id="stat-pos-${id}">${data.position ?? '-'}</div>
            </div>
            <div class="stat-item stat-vin">
                <div class="stat-label">电压</div>
                <div class="stat-value" id="stat-vin-${id}">${data.voltage != null ? (data.voltage / 1000).toFixed(1) : '-'}<span class="stat-unit">V</span></div>
            </div>
            <div class="stat-item stat-temp">
                <div class="stat-label">温度</div>
                <div class="stat-value" id="stat-temp-${id}">${data.temperature ?? '-'}<span class="stat-unit">°C</span></div>
            </div>
        </div>

        <div class="card-control">
            <div class="control-row">
                <label>位置</label>
                <div class="slider-container">
                    <input type="range" min="0" max="1000" value="${data.position ?? 500}"
                           id="slider-${id}" oninput="onSliderChange(${id}, this.value)">
                </div>
                <input type="number" class="input-sm pos-input" min="0" max="1000"
                       value="${data.position ?? 500}" id="pos-input-${id}"
                       onchange="onPosInputChange(${id}, this.value)">
            </div>
            <div class="control-row">
                <label>时间</label>
                <input type="number" class="input-sm time-input" min="50" max="30000" value="500"
                       id="time-input-${id}"> <span style="font-size:12px;color:var(--text-muted)">ms</span>
                <button class="btn btn-sm btn-primary" onclick="moveServo(${id})">移动</button>
            </div>
        </div>

        <div class="card-chart">
            <canvas id="chart-${id}"></canvas>
        </div>
    `;

    container.appendChild(card);
    initChart(id);
}

function updateServoStats(id) {
    const data = APP.servos[id];
    if (!data) return;
    const posEl = document.getElementById(`stat-pos-${id}`);
    const vinEl = document.getElementById(`stat-vin-${id}`);
    const tempEl = document.getElementById(`stat-temp-${id}`);
    if (posEl && data.position != null) posEl.textContent = data.position;
    if (vinEl && data.voltage != null) vinEl.innerHTML = `${(data.voltage / 1000).toFixed(1)}<span class="stat-unit">V</span>`;
    if (tempEl && data.temperature != null) tempEl.innerHTML = `${data.temperature}<span class="stat-unit">°C</span>`;
}

// ============================================================
// Chart.js
// ============================================================
function initChart(id) {
    const ctx = document.getElementById(`chart-${id}`);
    if (!ctx) return;
    APP.chartData[id] = { labels: [], pos: [], vin: [], temp: [] };

    APP.charts[id] = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: '位置', data: [], borderColor: '#4fc3f7', backgroundColor: 'rgba(79,195,247,0.1)', borderWidth: 2, pointRadius: 0, tension: 0.3, yAxisID: 'y' },
                { label: '电压(V)', data: [], borderColor: '#00d68f', backgroundColor: 'rgba(0,214,143,0.1)', borderWidth: 2, pointRadius: 0, tension: 0.3, yAxisID: 'y1' },
                { label: '温度(°C)', data: [], borderColor: '#ffc048', backgroundColor: 'rgba(255,192,72,0.1)', borderWidth: 2, pointRadius: 0, tension: 0.3, yAxisID: 'y1' },
            ],
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: { duration: 200 },
            interaction: { mode: 'index', intersect: false },
            plugins: { legend: { display: true, position: 'top', labels: { color: '#9da3b8', font: { size: 10 }, boxWidth: 12 } } },
            scales: {
                x: { display: true, ticks: { color: '#6b7194', font: { size: 9 }, maxTicksLimit: 8 }, grid: { color: 'rgba(42,46,63,0.5)' } },
                y: { display: true, position: 'left', title: { display: true, text: '位置', color: '#4fc3f7', font: { size: 10 } }, min: 0, max: 1050, ticks: { color: '#4fc3f7', font: { size: 9 } }, grid: { color: 'rgba(42,46,63,0.3)' } },
                y1: { display: true, position: 'right', title: { display: true, text: 'V / °C', color: '#9da3b8', font: { size: 10 } }, ticks: { color: '#9da3b8', font: { size: 9 } }, grid: { display: false } },
            },
        },
    });
}

function appendChartData(id, msg) {
    const cd = APP.chartData[id];
    const chart = APP.charts[id];
    if (!cd || !chart) return;

    cd.labels.push(formatTime(msg.timestamp));
    cd.pos.push(msg.position);
    cd.vin.push(msg.voltage != null ? msg.voltage / 1000 : null);
    cd.temp.push(msg.temperature);

    if (cd.labels.length > APP.maxDataPoints) { cd.labels.shift(); cd.pos.shift(); cd.vin.shift(); cd.temp.shift(); }

    chart.data.labels = cd.labels;
    chart.data.datasets[0].data = cd.pos;
    chart.data.datasets[1].data = cd.vin;
    chart.data.datasets[2].data = cd.temp;
    chart.update('none');
}

// ============================================================
// 舵机控制 — 用户操作暂停轮询，发送后恢复
// ============================================================
let moveDebounce = null;

function onSliderChange(id, val) {
    document.getElementById(`pos-input-${id}`).value = val;
    clearTimeout(moveDebounce);
    moveDebounce = setTimeout(async () => {
        const time = parseInt(document.getElementById(`time-input-${id}`).value) || 500;
        const wasPolling = !!APP.pollingTimer;
        stopPolling();
        // 等待当前串口操作完成
        await APP.serialLock;
        await lxMoveServo(id, parseInt(val), time);
        if (wasPolling) startPolling();
    }, 150);
}

function onPosInputChange(id, val) {
    document.getElementById(`slider-${id}`).value = val;
}

async function moveServo(id) {
    const pos = parseInt(document.getElementById(`pos-input-${id}`).value);
    const time = parseInt(document.getElementById(`time-input-${id}`).value) || 500;
    const wasPolling = !!APP.pollingTimer;
    stopPolling();
    await APP.serialLock;
    await lxMoveServo(id, pos, time);
    showToast(`舵机 #${id} → 位置 ${pos}`, 'info');
    if (wasPolling) startPolling();
}

async function unloadServo(id) {
    const wasPolling = !!APP.pollingTimer;
    stopPolling();
    await APP.serialLock;
    await ctrlUnloadServos([id]);
    showToast(`舵机 #${id} 已卸载`, 'success');
    if (wasPolling) startPolling();
}

async function unloadAll() {
    const ids = Object.keys(APP.servos).map(Number);
    if (ids.length === 0) return;
    const wasPolling = !!APP.pollingTimer;
    stopPolling();
    await APP.serialLock;
    await ctrlUnloadServos(ids);
    showToast('所有舵机已卸载', 'success');
    if (wasPolling) startPolling();
}

// ============================================================
// 修改 ID
// ============================================================
let changeIdTarget = null;

function changeIdModal(id) {
    changeIdTarget = id;
    document.getElementById('modal-current-id').textContent = id;
    document.getElementById('modal-new-id').value = id;
    document.getElementById('modal-overlay').classList.remove('hidden');
}

function closeModal() {
    document.getElementById('modal-overlay').classList.add('hidden');
    changeIdTarget = null;
}

async function confirmChangeId() {
    const newId = parseInt(document.getElementById('modal-new-id').value);
    if (isNaN(newId) || newId < 0 || newId > 253) { showToast('ID 范围: 0~253', 'error'); return; }

    closeModal();
    showToast(`正在修改 ID → ${newId}...`, 'info');

    stopPolling();
    await new Promise(r => setTimeout(r, 300));

    const currentId = await lxReadId(LX_BROADCAST_ID);
    if (currentId === null) { showToast('未检测到舵机', 'error'); startPolling(); return; }

    await lxWriteId(currentId, newId);
    await new Promise(r => setTimeout(r, 500));

    const verify = await lxReadId(LX_BROADCAST_ID);
    if (verify === newId) {
        showToast(`ID 修改成功: ${currentId} → ${newId}`, 'success');
    } else {
        showToast(`ID 修改可能失败, 当前读到: ${verify}`, 'error');
    }

    // 重新扫描
    await scanServos();
}

// ============================================================
// 动作组控制
// ============================================================
async function runActionGroup() {
    const id = parseInt(document.getElementById('action-group-id').value);
    await ctrlRunActionGroup(id, 1);
    showToast(`动作组 ${id} 运行`, 'success');
}

async function stopActionGroup() {
    await ctrlStopActionGroup();
    showToast('动作组已停止', 'info');
}

async function setActionGroupSpeed() {
    const id = parseInt(document.getElementById('action-group-id').value);
    const speed = parseInt(document.getElementById('action-group-speed').value);
    await ctrlSetActionGroupSpeed(id, speed);
    showToast(`动作组 ${id} 速度 ${speed}%`, 'info');
}

// ============================================================
// UI 状态
// ============================================================
function updateUI() {
    const dot = document.getElementById('conn-status');
    const btnConnect = document.getElementById('btn-connect');
    const btnDisconnect = document.getElementById('btn-disconnect');
    const btnScan = document.getElementById('btn-scan');
    const btnUnloadAll = document.getElementById('btn-unload-all');
    const panel = document.getElementById('info-panel');
    const agBtns = ['btn-ag-run', 'btn-ag-stop', 'btn-ag-speed'];

    if (APP.connected) {
        dot.className = 'status-dot connected';
        dot.title = '已连接';
        btnConnect.disabled = true;
        btnDisconnect.disabled = false;
        btnScan.disabled = false;
        btnUnloadAll.disabled = false;
        agBtns.forEach(id => document.getElementById(id).disabled = false);
        panel.classList.remove('hidden');

        document.getElementById('info-port').textContent = 'Web Serial';
        document.getElementById('info-baud').textContent = APP.baudRate;
    } else {
        dot.className = 'status-dot disconnected';
        dot.title = '未连接';
        btnConnect.disabled = false;
        btnDisconnect.disabled = true;
        btnScan.disabled = true;
        btnUnloadAll.disabled = true;
        agBtns.forEach(id => document.getElementById(id).disabled = true);
        panel.classList.add('hidden');
    }
}

// ============================================================
// 检查 Web Serial API 支持
// ============================================================
function checkWebSerialSupport() {
    if (!('serial' in navigator)) {
        document.getElementById('servo-cards').innerHTML = `
            <div class="empty-state">
                <div class="empty-icon">⚠️</div>
                <h2>浏览器不支持 Web Serial API</h2>
                <p>请使用 <strong>Chrome 89+</strong> 或 <strong>Edge 89+</strong> 浏览器</p>
                <p style="margin-top:8px;font-size:13px;color:var(--text-muted)">
                    如果已经是 Chrome，请确认 chrome://flags/#enable-experimental-web-platform-features 已启用
                </p>
            </div>`;
    }
}

// ============================================================
// 事件绑定
// ============================================================
document.addEventListener('DOMContentLoaded', () => {
    checkWebSerialSupport();
    updateUI();

    document.getElementById('btn-connect').addEventListener('click', connectSerial);
    document.getElementById('btn-disconnect').addEventListener('click', disconnectSerial);
    document.getElementById('btn-scan').addEventListener('click', scanServos);
    document.getElementById('btn-unload-all').addEventListener('click', unloadAll);
    document.getElementById('btn-modal-cancel').addEventListener('click', closeModal);
    document.getElementById('btn-modal-confirm').addEventListener('click', confirmChangeId);
    document.getElementById('btn-ag-run').addEventListener('click', runActionGroup);
    document.getElementById('btn-ag-stop').addEventListener('click', stopActionGroup);
    document.getElementById('btn-ag-speed').addEventListener('click', setActionGroupSpeed);
});
