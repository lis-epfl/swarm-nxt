class DroneGUI {
    constructor() {
        this.socket = null;
        this.droneData = {};
        this.isConnected = false;
        this.lastUpdate = null;

        this.initializeSocket();
        this.setupEventListeners();
        this.updateConnectionStatus();
    }

    initializeSocket() {
        this.socket = io();

        this.socket.on('connect', () => {
            console.log('Connected to server');
            this.isConnected = true;
            this.updateConnectionStatus();
        });

        this.socket.on('disconnect', () => {
            console.log('Disconnected from server');
            this.isConnected = false;
            this.updateConnectionStatus();
        });

        this.socket.on('drone_update', (data) => {
            this.droneData = data;
            this.lastUpdate = new Date();
            this.updateDroneCards();
            this.updateLastUpdateTime();
        });

        this.socket.on('roaming_status', (data) => {
            console.log('Roaming status update:', data);
            document.getElementById('global-start-roaming').disabled = data.active;
            document.getElementById('global-stop-roaming').disabled = !data.active;
        });

        this.socket.on('connect_error', (error) => {
            console.error('Connection error:', error);
            this.isConnected = false;
            this.updateConnectionStatus();
        });
    }

    setupEventListeners() {
        window.addEventListener('beforeunload', () => {
            if (this.socket) {
                this.socket.disconnect();
            }
        });
    }

    updateConnectionStatus() {
        const statusIndicator = document.getElementById('status-indicator');
        const statusText = document.getElementById('status-text');

        if (this.isConnected) {
            statusIndicator.classList.add('connected');
            statusText.textContent = 'Connected';
        } else {
            statusIndicator.classList.remove('connected');
            statusText.textContent = 'Disconnected';
        }
    }

    updateDroneCards() {
        const container = document.getElementById('drone-cards');

        if (Object.keys(this.droneData).length === 0) {
            container.innerHTML = '<div class="loading">No drone data available</div>';
            return;
        }

        const updatedCardIds = new Set();

        Object.values(this.droneData).forEach(drone => {
            const cardId = `drone-card-${drone.name}`;
            updatedCardIds.add(cardId);

            let existingCard = document.getElementById(cardId);

            if (!existingCard) {
                existingCard = this.createDroneCard(drone);
                container.appendChild(existingCard);
            } else {
                this.updateCardData(existingCard, drone);
            }
        });

        if (container.children.length > Object.keys(this.droneData).length) {
            for (const card of container.children) {
                if (card.id && !updatedCardIds.has(card.id)) {
                    card.remove();
                }
            }
        }
    }

    createDroneCard(drone) {
        const card = document.createElement('div');
        card.className = 'drone-card';
        card.id = `drone-card-${drone.name}`;

        card.innerHTML = `
            <div class="drone-header">
                <div class="drone-name">${drone.name.toUpperCase()} (Agent ${drone.agent_id})</div>
                <div class="drone-status">
                    <span class="status-badge conn-badge"></span>
                    <span class="status-badge armed-badge"></span>
                </div>
            </div>

            <div class="drone-info">
                <div class="info-row battery-info">
                    <span class="info-label">Battery:</span>
                    <div class="battery-display">
                        <div class="battery-bar-container">
                            <div class="battery-bar-level"></div>
                        </div>
                        <span class="info-value battery-text"></span>
                    </div>
                </div>
                <div class="info-row latency-info">
                    <span class="info-label">Latency:</span>
                    <span class="info-value latency-text"></span>
                </div>
                <div class="info-row">
                    <span class="info-label">Flight Mode:</span>
                    <span class="info-value flight-mode-value"></span>
                </div>
                <div class="info-row">
                    <span class="info-label">Drone State:</span>
                    <span class="info-value drone-state-value"></span>
                </div>
                <div class="info-row">
                    <span class="info-label">Status:</span>
                    <span class="info-value overall-status-value"></span>
                </div>
            </div>

            <div class="drone-controls-container">
                <div class="drone-controls">
                    <button class="btn btn-success btn-small btn-arm" onclick="gui.sendIndividualCommand('${drone.name}', 'arm')">
                        ✈️ Arm
                    </button>
                    <button class="btn btn-danger btn-small btn-disarm" onclick="gui.sendIndividualCommand('${drone.name}', 'disarm')">
                        ⛔ Disarm
                    </button>
                    <button class="btn btn-primary btn-small btn-takeoff" onclick="gui.sendIndividualCommand('${drone.name}', 'takeoff')">
                        🚀 Takeoff
                    </button>
                    <button class="btn btn-warning btn-small btn-land" onclick="gui.sendIndividualCommand('${drone.name}', 'land')">
                        🛬 Land
                    </button>
                </div>
                <div class="drone-controls planning-controls">
                    <button class="btn btn-info btn-small btn-plan-start" onclick="gui.sendIndividualCommand('${drone.name}', 'planning_start')">
                        🧠 Start Planning
                    </button>
                    <button class="btn btn-danger btn-small btn-plan-stop" onclick="gui.sendIndividualCommand('${drone.name}', 'planning_stop')">
                        🛑 Stop Planning
                    </button>
                </div>
                <div class="drone-controls goal-controls">
                    <input type="number" class="goal-input" id="goal-x-${drone.name}" placeholder="X">
                    <input type="number" class="goal-input" id="goal-y-${drone.name}" placeholder="Y">
                    <input type="number" class="goal-input" id="goal-z-${drone.name}" placeholder="Z">
                    <button class="btn btn-success btn-small goal-button btn-send-goal" onclick="gui.sendGoalCommand('${drone.name}')">
                        🎯 Send Goal
                    </button>
                </div>
                <div class="drone-controls kill-controls">
                    <button class="btn btn-danger btn-small btn-kill" onclick="gui.sendIndividualCommand('${drone.name}', 'kill')">
                        ☠️ KILL
                    </button>
                </div>
            </div>
        `;

        this.updateCardData(card, drone);
        return card;
    }

    updateCardData(card, drone) {
        card.classList.toggle('connected', drone.connected);
        card.classList.toggle('armed', drone.armed);

        // --- Battery Logic ---
        const battery = drone.battery_percent;
        let batteryClass = 'unknown';
        const isLow = (battery <= 20 && battery >= 0) || (drone.voltage_v > 0 && drone.voltage_v <= 21.0);

        if (battery < 0) {
            batteryClass = 'unknown';
        } else if (isLow) {
            batteryClass = 'low';
        } else if (battery <= 50) {
            batteryClass = 'medium';
        }

        const batteryText = battery < 0 ? 'N/A' : `${battery}% (${drone.voltage_v.toFixed(2)} V)`;

        card.querySelector('.battery-info').className = `info-row battery-info ${batteryClass}`;
        card.querySelector('.battery-bar-level').style.width = `${battery < 0 ? 0 : battery}%`;
        card.querySelector('.battery-text').textContent = batteryText;
        card.classList.toggle('low-battery', batteryClass === 'low');

        // --- Latency Logic ---
        const latency = drone.latency_ms;
        let latencyClass = (latency < 0) ? 'unknown' : (latency > 10.0 ? 'high' : 'normal');
        const latencyText = latency < 0 ? 'N/A' : `${latency.toFixed(1)} ms`;

        card.querySelector('.latency-info').className = `info-row latency-info ${latencyClass}`;
        card.querySelector('.latency-text').textContent = latencyText;

        // --- MODIFIED: Update Status Badges (THE FIX) ---
        const connBadge = card.querySelector('.conn-badge'); // Select by stable class
        connBadge.textContent = drone.connected ? 'ONLINE' : 'OFFLINE';
        // Use classList to safely add/remove dynamic classes
        connBadge.classList.toggle('status-connected', drone.connected);
        connBadge.classList.toggle('status-disconnected', !drone.connected);

        const armedBadge = card.querySelector('.armed-badge'); // Select by stable class
        armedBadge.textContent = drone.armed ? 'ARMED' : 'DISARMED';
        // Use classList
        armedBadge.classList.toggle('status-armed', drone.armed);
        armedBadge.classList.toggle('status-disarmed', !drone.armed);
        // --- END MODIFICATION ---

        // --- Update Info Text ---
        card.querySelector('.flight-mode-value').textContent = this.formatModeName(drone.mode);
        card.querySelector('.drone-state-value').textContent = this.formatStateName(drone.state);
        card.querySelector('.overall-status-value').textContent = this.getOverallStatus(drone);

        // --- Update Button Disabled States ---
        card.querySelector('.btn-arm').disabled = !drone.connected || drone.armed;
        card.querySelector('.btn-disarm').disabled = !drone.connected || !drone.armed;
        card.querySelector('.btn-takeoff').disabled = !drone.connected || !drone.armed || drone.state === 'TAKING_OFF';
        card.querySelector('.btn-land').disabled = !drone.connected || drone.state === 'IDLE' || drone.state === 'LANDING';
        card.querySelector('.btn-plan-start').disabled = !drone.connected;
        card.querySelector('.btn-plan-stop').disabled = !drone.connected;
        card.querySelector('.btn-send-goal').disabled = !drone.connected;
        card.querySelector('.btn-kill').disabled = !drone.connected;
    }

    formatStateName(state) {
        const stateNames = {
            'IDLE': '🟢 Idle', 'TAKING_OFF': '🚀 Taking Off', 'HOVERING': '⏸️ Hovering',
            'OFFBOARD': '🎮 Offboard', 'LANDING': '🛬 Landing', 'UNKNOWN': '❓ Unknown'
        };
        return stateNames[state] || state;
    }

    formatModeName(mode) {
        const modeNames = {
            'MANUAL': '🕹️ Manual', 'POSCTL': '📍 Position', 'AUTO.LOITER': '⭕ Loiter',
            'AUTO.TAKEOFF': '🚀 Auto Takeoff', 'AUTO.LAND': '🛬 Auto Land',
            'OFFBOARD': '🎮 Offboard', 'UNKNOWN': '❓ Unknown'
        };
        return modeNames[mode] || mode;
    }

    getOverallStatus(drone) {
        if (!drone.connected) { return '❌ Disconnected'; }

        const isLow = (drone.battery_percent <= 20 && drone.battery_percent >= 0) || (drone.voltage_v > 0 && drone.voltage_v <= 21.0);
        if (isLow) { return '⚠️ Low Battery'; }

        if (drone.latency_ms > 10.0) { return '📡 High Latency'; }
        if (drone.state === 'TAKING_OFF') { return '🚀 Taking Off...'; }
        if (drone.state === 'LANDING') { return '🛬 Landing...'; }
        if (drone.state === 'HOVERING') { return '⏸️ Hovering'; }
        if (drone.state === 'OFFBOARD') { return '🎮 In Mission'; }
        if (drone.armed) { return '⚡ Armed & Ready'; }
        return '💤 Standby';
    }

    updateLastUpdateTime() {
        const element = document.getElementById('last-update');
        if (this.lastUpdate) {
            element.textContent = this.lastUpdate.toLocaleTimeString();
        }
    }

    sendGlobalCommand(command) {
        if (!this.isConnected) {
            this.showError('Not connected to server');
            return;
        }
        if (command === 'kill') {
            if (!confirm('ARE YOU SURE YOU WANT TO KILL ALL DRONES? This is irreversible.')) {
                return;
            }
        }
        console.log(`Sending global command: ${command}`);
        this.socket.emit('global_command', { command: command });
        this.showNotification(`Global ${command} command sent`);

        let buttonId = `global-${command}`;
        if (command === 'planning_start' || command === 'planning_stop') {
            buttonId = `global-${command.replace('_', '-')}`;
        }
        const button = document.getElementById(buttonId);
        if (button && command !== 'kill' && command !== 'land') {
            button.disabled = true;
            setTimeout(() => { button.disabled = false; }, 2000);
        }
    }

    sendIndividualCommand(drone, command) {
        if (!this.isConnected) {
            this.showError('Not connected to server');
            return;
        }
        if (command === 'kill') {
            if (!confirm(`ARE YOU SURE YOU WANT TO KILL ${drone.toUpperCase()}? This is irreversible.`)) {
                return;
            }
        }
        console.log(`Sending command to ${drone}: ${command}`);
        this.socket.emit('individual_command', {
            drone: drone, command: command
        });
        this.showNotification(`${command} command sent to ${drone.toUpperCase()}`);
    }

    sendGoalCommand(drone) {
        if (!this.isConnected) {
            this.showError('Not connected to server');
            return;
        }
        const x = document.getElementById(`goal-x-${drone}`).value;
        const y = document.getElementById(`goal-y-${drone}`).value;
        const z = document.getElementById(`goal-z-${drone}`).value;
        if (x === '' || y === '' || z === '') {
            this.showError('All X, Y, and Z coordinates are required');
            return;
        }
        const goal = {
            drone: drone, x: parseFloat(x), y: parseFloat(y), z: parseFloat(z)
        };
        console.log(`Sending goal to ${drone}:`, goal);
        this.socket.emit('individual_goal', goal);
        this.showNotification(`Goal sent to ${drone.toUpperCase()}`);
    }

    sendRoamingCommand(command) {
        if (!this.isConnected) {
            this.showError('Not connected to server');
            return;
        }

        if (command === 'start') {
            const radius = document.getElementById('roam-radius').value;
            const center_height = document.getElementById('roam-center-height').value;
            const cylinder_height = document.getElementById('roam-cyl-height').value;

            if (radius === '' || center_height === '' || cylinder_height === '') {
                this.showError('All roaming parameters are required');
                return;
            }

            const data = {
                radius: parseFloat(radius),
                center_height: parseFloat(center_height),
                cylinder_height: parseFloat(cylinder_height)
            };

            this.socket.emit('start_roaming', data);
            this.showNotification('Start Roaming command sent');

        } else if (command === 'stop') {
            this.socket.emit('stop_roaming', {});
            this.showNotification('Stop Roaming command sent');
        }
    }

    showNotification(message) {
        const notification = document.createElement('div');
        notification.style.cssText = `
            position: fixed; top: 20px; right: 20px; background: #4CAF50; color: white;
            padding: 15px 20px; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);
            z-index: 1000; font-weight: 600;`;
        notification.textContent = message;
        document.body.appendChild(notification);
        setTimeout(() => { notification.remove(); }, 3000);
    }

    showError(message) {
        const notification = document.createElement('div');
        notification.style.cssText = `
            position: fixed; top: 20px; right: 20px; background: #f44336; color: white;
            padding: 15px 20px; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);
            z-index: 1000; font-weight: 600;`;
        notification.textContent = `Error: ${message}`;
        document.body.appendChild(notification);
        setTimeout(() => { notification.remove(); }, 5000);
    }
}

// Global functions
function sendGlobalCommand(command) { gui.sendGlobalCommand(command); }
function sendIndividualCommand(drone, command) { gui.sendIndividualCommand(drone, command); }
function sendGoalCommand(drone) { gui.sendGoalCommand(drone); }
function sendRoamingCommand(command) { gui.sendRoamingCommand(command); }

// Initialize
let gui;
document.addEventListener('DOMContentLoaded', () => {
    gui = new DroneGUI();
});
