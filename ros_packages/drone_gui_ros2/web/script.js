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
        
        this.socket.on('connect_error', (error) => {
            console.error('Connection error:', error);
            this.isConnected = false;
            this.updateConnectionStatus();
        });
    }
    
    setupEventListeners() {
        // Add any additional event listeners here
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
        
        container.innerHTML = '';
        
        Object.values(this.droneData).forEach(drone => {
            const card = this.createDroneCard(drone);
            container.appendChild(card);
        });
    }
    
    createDroneCard(drone) {
        const card = document.createElement('div');
        card.className = 'drone-card';
        
        if (drone.connected) {
            card.classList.add('connected');
        }
        if (drone.armed) {
            card.classList.add('armed');
        }
        
        const stateDisplayName = this.formatStateName(drone.state);
        const modeDisplayName = this.formatModeName(drone.mode);
        
        card.innerHTML = `
            <div class="drone-header">
                <div class="drone-name">${drone.name.toUpperCase()} (Agent ${drone.agent_id})</div>
                <div class="drone-status">
                    <span class="status-badge ${drone.connected ? 'status-connected' : 'status-disconnected'}">
                        ${drone.connected ? 'ONLINE' : 'OFFLINE'}
                    </span>
                    <span class="status-badge ${drone.armed ? 'status-armed' : 'status-disarmed'}">
                        ${drone.armed ? 'ARMED' : 'DISARMED'}
                    </span>
                </div>
            </div>
            
            <div class="drone-info">
                <div class="info-row">
                    <span class="info-label">Flight Mode:</span>
                    <span class="info-value">${modeDisplayName}</span>
                </div>
                <div class="info-row">
                    <span class="info-label">Drone State:</span>
                    <span class="info-value">${stateDisplayName}</span>
                </div>
                <div class="info-row">
                    <span class="info-label">Status:</span>
                    <span class="info-value">${this.getOverallStatus(drone)}</span>
                </div>
            </div>
            
            <div class="drone-controls">
                <button class="btn btn-success btn-small" 
                        onclick="gui.sendIndividualCommand('${drone.name}', 'arm')"
                        ${!drone.connected || drone.armed ? 'disabled' : ''}>
                    ✈️ Arm
                </button>
                <button class="btn btn-danger btn-small" 
                        onclick="gui.sendIndividualCommand('${drone.name}', 'disarm')"
                        ${!drone.connected || !drone.armed ? 'disabled' : ''}>
                    ⛔ Disarm
                </button>
                <button class="btn btn-primary btn-small" 
                        onclick="gui.sendIndividualCommand('${drone.name}', 'takeoff')"
                        ${!drone.connected || !drone.armed || drone.state === 'TAKING_OFF' ? 'disabled' : ''}>
                    🚀 Takeoff
                </button>
                <button class="btn btn-warning btn-small" 
                        onclick="gui.sendIndividualCommand('${drone.name}', 'land')"
                        ${!drone.connected || drone.state === 'IDLE' || drone.state === 'LANDING' ? 'disabled' : ''}>
                    🛬 Land
                </button>
            </div>
            
            <div class="drone-controls">
                <button class="btn btn-info btn-small" 
                        onclick="gui.sendIndividualCommand('${drone.name}', 'planning_start')"
                        ${!drone.connected ? 'disabled' : ''}>
                    🧠 Start Planning
                </button>
                <button class="btn btn-secondary btn-small" 
                        onclick="gui.sendIndividualCommand('${drone.name}', 'planning_stop')"
                        ${!drone.connected ? 'disabled' : ''}>
                    🛑 Stop Planning
                </button>
            </div>
        `;
        
        return card;
    }
    
    formatStateName(state) {
        const stateNames = {
            'IDLE': '🟢 Idle',
            'TAKING_OFF': '🚀 Taking Off',
            'HOVERING': '⏸️ Hovering',
            'OFFBOARD': '🎮 Offboard',
            'LANDING': '🛬 Landing',
            'UNKNOWN': '❓ Unknown'
        };
        return stateNames[state] || state;
    }
    
    formatModeName(mode) {
        const modeNames = {
            'MANUAL': '🕹️ Manual',
            'POSCTL': '📍 Position',
            'AUTO.LOITER': '⭕ Loiter',
            'AUTO.TAKEOFF': '🚀 Auto Takeoff',
            'AUTO.LAND': '🛬 Auto Land',
            'OFFBOARD': '🎮 Offboard',
            'UNKNOWN': '❓ Unknown'
        };
        return modeNames[mode] || mode;
    }
    
    getOverallStatus(drone) {
        if (!drone.connected) {
            return '❌ Disconnected';
        }
        
        if (drone.state === 'TAKING_OFF') {
            return '🚀 Taking Off...';
        } else if (drone.state === 'LANDING') {
            return '🛬 Landing...';
        } else if (drone.state === 'HOVERING') {
            return '⏸️ Hovering';
        } else if (drone.state === 'OFFBOARD') {
            return '🎮 In Mission';
        } else if (drone.armed) {
            return '⚡ Armed & Ready';
        } else {
            return '💤 Standby';
        }
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
        
        console.log(`Sending global command: ${command}`);
        this.socket.emit('global_command', { command: command });
        
        // Show user feedback
        this.showNotification(`Global ${command} command sent`);
        
        // Temporarily disable the button
        let buttonId;
        if (command === 'controller_enable' || command === 'controller_disable') {
            buttonId = `global-${command.replace('_', '-')}`;
        } else if (command === 'planning_start' || command === 'planning_stop') {
            buttonId = `global-${command.replace('_', '-')}`;
        } else {
            buttonId = `global-${command}`;
        }
        const button = document.getElementById(buttonId);
        if (button) {
            button.disabled = true;
            setTimeout(() => {
                button.disabled = false;
            }, 2000);
        }
    }
    
    sendIndividualCommand(drone, command) {
        if (!this.isConnected) {
            this.showError('Not connected to server');
            return;
        }
        
        console.log(`Sending command to ${drone}: ${command}`);
        this.socket.emit('individual_command', { 
            drone: drone, 
            command: command 
        });
        
        // Show user feedback
        this.showNotification(`${command} command sent to ${drone.toUpperCase()}`);
    }
    
    showNotification(message) {
        // Create a simple notification
        const notification = document.createElement('div');
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: #4CAF50;
            color: white;
            padding: 15px 20px;
            border-radius: 8px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.2);
            z-index: 1000;
            font-weight: 600;
        `;
        notification.textContent = message;
        document.body.appendChild(notification);
        
        setTimeout(() => {
            notification.remove();
        }, 3000);
    }
    
    showError(message) {
        // Create a simple error notification
        const notification = document.createElement('div');
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: #f44336;
            color: white;
            padding: 15px 20px;
            border-radius: 8px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.2);
            z-index: 1000;
            font-weight: 600;
        `;
        notification.textContent = `Error: ${message}`;
        document.body.appendChild(notification);
        
        setTimeout(() => {
            notification.remove();
        }, 5000);
    }
}

// Global functions for HTML onclick handlers
function sendGlobalCommand(command) {
    gui.sendGlobalCommand(command);
}

function sendIndividualCommand(drone, command) {
    gui.sendIndividualCommand(drone, command);
}

// Initialize the GUI when the page loads
let gui;
document.addEventListener('DOMContentLoaded', () => {
    gui = new DroneGUI();
});