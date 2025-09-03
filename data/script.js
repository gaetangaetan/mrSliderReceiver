// Configuration
const UPDATE_INTERVAL = 100; // 100ms
const API_BASE = '';

// État de l'application
let isUpdating = false;
let lastUpdate = 0;

// Éléments DOM
const sliders = {
    posx: document.getElementById('posx'),
    pan: document.getElementById('pan'),
    tilt: document.getElementById('tilt'),
    speed_posx: document.getElementById('speed_posx'),
    speed_pan: document.getElementById('speed_pan'),
    speed_tilt: document.getElementById('speed_tilt'),
    acceleration: document.getElementById('acceleration')
};

const values = {
    posx: document.getElementById('posx-value'),
    pan: document.getElementById('pan-value'),
    tilt: document.getElementById('tilt-value'),
    speed_posx: document.getElementById('speed_posx-value'),
    speed_pan: document.getElementById('speed_pan-value'),
    speed_tilt: document.getElementById('speed_tilt-value'),
    acceleration: document.getElementById('acceleration-value')
};

const saveButtons = document.querySelectorAll('.save-btn');
const loadButtons = document.querySelectorAll('.load-btn');
const updateIndicator = document.getElementById('update-indicator');
const connectionStatus = document.getElementById('connection-status');

// Éléments du mode automatique
const presetASelect = document.getElementById('preset-a');
const presetBSelect = document.getElementById('preset-b');
const autoDelaySlider = document.getElementById('auto-delay');
const autoDelayValue = document.getElementById('auto-delay-value');
const autoToggle = document.getElementById('auto-toggle');
const autoStatus = document.getElementById('auto-status');
const statusTimer = document.getElementById('status-timer');

// Fonction pour afficher un message de statut
function showStatus(message, type = 'success') {
    // Supprimer les anciens messages
    const existingMessages = document.querySelectorAll('.status-message');
    existingMessages.forEach(msg => msg.remove());
    
    const statusDiv = document.createElement('div');
    statusDiv.className = `status-message ${type}`;
    statusDiv.textContent = message;
    document.body.appendChild(statusDiv);
    
    // Afficher le message
    setTimeout(() => statusDiv.classList.add('show'), 100);
    
    // Masquer après 3 secondes
    setTimeout(() => {
        statusDiv.classList.remove('show');
        setTimeout(() => statusDiv.remove(), 300);
    }, 3000);
}

// Fonction pour mettre à jour les valeurs affichées
function updateDisplayValues() {
    Object.keys(sliders).forEach(key => {
        if (sliders[key] && values[key]) {
            values[key].textContent = sliders[key].value;
        }
    });
}

// Fonction pour mettre à jour les limites des sliders
function updateSliderLimits(limits) {
    if (!limits) return;
    
    // Position X
    if (sliders.posx && limits.posx_min !== undefined && limits.posx_max !== undefined) {
        sliders.posx.min = limits.posx_min;
        sliders.posx.max = limits.posx_max;
        // S'assurer que la valeur actuelle est dans les nouvelles limites
        const currentVal = parseInt(sliders.posx.value);
        if (currentVal < limits.posx_min) sliders.posx.value = limits.posx_min;
        if (currentVal > limits.posx_max) sliders.posx.value = limits.posx_max;
    }
    
    // Pan
    if (sliders.pan && limits.pan_min !== undefined && limits.pan_max !== undefined) {
        sliders.pan.min = limits.pan_min;
        sliders.pan.max = limits.pan_max;
        const currentVal = parseInt(sliders.pan.value);
        if (currentVal < limits.pan_min) sliders.pan.value = limits.pan_min;
        if (currentVal > limits.pan_max) sliders.pan.value = limits.pan_max;
    }
    
    // Tilt
    if (sliders.tilt && limits.tilt_min !== undefined && limits.tilt_max !== undefined) {
        sliders.tilt.min = limits.tilt_min;
        sliders.tilt.max = limits.tilt_max;
        const currentVal = parseInt(sliders.tilt.value);
        if (currentVal < limits.tilt_min) sliders.tilt.value = limits.tilt_min;
        if (currentVal > limits.tilt_max) sliders.tilt.value = limits.tilt_max;
    }
    
    // Vitesses (toutes utilisent les mêmes limites)
    if (limits.speed_min !== undefined && limits.speed_max !== undefined) {
        ['speed_posx', 'speed_pan', 'speed_tilt'].forEach(key => {
            if (sliders[key]) {
                sliders[key].min = limits.speed_min;
                sliders[key].max = limits.speed_max;
                const currentVal = parseInt(sliders[key].value);
                if (currentVal < limits.speed_min) sliders[key].value = limits.speed_min;
                if (currentVal > limits.speed_max) sliders[key].value = limits.speed_max;
            }
        });
    }
    
    // Accélération
    if (sliders.acceleration && limits.accel_min !== undefined && limits.accel_max !== undefined) {
        sliders.acceleration.min = limits.accel_min;
        sliders.acceleration.max = limits.accel_max;
        const currentVal = parseInt(sliders.acceleration.value);
        if (currentVal < limits.accel_min) sliders.acceleration.value = limits.accel_min;
        if (currentVal > limits.accel_max) sliders.acceleration.value = limits.accel_max;
    }
    
    console.log('Limites des sliders mises à jour:', limits);
}

// Fonction pour envoyer les paramètres au serveur
async function sendUpdate() {
    if (isUpdating) return;
    
    const now = Date.now();
    if (now - lastUpdate < UPDATE_INTERVAL) return;
    
    isUpdating = true;
    lastUpdate = now;
    
    try {
        const params = new URLSearchParams();
        Object.keys(sliders).forEach(key => {
            if (sliders[key]) {
                params.append(key, sliders[key].value);
            }
        });
        
        updateIndicator.style.color = '#ffa726'; // Orange pendant l'envoi
        
        const response = await fetch(`/api/update?${params.toString()}`);
        
        if (response.ok) {
            updateIndicator.style.color = '#2ed573'; // Vert si succès
        } else {
            throw new Error('Erreur serveur');
        }
    } catch (error) {
        console.error('Erreur lors de la mise à jour:', error);
        updateIndicator.style.color = '#ff4757'; // Rouge si erreur
        connectionStatus.textContent = 'Erreur de connexion';
    } finally {
        isUpdating = false;
    }
}

// Fonction pour sauvegarder un preset
async function savePreset(slot) {
    try {
        const preset = {};
        Object.keys(sliders).forEach(key => {
            if (sliders[key]) {
                preset[key] = parseInt(sliders[key].value);
            }
        });
        
        const response = await fetch('/api/preset/save', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                slot: slot,
                preset: preset
            })
        });
        
        if (response.ok) {
            showStatus(`Preset ${slot} sauvegardé`, 'success');
            updatePresetButtons();
        } else {
            throw new Error('Erreur lors de la sauvegarde');
        }
    } catch (error) {
        console.error('Erreur sauvegarde preset:', error);
        showStatus(`Erreur sauvegarde preset ${slot}`, 'error');
    }
}

// Fonction pour charger un preset
async function loadPreset(slot) {
    try {
        const response = await fetch(`/api/preset/load?slot=${slot}`);
        
        if (response.ok) {
            const data = await response.json();
            
            // Mettre à jour les sliders
            Object.keys(data.preset).forEach(key => {
                if (sliders[key]) {
                    sliders[key].value = data.preset[key];
                }
            });
            
            updateDisplayValues();
            sendUpdate(); // Envoyer immédiatement les nouvelles valeurs
            showStatus(`Preset ${slot} chargé`, 'success');
        } else {
            throw new Error('Preset non trouvé');
        }
    } catch (error) {
        console.error('Erreur chargement preset:', error);
        showStatus(`Erreur chargement preset ${slot}`, 'error');
    }
}

// Fonction pour vérifier quels presets existent
async function updatePresetButtons() {
    try {
        const response = await fetch('/api/preset/list');
        if (response.ok) {
            const data = await response.json();
            
            loadButtons.forEach(btn => {
                const slot = parseInt(btn.dataset.slot);
                if (data.presets.includes(slot)) {
                    btn.classList.add('has-preset');
                } else {
                    btn.classList.remove('has-preset');
                }
            });
        }
    } catch (error) {
        console.error('Erreur lors de la vérification des presets:', error);
    }
}

// Fonction pour obtenir l'état actuel du serveur
async function getCurrentState() {
    try {
        const response = await fetch('/api/status');
        if (response.ok) {
            const data = await response.json();
            
            // Mettre à jour les limites des sliders si elles sont présentes
            if (data.limits) {
                updateSliderLimits(data.limits);
            }
            
            // Mettre à jour les sliders avec les valeurs du serveur
            Object.keys(data).forEach(key => {
                if (sliders[key] && data[key] !== undefined) {
                    sliders[key].value = data[key];
                }
            });
            
            // Mettre à jour l'état du mode automatique
            if (data.auto_mode) {
                updateAutoModeUI(data.auto_mode);
            }
            
            updateDisplayValues();
            connectionStatus.textContent = 'Connecté';
        }
    } catch (error) {
        console.error('Erreur lors de la récupération de l\'état:', error);
        connectionStatus.textContent = 'Déconnecté';
    }
}

// Fonction pour configurer le mode automatique
async function configureAutoMode() {
    try {
        const config = {
            enabled: autoToggle.checked,
            preset_a: parseInt(presetASelect.value),
            preset_b: parseInt(presetBSelect.value),
            delay: parseInt(autoDelaySlider.value)
        };
        
        const response = await fetch('/api/auto', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(config)
        });
        
        const result = await response.json();
        
        if (response.ok) {
            showStatus(`Mode automatique ${config.enabled ? 'activé' : 'désactivé'}`, 'success');
            // Mettre à jour immédiatement l'état
            getCurrentState();
        } else {
            showStatus(`Erreur: ${result.error}`, 'error');
            // Réinitialiser le toggle en cas d'erreur
            autoToggle.checked = false;
        }
    } catch (error) {
        console.error('Erreur lors de la configuration du mode auto:', error);
        showStatus('Erreur de communication', 'error');
        autoToggle.checked = false;
    }
}

// Fonction pour mettre à jour l'interface du mode automatique
function updateAutoModeUI(autoModeData) {
    // Mettre à jour les contrôles
    presetASelect.value = autoModeData.preset_a;
    presetBSelect.value = autoModeData.preset_b;
    autoDelaySlider.value = autoModeData.delay;
    autoDelayValue.textContent = autoModeData.delay;
    autoToggle.checked = autoModeData.enabled;
    
    // Mettre à jour l'affichage du statut
    if (autoModeData.enabled) {
        autoStatus.classList.add('active');
        const currentPreset = autoModeData.current_preset_is_a ? 'A' : 'B';
        const presetNumber = autoModeData.current_preset_is_a ? autoModeData.preset_a : autoModeData.preset_b;
        autoStatus.querySelector('.status-text').textContent = `Mode auto actif - Preset ${currentPreset} (${presetNumber})`;
        
        // Afficher le temps restant
        if (autoModeData.time_remaining > 0) {
            const seconds = Math.ceil(autoModeData.time_remaining / 1000);
            statusTimer.textContent = `${seconds}s`;
        } else {
            statusTimer.textContent = 'Changement...';
        }
    } else {
        autoStatus.classList.remove('active');
        autoStatus.querySelector('.status-text').textContent = 'Mode manuel';
        statusTimer.textContent = '';
    }
}

// Fonction pour mettre à jour la valeur affichée du délai
function updateAutoDelayValue() {
    autoDelayValue.textContent = autoDelaySlider.value;
}

// Initialisation des event listeners
function initEventListeners() {
    // Event listeners pour les sliders
    Object.keys(sliders).forEach(key => {
        if (sliders[key]) {
            sliders[key].addEventListener('input', () => {
                updateDisplayValues();
                sendUpdate();
            });
        }
    });
    
    // Event listeners pour les boutons de sauvegarde
    saveButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            savePreset(slot);
        });
    });
    
    // Event listeners pour les boutons de chargement
    loadButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const slot = parseInt(btn.dataset.slot);
            loadPreset(slot);
        });
    });
    
    // Event listeners pour le mode automatique
    if (autoDelaySlider) {
        autoDelaySlider.addEventListener('input', updateAutoDelayValue);
    }
    
    if (autoToggle) {
        autoToggle.addEventListener('change', configureAutoMode);
    }
    
    if (presetASelect) {
        presetASelect.addEventListener('change', () => {
            if (autoToggle.checked) {
                configureAutoMode();
            }
        });
    }
    
    if (presetBSelect) {
        presetBSelect.addEventListener('change', () => {
            if (autoToggle.checked) {
                configureAutoMode();
            }
        });
    }
    
    // Event listener pour le changement de délai (seulement si le mode auto est actif)
    if (autoDelaySlider) {
        autoDelaySlider.addEventListener('change', () => {
            if (autoToggle.checked) {
                configureAutoMode();
            }
        });
    }
}

// Fonction d'initialisation
function init() {
    initEventListeners();
    updateDisplayValues();
    updateAutoDelayValue(); // Initialiser la valeur du délai automatique
    getCurrentState();
    updatePresetButtons();
    
    // Vérification périodique de la connexion
    setInterval(() => {
        if (Date.now() - lastUpdate > 5000) { // Si pas de mise à jour depuis 5s
            getCurrentState();
        }
    }, 5000);
    
    // Event listener pour détecter quand on revient sur la page (focus)
    window.addEventListener('focus', () => {
        console.log('Page focused - rechargement des limites');
        getCurrentState(); // Recharger l'état complet avec les nouvelles limites
    });
    
    // Event listener pour la visibilité de la page
    document.addEventListener('visibilitychange', () => {
        if (!document.hidden) {
            console.log('Page visible - rechargement des limites');
            getCurrentState(); // Recharger l'état complet avec les nouvelles limites
        }
    });
    
    console.log('MrSlider Control Interface initialisée');
}

// Démarrage de l'application
document.addEventListener('DOMContentLoaded', init);
