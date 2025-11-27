const express = require('express');
const cors = require('cors');
const bodyParser = require('body-parser');
const path = require('path');

const app = express();
const PORT = 8081;

// Middleware
app.use(cors());
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: true }));
app.use(express.static(path.join(__dirname, 'public')));

// Almacén mejorado de conexiones
let connectionLog = {
    activeConnections: [],
    connectionHistory: [],
    devices: {
        metaquest: [],
        esp32: [],
        browser: [],
        unknown: []
    },
    statistics: {
        totalConnections: 0,
        uniqueDevices: 0
    }
};

// Función para registrar conexiones
function logConnection(req, deviceType = 'unknown') {
    const clientIP = req.ip || req.connection.remoteAddress || req.socket.remoteAddress;
    
    const connection = {
        id: Math.random().toString(36).substr(2, 9) + Date.now().toString(36),
        ip: clientIP,
        userAgent: req.get('User-Agent') || 'Unknown',
        deviceType: deviceType,
        endpoint: req.path,
        method: req.method,
        timestamp: new Date().toISOString(),
        clientId: req.headers['client-id'] || 'unknown'
    };

    // Agregar a conexiones activas (solo mantener últimas 50)
    connectionLog.activeConnections.unshift(connection);
    if (connectionLog.activeConnections.length > 50) {
        connectionLog.activeConnections.pop();
    }

    // Agregar al historial (solo mantener últimas 200)
    connectionLog.connectionHistory.unshift(connection);
    if (connectionLog.connectionHistory.length > 200) {
        connectionLog.connectionHistory.pop();
    }

    // Actualizar estadísticas
    connectionLog.statistics.totalConnections++;

    // Agregar dispositivo si es nuevo
    const deviceIdentifier = connection.clientId + '-' + connection.ip;
    if (!connectionLog.devices[deviceType].includes(deviceIdentifier)) {
        connectionLog.devices[deviceType].push(deviceIdentifier);
        connectionLog.statistics.uniqueDevices = Object.values(connectionLog.devices).reduce((acc, arr) => acc + arr.length, 0);
    }

    console.log(`🔗 [${deviceType.toUpperCase()}] ${connection.ip} - ${req.method} ${req.path}`);
    
    return connection;
}

// Middleware de logging mejorado
app.use((req, res, next) => {
    // Determinar tipo de dispositivo
    let deviceType = 'unknown';
    const userAgent = req.get('User-Agent') || '';
    const path = req.path.toLowerCase();

    if (path.includes('metaquest')) {
        deviceType = 'metaquest';
    } else if (path.includes('esp32')) {
        deviceType = 'esp32';
    } else if (userAgent.includes('Mozilla') || userAgent.includes('Chrome') || userAgent.includes('Safari')) {
        deviceType = 'browser';
    }

    logConnection(req, deviceType);
    next();
});

// ==================== ENDPOINTS DE MONITOREO ====================

// Endpoint para ver conexiones activas
app.get('/monitor/connections', (req, res) => {
    try {
        const now = new Date();
        const fiveMinutesAgo = new Date(now.getTime() - 5 * 60 * 1000);
        
        const activeLast5min = connectionLog.activeConnections.filter(conn => {
            const connTime = new Date(conn.timestamp);
            return connTime > fiveMinutesAgo;
        });

        res.json({
            status: 'success',
            active_connections_last_5min: activeLast5min.length,
            active_connections: activeLast5min,
            statistics: connectionLog.statistics,
            devices_connected: {
                metaquest: connectionLog.devices.metaquest.length,
                esp32: connectionLog.devices.esp32.length,
                browser: connectionLog.devices.browser.length,
                total_unique: connectionLog.statistics.uniqueDevices
            },
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        res.status(500).json({
            status: 'error',
            message: error.message
        });
    }
});

// Endpoint para ver historial completo
app.get('/monitor/history', (req, res) => {
    try {
        const limit = parseInt(req.query.limit) || 20;
        const history = connectionLog.connectionHistory.slice(0, limit);
        
        res.json({
            status: 'success',
            total_connections: connectionLog.connectionHistory.length,
            showing_last: limit,
            history: history,
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        res.status(500).json({
            status: 'error',
            message: error.message
        });
    }
});

// Endpoint para ver dispositivos conectados
app.get('/monitor/devices', (req, res) => {
    try {
        res.json({
            status: 'success',
            devices: connectionLog.devices,
            statistics: {
                total_unique_devices: connectionLog.statistics.uniqueDevices,
                by_type: {
                    metaquest: connectionLog.devices.metaquest.length,
                    esp32: connectionLog.devices.esp32.length,
                    browser: connectionLog.devices.browser.length,
                    unknown: connectionLog.devices.unknown.length
                }
            },
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        res.status(500).json({
            status: 'error',
            message: error.message
        });
    }
});

// Endpoint para limpiar conexiones antiguas
app.delete('/monitor/clear', (req, res) => {
    try {
        const hoursToKeep = parseInt(req.query.hours) || 1;
        const cutoffTime = new Date(Date.now() - hoursToKeep * 60 * 60 * 1000);
        
        const initialCount = connectionLog.connectionHistory.length;
        
        connectionLog.connectionHistory = connectionLog.connectionHistory.filter(conn => 
            new Date(conn.timestamp) > cutoffTime
        );
        
        const removedCount = initialCount - connectionLog.connectionHistory.length;
        
        res.json({
            status: 'success',
            message: `Historial limpiado`,
            removed_entries: removedCount,
            remaining_entries: connectionLog.connectionHistory.length,
            kept_hours: hoursToKeep,
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        res.status(500).json({
            status: 'error',
            message: error.message
        });
    }
});

// ==================== ENDPOINTS PRINCIPALES ====================

// Endpoint raíz - Página de prueba
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Endpoint para verificar estado del servidor
app.get('/status', (req, res) => {
    const now = new Date();
    const fiveMinutesAgo = new Date(now.getTime() - 5 * 60 * 1000);
    
    const activeLast5min = connectionLog.activeConnections.filter(conn => {
        const connTime = new Date(conn.timestamp);
        return connTime > fiveMinutesAgo;
    });

    res.json({
        status: 'online',
        server: 'HTTP Server for MetaQuest/ESP32',
        port: PORT,
        timestamp: new Date().toISOString(),
        uptime: process.uptime(),
        connections: {
            active_last_5min: activeLast5min.length,
            total_unique_devices: connectionLog.statistics.uniqueDevices,
            by_type: {
                metaquest: connectionLog.devices.metaquest.length,
                esp32: connectionLog.devices.esp32.length,
                browser: connectionLog.devices.browser.length
            }
        }
    });
});

// Endpoints para MetaQuest
app.post('/metaquest/send', (req, res) => {
    const data = req.body;
    const clientId = req.headers['client-id'] || 'unknown-metaquest';
    
    console.log(`📱 Datos de MetaQuest (${clientId}):`, data);
    
    res.json({
        status: 'success',
        message: 'Datos recibidos desde MetaQuest',
        received: data,
        client_id: clientId,
        timestamp: new Date().toISOString()
    });
});

app.get('/metaquest/receive', (req, res) => {
    res.json({
        status: 'success',
        message: 'Datos para MetaQuest',
        data_available: false,
        timestamp: new Date().toISOString()
    });
});

// Almacén para comandos pendientes
let pendingCommands = {
    metaquest: null,
    esp32: null
};

// Endpoint para enviar comandos al MetaQuest
app.post('/metaquest/command', (req, res) => {
    const { intensity, duration, pattern } = req.body;
    
    // Validar intensidad (1-10)
    if (intensity && (intensity < 1 || intensity > 10)) {
        return res.status(400).json({
            status: 'error',
            message: 'Intensidad debe ser entre 1 y 10'
        });
    }

    const command = {
        id: Math.random().toString(36).substr(2, 9),
        intensity: intensity || 5, // Default 5
        duration: duration || 1000, // Default 1 segundo
        pattern: pattern || 'continuous',
        timestamp: new Date().toISOString(),
        acknowledged: false
    };

    pendingCommands.metaquest = command;
    
    console.log(`🎮 Comando enviado a MetaQuest:`, command);
    
    res.json({
        status: 'success',
        message: 'Comando enviado al MetaQuest',
        command: command
    });
});

// Endpoint para que el MetaQuest consulte comandos
app.get('/metaquest/command', (req, res) => {
    const command = pendingCommands.metaquest;
    
    if (command && !command.acknowledged) {
        // Marcar como leído
        pendingCommands.metaquest.acknowledged = true;
        
        console.log(`📱 MetaQuest recibió comando: ${command.intensity}/10`);
        
        res.json({
            status: 'success',
            command_available: true,
            command: command
        });
    } else {
        res.json({
            status: 'success', 
            command_available: false,
            message: 'No hay comandos pendientes'
        });
    }
});

// Endpoint para limpiar comandos viejos
app.delete('/metaquest/command', (req, res) => {
    pendingCommands.metaquest = null;
    res.json({
        status: 'success',
        message: 'Comandos limpiados'
    });
});

// Endpoints para ESP32
app.post('/esp32/send', (req, res) => {
    const data = req.body;
    const clientId = req.headers['client-id'] || 'unknown-esp32';
    
    console.log(`🔌 Datos de ESP32 (${clientId}):`, data);
    
    res.json({
        status: 'success',
        message: 'Datos recibidos desde ESP32',
        received: data,
        client_id: clientId,
        timestamp: new Date().toISOString()
    });
});

app.get('/esp32/receive', (req, res) => {
    res.json({
        status: 'success',
        message: 'Datos para ESP32',
        data_available: false,
        timestamp: new Date().toISOString()
    });
});

// Endpoint para enviar comando al ESP32 (desde UI/MetaQuest/otro)
app.post('/esp32/command', (req, res) => {
    const { action, value, priority } = req.body;

    // Validaciones simples
    if (priority && (priority < 0 || priority > 10)) {
        return res.status(400).json({
            status: 'error',
            message: 'Priority debe estar entre 0 y 10'
        });
    }

    const command = {
        id: Math.random().toString(36).substr(2, 9),
        action: action || 'noop',
        value: value || null,
        priority: priority || 5,
        timestamp: new Date().toISOString(),
        acknowledged: false
    };

    pendingCommands.esp32 = command;

    console.log(`🔌 Comando enviado a ESP32:`, command);

    res.json({
        status: 'success',
        message: 'Comando enviado al ESP32',
        command: command
    });
});

// Endpoint para que la ESP32 consulte comandos (long-polling simple)
app.get('/esp32/command', (req, res) => {
    const command = pendingCommands.esp32;

    if (command && !command.acknowledged) {
        // marcar como leído para no devolverlo de nuevo
        pendingCommands.esp32.acknowledged = true;
        console.log(`🔌 ESP32 recibió comando: ${command.action}`);
        res.json({
            status: 'success',
            command_available: true,
            command: command
        });
    } else {
        res.json({
            status: 'success',
            command_available: false,
            message: 'No hay comandos pendientes'
        });
    }
});

// Endpoint para limpiar comandos viejos de ESP32
app.delete('/esp32/command', (req, res) => {
    pendingCommands.esp32 = null;
    res.json({
        status: 'success',
        message: 'Comandos ESP32 limpiados'
    });
});

// Endpoint para debug
app.get('/debug/data', (req, res) => {
    res.json({
        connection_log: connectionLog,
        timestamp: new Date().toISOString()
    });
});

// Endpoint 404 para rutas no encontradas
app.use('*', (req, res) => {
    res.status(404).json({
        status: 'error',
        message: 'Endpoint no encontrado',
        path: req.originalUrl
    });
});

// ==================== INICIAR SERVIDOR ====================

app.listen(PORT, '0.0.0.0', () => {
    console.log(`🚀 Servidor HTTP ejecutándose en puerto ${PORT}`);
    console.log(`📊 Endpoints de monitor:`);
    console.log(`   GET  /monitor/connections - Conexiones activas`);
    console.log(`   GET  /monitor/history     - Historial completo`);
    console.log(`   GET  /monitor/devices     - Dispositivos conectados`);
    console.log(`   GET  /status              - Estado del servidor`);
    console.log(`   GET  /debug/data          - Todos los datos`);
});