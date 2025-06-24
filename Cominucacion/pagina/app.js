// app.js

// --- Elementos del DOM (del header) ---
const connectButton = document.getElementById('connectButton');
const statusDisplay = document.getElementById('status');
const rawDataOutput = document.getElementById('rawDataOutput');

// --- Variables Globales ---
const MAX_CHART_POINTS = 50;
let port, reader, writer;
let keepReading = true, lineBuffer = '', isConnected = false, firstFix = true;

// Three.js
let scene, camera, renderer, cylinder, ambientLight, directionalLight;
const initialQuaternion = new THREE.Quaternion(); // Para resetear orientación
let lastTimestamp = 0; // Para cálculo de delta en rotación

// Chart.js
let tempChart, pressureChart, humidityChart, speedSatChart; // Instancias de Chart
let tempCanvasCtx, pressureCanvasCtx, humidityCanvasCtx, speedSatCanvasCtx; // Contextos de los canvas
const chartLabels = [];
const tempData = { mpu: [], bmp: [], dht: [] };
const pressureData = [], humidityData = [], speedData = [], satsData = [];

// Leaflet
let map, marker;

// GoldenLayout
let myLayout;

// --- Configuración de GoldenLayout ---
const goldenLayoutConfig = {
    settings: {
        hasHeaders: true, constrainDragToContainer: true, reorderEnabled: true,
        selectionEnabled: false, popoutWholeStack: false, blockedPopoutsThrowError: true,
        closePopoutsOnUnload: true, showPopoutIcon: false, showMaximiseIcon: true,
        showCloseIcon: false,
    },
    dimensions: {
        borderWidth: 5, minItemHeight: 150, minItemWidth: 200, headerHeight: 28,
        dragProxyWidth: 300, dragProxyHeight: 200
    },
    content: [{
        type: 'row',
        content: [
            { // Columna Izquierda (Modelo 3D y Mapa)
                type: 'column', width: 30,
                content: [
                    { type: 'component', componentName: 'model3DComponent', title: 'Orientación 3D', isClosable: false },
                    { type: 'component', componentName: 'mapComponent', title: 'Posición GPS', isClosable: false }
                ]
            },
            { // Columna Derecha (Gráficos)
                type: 'column', width: 70,
                content: [
                    { // Stack Superior de Gráficos
                        type: 'stack',
                        content: [
                            { type: 'component', componentName: 'tempChartComponent', title: 'Temperaturas', isClosable: false },
                            { type: 'component', componentName: 'pressureChartComponent', title: 'Presión', isClosable: false }
                        ]
                    },
                    { // Stack Inferior de Gráficos
                        type: 'stack',
                        content: [
                            { type: 'component', componentName: 'humidityChartComponent', title: 'Humedad', isClosable: false },
                            { type: 'component', componentName: 'speedSatChartComponent', title: 'GPS Vel/Sat', isClosable: false }
                        ]
                    }
                ]
            }
        ]
    }]
};

// --- Inicialización General ---
function init() {
    console.log("Inicializando aplicación...");
    if (!("serial" in navigator)) {
        statusDisplay.textContent = "Error: Web Serial API no soportada.";
        connectButton.disabled = true;
        return;
    }
    setupEventListeners();
    initGoldenLayout();
    animate(); // Inicia el bucle de renderizado de Three.js
    console.log("Inicialización completa.");
}

function initGoldenLayout() {
    const layoutContainer = document.getElementById('layoutContainer');
    if (!layoutContainer) {
        console.error("Contenedor de GoldenLayout no encontrado!");
        return;
    }
    myLayout = new GoldenLayout(goldenLayoutConfig, layoutContainer);

    // Registrar Componente: Modelo 3D
    myLayout.registerComponent('model3DComponent', function (container, componentState) {
        const panelRoot = container.getElement()[0];
        panelRoot.classList.add('panel-content');

        const titleEl = document.createElement('h3');
        titleEl.textContent = 'Orientación (Giroscopio)'; // Título dentro del panel
        panelRoot.appendChild(titleEl);

        const threeDiv = document.createElement('div');
        threeDiv.style.width = '100%';
        threeDiv.style.height = 'calc(100% - 40px)'; // Espacio para el título
        threeDiv.style.flexGrow = '1';
        panelRoot.appendChild(threeDiv);

        initThreeJS(threeDiv);
        container.on('resize', () => onPanelResizeThree(threeDiv));
    });

    // Registrar Componente: Mapa
    myLayout.registerComponent('mapComponent', function (container, componentState) {
        const panelRoot = container.getElement()[0];
        panelRoot.classList.add('panel-content');

        const titleEl = document.createElement('h3');
        titleEl.textContent = 'Posición GPS';
        panelRoot.appendChild(titleEl);

        const mapDiv = document.createElement('div');
        mapDiv.style.width = '100%';
        mapDiv.style.height = 'calc(100% - 40px)';
        mapDiv.style.flexGrow = '1';
        panelRoot.appendChild(mapDiv);

        initMap(mapDiv);
        container.on('resize', () => { if (map) map.invalidateSize(); });
    });

    // Registrar Componente: Gráfico de Temperatura
    myLayout.registerComponent('tempChartComponent', function (container, componentState) {
        const panelRoot = container.getElement()[0];
        panelRoot.classList.add('panel-content');
        // El título del panel de GoldenLayout es suficiente, no añadimos h3 interno
        const chartWrapper = document.createElement('div');
        chartWrapper.className = 'chart-wrapper';
        const canvas = document.createElement('canvas');
        chartWrapper.appendChild(canvas);
        panelRoot.appendChild(chartWrapper);
        tempCanvasCtx = canvas.getContext('2d');
        initTempChart();
        container.on('resize', () => { if (tempChart) tempChart.resize(); });
    });

    // Registrar Componente: Gráfico de Presión
    myLayout.registerComponent('pressureChartComponent', function (container, componentState) {
        const panelRoot = container.getElement()[0];
        panelRoot.classList.add('panel-content');
        const chartWrapper = document.createElement('div');
        chartWrapper.className = 'chart-wrapper';
        const canvas = document.createElement('canvas');
        chartWrapper.appendChild(canvas);
        panelRoot.appendChild(chartWrapper);
        pressureCanvasCtx = canvas.getContext('2d');
        initPressureChart();
        container.on('resize', () => { if (pressureChart) pressureChart.resize(); });
    });

    // Registrar Componente: Gráfico de Humedad
    myLayout.registerComponent('humidityChartComponent', function (container, componentState) {
        const panelRoot = container.getElement()[0];
        panelRoot.classList.add('panel-content');
        const chartWrapper = document.createElement('div');
        chartWrapper.className = 'chart-wrapper';
        const canvas = document.createElement('canvas');
        chartWrapper.appendChild(canvas);
        panelRoot.appendChild(chartWrapper);
        humidityCanvasCtx = canvas.getContext('2d');
        initHumidityChart();
        container.on('resize', () => { if (humidityChart) humidityChart.resize(); });
    });

    // Registrar Componente: Gráfico de Velocidad/Satélites
    myLayout.registerComponent('speedSatChartComponent', function (container, componentState) {
        const panelRoot = container.getElement()[0];
        panelRoot.classList.add('panel-content');
        const chartWrapper = document.createElement('div');
        chartWrapper.className = 'chart-wrapper';
        const canvas = document.createElement('canvas');
        chartWrapper.appendChild(canvas);
        panelRoot.appendChild(chartWrapper);
        speedSatCanvasCtx = canvas.getContext('2d');
        initSpeedSatChart();
        container.on('resize', () => { if (speedSatChart) speedSatChart.resize(); });
    });

    myLayout.init();
    window.addEventListener('resize', () => {
        if (myLayout) myLayout.updateSize(layoutContainer.offsetWidth, layoutContainer.offsetHeight);
    });
    setTimeout(() => { // Forzar un update inicial
        if (myLayout && layoutContainer) myLayout.updateSize(layoutContainer.offsetWidth, layoutContainer.offsetHeight);
    }, 150);
}

// --- Inicialización de Componentes Específicos ---
function initThreeJS(containerDiv) {
    if (!containerDiv) { console.error("Contenedor Three.js no provisto."); return; }
    console.log("Inicializando Three.js en", containerDiv);

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x2a2a2a);

    const rect = containerDiv.getBoundingClientRect();
    const aspect = (rect.width > 0 && rect.height > 0) ? rect.width / rect.height : 1;
    camera = new THREE.PerspectiveCamera(60, aspect, 0.1, 1000); // FOV ajustado
    camera.position.set(0, 0.5, 4); // Posición ajustada para ver mejor el cilindro

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(rect.width, rect.height);
    containerDiv.innerHTML = ''; // Limpiar por si acaso
    containerDiv.appendChild(renderer.domElement);

    ambientLight = new THREE.AmbientLight(0x707070); // Luz ambiente un poco más brillante
    scene.add(ambientLight);
    directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(1, 2, 1.5).normalize();
    scene.add(directionalLight);

    const geometry = new THREE.CylinderGeometry(0.4, 0.4, 1.5, 32); // Ajustar tamaño
    const material = new THREE.MeshStandardMaterial({ color: 0x4682B4, metalness: 0.4, roughness: 0.5 });
    cylinder = new THREE.Mesh(geometry, material);
    scene.add(cylinder);
    resetCylinderOrientation(); // Establecer orientación inicial
    console.log("Three.js inicializado.");
}

function onPanelResizeThree(containerDiv) {
    if (!camera || !renderer || !containerDiv) return;
    const rect = containerDiv.getBoundingClientRect();
    if (rect.width > 0 && rect.height > 0) {
        camera.aspect = rect.width / rect.height;
        camera.updateProjectionMatrix();
        renderer.setSize(rect.width, rect.height);
    }
}

function initMap(containerDiv) {
    if (!containerDiv) { console.error("Contenedor Leaflet no provisto."); return; }
    console.log("Inicializando Leaflet en", containerDiv);
    try {
        map = L.map(containerDiv, { attributionControl: false, zoomControl: false }).setView([0, 0], 2);
        L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
            attribution: '© OSM © CARTO', subdomains: 'abcd', maxZoom: 19
        }).addTo(map);
        L.control.attribution({ prefix: '' }).addTo(map); // Atribución simple
        L.control.zoom({ position: 'bottomright' }).addTo(map); // Controles de zoom

        marker = L.marker([0, 0]).addTo(map).bindPopup('Esperando fix GPS...');
        map.invalidateSize();
        console.log("Leaflet inicializado.");
    } catch (error) {
        console.error("Error inicializando Leaflet:", error);
        containerDiv.innerHTML = "Error al cargar mapa.";
    }
}

function getCommonChartOptions() {
    Chart.defaults.color = '#e0e0e0';
    Chart.defaults.borderColor = '#444'; // Líneas de rejilla más oscuras
    return {
        responsive: true, maintainAspectRatio: false,
        scales: {
            x: { display: true, ticks: { autoSkip: true, maxTicksLimit: 8, color: '#b0b0b0', font: {size: 10} }, grid: { color: '#383838'} },
            y: { beginAtZero: false, ticks: { color: '#b0b0b0', font: {size: 10} }, grid: { color: '#383838'} }
        },
        animation: { duration: 100 },
        plugins: {
            legend: { display: true, position: 'top', labels: { color: '#e0e0e0', font: {size: 11}, boxWidth: 12, padding: 8 } },
            title: { display: false } // El título del panel de GL es suficiente
        }
    };
}

function initTempChart() {
    if (!tempCanvasCtx) return;
    tempChart = new Chart(tempCanvasCtx, {
        type: 'line',
        data: { labels: chartLabels, datasets: [
            { label: 'MPU', data: tempData.mpu, borderColor: '#FF6384', tension: 0.2, pointRadius: 0, borderWidth: 1.5 },
            { label: 'BMP', data: tempData.bmp, borderColor: '#36A2EB', tension: 0.2, pointRadius: 0, borderWidth: 1.5 },
            { label: 'DHT', data: tempData.dht, borderColor: '#4BC0C0', tension: 0.2, pointRadius: 0, borderWidth: 1.5 }
        ]},
        options: getCommonChartOptions()
    });
}
function initPressureChart() {
    if (!pressureCanvasCtx) return;
    pressureChart = new Chart(pressureCanvasCtx, {
        type: 'line',
        data: { labels: chartLabels, datasets: [{ label: 'Presión (Pa)', data: pressureData, borderColor: '#FF9F40', tension: 0.2, pointRadius: 0, borderWidth: 1.5 }] },
        options: getCommonChartOptions()
    });
}
function initHumidityChart() {
    if (!humidityCanvasCtx) return;
    const opts = getCommonChartOptions();
    opts.scales.y.min = 0; opts.scales.y.max = 100; // Específico para humedad
    humidityChart = new Chart(humidityCanvasCtx, {
        type: 'line',
        data: { labels: chartLabels, datasets: [{ label: 'Humedad (%)', data: humidityData, borderColor: '#9966FF', tension: 0.2, pointRadius: 0, borderWidth: 1.5 }] },
        options: opts
    });
}
function initSpeedSatChart() {
    if (!speedSatCanvasCtx) return;
    const opts = getCommonChartOptions();
    opts.scales.ySpeed = { type: 'linear', display: true, position: 'left', title: { display: false }, ticks:{color:'#b0b0b0', font: {size: 10}}, grid:{color:'#383838'} };
    opts.scales.ySats = { type: 'linear', display: true, position: 'right', title: { display: false }, beginAtZero: true, ticks:{color:'#b0b0b0', stepSize: 1, font: {size: 10}}, grid: { drawOnChartArea: false } };
    delete opts.scales.y; // Eliminar eje y genérico

    speedSatChart = new Chart(speedSatCanvasCtx, {
        type: 'line',
        data: { labels: chartLabels, datasets: [
            { label: 'Vel (m/s)', data: speedData, borderColor: '#FFCD56', yAxisID: 'ySpeed', tension: 0.2, pointRadius: 0, borderWidth: 1.5 },
            { label: 'Sats', data: satsData, backgroundColor: 'rgba(201, 203, 207, 0.4)', borderColor: '#C9CBCF', yAxisID: 'ySats', type: 'bar' }
        ]},
        options: opts
    });
}

// --- Lógica de Conexión Serial ---
function setupEventListeners() {
    connectButton.addEventListener('click', async () => {
        if (isConnected) await disconnect(); else await connect();
    });
    navigator.serial?.addEventListener('disconnect', (event) => {
        if (port && event.target === port) {
            console.log("Puerto desconectado externamente.");
            handleDisconnectionUI("Desconectado (Externamente)");
        }
    });
}

async function connect() {
    try {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
        isConnected = true;
        statusDisplay.textContent = 'Estado: Conectando e importando "datos"...';
        connectButton.textContent = 'Desconectar';
        rawDataOutput.textContent = '(Esperando inicio de datos...)';
        keepReading = true; lineBuffer = ''; firstFix = true;
        resetCylinderOrientation(); // Resetear orientación al conectar

        if (port.writable) {
            writer = port.writable.getWriter();
            //const commandToSend = "import datos\r\n";
            const commandToSend = "import datos\r\ndatos.print_data()\r\n";
            const dataEncoded = new TextEncoder().encode(commandToSend);
            try {
                await writer.write(dataEncoded);
                console.log(`Comando enviado: ${commandToSend.trim()}`);
                statusDisplay.textContent = 'Estado: Conectado, "datos" importado.';
            } catch (writeError) {
                console.error("Error al enviar comando:", writeError);
                statusDisplay.textContent = 'Error al importar "datos".';
                await disconnect(); return;
            } finally {
                if (writer) { writer.releaseLock(); writer = undefined; }
            }
        } else {
            console.error("Puerto no escribible."); statusDisplay.textContent = 'Error: Puerto no escribible.';
            await disconnect(); return;
        }
        readLoop();
    } catch (err) {
        console.error("Error al conectar:", err); statusDisplay.textContent = `Error: ${err.message}`;
        isConnected = false; port = undefined; writer = undefined;
    }
}

async function disconnect() {
    console.log("Iniciando desconexión...");
    keepReading = false;
    if (port && port.writable) {
        try {
            writer = port.writable.getWriter();
            await writer.write(new Uint8Array([3])); // Ctrl+C
            console.log("Ctrl+C enviado.");
        } catch (e) { console.warn("Error enviando Ctrl+C:", e); }
        finally { if (writer) { writer.releaseLock(); writer = undefined; } }
    }
    if (reader) {
        try { await reader.cancel("Desconexión solicitada"); }
        catch (e) { console.warn("Error cancelando lector:", e); }
    }
    if (port) {
        try { await port.close(); console.log("Puerto cerrado."); }
        catch (e) { console.error("Error cerrando puerto:", e); }
    }
    handleDisconnectionUI();
    console.log("Desconexión completada.");
}

function handleDisconnectionUI(message = "Desconectado") {
    if (isConnected || statusDisplay.textContent.startsWith("Estado: Conectado")) { // Capturar más casos
        statusDisplay.textContent = `Estado: ${message}`;
        connectButton.textContent = 'Conectar al Puerto Serial';
        isConnected = false; port = undefined; reader = undefined; writer = undefined;
    }
}

async function readLoop() {
    if (!port || !port.readable) { handleDisconnectionUI("Error de Puerto"); return; }
    const textDecoder = new TextDecoderStream();
    const readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
    reader = textDecoder.readable.getReader();
    console.log("Iniciando bucle de lectura...");
    try {
        while (keepReading) {
            const { value, done } = await reader.read();
            if (done) { console.log("Stream cerrado."); break; }
            lineBuffer += value;
            let newlineIndex;
            while ((newlineIndex = lineBuffer.indexOf('\n')) >= 0) {
                const line = lineBuffer.slice(0, newlineIndex).trim();
                lineBuffer = lineBuffer.slice(newlineIndex + 1);
                if (line) {
                    rawDataOutput.textContent = `Raw: ${line}`;
                    parseAndProcessData(line);
                }
            }
        }
    } catch (error) {
        if (error.name !== 'AbortError') console.error('Error lectura:', error);
    } finally {
        if (reader) { try { reader.releaseLock(); } catch(e) {} reader = undefined; }
        // La desconexión formal y actualización UI la maneja disconnect() o el evento 'disconnect' de Web Serial
        // Pero si el bucle termina por otra razón, actualizamos UI:
        if (keepReading) { // Si keepReading es true, el bucle terminó inesperadamente
            handleDisconnectionUI("Desconectado (Stream finalizado)");
        }
    }
}

// --- Procesamiento de Datos y Actualización de UI ---
function parseAndProcessData(line) {
    const parts = line.split(',');
    if (parts.length !== 15) {
        console.warn(`Línea ignorada (partes ${parts.length}): ${line}`); return;
    }
    try {
        const data = {
            ax: parseFloat(parts[0]), ay: parseFloat(parts[1]), az: parseFloat(parts[2]),
            gx: parseFloat(parts[3]), gy: parseFloat(parts[4]), gz: parseFloat(parts[5]),
            temp_mpu: parseFloat(parts[6]), temp_bmp: parseFloat(parts[7]),
            presion: parseFloat(parts[8]), temp_dht: parseFloat(parts[9]),
            hum: parseFloat(parts[10]), lat: parseFloat(parts[11]),
            lon: parseFloat(parts[12]), sats: parseInt(parts[13], 10),
            speed: parseFloat(parts[14])
        };
        for (const key in data) if (isNaN(data[key])) { console.warn(`NaN para ${key}`); return; }

        updateCylinderRotation(data.gx, data.gy, data.gz);
        updateCharts(data);
        updateMap(data.lat, data.lon);
    } catch (e) { console.error(`Error parseando: ${line}`, e); }
}

function updateCylinderRotation(gx, gy, gz) {
    if (!cylinder) return;
    const currentTimestamp = performance.now();
    if (lastTimestamp === 0) { lastTimestamp = currentTimestamp; return; }
    const deltaTime = (currentTimestamp - lastTimestamp) / 1000.0;
    lastTimestamp = currentTimestamp;

    const deltaRadX = (gy * Math.PI / 180.0) * deltaTime;
    const deltaRadY = (gx * Math.PI / 180.0) * deltaTime;
    const deltaRadZ = (gz * Math.PI / 180.0) * deltaTime;

    const qDelta = new THREE.Quaternion();
    const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), deltaRadX);
    const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), deltaRadY);
    const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), deltaRadZ);

    qDelta.multiply(qz).multiply(qx).multiply(qy); // O el orden que funcione mejor
    cylinder.quaternion.multiplyQuaternions(qDelta, cylinder.quaternion).normalize();
}

function resetCylinderOrientation() {
    if (cylinder) cylinder.quaternion.copy(initialQuaternion);
    lastTimestamp = 0;
}

function updateCharts(data) {
    const nowLabel = chartLabels.length.toString(); // Etiqueta simple
    chartLabels.push(nowLabel);
    tempData.mpu.push(data.temp_mpu); tempData.bmp.push(data.temp_bmp); tempData.dht.push(data.temp_dht);
    pressureData.push(data.presion); humidityData.push(data.hum);
    speedData.push(data.speed); satsData.push(data.sats);

    if (chartLabels.length > MAX_CHART_POINTS) {
        chartLabels.shift();
        tempData.mpu.shift(); tempData.bmp.shift(); tempData.dht.shift();
        pressureData.shift(); humidityData.shift();
        speedData.shift(); satsData.shift();
    }
    if (tempChart) tempChart.update('none'); // 'none' para evitar re-renderizado excesivo de animación
    if (pressureChart) pressureChart.update('none');
    if (humidityChart) humidityChart.update('none');
    if (speedSatChart) speedSatChart.update('none');
}

function updateMap(lat, lon) {
    if (!map || !marker) return;
    if (lat !== 0.0 && lon !== 0.0) {
        const newLatLng = [lat, lon];
        marker.setLatLng(newLatLng);
        marker.setPopupContent(`Lat: ${lat.toFixed(5)}, Lon: ${lon.toFixed(5)}`);
        if (!map.getBounds().contains(newLatLng)) marker.openPopup(); // Abrir solo si está fuera de vista

        if (firstFix) {
            map.setView(newLatLng, 15); firstFix = false;
        }
    } else {
        marker.setPopupContent("Esperando fix GPS...");
    }
}

// --- Bucle de Animación Three.js ---
function animate() {
    requestAnimationFrame(animate);
    if (renderer && scene && camera) {
        renderer.render(scene, camera);
    }
}

// --- Iniciar la aplicación ---
document.addEventListener('DOMContentLoaded', init);
