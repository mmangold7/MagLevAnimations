import * as three from 'three';
import { TextGeometry } from 'three/addons/geometries/TextGeometry.js';
import { FontLoader } from 'three/addons/loaders/FontLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { elementSpectra, getElementSymbol, fraunhoferLines } from './elementSpectra';

var scene, camera, renderer, controls, fontLoader;
var simulationGroup, halfBoundingBoxHeight;
var analogClock, digitalClock, ambientLight, ceilingLight;

const littleStripeWidth = 20;
const littleStripeHeight = 5;
const labelHeight = 5;
const bigCellPadding = 2;
const littleCellPadding = 2;
const cellWidth = littleStripeWidth + littleCellPadding;
const cellHeight = littleStripeHeight + labelHeight + littleCellPadding;
const wallWidth = 200;
const wallHeight = 80;
const bigStripeHeight = wallHeight / 3;
const numColumns = Math.floor(wallWidth / cellWidth);
const numRows = Math.floor(bigStripeHeight / cellHeight);

//public api
function initializeThreeJs(inputDimensions, drawingParameters) {
    const canvas = document.getElementById('threejs-canvas');
    if (!canvas) {
        console.error('Canvas element not found');
        return;
    }

    halfBoundingBoxHeight = inputDimensions.height / 2;
    renderer = new three.WebGLRenderer({ canvas: canvas, antialias: true });
    scene = new three.Scene();
    camera = new three.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(0, 1, 2);
    camera.lookAt(new three.Vector3(0, 0, 0));

    fontLoader = new FontLoader();

    addLights(drawingParameters.ceilingLightLevel, drawingParameters.ambientLightLevel);
    addFloor();
    addWalls();
    addCeiling();

    addTable();

    //addBackWallRainbow();

    const visibleSpectrumTexture = generateVisibleSpectrumTexture();
    addSpectrumStripe(visibleSpectrumTexture, 0); // Visible spectrum on top
    const solarSpectrumTexture = generateSolarSpectrumTexture();
    addSpectrumStripe(solarSpectrumTexture, 1); // Solar spectrum in the middle
    drawElementSpectra();

    addDigitalClock();
    addAnalogClock();

    simulationGroup = new three.Group();
    addSimulationBox(inputDimensions);
    simulationGroup.position.set(0, 20.5 + 0.5, 0);
    scene.add(simulationGroup);

    setupOrbitControls();
    resetCameraToBoundingBox(inputDimensions);

    animate();
};
function updateThreeJsScene(state, drawingParameters) {
    const magnets = state.magnets;
    const gravityField = state.gravityFieldData;
    const magneticField = state.magneticFieldData;
    const timeSinceStart = state.timeSinceStart;

    if (ambientLight) ambientLight.intensity = drawingParameters.ambientLightLevel;
    if (ceilingLight) ceilingLight.intensity = drawingParameters.ceilingLightLevel;

    clearMagnetsAndFields();

    if (magnets) {
        const magnetsArray = Array.isArray(magnets) ? magnets : [magnets];
        magnetsArray.forEach(magnet => {
            const magnetMesh = (drawingParameters.showCoils ? createLoopsMagnet(magnet) : createCylinderMagnet(magnet));
            simulationGroup.add(magnetMesh);
            addMagnetOrientationIndicator(magnet);
        });
    }

    if (gravityField && drawingParameters.showGravityField) {
        drawFieldVectors(gravityField, 0x00ff00, "arrows", "gravity");
    } else if (!drawingParameters.showGravityField) {
        clearFieldVectorsOfType("gravity");
    }

    if (magneticField && drawingParameters.showMagneticField) {
        drawFieldVectors(magneticField, 0xff0000, drawingParameters.fieldDrawingStyle, "magnetic");
    } else if (!drawingParameters.showMagneticField) {
        clearFieldVectorsOfType("magnetic");
    }

    if (analogClock) analogClock.updateTime(timeSinceStart);
    if (digitalClock) digitalClock.updateTime(timeSinceStart);
}

//init helpers
function resetCameraToBoundingBox(dimensions) {
    const boundingBoxCenter = new three.Vector3(0, 20.5 + halfBoundingBoxHeight, 0);
    const maxDimension = Math.max(dimensions.width, dimensions.height, dimensions.depth);
    const fov = camera.fov * (Math.PI / 180);
    const cameraZ = Math.abs(maxDimension / 2 * Math.tan(fov / 2) * 2);
    camera.position.set(boundingBoxCenter.x, boundingBoxCenter.y, boundingBoxCenter.z + cameraZ);
    controls.target.copy(boundingBoxCenter);
    controls.update();
}
function setupOrbitControls() {
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.maxPolarAngle = Math.PI / 2;
    controls.zoomSpeed = 1.0;
    controls.target.set(0, 0, 0);
    controls.update();
}
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    render();
}
function render() {
    if (resizeRendererToDisplaySize(renderer)) {
        const canvas = renderer.domElement;
        camera.aspect = canvas.clientWidth / canvas.clientHeight;
        camera.updateProjectionMatrix();
    }
    renderer.render(scene, camera);
}
function resizeRendererToDisplaySize(renderer) {
    const canvas = renderer.domElement;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const needResize = canvas.width !== width || canvas.height !== height;
    if (needResize) {
        renderer.setSize(width, height, false);
    }
    return needResize;
}

//drawing helpers
function clearMagnetsAndFields() {
    const toRemove = [];
    simulationGroup.traverse((child) => {
        if (child.name === "magnetObject" || child.name.startsWith("vectorObject") || child.name.startsWith("lineObject")) {
            toRemove.push(child);
        }
    });
    toRemove.forEach((object) => {
        simulationGroup.remove(object);
    });
}
function clearFieldVectorsOfType(fieldType) {
    const toRemove = [];
    simulationGroup.traverse((child) => {
        if (child.name.startsWith(fieldType)) {
            toRemove.push(child);
        }
    });
    toRemove.forEach((object) => {
        simulationGroup.remove(object);
    });
}

//drawing the room/scene
function addLights(ceilingLightLevel, ambientLightLevel) {
    ceilingLight = new three.PointLight(0xffffff, ceilingLightLevel, 300, 0);
    ceilingLight.position.set(0, 70, 0);
    scene.add(ceilingLight);

    ambientLight = new three.AmbientLight(0xffffff, ambientLightLevel);
    scene.add(ambientLight);
}
function addFloor() {
    const tileSize = 10;
    const tilesPerSide = 20;
    const floorGeometry = new three.PlaneGeometry(tileSize, tileSize);

    for (let x = 0; x < tilesPerSide; x++) {
        for (let z = 0; z < tilesPerSide; z++) {
            const isWhite = (x + z) % 2 === 0;
            const floorMaterial = new three.MeshLambertMaterial({
                color: isWhite ? 0xFFFFFF : 0x000000,
                side: three.DoubleSide
            });

            const floorTile = new three.Mesh(floorGeometry, floorMaterial);
            floorTile.rotation.x = -Math.PI / 2;
            floorTile.position.set(x * tileSize - (tilesPerSide / 2) * tileSize + tileSize / 2, 0, z * tileSize - (tilesPerSide / 2) * tileSize + tileSize / 2);

            scene.add(floorTile);
        }
    }
}
function addWalls() {
    const wallMaterial = new three.MeshLambertMaterial({
        color: 0x555555,
        side: three.DoubleSide
    });

    const backWallGeometry = new three.PlaneGeometry(200, 80);
    const frontWallGeometry = new three.PlaneGeometry(200, 80);
    const sideWallGeometry = new three.PlaneGeometry(200, 80);

    const walls = [
        { geometry: backWallGeometry, position: new three.Vector3(0, 40, -100) },
        { geometry: frontWallGeometry, position: new three.Vector3(0, 40, 100) },
        { geometry: sideWallGeometry, position: new three.Vector3(-100, 40, 0), rotation: Math.PI / 2 },
        { geometry: sideWallGeometry, position: new three.Vector3(100, 40, 0), rotation: Math.PI / 2 }
    ];

    walls.forEach((wallSpec) => {
        const wall = new three.Mesh(wallSpec.geometry, wallMaterial);
        wall.position.copy(wallSpec.position);
        if (wallSpec.rotation) wall.rotation.y = wallSpec.rotation;
        scene.add(wall);
    });
}
function addCeiling() {
    const ceilingGeometry = new three.PlaneGeometry(200, 200);
    const ceilingMaterial = new three.MeshStandardMaterial({ color: 0x808080, side: three.DoubleSide, roughness: 0.4 });
    const ceiling = new three.Mesh(ceilingGeometry, ceilingMaterial);
    ceiling.rotation.x = Math.PI / 2;
    ceiling.position.y = 80;
    scene.add(ceiling);
}
function addTable() {
    const tableMaterial = new three.MeshStandardMaterial({
        color: 0x202020,
        roughness: 0.5,
        metalness: 0.1
    });
    const tableTop = new three.Mesh(new three.BoxGeometry(40, 1, 20), tableMaterial);
    tableTop.position.set(0, 20.5, 0);
    scene.add(tableTop);

    const legMaterial = new three.MeshStandardMaterial({
        color: 0x909090,
        roughness: 0.5,
        metalness: 0.1
    });
    const legPositions = [
        new three.Vector3(-19.5, 10, -9.5),
        new three.Vector3(19.5, 10, -9.5),
        new three.Vector3(-19.5, 10, 9.5),
        new three.Vector3(19.5, 10, 9.5)
    ];
    legPositions.forEach(position => {
        const leg = new three.Mesh(new three.CylinderGeometry(0.5, 0.5, 20, 32), legMaterial);
        leg.position.copy(position);
        scene.add(leg);
    });
}
function addBackWallRainbow() {
    const colors = ['#FF0000', '#FF7F00', '#FFFF00', '#00FF00', '#0000FF', '#4B0082', '#9400D3'];
    const panelWidth = 200 / colors.length;

    colors.forEach((color, i) => {
        const panelMaterial = new three.MeshLambertMaterial({ color });
        const panelGeometry = new three.PlaneGeometry(panelWidth, 80);
        const panel = new three.Mesh(panelGeometry, panelMaterial);
        panel.position.set(-100 + panelWidth * i + panelWidth / 2, 40, -99.9);
        scene.add(panel);
    });
}

//drawing spectra
function addSpectrumStripe(texture, stripeIndex) {
    const paddedWidth = wallWidth - 2 * bigCellPadding;
    const paddedHeight = bigStripeHeight - 2 * bigCellPadding;
    const geometry = new three.PlaneGeometry(paddedWidth, paddedHeight);
    const material = new three.MeshBasicMaterial({ map: texture, side: three.DoubleSide });
    const plane = new three.Mesh(geometry, material);
    const yPos = (wallHeight) - (bigStripeHeight / 2) - (stripeIndex * bigStripeHeight);
    plane.position.set(0, yPos, -99.9);
    plane.scale.x = -1;
    scene.add(plane);
}
function drawElementSpectra() {
    elementSpectra.forEach((element, index) => {
        const column = index % numColumns;
        const row = Math.floor(index / numColumns);
        if (row < numRows) {
            addElementSpectrumStripe(element, column, row);
        }
    });
}
function addElementSpectrumStripe(element, column, row) {
    const xOffset = -wallWidth / 2 + cellWidth / 2;
    const yOffset = (wallHeight) - (bigStripeHeight / 2) - (2 * bigStripeHeight);

    const xPos = xOffset + column * cellWidth;
    const yPos = yOffset + row * cellHeight;

    const geometry = new three.PlaneGeometry(littleStripeWidth, littleStripeHeight);
    const texture = generateElementSpectrumTexture(element);
    const material = new three.MeshBasicMaterial({ map: texture, side: three.DoubleSide });
    const plane = new three.Mesh(geometry, material);
    plane.position.set(xPos, yPos - labelHeight / 2, -99.9);
    plane.scale.x = -1;
    scene.add(plane);

    const elementSymbol = getElementSymbol(element.name);
    addElementLabel(elementSymbol, xPos, yPos - littleStripeHeight - labelHeight);
}
function generateVisibleSpectrumTexture() {
    const width = 512;
    const height = 1;
    const size = width * height;
    const data = new Uint8Array(4 * size);

    for (let i = 0; i < size; i++) {
        const wavelength = 380 + (i / width) * (700 - 380);
        const rgb = wavelengthToRgb(wavelength);
        const stride = i * 4;
        data[stride] = rgb.r;
        data[stride + 1] = rgb.g;
        data[stride + 2] = rgb.b;
        data[stride + 3] = 255;
    }

    const texture = new three.DataTexture(data, width, height, three.RGBAFormat);
    texture.needsUpdate = true;
    return texture;
}
function generateSolarSpectrumTexture() {
    const width = 512;
    const height = 1;
    const size = width * height;
    const data = new Uint8Array(4 * size);

    for (let i = 0; i < size; i++) {
        const position = i / width;
        const wavelength = 380 + position * (700 - 380);

        const isFraunhoferLine = fraunhoferLines.some(lineWavelength =>
            Math.abs(wavelength - lineWavelength) < ((700 - 380) / width)
        );

        const rgb = isFraunhoferLine ? { r: 0, g: 0, b: 0 } : wavelengthToRgb(wavelength);
        const stride = i * 4;
        data[stride] = rgb.r;
        data[stride + 1] = rgb.g;
        data[stride + 2] = rgb.b;
        data[stride + 3] = 255;
    }

    const texture = new three.DataTexture(data, width, height, three.RGBAFormat);
    texture.needsUpdate = true;
    return texture;
}
function generateElementSpectrumTexture(element) {
    const width = 512;
    const height = 1;
    const size = width * height;
    const data = new Uint8Array(4 * size);

    for (let i = 0; i < size; i++) {
        const stride = i * 4;
        data[stride] = 0; // R
        data[stride + 1] = 0; // G
        data[stride + 2] = 0; // B
        data[stride + 3] = 255; // A
    }

    element.lines.forEach(line => {
        const wavelength = line[0];
        const intensity = line[1];

        const position = Math.floor(((wavelength - 380) / (780 - 380)) * width);

        if (position >= 0 && position < width) {
            const rgb = wavelengthToRgb(wavelength);
            const stride = position * 4;

            data[stride] = rgb.r;
            data[stride + 1] = rgb.g;
            data[stride + 2] = rgb.b;
            data[stride + 2] = 255 * intensity;
        }
    });

    const texture = new three.DataTexture(data, width, height, three.RGBAFormat);
    texture.needsUpdate = true;
    return texture;
}
function wavelengthToRgb(wavelength) {
    let gamma = 0.8;
    let intensityMax = 255;
    let factor;
    let red, green, blue;

    if ((wavelength >= 380) && (wavelength < 440)) {
        red = -(wavelength - 440) / (440 - 380);
        green = 0.0;
        blue = 1.0;
    } else if ((wavelength >= 440) && (wavelength < 490)) {
        red = 0.0;
        green = (wavelength - 440) / (490 - 440);
        blue = 1.0;
    } else if ((wavelength >= 490) && (wavelength < 510)) {
        red = 0.0;
        green = 1.0;
        blue = -(wavelength - 510) / (510 - 490);
    } else if ((wavelength >= 510) && (wavelength < 580)) {
        red = (wavelength - 510) / (580 - 510);
        green = 1.0;
        blue = 0.0;
    } else if ((wavelength >= 580) && (wavelength < 645)) {
        red = 1.0;
        green = -(wavelength - 645) / (645 - 580);
        blue = 0.0;
    } else if ((wavelength >= 645) && (wavelength <= 780)) {
        red = 1.0;
        green = 0.0;
        blue = 0.0;
    } else {
        red = 0.0;
        green = 0.0;
        blue = 0.0;
    }

    // Let the intensity fall off near the vision limits
    if ((wavelength >= 380) && (wavelength < 420)) {
        factor = 0.3 + 0.7 * (wavelength - 380) / (420 - 380);
    } else if ((wavelength >= 420) && (wavelength < 645)) {
        factor = 1.0;
    } else if ((wavelength >= 645) && (wavelength <= 780)) {
        factor = 0.3 + 0.7 * (780 - wavelength) / (780 - 645);
    } else {
        factor = 0.0;
    }

    const rgb = {
        r: adjustColor(red, factor, intensityMax, gamma),
        g: adjustColor(green, factor, intensityMax, gamma),
        b: adjustColor(blue, factor, intensityMax, gamma)
    };

    return rgb;
}
function adjustColor(color, factor, intensityMax, gamma) {
    if (color === 0.0) {
        return 0;
    } else {
        return Math.round(intensityMax * Math.pow(color * factor, gamma));
    }
}
function addElementLabel(text, xPos, yPos) {
    fontLoader.load('https://threejs.org/examples/fonts/helvetiker_regular.typeface.json', function (font) {
        const textGeo = new TextGeometry(text, {
            font: font,
            size: 2,
            height: 0.1
        });
        const textMaterial = new three.MeshBasicMaterial({ color: 0xffffff });
        const mesh = new three.Mesh(textGeo, textMaterial);
        mesh.position.set(xPos, yPos, -99.8);
        scene.add(mesh);
    });
}

//drawing clocks
function addDigitalClock() {
    digitalClock = new three.Group();

    digitalClock.updateTime = function () {
        console.warn('Font not loaded yet.');
    };

    fontLoader.load('https://threejs.org/examples/fonts/helvetiker_regular.typeface.json', function (font) {
        const geometry = new TextGeometry('00:00:00', {
            font: font,
            size: 10,
            height: 2,
            curveSegments: 12,
            bevelEnabled: true,
            bevelThickness: 0.1,
            bevelSize: 0.1,
            bevelSegments: 5
        });

        const material = new three.MeshBasicMaterial({ color: 0xff0000 });
        const mesh = new three.Mesh(geometry, material);
        mesh.position.set(-10, 0, 0);
        mesh.rotation.y = Math.PI / 2;
        mesh.position.set(-99.5, 40 - 5, 50 + 20);
        scene.add(mesh);

        digitalClock.updateTime = function (simulationTime) {
            const time = new Date(simulationTime * 1000);

            let hours = time.getUTCHours();
            let minutes = time.getUTCMinutes();
            let seconds = time.getUTCSeconds();

            if (hours > 12) {
                hours -= 12;
            }

            hours = String(hours).padStart(2, '0');
            minutes = String(minutes).padStart(2, '0');
            seconds = String(seconds).padStart(2, '0');

            const timeString = `${hours}:${minutes}:${seconds}`;

            mesh.geometry = new TextGeometry(timeString, {
                font: font,
                size: 10,
                height: 2,
                curveSegments: 12,
                bevelEnabled: true,
                bevelThickness: 0.1,
                bevelSize: 0.1,
                bevelSegments: 5
            });
        };
    });

    scene.add(digitalClock);
}
function addAnalogClock() {
    const radius = 30;
    const clockGeometry = new three.CircleGeometry(radius, 32);
    const clockMaterial = new three.MeshLambertMaterial({ color: 0xCCCCCC });
    analogClock = new three.Mesh(clockGeometry, clockMaterial);
    analogClock.position.set(-99.9, 40, -50);
    analogClock.rotation.y = Math.PI / 2;
    scene.add(analogClock);

    const hourHand = createClockHand(2, 20, 0x000000, 0.1);
    const minuteHand = createClockHand(1.5, 25, 0x000000, 0.2);
    const secondHand = createClockHand(1, 30, 0xFF0000, 0.3);

    analogClock.add(hourHand);
    analogClock.add(minuteHand);
    analogClock.add(secondHand);

    analogClock.updateTime = function (simulationTime) {
        const hours = (simulationTime / 3600) % 12;
        const minutes = (simulationTime % 3600) / 60;
        const seconds = simulationTime % 60;

        hourHand.rotation.z = -hours * (Math.PI / 6) - (Math.PI / 360) * minutes;
        minuteHand.rotation.z = -minutes * (Math.PI / 30);
        secondHand.rotation.z = -seconds * (Math.PI / 30);
    };
}
function createClockHand(width, length, color, axisOffset = 0) {
    const handGeometry = new three.PlaneGeometry(width, length);
    const handMaterial = new three.MeshLambertMaterial({ color: color });
    const hand = new three.Mesh(handGeometry, handMaterial);

    hand.geometry.translate(0, length / 2, axisOffset);

    return hand;
}

//drawing magnets
function createCylinderMagnet(magnet) {
    const geometry = new three.CylinderGeometry(magnet.radius, magnet.radius, magnet.length, 32);
    const neodymiumMaterial = new three.MeshPhysicalMaterial({
        color: 0xffffff,
        roughness: 0.1
    });

    const cylinder = new three.Mesh(geometry, neodymiumMaterial);
    cylinder.name = "magnetObject";

    cylinder.position.set(magnet.position.x, magnet.position.y, magnet.position.z);

    const axis = new three.Vector3(0, 1, 0);
    const desiredOrientation = new three.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
    const angle = Math.acos(axis.dot(desiredOrientation));
    const rotationAxis = new three.Vector3().crossVectors(axis, desiredOrientation).normalize();
    cylinder.setRotationFromAxisAngle(rotationAxis, angle);

    cylinder.position.set(magnet.position.x, magnet.position.y + halfBoundingBoxHeight, magnet.position.z);
    simulationGroup.add(cylinder);
    return cylinder;
}
function createLoopsMagnet(magnet) {
    const segments = 10;
    const loopRadius = magnet.radius;
    const segmentHeight = magnet.length / segments;

    const enamelMaterial = new three.MeshPhysicalMaterial({
        color: 0xc54c12,
        metalness: 0.8,
        roughness: 0.0,
        clearcoat: 1.0,
        clearcoatRoughness: 0.05,
        sheen: 0.5,
        transmission: 0.2,
        thickness: 0.5
        //reflectivity?
    });

    for (let i = 0; i < segments; i++) {
        const loopGeometry = new three.TorusGeometry(loopRadius, magnet.length / segments / 2, 16, 100);
        const loop = new three.Mesh(loopGeometry, enamelMaterial);
        loop.name = "magnetObject";

        loop.position.set(magnet.position.x, magnet.position.y - magnet.length / 2 + segmentHeight * i + segmentHeight / 2, magnet.position.z);

        const axis = new three.Vector3(0, 0, 1);
        const desiredOrientation = new three.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
        const angle = Math.acos(axis.dot(desiredOrientation));
        const rotationAxis = new three.Vector3().crossVectors(axis, desiredOrientation).normalize();
        loop.setRotationFromAxisAngle(rotationAxis, angle);

        simulationGroup.add(loop);
    }
}

//drawing indicators
function addSimulationBox(dimensions) {
    const geometry = new three.BoxGeometry(dimensions.width, dimensions.height, dimensions.depth);
    const edges = new three.EdgesGeometry(geometry);
    const line = new three.LineSegments(edges, new three.LineBasicMaterial({ color: 0xffffff }));

    line.position.set(0, halfBoundingBoxHeight, 0);
    line.name = "boundingBox";
    simulationGroup.add(line);

    const gridHelper = new three.GridHelper(dimensions.width, 10);
    gridHelper.position.set(0, 0.1, 0);
    simulationGroup.add(gridHelper);

    const axesHelper = new three.AxesHelper(1);
    axesHelper.position.set(-dimensions.width / 2 + 0.1, 0.2, -dimensions.depth / 2 + 0.1);
    simulationGroup.add(axesHelper);
}
function drawFieldVectors(fieldData, color, fieldDrawMethod, fieldType) {
    switch (fieldDrawMethod) {
        case 'arrows':
            fieldData.forEach(vector => {
                drawArrow(vector, color, fieldType);
            });
            break;

        case 'fieldLines':
            fieldData.forEach(vector => {
                drawFieldLine(vector, color, fieldType);
            });
            break;

        case 'colorMapping':
            fieldData.forEach(vector => {
                drawColorMappedArrow(vector, fieldType);
            });
            break;

        case 'volumeRendering':
            console.log("Volume rendering is not implemented yet.");
            break;

        default:
            console.warn("Unknown field draw method:", fieldDrawMethod);
            break;
    }
}
function drawArrow(vector, color, fieldType) {
    const vectorName = `${fieldType}-vectorObject-${vector.index}`;
    let arrowHelper = simulationGroup.getObjectByName(vectorName);
    const arrowDirection = new three.Vector3(vector.direction.x, vector.direction.y, vector.direction.z).normalize();
    const arrowPosition = new three.Vector3(vector.position.x, vector.position.y + halfBoundingBoxHeight, vector.position.z);
    const arrowLength = vector.magnitude;

    if (!arrowHelper) {
        arrowHelper = new three.ArrowHelper(arrowDirection, arrowPosition, arrowLength, color);
        arrowHelper.name = vectorName;
        simulationGroup.add(arrowHelper);
    } else {
        arrowHelper.setDirection(arrowDirection);
        arrowHelper.setLength(arrowLength);
        arrowHelper.setColor(new three.Color(color));
        arrowHelper.position.copy(arrowPosition);
    }
}
function drawFieldLine(vector, color, fieldType) {
    const vectorName = `${fieldType}-lineObject-${vector.index}`;
    let lineObject = simulationGroup.getObjectByName(vectorName);

    const maxPoints = 100;
    const stepSize = 0.01;
    const positions = new Float32Array(maxPoints * 3);

    let currentPosition = new three.Vector3(vector.position.x, vector.position.y + halfBoundingBoxHeight, vector.position.z);
    for (let i = 0; i < maxPoints; i++) {
        positions[i * 3] = currentPosition.x;
        positions[i * 3 + 1] = currentPosition.y;
        positions[i * 3 + 2] = currentPosition.z;

        const fieldDirection = new three.Vector3(vector.direction.x, vector.direction.y, vector.direction.z).normalize();
        currentPosition.addScaledVector(fieldDirection, stepSize);
    }

    if (!lineObject) {
        const geometry = new three.BufferGeometry();
        geometry.setAttribute('position', new three.BufferAttribute(positions, 3));
        const lineMaterial = new three.LineBasicMaterial({ color: color });
        lineObject = new three.Line(geometry, lineMaterial);
        lineObject.name = vectorName;
        simulationGroup.add(lineObject);
    } else {
        lineObject.geometry.attributes.position.array = positions;
        lineObject.geometry.attributes.position.needsUpdate = true;
    }
}
function drawColorMappedArrow(vector, fieldType) {
    const vectorName = `${fieldType}-vectorObject-${vector.index}`;
    let arrowHelper = simulationGroup.getObjectByName(vectorName);
    const arrowDirection = new three.Vector3(vector.direction.x, vector.direction.y, vector.direction.z).normalize();
    const arrowPosition = new three.Vector3(vector.position.x, vector.position.y + halfBoundingBoxHeight, vector.position.z);

    //old
    //const arrowLength = vector.magnitude;
    //new
    const scaledMagnitude = Math.log(1 + vector.magnitude);
    const arrowLength = Math.max(scaledMagnitude, 0.1);

    const magnitudeNormalized = Math.min(vector.magnitude, 1);
    const colorScale = new three.Color().setHSL(0.7 * (1 - magnitudeNormalized), 1, 0.5);

    if (!arrowHelper) {
        arrowHelper = new three.ArrowHelper(arrowDirection, arrowPosition, arrowLength, colorScale);
        arrowHelper.name = vectorName;
        simulationGroup.add(arrowHelper);
    } else {
        arrowHelper.setDirection(arrowDirection);
        arrowHelper.setLength(arrowLength);
        arrowHelper.setColor(colorScale);
        arrowHelper.position.copy(arrowPosition);
    }
}
function addMagnetOrientationIndicator(magnet) {
    var dir = new three.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
    var origin = new three.Vector3(magnet.position.x, magnet.position.y + halfBoundingBoxHeight, magnet.position.z);
    var length = 1.0;
    var hex = 0xffff00;

    var arrowHelper = new three.ArrowHelper(dir, origin, length, hex);
    arrowHelper.name = "magnetObject";
    simulationGroup.add(arrowHelper);
}

export { updateThreeJsScene, initializeThreeJs };