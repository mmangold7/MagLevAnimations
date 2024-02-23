import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

var scene, camera, renderer, controls;
var simulationGroup, halfBoundingBoxHeight;
var analogClock, digitalClock, ambientLight, ceilingLight;

const numeralSegments = {
    '0': [true, true, true, false, true, true, true],
    '1': [false, true, true, false, false, false, false],
    '2': [true, true, false, true, true, false, true],
    '3': [true, true, true, true, false, false, true],
    '4': [false, true, true, true, false, true, false],
    '5': [true, false, true, true, false, true, true],
    '6': [true, false, true, true, true, true, true],
    '7': [true, true, true, false, false, false, false],
    '8': [true, true, true, true, true, true, true],
    '9': [true, true, true, true, false, true, true]
};

//public api
function initializeThreeJs(inputDimensions, drawingParameters) {
    const canvas = document.getElementById('threejs-canvas');
    if (!canvas) {
        console.error('Canvas element not found');
        return;
    }

    halfBoundingBoxHeight = inputDimensions.height / 2;
    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(0, 1, 2);
    camera.lookAt(new THREE.Vector3(0, 0, 0));

    addLights(drawingParameters.ceilingLightLevel, drawingParameters.ambientLightLevel);
    addFloor();
    addWalls();
    addCeiling();

    addTable();
    addBackWallRainbow();
    addDigitalClock();
    addAnalogClock();

    simulationGroup = new THREE.Group();
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
    const boundingBoxCenter = new THREE.Vector3(0, 20.5 + halfBoundingBoxHeight, 0);
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
    ceilingLight = new THREE.PointLight(0xffffff, ceilingLightLevel, 300, 0);
    ceilingLight.position.set(0, 70, 0);
    scene.add(ceilingLight);

    ambientLight = new THREE.AmbientLight(0xffffff, ambientLightLevel);
    scene.add(ambientLight);
}
function addFloor() {
    const tileSize = 10;
    const tilesPerSide = 20;
    const floorGeometry = new THREE.PlaneGeometry(tileSize, tileSize);

    for (let x = 0; x < tilesPerSide; x++) {
        for (let z = 0; z < tilesPerSide; z++) {
            const isWhite = (x + z) % 2 === 0;
            const floorMaterial = new THREE.MeshLambertMaterial({
                color: isWhite ? 0x9F9F9F : 0x000000,
                side: THREE.DoubleSide
            });

            const floorTile = new THREE.Mesh(floorGeometry, floorMaterial);
            floorTile.rotation.x = -Math.PI / 2;
            floorTile.position.set(x * tileSize - (tilesPerSide / 2) * tileSize + tileSize / 2, 0, z * tileSize - (tilesPerSide / 2) * tileSize + tileSize / 2);

            scene.add(floorTile);
        }
    }
}
function addWalls() {
    const wallMaterial = new THREE.MeshLambertMaterial({
        color: 0xFFFFFF,
        side: THREE.DoubleSide
    });

    const backWallGeometry = new THREE.PlaneGeometry(200, 80);
    const frontWallGeometry = new THREE.PlaneGeometry(200, 80);
    const sideWallGeometry = new THREE.PlaneGeometry(200, 80);

    const walls = [
        { geometry: backWallGeometry, position: new THREE.Vector3(0, 40, -100) },
        { geometry: frontWallGeometry, position: new THREE.Vector3(0, 40, 100) },
        { geometry: sideWallGeometry, position: new THREE.Vector3(-100, 40, 0), rotation: Math.PI / 2 },
        { geometry: sideWallGeometry, position: new THREE.Vector3(100, 40, 0), rotation: Math.PI / 2 }
    ];

    walls.forEach((wallSpec) => {
        const wall = new THREE.Mesh(wallSpec.geometry, wallMaterial);
        wall.position.copy(wallSpec.position);
        if (wallSpec.rotation) wall.rotation.y = wallSpec.rotation;
        scene.add(wall);
    });
}
function addCeiling() {
    const ceilingGeometry = new THREE.PlaneGeometry(200, 200);
    const ceilingMaterial = new THREE.MeshStandardMaterial({ color: 0x808080, side: THREE.DoubleSide, roughness: 0.4 });
    const ceiling = new THREE.Mesh(ceilingGeometry, ceilingMaterial);
    ceiling.rotation.x = Math.PI / 2;
    ceiling.position.y = 80;
    scene.add(ceiling);
}
function addTable() {
    const tableMaterial = new THREE.MeshStandardMaterial({
        color: 0x7F7F7F,
        roughness: 0.5,
        metalness: 0.1
    });
    const tableTop = new THREE.Mesh(new THREE.BoxGeometry(40, 1, 20), tableMaterial);
    tableTop.position.set(0, 20.5, 0);
    scene.add(tableTop);

    const legMaterial = new THREE.MeshStandardMaterial({
        color: 0x606060,
        roughness: 0.5,
        metalness: 0.1
    });
    const legPositions = [
        new THREE.Vector3(-19.5, 10, -9.5),
        new THREE.Vector3(19.5, 10, -9.5),
        new THREE.Vector3(-19.5, 10, 9.5),
        new THREE.Vector3(19.5, 10, 9.5)
    ];
    legPositions.forEach(position => {
        const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.5, 20, 32), legMaterial);
        leg.position.copy(position);
        scene.add(leg);
    });
}
function addBackWallRainbow() {
    const colors = ['#FF0000', '#FF7F00', '#FFFF00', '#00FF00', '#0000FF', '#4B0082', '#9400D3'];
    const panelWidth = 200 / colors.length;

    colors.forEach((color, i) => {
        const panelMaterial = new THREE.MeshLambertMaterial({ color });
        const panelGeometry = new THREE.PlaneGeometry(panelWidth, 80);
        const panel = new THREE.Mesh(panelGeometry, panelMaterial);
        panel.position.set(-100 + panelWidth * i + panelWidth / 2, 40, -99.9);
        scene.add(panel);
    });
}

//drawing clocks
function addDigitalClock() {
    digitalClock = new THREE.Group();
    digitalClock.rotation.y = Math.PI / 2;
    digitalClock.position.set(-95, 40, 50);

    const digits = [];
    for (let i = 0; i < 6; i++) { // HH:MM:SS
        const digit = createNumeral(0); // Start with '0'
        digit.position.x = (i - 2.5) * 12; // Align digits horizontally within the group
        if (i === 2 || i === 4) digit.position.x += 3; // Add space for colon
        digitalClock.add(digit);
        digits.push(digit);
    }

    const colonMaterial = new THREE.MeshBasicMaterial({ color: 0xFF0000 });
    const colonGeometry = new THREE.CircleGeometry(0.5, 32);
    [1, 3].forEach(i => {
        const colonTop = new THREE.Mesh(colonGeometry, colonMaterial);
        const colonBottom = new THREE.Mesh(colonGeometry, colonMaterial);
        colonTop.position.set((i - 2.5) * 12 + 6, 3, 0);
        colonBottom.position.set((i - 2.5) * 12 + 6, -3, 0);
        digitalClock.add(colonTop);
        digitalClock.add(colonBottom);
    });

    digitalClock.updateTime = function (simulationTime) {
        const time = new Date(simulationTime * 1000);
        const hours = time.getHours().toString().padStart(2, '0');
        const minutes = time.getMinutes().toString().padStart(2, '0');
        const seconds = time.getSeconds().toString().padStart(2, '0');
        const timeString = hours + minutes + seconds;

        timeString.split('').forEach((num, index) => {
            while (digits[index].children.length) {
                digits[index].remove(digits[index].children[0]);
            }

            const numeral = createNumeral(parseInt(num));
            digits[index].add(...numeral.children);
        });
    };

    scene.add(digitalClock);
}
function createSegment() {
    const segmentLength = 5;
    const segmentThickness = 0.5;
    const diamondTipLength = 1;

    const shape = new THREE.Shape();
    shape.moveTo(-segmentThickness / 2, diamondTipLength);
    shape.lineTo(0, 0);
    shape.lineTo(segmentThickness / 2, diamondTipLength);
    shape.lineTo(segmentThickness / 2, segmentLength - diamondTipLength);
    shape.lineTo(0, segmentLength);
    shape.lineTo(-segmentThickness / 2, segmentLength - diamondTipLength);
    shape.lineTo(-segmentThickness / 2, diamondTipLength);

    const extrudeSettings = {
        steps: 1,
        depth: segmentThickness,
        bevelEnabled: false,
    };

    const geometry = new THREE.ExtrudeGeometry(shape, extrudeSettings);
    const material = new THREE.MeshBasicMaterial({ color: 0xFF0000 });
    const mesh = new THREE.Mesh(geometry, material);

    return mesh;
}
function createNumeral(num) {
    const numeral = new THREE.Group();
    const segmentPositions = [
        { x: 0, y: 10, rotation: 0 }, // Top
        { x: 5, y: 5, rotation: Math.PI / 2 }, // Top Right
        { x: 5, y: -5, rotation: Math.PI / 2 }, // Bottom Right
        { x: 0, y: -10, rotation: 0 }, // Bottom
        { x: -5, y: -5, rotation: Math.PI / 2 }, // Bottom Left
        { x: -5, y: 5, rotation: Math.PI / 2 }, // Top Left
        { x: 0, y: 0, rotation: 0 } // Middle
    ];

    numeralSegments[num.toString()].forEach((isActive, index) => {
        if (isActive) {
            const segment = createSegment();
            segment.position.set(segmentPositions[index].x, segmentPositions[index].y, 0);
            segment.rotation.z = segmentPositions[index].rotation;
            numeral.add(segment);
        }
    });

    return numeral;
}
function addAnalogClock() {
    const radius = 20;
    const clockGeometry = new THREE.CircleGeometry(radius, 32);
    const clockMaterial = new THREE.MeshLambertMaterial({ color: 0xFFFFFF });
    analogClock = new THREE.Mesh(clockGeometry, clockMaterial);
    analogClock.position.set(-95, 40 - radius, 50);
    analogClock.rotation.y = Math.PI / 2;
    scene.add(analogClock);

    const hourHand = createClockHand(1, 10, 0x000000, -5);
    const minuteHand = createClockHand(0.5, 15, 0x000000, -7.5);
    const secondHand = createClockHand(0.2, 18, 0xFF0000, -9);

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
function createClockHand(width, length, color, pivotOffset = 0) {
    const handGeometry = new THREE.PlaneGeometry(width, length);
    const handMaterial = new THREE.MeshLambertMaterial({ color: color });
    const hand = new THREE.Mesh(handGeometry, handMaterial);

    hand.geometry.translate(0, length / 2, 0);

    return hand;
}

//drawing magnets
function createCylinderMagnet(magnet) {
    const geometry = new THREE.CylinderGeometry(magnet.radius, magnet.radius, magnet.length, 32);
    const neodymiumMaterial = new THREE.MeshPhysicalMaterial({
        color: 0xffffff,
        roughness: 0.1,
    });

    const cylinder = new THREE.Mesh(geometry, neodymiumMaterial);
    cylinder.name = "magnetObject";

    cylinder.position.set(magnet.position.x, magnet.position.y, magnet.position.z);

    const axis = new THREE.Vector3(0, 1, 0);
    const desiredOrientation = new THREE.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
    const angle = Math.acos(axis.dot(desiredOrientation));
    const rotationAxis = new THREE.Vector3().crossVectors(axis, desiredOrientation).normalize();
    cylinder.setRotationFromAxisAngle(rotationAxis, angle);

    cylinder.position.set(magnet.position.x, magnet.position.y + halfBoundingBoxHeight, magnet.position.z);
    simulationGroup.add(cylinder);
    return cylinder;
}
function createLoopsMagnet(magnet) {
    const segments = 10;
    const loopRadius = magnet.radius;
    const segmentHeight = magnet.length / segments;

    const enamelMaterial = new THREE.MeshPhysicalMaterial({
        color: 0xc54c12,
        metalness: 0.8,
        roughness: 0.0,
        clearcoat: 1.0,
        clearcoatRoughness: 0.05,
        sheen: 0.5,
        transmission: 0.2,
        thickness: 0.5,
        //reflectivity?
    });

    for (let i = 0; i < segments; i++) {
        const loopGeometry = new THREE.TorusGeometry(loopRadius, magnet.length / segments / 2, 16, 100);
        const loop = new THREE.Mesh(loopGeometry, enamelMaterial);
        loop.name = "magnetObject";

        loop.position.set(magnet.position.x, magnet.position.y - magnet.length / 2 + segmentHeight * i + segmentHeight / 2, magnet.position.z);

        const axis = new THREE.Vector3(0, 0, 1);
        const desiredOrientation = new THREE.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
        const angle = Math.acos(axis.dot(desiredOrientation));
        const rotationAxis = new THREE.Vector3().crossVectors(axis, desiredOrientation).normalize();
        loop.setRotationFromAxisAngle(rotationAxis, angle);

        simulationGroup.add(loop);
    }
}

//drawing indicators
function addSimulationBox(dimensions) {
    const geometry = new THREE.BoxGeometry(dimensions.width, dimensions.height, dimensions.depth);
    const edges = new THREE.EdgesGeometry(geometry);
    const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0xffffff }));

    line.position.set(0, halfBoundingBoxHeight, 0);
    line.name = "boundingBox";
    simulationGroup.add(line);

    const gridHelper = new THREE.GridHelper(dimensions.width, 10);
    gridHelper.position.set(0, 0.1, 0);
    simulationGroup.add(gridHelper);

    const axesHelper = new THREE.AxesHelper(1);
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
    const arrowDirection = new THREE.Vector3(vector.direction.x, vector.direction.y, vector.direction.z).normalize();
    const arrowPosition = new THREE.Vector3(vector.position.x, vector.position.y + halfBoundingBoxHeight, vector.position.z);
    const arrowLength = vector.magnitude;

    if (!arrowHelper) {
        arrowHelper = new THREE.ArrowHelper(arrowDirection, arrowPosition, arrowLength, color);
        arrowHelper.name = vectorName;
        simulationGroup.add(arrowHelper);
    } else {
        arrowHelper.setDirection(arrowDirection);
        arrowHelper.setLength(arrowLength);
        arrowHelper.setColor(new THREE.Color(color));
        arrowHelper.position.copy(arrowPosition);
    }
}
function drawFieldLine(vector, color, fieldType) {
    const vectorName = `${fieldType}-lineObject-${vector.index}`;
    let lineObject = simulationGroup.getObjectByName(vectorName);

    const maxPoints = 100;
    const stepSize = 0.01;
    const positions = new Float32Array(maxPoints * 3);

    let currentPosition = new THREE.Vector3(vector.position.x, vector.position.y + halfBoundingBoxHeight, vector.position.z);
    for (let i = 0; i < maxPoints; i++) {
        positions[i * 3] = currentPosition.x;
        positions[i * 3 + 1] = currentPosition.y;
        positions[i * 3 + 2] = currentPosition.z;

        const fieldDirection = new THREE.Vector3(vector.direction.x, vector.direction.y, vector.direction.z).normalize();
        currentPosition.addScaledVector(fieldDirection, stepSize);
    }

    if (!lineObject) {
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        const lineMaterial = new THREE.LineBasicMaterial({ color: color });
        lineObject = new THREE.Line(geometry, lineMaterial);
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
    const arrowDirection = new THREE.Vector3(vector.direction.x, vector.direction.y, vector.direction.z).normalize();
    const arrowPosition = new THREE.Vector3(vector.position.x, vector.position.y + halfBoundingBoxHeight, vector.position.z);
    const arrowLength = vector.magnitude;

    const magnitudeNormalized = Math.min(vector.magnitude, 1);
    const colorScale = new THREE.Color().setHSL(0.7 * (1 - magnitudeNormalized), 1, 0.5);

    if (!arrowHelper) {
        arrowHelper = new THREE.ArrowHelper(arrowDirection, arrowPosition, arrowLength, colorScale);
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
    var dir = new THREE.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
    var origin = new THREE.Vector3(magnet.position.x, magnet.position.y + halfBoundingBoxHeight, magnet.position.z);
    var length = 1.0;
    var hex = 0xffff00;

    var arrowHelper = new THREE.ArrowHelper(dir, origin, length, hex);
    arrowHelper.name = "magnetObject";
    simulationGroup.add(arrowHelper);
}

export { updateThreeJsScene, initializeThreeJs };