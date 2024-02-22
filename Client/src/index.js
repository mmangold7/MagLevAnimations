import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

var scene, camera, renderer, controls, simulationGroup;

function initializeThreeJs(dimensions) {
    const canvas = document.getElementById('threejs-canvas');
    if (!canvas) {
        console.error('Canvas element not found');
        return;
    }

    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    scene = new THREE.Scene();
    simulationGroup = new THREE.Group();

    setupLightsAndCamera();
    setupOrbitControls();
    addBoundingBox(dimensions);
    addTableAndRoom();
    addWalls();
    addCeilingLight();
    addCeiling();
    resetCameraToBoundingBox(dimensions);

    simulationGroup.position.set(0, 20.5 + 0.5, 0);
    scene.add(simulationGroup);

    animate();
};

function setupLightsAndCamera() {
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(0, 1, 2);
    camera.lookAt(new THREE.Vector3(0, 0, 0));

    scene.add(new THREE.AmbientLight(0xffffff, 5));

    //const light = new THREE.PointLight(0xffffff, 10, 0, 0);
    //light.position.copy(camera.position);
    //scene.add(light);
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

function createCheckerboardTexture() {
    const size = 64; // Texture size
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = size;
    const context = canvas.getContext('2d');

    for (let x = 0; x < size; x += 8) {
        for (let y = 0; y < size; y += 8) {
            context.fillStyle = (x ^ y) & 8 ? '#FFF' : '#CCC'; // Alternate colors
            context.fillRect(x, y, 8, 8);
        }
    }

    const texture = new THREE.CanvasTexture(canvas);
    texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
    texture.repeat.set(25, 25); // Adjust based on your scene
    return texture;
}

function addBoundingBox(dimensions) {
    const geometry = new THREE.BoxGeometry(dimensions.width, dimensions.height, dimensions.depth);
    const edges = new THREE.EdgesGeometry(geometry);
    const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0xffffff }));
    // Align the bottom of the bounding box with the top surface of the desk
    line.position.set(0, 5, 0); // Desk height (20.5) + half of the bounding box's height to sit on the surface
    line.name = "boundingBox";
    simulationGroup.add(line);

    const gridHelper = new THREE.GridHelper(dimensions.width, 10);
    gridHelper.position.set(0, 0, 0); // Adjusted to sit right on the desk surface within the bounding box
    simulationGroup.add(gridHelper);

    const axesHelper = new THREE.AxesHelper(5);
    axesHelper.position.set(-dimensions.width / 2, 0, dimensions.depth / 2); // Adjust to sit at the corner of the grid
    simulationGroup.add(axesHelper);
}

function addTableAndRoom() {
    // Desk surface
    const deskGeometry = new THREE.BoxGeometry(40, 1, 20);
    const deskMaterial = new THREE.MeshStandardMaterial({
        color: '#8B4513', // SaddleBrown color
        roughness: 0.6,
        metalness: 0.2
    });
    const desk = new THREE.Mesh(new THREE.BoxGeometry(40, 1, 20), deskMaterial);
    desk.position.set(0, 20.5, 0);
    scene.add(desk);

    // Desk legs
    const legGeometry = new THREE.CylinderGeometry(0.5, 0.5, 20, 32);
    const legMaterial = new THREE.MeshStandardMaterial({ color: 0x654321 });
    const legPositions = [
        new THREE.Vector3(-19.5, 10, -9.5),
        new THREE.Vector3(19.5, 10, -9.5),
        new THREE.Vector3(-19.5, 10, 9.5),
        new THREE.Vector3(19.5, 10, 9.5)
    ];

    legPositions.forEach((position) => {
        const leg = new THREE.Mesh(legGeometry, legMaterial);
        leg.position.copy(position);
        scene.add(leg);
    });

    // Floor
    const floorMaterial = new THREE.MeshStandardMaterial({
        map: createCheckerboardTexture()
    });
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(200, 200), floorMaterial);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = 0;
    scene.add(floor);

    simulationGroup.position.set(0, 20.5 + 0.5, 0);
}

function addWalls() {
    const wallMaterial = new THREE.MeshLambertMaterial({ color: 0xFFFFFF, side: THREE.DoubleSide });

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

    const colors = ['#FF0000', '#FF7F00', '#FFFF00', '#00FF00', '#0000FF', '#4B0082', '#9400D3'];
    const panelWidth = 200 / colors.length;

    colors.forEach((color, i) => {
        const panelMaterial = new THREE.MeshLambertMaterial({ color });
        const panelGeometry = new THREE.PlaneGeometry(panelWidth, 80);
        const panel = new THREE.Mesh(panelGeometry, panelMaterial);
        panel.position.set(-100 + panelWidth * i + panelWidth / 2, 40, -100); // Position each panel
        scene.add(panel);
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

function addCeilingLight() {
    // Light source
    const light = new THREE.PointLight(0xffffff, 1, 0, 2);
    light.position.set(0, 75, 0);
    scene.add(light);
}

function resetCameraToBoundingBox(dimensions) {
    const boundingBoxCenter = new THREE.Vector3(0, 21, 0); // Centered and adjusted for height on the desk
    const maxDimension = Math.max(dimensions.width, dimensions.height, dimensions.depth);
    const fov = camera.fov * (Math.PI / 180);
    const cameraZ = Math.abs(maxDimension / 2 * Math.tan(fov / 2) * 2);
    camera.position.set(boundingBoxCenter.x, boundingBoxCenter.y, boundingBoxCenter.z + cameraZ);
    controls.target.copy(boundingBoxCenter);
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

function updateThreeJsScene(state, drawingParameters) {
    const magnets = state.magnets;
    const gravityField = state.gravityFieldData;
    const magneticField = state.magneticFieldData;

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
}

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

function drawArrow(vector, color, fieldType) {
    const vectorName = `${fieldType}-vectorObject-${vector.index}`;
    let arrowHelper = simulationGroup.getObjectByName(vectorName);
    const arrowDirection = new THREE.Vector3(vector.direction.x, vector.direction.y, vector.direction.z).normalize();
    const arrowPosition = new THREE.Vector3(vector.position.x, vector.position.y + 5, vector.position.z);
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

    let currentPosition = new THREE.Vector3(vector.position.x, vector.position.y + 5, vector.position.z);
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
    const arrowPosition = new THREE.Vector3(vector.position.x, vector.position.y + 5, vector.position.z);
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

function createCylinderMagnet(magnet) {
    const geometry = new THREE.CylinderGeometry(magnet.radius, magnet.radius, magnet.length, 32);
    const neodymiumMaterial = new THREE.MeshPhysicalMaterial({
        color: 0xffffff,
        metalness: 0.99,
        roughness: 0.0,
        clearcoat: 1.0,
        clearcoatRoughness: 0.05,
        sheen: 0.5,
        transmission: 0.2,
        thickness: 0.5,
    });

    const cylinder = new THREE.Mesh(geometry, neodymiumMaterial);
    cylinder.name = "magnetObject";

    cylinder.position.set(magnet.position.x, magnet.position.y, magnet.position.z);

    const axis = new THREE.Vector3(0, 1, 0);
    const desiredOrientation = new THREE.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
    const angle = Math.acos(axis.dot(desiredOrientation));
    const rotationAxis = new THREE.Vector3().crossVectors(axis, desiredOrientation).normalize();
    cylinder.setRotationFromAxisAngle(rotationAxis, angle);

    cylinder.position.set(magnet.position.x, magnet.position.y + 5, magnet.position.z);
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

function addMagnetOrientationIndicator(magnet) {
    var dir = new THREE.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
    var origin = new THREE.Vector3(magnet.position.x, magnet.position.y + 5, magnet.position.z);
    var length = 1.0;
    var hex = 0xffff00;

    var arrowHelper = new THREE.ArrowHelper(dir, origin, length, hex);
    arrowHelper.name = "arrowObject";
    simulationGroup.add(arrowHelper);
}

export { updateThreeJsScene, initializeThreeJs };