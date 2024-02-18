import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

var scene, camera, renderer, controls;

function initializeThreeJs(dimensions) {
    scene = new THREE.Scene();
    //scene.background = new THREE.Color(0xbbbbbb);

    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(0, 1, 2);
    camera.lookAt(new THREE.Vector3(0, 0, 0));

    scene.add(new THREE.AmbientLight(0xffffff, 10));
    const light = new THREE.PointLight(0xffffff, 10, 0, 0);
    light.position.copy(camera.position);
    scene.add(light);

    const axesHelper = new THREE.AxesHelper(1);

    axesHelper.position.set(dimensions.width / 2, -dimensions.height / 2, dimensions.depth / 2);

    // Add the AxesHelper to the scene
    scene.add(axesHelper);

    const canvasContainer = document.querySelector('.canvas-container');
    const canvas = document.getElementById('threejs-canvas');

    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);

    setupOrbitControls(dimensions);
    addBoundingBox(dimensions);
    resetCameraToBoundingBox(dimensions);
    animate();
};

function setupOrbitControls(dimensions) {
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.maxPolarAngle = Math.PI / 2;
    controls.zoomSpeed = 1.0;
    controls.target.set(0, 0, 0);
    controls.update();
}

function addBoundingBox(dimensions) {
    const geometry = new THREE.BoxGeometry(dimensions.width, dimensions.height, dimensions.depth);
    const edges = new THREE.EdgesGeometry(geometry);
    const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0xffffff }));
    line.position.set(0, 0, 0);
    line.name = "boundingBox";
    scene.add(line);

    const gridHelper = new THREE.GridHelper(dimensions.width, 10);
    gridHelper.position.y = -(dimensions.height / 2);
    scene.add(gridHelper);
}

function resetCameraToBoundingBox(dimensions) {
    const maxDimension = Math.max(dimensions.width, dimensions.height, dimensions.depth);
    const fov = camera.fov * (Math.PI / 180);
    const cameraZ = Math.abs(maxDimension / 2 * Math.tan(fov / 2) * 2);
    camera.position.z = cameraZ;

    const center = new THREE.Vector3(0, 0, 0);
    controls.target = center;
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

function updateThreeJsScene(magnets, showLoops, gravityField, magneticField) {
    clearMagnetsAndFields();

    const magnetsArray = Array.isArray(magnets) ? magnets : [magnets];

    magnetsArray.forEach(magnet => {
        if (showLoops) {
            createLoopsMagnet(magnet);
            addMagnetOrientationIndicator(magnet);
        } else {
            createCylinderMagnet(magnet);
            addMagnetOrientationIndicator(magnet);
        }
    });

    if (gravityField) {
        drawFieldVectors(gravityField, 0x00ff00); // Green for gravity
    }

    if (magneticField) {
        drawFieldVectors(magneticField, 0xff0000); // Red for magnetic
    }
}

function clearMagnetsAndFields() {
    const toRemove = [];
    scene.traverse((child) => {
        if (child.name === "magnetObject" || child.name === "arrowObject") {
            toRemove.push(child);
        }
    });
    toRemove.forEach((object) => {
        scene.remove(object);
    });
}

function drawFieldVectors(fieldData, color) {
    fieldData.forEach(vector => {
        const arrowDirection = new THREE.Vector3(vector.direction.x, vector.direction.y, vector.direction.z);
        const arrowPosition = new THREE.Vector3(vector.position.x, vector.position.y, vector.position.z);
        const arrowLength = vector.magnitude; // Adjust based on your scale

        const arrowHelper = new THREE.ArrowHelper(arrowDirection.normalize(), arrowPosition, arrowLength, color);
        arrowHelper.name = "arrowObject";
        scene.add(arrowHelper);
    });
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

    scene.add(cylinder);
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
        //reflectivity 
    });

    for (let i = 0; i < segments; i++) {
        const loopGeometry = new THREE.TorusGeometry(loopRadius, 0.005, 16, 100);
        const loop = new THREE.Mesh(loopGeometry, enamelMaterial);
        loop.name = "magnetObject";

        loop.position.set(magnet.position.x, magnet.position.y - magnet.length / 2 + segmentHeight * i + segmentHeight / 2, magnet.position.z);

        const axis = new THREE.Vector3(0, 0, 1);
        const desiredOrientation = new THREE.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
        const angle = Math.acos(axis.dot(desiredOrientation));
        const rotationAxis = new THREE.Vector3().crossVectors(axis, desiredOrientation).normalize();
        loop.setRotationFromAxisAngle(rotationAxis, angle);

        scene.add(loop);
    }
}

function addMagnetOrientationIndicator(magnet) {
    // Assuming 'magnet' is an object with properties 'position' and 'magnetization'
    var dir = new THREE.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
    var origin = new THREE.Vector3(magnet.position.x, magnet.position.y, magnet.position.z);
    var length = 0.2; // Length of the arrow
    var hex = 0xffff00; // Color of the arrow

    var arrowHelper = new THREE.ArrowHelper(dir, origin, length, hex);
    scene.add(arrowHelper);
}

export { updateThreeJsScene, initializeThreeJs };