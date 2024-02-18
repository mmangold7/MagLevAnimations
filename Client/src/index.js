import * as three from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

var scene, camera, renderer, controls;

function initializeThreeJs(dimensions) {
    scene = new three.Scene();

    camera = new three.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(0, 1, 2);
    camera.lookAt(new three.Vector3(0, 0, 0));

    const canvasContainer = document.querySelector('.canvas-container');
    const canvas = document.getElementById('threejs-canvas');

    renderer = new three.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setClearColor(0xffffff, 1);
    renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);

    setupOrbitControls(dimensions);
    addBoundingBox(dimensions);
    resetCameraToBoundingBox(dimensions);
    animate();
};

function setupOrbitControls(dimensions) {
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; // Optional: enable damping (inertia)
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.maxPolarAngle = Math.PI / 2;
    controls.zoomSpeed = 1.0;
    controls.target.set(0, 0, 0);
    controls.update();
}

function addBoundingBox(dimensions) {
    const geometry = new three.BoxGeometry(dimensions.width, dimensions.height, dimensions.depth);
    const edges = new three.EdgesGeometry(geometry);
    const line = new three.LineSegments(edges, new three.LineBasicMaterial({ color: 0x0000ff }));
    line.position.set(0, 0, 0);
    line.name = "boundingBox";
    scene.add(line);

    const gridHelper = new three.GridHelper(dimensions.width, 10);
    gridHelper.position.y = -(dimensions.height / 2);
    scene.add(gridHelper);
}

function resetCameraToBoundingBox(dimensions) {
    const maxDimension = Math.max(dimensions.width, dimensions.height, dimensions.depth);
    const fov = camera.fov * (Math.PI / 180);
    const cameraZ = Math.abs(maxDimension / 2 * Math.tan(fov / 2) * 2);
    camera.position.z = cameraZ;

    const center = new three.Vector3(0, 0, 0);
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

function updateThreeJsScene(magnets, showLoops) {
    clearMagnets();

    const magnetsArray = Array.isArray(magnets) ? magnets : [magnets];

    magnetsArray.forEach(magnet => {
        if (showLoops) {
            createLoopsMagnet(magnet);
        } else {
            createCylinderMagnet(magnet);
        }
    });
}

function clearMagnets() {
    const toRemove = [];
    scene.traverse((child) => {
        if (child.name === "magnetObject") {
            toRemove.push(child);
        }
    });
    toRemove.forEach((object) => {
        scene.remove(object);
    });
}

function createCylinderMagnet(magnet) {
    const geometry = new three.CylinderGeometry(magnet.radius, magnet.radius, magnet.length, 32);
    const material = new three.MeshBasicMaterial({
        color: 0xff0000,
    });

    const cylinder = new three.Mesh(geometry, material);
    cylinder.name = "magnetObject";

    cylinder.position.set(magnet.position.x, magnet.position.y, magnet.position.z);

    const axis = new three.Vector3(0, 1, 0);
    const desiredOrientation = new three.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
    const angle = Math.acos(axis.dot(desiredOrientation));
    const rotationAxis = new three.Vector3().crossVectors(axis, desiredOrientation).normalize();
    cylinder.setRotationFromAxisAngle(rotationAxis, angle);

    scene.add(cylinder);
    return cylinder;
}

function createLoopsMagnet(magnet) {
    const segments = 10;
    const loopRadius = magnet.radius;
    const segmentHeight = magnet.length / segments;

    for (let i = 0; i < segments; i++) {
        const loopGeometry = new three.TorusGeometry(loopRadius, 0.005, 16, 100);
        const loopMaterial = new three.MeshBasicMaterial({
            color: 0x0000ff,
        });
        const loop = new three.Mesh(loopGeometry, loopMaterial);
        loop.name = "magnetObject";

        loop.position.set(magnet.position.x, magnet.position.y - magnet.length / 2 + segmentHeight * i + segmentHeight / 2, magnet.position.z);

        const axis = new three.Vector3(0, 0, 1);
        const desiredOrientation = new three.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
        const angle = Math.acos(axis.dot(desiredOrientation));
        const rotationAxis = new three.Vector3().crossVectors(axis, desiredOrientation).normalize();
        loop.setRotationFromAxisAngle(rotationAxis, angle);

        scene.add(loop);
    }
}

export { updateThreeJsScene, initializeThreeJs };