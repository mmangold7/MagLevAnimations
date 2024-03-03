import * as three from 'three';
import { FontLoader } from 'three/addons/loaders/FontLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

import { clearFieldVectorsOfType, clearMagnetsAndFields, drawFieldVectors, addMagnetOrientationIndicator,
    createLoopsMagnet, createCylinderMagnet, addSimulationBox } from './physics';
import { addLights, addFloor, addWalls, addCeiling, addTable } from './room';
import { generateVisibleSpectrumTexture, generateSolarSpectrumTexture, addSpectrumStripe, drawElementSpectra } from './spectra';
import { addDigitalClock, addAnalogClock } from './clocks';

var scene, camera, renderer, controls, fontLoader;
var simulationGroup, halfBoundingBoxHeight;
var analogClock, digitalClock, ambientLight, ceilingLight;

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

    addLights(scene, drawingParameters.ceilingLightLevel, drawingParameters.ambientLightLevel);
    addFloor(scene);
    addWalls(scene);
    addCeiling(scene);

    addTable(scene);

    //const visibleSpectrumTexture = generateVisibleSpectrumTexture();
    //addSpectrumStripe(scene, visibleSpectrumTexture, 0);
    const solarSpectrumTexture = generateSolarSpectrumTexture();
    addSpectrumStripe(scene, solarSpectrumTexture, 0);
    const maxAnisotropy = renderer.capabilities.getMaxAnisotropy();
    drawElementSpectra(scene, fontLoader, maxAnisotropy);

    digitalClock = addDigitalClock(scene, fontLoader);
    analogClock = addAnalogClock(scene);

    simulationGroup = new three.Group();
    addSimulationBox(simulationGroup, halfBoundingBoxHeight, inputDimensions, fontLoader);
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

    clearMagnetsAndFields(simulationGroup);

    if (magnets) {
        const magnetsArray = Array.isArray(magnets) ? magnets : [magnets];
        magnetsArray.forEach(magnet => {
            const magnetMesh = (drawingParameters.showCoils ? createLoopsMagnet(simulationGroup, halfBoundingBoxHeight, magnet) : createCylinderMagnet(simulationGroup, halfBoundingBoxHeight, magnet));
            simulationGroup.add(magnetMesh);
            addMagnetOrientationIndicator(simulationGroup, halfBoundingBoxHeight, magnet);
        });
    }

    if (gravityField && drawingParameters.showGravityField) {
        drawFieldVectors(simulationGroup, halfBoundingBoxHeight, gravityField, 0x00ff00, "arrows", "gravity");
    } else if (!drawingParameters.showGravityField) {
        clearFieldVectorsOfType(simulationGroup, "gravity");
    }

    if (magneticField && drawingParameters.showMagneticField) {
        drawFieldVectors(simulationGroup, halfBoundingBoxHeight, magneticField, 0xff0000, drawingParameters.fieldDrawingStyle, "magnetic");
    } else if (!drawingParameters.showMagneticField) {
        clearFieldVectorsOfType(simulationGroup, "magnetic");
    }

    if (analogClock) analogClock.updateTime(timeSinceStart);
    if (digitalClock) digitalClock.updateTime(timeSinceStart);
}

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

export { updateThreeJsScene, initializeThreeJs };