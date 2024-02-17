import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

// Global variables for the scene, camera, and renderer
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Function to create a simple cylinder representation of a magnet
function createCylinder(magnet) {
    const geometry = new THREE.CylinderGeometry(magnet.radius, magnet.radius, magnet.length, 32);
    const material = new THREE.MeshBasicMaterial({ color: magnet.color });
    const cylinder = new THREE.Mesh(geometry, material);
    cylinder.position.set(magnet.position.x, magnet.position.y, magnet.position.z);
    cylinder.rotation.x = Math.PI / 2; // Align the cylinder along the Z-axis
    scene.add(cylinder);
}

// Function to create loop segments representation of a magnet
function createLoopSegments(magnet) {
    const segments = 10; // Number of loop segments along the magnet's length
    const loopRadius = magnet.radius;
    const segmentHeight = magnet.length / segments;

    for (let i = 0; i < segments; i++) {
        const loopGeometry = new THREE.TorusGeometry(loopRadius, 0.005, 16, 100);
        const loopMaterial = new THREE.MeshBasicMaterial({ color: magnet.color });
        const loop = new THREE.Mesh(loopGeometry, loopMaterial);

        // Position each loop segment
        loop.position.set(magnet.position.x, magnet.position.y, magnet.position.z - magnet.length / 2 + segmentHeight * (i + 0.5));
        scene.add(loop);
    }
}

// Function to update the visualization
function updateVisualization(magnet, showLoops) {
    clearScene(); // Clear existing objects from the scene

    if (showLoops) {
        createLoopSegments(magnet);
    } else {
        createCylinder(magnet);
    }

    // Ensure the camera and renderer are set up correctly
    camera.position.set(0, 0, 3);
    renderer.render(scene, camera);
}

// Function to clear all objects from the scene
function clearScene() {
    while (scene.children.length > 0) {
        scene.remove(scene.children[0]);
    }
}

// Example magnet object
const exampleMagnet = {
    position: { x: 0, y: 0, z: 0 },
    radius: 0.1,
    length: 0.2,
    color: 0xff0000 // Red
};

// Initial visualization
updateVisualization(exampleMagnet, false); // Start with the simple cylinder representation
