import * as three from 'three';
import { TextGeometry } from 'three/addons/geometries/TextGeometry.js';

var analogClock, digitalClock;

function addDigitalClock(scene, fontLoader) {
    digitalClock = new three.Group();

    digitalClock.updateTime = function () {
        console.warn('Font not loaded yet.');
    };

    fontLoader.load('https://threejs.org/examples/fonts/helvetiker_regular.typeface.json', function (font) {
        const geometry = new TextGeometry('00:00:00', {
            font: font,
            size: 5,
            height: .1,
            curveSegments: 12,
            bevelEnabled: true,
            bevelThickness: 0.1,
            bevelSize: 0.1,
            bevelSegments: 5
        });

        const material = new three.MeshBasicMaterial({ color: 0xff0000 });
        const mesh = new three.Mesh(geometry, material);
        mesh.rotation.y = Math.PI / 2;
        mesh.position.set(-99.5, 40 - 1.25 - 10, -(40 - 1.25));
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
                size: 5,
                height: .1,
                curveSegments: 12,
                bevelEnabled: true,
                bevelThickness: 0.1,
                bevelSize: 0.1,
                bevelSegments: 5
            });
        };
    });

    scene.add(digitalClock);

    return digitalClock;
}

function addAnalogClock(scene) {
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

    return analogClock;
}

function createClockHand(width, length, color, axisOffset = 0) {
    const handGeometry = new three.PlaneGeometry(width, length);
    const handMaterial = new three.MeshLambertMaterial({ color: color });
    const hand = new three.Mesh(handGeometry, handMaterial);

    hand.geometry.translate(0, length / 2, axisOffset);

    return hand;
}

export { addDigitalClock, addAnalogClock };