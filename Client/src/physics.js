import * as three from 'three';

function createCylinderMagnet(simulationGroup, halfBoundingBoxHeight, magnet) {
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

function createLoopsMagnet(simulationGroup, magnet) {
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

function addSimulationBox(simulationGroup, halfBoundingBoxHeight, dimensions) {
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

function drawFieldVectors(simulationGroup, halfBoundingBoxHeight, fieldData, color, fieldDrawMethod, fieldType) {
    switch (fieldDrawMethod) {
        case 'arrows':
            fieldData.forEach(vector => {
                drawArrow(simulationGroup, halfBoundingBoxHeight, vector, color, fieldType);
            });
            break;

        case 'fieldLines':
            fieldData.forEach(vector => {
                drawFieldLine(simulationGroup, halfBoundingBoxHeight, vector, color, fieldType);
            });
            break;

        case 'colorMapping':
            fieldData.forEach(vector => {
                drawColorMappedArrow(simulationGroup, halfBoundingBoxHeight, vector, fieldType);
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

function drawArrow(simulationGroup, halfBoundingBoxHeight, vector, color, fieldType) {
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

function drawFieldLine(simulationGroup, halfBoundingBoxHeight, vector, color, fieldType) {
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

function drawColorMappedArrow(simulationGroup, halfBoundingBoxHeight, vector, fieldType) {
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

function addMagnetOrientationIndicator(simulationGroup, halfBoundingBoxHeight, magnet) {
    var dir = new three.Vector3(magnet.magnetization.x, magnet.magnetization.y, magnet.magnetization.z).normalize();
    var origin = new three.Vector3(magnet.position.x, magnet.position.y + halfBoundingBoxHeight, magnet.position.z);
    var length = 1.0;
    var hex = 0xffff00;

    var arrowHelper = new three.ArrowHelper(dir, origin, length, hex);
    arrowHelper.name = "magnetObject";
    simulationGroup.add(arrowHelper);
}

function clearMagnetsAndFields(simulationGroup) {
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

function clearFieldVectorsOfType(simulationGroup, fieldType) {
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

export {
    clearFieldVectorsOfType, clearMagnetsAndFields,
    drawFieldVectors, addMagnetOrientationIndicator,
    createLoopsMagnet, createCylinderMagnet, addSimulationBox };