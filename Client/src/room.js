import * as three from 'three';

var ambientLight, ceilingLight;

function addLights(scene, ceilingLightLevel, ambientLightLevel) {
    ceilingLight = new three.PointLight(0xffffff, ceilingLightLevel, 300, 0);
    ceilingLight.position.set(0, 70, 0);
    scene.add(ceilingLight);

    ambientLight = new three.AmbientLight(0xffffff, ambientLightLevel);
    scene.add(ambientLight);
}

function addFloor(scene) {
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

function addWalls(scene) {
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

function addCeiling(scene) {
    const ceilingGeometry = new three.PlaneGeometry(200, 200);
    const ceilingMaterial = new three.MeshStandardMaterial({ color: 0x808080, side: three.DoubleSide, roughness: 0.4 });
    const ceiling = new three.Mesh(ceilingGeometry, ceilingMaterial);
    ceiling.rotation.x = Math.PI / 2;
    ceiling.position.y = 80;
    scene.add(ceiling);
}

function addTable(scene) {
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

export { addLights, addFloor, addWalls, addCeiling, addTable };