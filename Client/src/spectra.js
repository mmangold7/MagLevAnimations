import * as three from 'three';
import { TextGeometry } from 'three/addons/geometries/TextGeometry.js';
import { elementSpectra, getElementSymbol, fraunhoferLines } from './elementSpectra';

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

function addSpectrumStripe(scene, texture, stripeIndex) {
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

function drawElementSpectra(scene, fontLoader) {
    elementSpectra.forEach((element, index) => {
        const column = index % numColumns;
        const row = Math.floor(index / numColumns);
        if (row < numRows) {
            addElementSpectrumStripe(scene, fontLoader, element, column, row);
        }
    });
}

function addElementSpectrumStripe(scene, fontLoader, element, column, row) {
    const xOffset = -wallWidth / 2 + cellWidth / 2;
    const yOffset = (wallHeight) - (bigStripeHeight / 2) - (bigStripeHeight);

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
    addElementLabel(scene, fontLoader, elementSymbol, xPos, yPos - littleStripeHeight - labelHeight);
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

function addElementLabel(scene, fontLoader, text, xPos, yPos) {
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

export { generateVisibleSpectrumTexture, generateSolarSpectrumTexture, addSpectrumStripe, drawElementSpectra };