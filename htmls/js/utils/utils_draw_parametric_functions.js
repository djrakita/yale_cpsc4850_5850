import * as THREE from 'three';
import { ParametricGeometry } from 'three/examples/jsm/geometries/ParametricGeometry.js';

/**
 * Draws a 2D function as a series of connected lines in the scene.
 *
 * @param {Object} engine - The rendering engine, including the scene, camera, and renderer.
 * @param {Function} function_to_draw - The function to graph. It should take a single argument (x) and return a scalar value (y).
 * @param {Array<number>} [domain=null] - The domain of the function to graph, as a two-element array [min, max].
 * @param {number} [samples=null] - The number of samples to take across the domain.
 * @param {number} [color=0x000000] - The color of the graph.
 * @returns {Object} An object containing the `update` method to refresh the graph.
 */
export function draw_2d_function(engine, function_to_draw, domain = null, samples = null, color = 0x000000) {
    /**
     * Updates the graph based on the function and parameters provided.
     *
     * @param {boolean} [changed=true] - Whether the graph should be updated.
     */
    function update(changed = true) {
        if (!changed) {
            return;
        }

        // Get viewing space dimensions
        const viewWidth = window.innerWidth;
        const viewHeight = window.innerHeight;

        // If the domain is not preset, set it to fill the viewing space
        let curr_domain = domain;
        if (domain == null) {
            curr_domain = [-viewWidth / 2, viewWidth / 2];
        }

        // Set default sample number
        let curr_samples = samples;
        if (samples == null) {
            curr_samples = 10 * Math.ceil(curr_domain[1] - curr_domain[0]);
        }

        const step = (curr_domain[1] - curr_domain[0]) / curr_samples;
        let p1, p2;
        for (let i = 0; i < curr_samples; i++) {
            let x1 = curr_domain[0] + i * step;
            let x2 = curr_domain[0] + (i + 1) * step;

            p1 = [x1, function_to_draw(x1)];
            p2 = [x2, function_to_draw(x2)];

            engine.draw_debug_line(p1, p2, true, 0.01, color);
        }
    }
    update();

    // Return the update function
    return {
        update: update
    };
}

/**
 * Creates a parametric geometry based on a function and domain.
 *
 * @param {Function} function_to_draw - The function to graph. It should take two arguments (x, y) and return a scalar value (z).
 * @param {number} width_segments - The number of segments along the width.
 * @param {number} height_segments - The number of segments along the height.
 * @param {Array<Array<number>>} domain - The domain of the function to graph, as a 2D array [[x_min, x_max], [y_min, y_max]].
 * @returns {ParametricGeometry} The generated parametric geometry.
 */
function create_parameterization(function_to_draw, width_segments, height_segments, domain) {
    return new ParametricGeometry((u, v, target) => {
        const x = domain[0][0] + u * (domain[0][1] - domain[0][0]);
        const y = domain[1][0] + v * (domain[1][1] - domain[1][0]);
        const z = function_to_draw(x, y);
        target.set(x, z, y); // up direction is y
    }, width_segments, height_segments);
}

/**
 * Calculates the visible domain of the camera based on its frustum and a specified distance.
 *
 * @param {THREE.Camera} camera - The camera used to calculate the visible domain.
 * @param {number} z_dist - The distance along the Z-axis.
 * @returns {Array<Array<number>>} The calculated visible domain as a 2D array [[x_min, x_max], [y_min, y_max]].
 */
function calculate_visible_domain(camera, z_dist) {
    const frustumHeight = Math.abs(2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) * z_dist);
    const frustumWidth = Math.abs(frustumHeight * camera.aspect);
    const largest = Math.round(Math.max(frustumWidth, frustumHeight)) + 1;
    return [
        [-largest, largest],
        [-largest, largest]
    ];
}

/**
 * Creates grid lines for visualizing a function over a domain.
 *
 * @param {Function} function_to_draw - The function to graph. It should take two arguments (x, y) and return a scalar value (z).
 * @param {number} width_segments - The number of segments along the width.
 * @param {number} height_segments - The number of segments along the height.
 * @param {Array<Array<number>>} domain - The domain of the function to graph, as a 2D array [[x_min, x_max], [y_min, y_max]].
 * @param {boolean} [draw_axes_only=false] - Whether to draw only the axes lines.
 * @returns {THREE.LineSegments} The generated grid lines.
 */
function create_gridlines(function_to_draw, width_segments, height_segments, domain, draw_axes_only = false) {
    const gridGeometry = new THREE.BufferGeometry();
    const vertices = [];

    // Vertical contours
    for (let i = 0; i <= width_segments; i++) {
        const u = i / width_segments;
        const x = domain[0][0] + u * (domain[0][1] - domain[0][0]);

        if (!draw_axes_only) {
            if (x - Math.floor(x) > 0.001 && Math.ceil(x) - x > 0.001) {
                continue;
            }
        } else {
            if (x !== 0) {
                continue;
            }
        }

        for (let j = 0; j < height_segments; j++) {
            const v1 = j / height_segments;
            const v2 = (j + 1) / height_segments;
            const y1 = domain[1][0] + v1 * (domain[1][1] - domain[1][0]);
            const y2 = domain[1][0] + v2 * (domain[1][1] - domain[1][0]);

            vertices.push(x, function_to_draw(x, y1), y1);
            vertices.push(x, function_to_draw(x, y2), y2);
        }
    }

    // Horizontal contours
    for (let j = 0; j <= height_segments; j++) {
        const v = j / height_segments;
        const y = domain[1][0] + v * (domain[1][1] - domain[1][0]);

        if (!draw_axes_only) {
            if (y - Math.floor(y) > 0.001 && Math.ceil(y) - y > 0.001) {
                continue;
            }
        } else {
            if (y !== 0) {
                continue;
            }
        }

        for (let i = 0; i < width_segments; i++) {
            const u1 = i / width_segments;
            const u2 = (i + 1) / width_segments;
            const x1 = domain[0][0] + u1 * (domain[0][1] - domain[0][0]);
            const x2 = domain[0][0] + u2 * (domain[0][1] - domain[0][0]);

            vertices.push(x1, function_to_draw(x1, y), y);
            vertices.push(x2, function_to_draw(x2, y), y);
        }
    }

    gridGeometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    const gridMaterial = new THREE.LineBasicMaterial({ color: 0x000000 });
    const gridLines = new THREE.LineSegments(gridGeometry, gridMaterial);

    return gridLines;
}

/**
 * Computes the distance from the camera to the origin.
 *
 * @param {THREE.Camera} camera - The camera from which to measure the distance.
 * @returns {number} The distance from the camera to the origin.
 */
export function distance_to_origin(camera) {
    const origin = new THREE.Vector3(0, 0, 0);
    const cameraPosition = camera.position.clone();
    return cameraPosition.distanceTo(origin);
}

/**
 * Draws a 3D function as a surface mesh in the scene.
 *
 * @param {Object} engine - The rendering engine, including the scene, camera, and renderer.
 * @param {Function} function_to_draw - The function to graph. It should take two arguments (x, y) and return a scalar value (z).
 * @param {number} [width_segments=null] - The number of segments along the width.
 * @param {number} [height_segments=null] - The number of segments along the height.
 * @param {Array<Array<number>>} [domain=null] - The domain of the function to graph, as a 2D array [[x_min, x_max], [y_min, y_max]].
 * @param {number} [color=0x00ffff] - The color of the surface mesh.
 * @returns {Object} An object containing the `update` method to refresh the graph.
 */
export function draw_3d_function(engine, function_to_draw, width_segments = null, height_segments = null, domain = null, color = 0x00ffff) {
    let mesh;
    let gridLines;

    /**
     * Updates the graph based on the function and parameters provided.
     *
     * @param {boolean} [changed=true] - Whether the graph should be updated.
     */
    function update(changed = true) {
        if (!changed) {
            return;
        }

        let z_dist = engine.camera.position.z;

        // Create a clipping plane at the specified height
        const clip_height = distance_to_origin(engine.camera) / 2;
        const clipping_plane = new THREE.Plane(new THREE.Vector3(0, -1, 0), clip_height);

        // Enable clipping in the renderer
        engine.renderer.clippingPlanes = [clipping_plane];

        // Calculate domain based on the camera frustum
        let curr_domain = domain;
        if (domain == null) {
            curr_domain = calculate_visible_domain(engine.camera, z_dist);
        }

        // Set default sample number for widthSegments and heightSegments
        let curr_width_segments = width_segments;
        let curr_height_segments = height_segments;
        if (width_segments == null) {
            curr_width_segments = Math.min(
                10 * Math.ceil(curr_domain[0][1] - curr_domain[0][0]),
                1000
            );
        }
        if (height_segments == null) {
            curr_height_segments = Math.min(
                10 * Math.ceil(curr_domain[0][1] - curr_domain[0][0]),
                1000
            );
        }

        if (mesh) {
            engine.scene.remove(mesh);
            mesh.geometry.dispose();
            mesh.material.dispose();
        }

        if (gridLines) {
            engine.scene.remove(gridLines);
            gridLines.geometry.dispose();
            gridLines.material.dispose();
        }

        const geometry = create_parameterization(function_to_draw, curr_width_segments, curr_height_segments, curr_domain);
        const material = new THREE.MeshBasicMaterial({
            color: color,
            transparent: true,
            opacity: 0.7,
            side: THREE.DoubleSide, // Make back faces visible
            clippingPlanes: [clipping_plane]  // Apply clipping plane
        });
        mesh = new THREE.Mesh(geometry, material);

        engine.scene.add(mesh);

        // Create and add gridlines
        let draw_axis_only = false;
        if (clip_height > 20) {
            draw_axis_only = true;
        }
        gridLines = create_gridlines(function_to_draw, curr_width_segments, curr_height_segments, curr_domain, draw_axis_only);
        engine.scene.add(gridLines);
    }

    return {
        update: update
    };
}
