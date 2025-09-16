import * as THREE from 'three';
import { MarchingCubes } from 'three/examples/jsm/objects/MarchingCubes';
import { distance_to_origin } from "./utils_draw_parametric_functions.js";

/**
 * Class representing an implicit function grapher using the Marching Cubes algorithm.
 */
export class ImplicitFunctionGrapher {
    /**
     * Creates an instance of ImplicitFunctionGrapher.
     *
     * @param {Object} engine - The rendering engine, including the scene, camera, and renderer.
     * @param {Function} func - The implicit function to graph. It should take three arguments (x, y, z) and return a scalar value.
     * @param {number} cubes_num - The number of cubes along each axis for the Marching Cubes algorithm.
     * @param {number} domain_size - The size of the domain to be graphed in each dimension.
     * @param {number} [color=0x00ffff] - The color of the graphed surface.
     */
    constructor(
        engine,
        func,
        cubes_num,
        domain_size,
        color = 0x00ffff,
    ) {
        /** @type {Object} */
        this.engine = engine;
        /** @type {Function} */
        this.func = func;
        /** @type {number} */
        this.cubes_num = cubes_num;
        /** @type {number} */
        this.domain_size = domain_size;
        /** @type {number} */
        this.color = color;

        /** @type {MarchingCubes} */
        this.marching_cubes = new MarchingCubes(cubes_num);
        this.marching_cubes.isolation = 0.0; // Sets the iso-surface value for Marching Cubes.

        // Create a clipping plane at the specified height
        const clip_height = distance_to_origin(engine.camera) / 2;
        /** @type {THREE.Plane} */
        const clipping_plane = new THREE.Plane(new THREE.Vector3(0, -1, 0), clip_height);
        engine.renderer.clippingPlanes = [clipping_plane];

        // Configure the material for the Marching Cubes mesh
        this.marching_cubes.material = new THREE.MeshBasicMaterial({
            color: color,
            transparent: true,
            opacity: 0.7,
            side: THREE.DoubleSide, // Make back faces visible
            clippingPlanes: [clipping_plane]  // Apply clipping plane
        });

        // Add the Marching Cubes mesh to the scene
        engine.scene.add(this.marching_cubes);
    }

    /**
     * Updates the plotted implicit function by recalculating the scalar field for each cell in the grid.
     */
    update_funciton_plot() {
        const half_size = this.domain_size / 2;
        for (let x = 0; x < this.cubes_num; x++) {
            for (let y = 0; y < this.cubes_num; y++) {
                for (let z = 0; z < this.cubes_num; z++) {
                    const fx = (x / (this.cubes_num - 1)) * this.domain_size - half_size;
                    console.log(fx);
                    const fy = (y / (this.cubes_num - 1)) * this.domain_size - half_size;
                    const fz = (z / (this.cubes_num - 1)) * this.domain_size - half_size;

                    const value = this.func(fx, fy, fz);
                    this.marching_cubes.setCell(x, y, z, value);
                }
            }
        }

        // Update the Marching Cubes mesh with the new values
        this.marching_cubes.update();
        this.marching_cubes.scale.set(this.domain_size, this.domain_size, this.domain_size);
    }
}
