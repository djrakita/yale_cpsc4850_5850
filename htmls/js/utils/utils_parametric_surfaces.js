/**
 * Author: Danny Rakita
 * Description: For CPSC-487-587 3D Spatial Modeling and Computing at Yale University
 */

import * as THREE from 'three';
import {
    convert_y_up_array_to_z_up_array,
    convert_z_up_array_to_y_up_array,
    get_default_lil_gui,
    spawn_line_specific
} from "./utils_three.js";
import {
    add_matrix_matrix,
    cross_product, frobenius_norm_matrix, mul_matrix_matrix,
    mul_matrix_scalar,
    normalized_matrix, roll_list_into_column_vec_matrix,
    sub_matrix_matrix, unroll_matrix_to_list
} from "./utils_math.js";
import {ParametricGeometry} from 'three/examples/jsm/geometries/ParametricGeometry.js';
import {
    optimization_bfgs,
    optimization_gradient_descent, optimization_powell,
} from "./utils_optimization.js";

/**
 * Base class for creating specific parametric surfaces in 3D space.
 * This is a template class and should not be instantiated directly.
 */
export class ParametricSurfaceBaseClass {
    constructor() {
        if (new.target === ParametricSurfaceBaseClass) {
            throw new Error("ParametricSurfaceBaseClass is a template class and cannot be instantiated directly.");
        }
        /**
         * Initializes the raw parametric function.
         * @type {Function}
         */
        this.raw_parametric_function = this.get_raw_parametric_function();
        /**
         * Initializes the raw parametric function in Z-up coordinate system.
         * @type {Function}
         */
        this.raw_parametric_function_z_up = this.get_raw_parametric_function_z_up();
    }

    // returns a function that takes in two parameters (u and v) that both vary on the range 0-1 and returns a
    // point in space [x, y, z].
    /**
     * Abstract method to return a function that represents the parametric surface.
     * Should be implemented by derived classes.
     * @abstract
     * @returns {Function} A function that takes parameters `u` and `v` and returns a point [x, y, z] on the surface.
     */
    get_raw_parametric_function() {
        throw new Error("Method 'get_raw_parametric_function()' must be implemented in the derived class.");
    }

    /**
     * Converts the Z-up coordinate system to Y-up.
     * @returns {Function} A function that takes parameters `u` and `v` and returns a point [x, y, z] on the surface.
     */
    get_raw_parametric_function_z_up() {
        return (u, v) => {
            return convert_z_up_array_to_y_up_array(this.raw_parametric_function(u, v));
        }
    }

    /**
     * Returns a function compatible with Three.js's ParametricGeometry.
     * @returns {Function} A function that sets a target vector to a point [x, y, z] on the surface.
     */
    get_three_parametric_function() {
        return (u, v, target) => {
            let point = this.raw_parametric_function_z_up(u, v);
            target.set(point[0], point[1], point[2]);
        }
    }

    /**
     * Projects a given point onto the surface using optimization techniques.
     * @param {number[]} point - The 3D point to project onto the surface.
     * @param {number} starting_u - The initial guess for the parameter `u`.
     * @param {number} starting_v - The initial guess for the parameter `v`.
     * @param {number} [max_iter=500] - The maximum number of iterations for the optimization.
     * @returns {[number, number, number[]]} A tuple `[u, v, point_on_surface]` where `u` and `v` are parameters on the surface and `point_on_surface` is the closest point on the surface.
     */
    project_onto_surface(point, starting_u, starting_v, max_iter=500) {
        let f = x => {
            let a = this.raw_parametric_function(x[0], x[1]);
            let dis = frobenius_norm_matrix(sub_matrix_matrix(a, point));
            return dis*dis;
        }

        let solution = optimization_gradient_descent(f, [starting_u, starting_v], max_iter);

        let uv = solution;
        let u = uv[0];
        let v = uv[1];

        return [ u, v, this.raw_parametric_function(u, v) ];
    }

    /*
    project_onto_surface_parallel_to_vector_from_u_v(starting_u, starting_v, vector) {
        let starting_point = this.raw_parametric_function(starting_u, starting_v);
        return this.project_onto_surface_parallel_to_vector(starting_point, vector);
    }

    project_onto_surface_parallel_to_vector(starting_point, vector, max_iter=100) {
        let ss = this.get_surface_normal_vector_at_point(0.000001, 0.0000001);
        let a = cross_product(ss, vector);
        let b = cross_product(a, vector);

        // let proj = this.project_onto_surface(add_matrix_matrix(starting_point, vector), starting_u, starting_v)[2];
        let f = x => {
            let t = x[0];

            let p1 = add_matrix_matrix(add_matrix_matrix(starting_point, vector), mul_matrix_scalar(b, t));
            let proj1 = this.project_onto_surface(p1, 0.5, 0.5)[2];

            let sub1 = sub_matrix_matrix(p1, proj1);
            return frobenius_norm_matrix(sub1);
        }

        let solution = optimization_gradient_descent(f, [0.5], max_iter);
        let t = solution[0];

        return add_matrix_matrix(add_matrix_matrix(starting_point, vector), mul_matrix_scalar(b, t));
    }
    */

    /**
     * Rotates a vector onto the surface, optimizing for the closest point on the surface.
     * @param {number[]} starting_point - The starting point on the surface.
     * @param {number[]} direction - The direction vector to rotate.
     * @returns {[number[], number[]]} A tuple `[rotated_vector, new_point]` where `rotated_vector` is the rotated vector and `new_point` is the corresponding point on the surface.
     */
    rotate_vector_onto_surface(starting_point, direction) {
        let direction_length = frobenius_norm_matrix(direction);
        let normalized_direction = normalized_matrix(direction);
        let up = [[0], [0], [1]];
        let side = normalized_matrix(cross_product(normalized_direction, up));
        let z = normalized_matrix(cross_product(normalized_direction, side));
        let x = normalized_direction;
        let y = side;
        let mat = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]];

        let f = x => {
            let theta = x[0];
            let rot = [[Math.cos(theta), 0, Math.sin(theta)], [0, 1, 0], [-Math.sin(theta), 0, Math.cos(theta)]];
            let rr = mul_matrix_matrix(mat, rot);
            let new_x = [ [rr[0][0]], [rr[1][0]], [rr[2][0]]  ];
            new_x = normalized_matrix(new_x);
            let scaled_new_x = mul_matrix_scalar(new_x, direction_length);
            let new_point = add_matrix_matrix(starting_point, scaled_new_x);
            let proj = this.project_onto_surface(new_point, 0.1, 0.1)[2];
            let sub = sub_matrix_matrix(proj, new_point);
            let norm = frobenius_norm_matrix(sub);
            return norm*norm;
        }

        let solution = optimization_bfgs(f, [0.0], 200);
        let theta = solution[0];
        let rot = [[Math.cos(theta), 0, Math.sin(theta)], [0, 1, 0], [-Math.sin(theta), 0, Math.cos(theta)]];
        let rr = mul_matrix_matrix(mat, rot);
        let new_x = [ [rr[0][0]], [rr[1][0]], [rr[2][0]]  ];
        new_x = normalized_matrix(new_x);
        let scaled_new_x = mul_matrix_scalar(new_x, direction_length);
        let new_point = add_matrix_matrix(starting_point, scaled_new_x);

        return [scaled_new_x, new_point];
    }

    /**
     * Returns two normalized vectors that span the surface at a given point (u, v).
     * @param {number} u - The parameter along the surface.
     * @param {number} v - The parameter along the surface.
     * @returns {[number[], number[]]} A tuple `[vector_u, vector_v]` where `vector_u` and `vector_v` are the spanning vectors.
     */
    get_surface_spanning_vectors_at_point(u, v) {
        let uu = u + 0.0001;
        let vv = v + 0.0001;

        let point = this.raw_parametric_function(u, v);
        let uu_point = this.raw_parametric_function(uu, v);
        let vv_point = this.raw_parametric_function(u, vv);

        let a1 = normalized_matrix(sub_matrix_matrix(uu_point, point));
        let a2 = normalized_matrix(sub_matrix_matrix(vv_point, point));

        return [a1, a2];
    }

    /**
     * Returns a tangent vector at a point on the surface in a specified direction.
     * @param {number} u - The parameter `u` on the surface.
     * @param {number} v - The parameter `v` on the surface.
     * @param {number} da - The change in `u` direction.
     * @param {number} db - The change in `v` direction.
     * @returns {number[]} The normalized tangent vector at the point.
     */
    get_tangent_vector_at_point_in_direction(u, v, da, db) {
        let uu = u + 0.000001;
        let vv = v + 0.000001;

        da *= 0.00001;
        db *= 0.00001;

        let point = this.raw_parametric_function(u, v);
        let uu_point = this.raw_parametric_function(uu, v);
        let vv_point = this.raw_parametric_function(u, vv);

        let distance_per_unit_in_u_dir = frobenius_norm_matrix(sub_matrix_matrix(uu_point, point)) / 0.000001;
        let distance_per_unit_in_v_dir = frobenius_norm_matrix(sub_matrix_matrix(vv_point, point)) / 0.000001;

        let du = da / distance_per_unit_in_u_dir;
        let dv = db / distance_per_unit_in_v_dir;

        let new_point = this.raw_parametric_function(u + du, v + dv);

        return normalized_matrix(sub_matrix_matrix(new_point, point));
    }

    /**
     * Returns the surface normal vector at a given point (u, v).
     * @param {number} u - The parameter `u` on the surface.
     * @param {number} v - The parameter `v` on the surface.
     * @returns {number[]} The normal vector at the point.
     */
    get_surface_normal_vector_at_point(u, v) {
        let [a1, a2] = this.get_surface_spanning_vectors_at_point(u, v);

        return cross_product(a1, a2);
    }

    /**
     * Spawns a static visualization of the parametric surface in a Three.js scene.
     * @param {Object} scene - The Three.js scene to add the surface to.
     * @param {number} [slices=100] - The number of slices in the parametric surface.
     * @param {number} [stacks=100] - The number of stacks in the parametric surface.
     * @param {number} [color=0x0000ff] - The color of the surface.
     * @param {number} [opacity=1.0] - The opacity of the surface.
     * @returns {Object} The Three.js mesh representing the surface.
     */
    spawn_static_parametric_surface(scene, slices=100, stacks=100, color=0x0000ff, opacity=1.0) {
        let geometry = new ParametricGeometry(this.get_three_parametric_function(), 200, 200);

        let material = new THREE.MeshStandardMaterial({ color: color, side: THREE.DoubleSide });

        let mesh = new THREE.Mesh(geometry, material);
        mesh.material.transparent = true;
        mesh.material.opacity = opacity;

        scene.add(mesh);

        return mesh;
    }

    /**
     * Draws static curves on the parametric surface in a Three.js scene.
     * @param {Object} scene - The Three.js scene to add the curves to.
     * @param {number} [num_us=30] - The number of curves in the `u` direction.
     * @param {number} [num_vs=30] - The number of curves in the `v` direction.
     * @param {number} [color=0x555555] - The color of the curves.
     * @param {number} [opacity=0.3] - The opacity of the curves.
     */
    draw_static_curves(scene, num_us=30, num_vs=30, color=0x555555, opacity=0.3) {
        let width = 0.002;
        let num_samples_per = 75;
        for(let i = 0; i < num_us; i++) {
            for(let j = 0; j < num_samples_per; j++) {
                let u = i / num_us;
                let v_curr = j / num_samples_per;
                let v_next = (j+1) / num_samples_per;

                let p1 = this.raw_parametric_function(u, v_curr);
                let p2 = this.raw_parametric_function(u, v_next);

                spawn_line_specific(scene, p1, p2, false, width, color, opacity);
            }
        }

        for(let i = 0; i < num_samples_per; i++) {
            for(let j = 0; j < num_vs; j++) {
                let u_curr = i / num_samples_per;
                let u_next = (i+1) / num_samples_per;
                let v = j / num_vs;

                let p1 = this.raw_parametric_function(u_curr, v);
                let p2 = this.raw_parametric_function(u_next, v);

                spawn_line_specific(scene, p1, p2, false, width, color, opacity);
            }
        }
    }
}

/**
 * A class to visualize parametric surfaces using Three.js.
 */
export class ParametricSurfaceThreeVisualizer {
    /**
     * Creates an instance of ParametricSurfaceThreeVisualizer.
     * @param {ParametricSurfaceBaseClass} parametric_surface - An instance of a parametric surface.
     * @param {number} [starting_u=0.5] - Initial value for the `u` parameter.
     * @param {number} [starting_v=0.5] - Initial value for the `v` parameter.
     * @param {boolean} [draw_tangent_space=false] - Whether to draw the tangent space vectors.
     * @param {boolean} [draw_point_plus_tangent_space=false] - Whether to draw both the point and tangent space vectors.
     * @param {boolean} [uv_sliders=true] - Whether to include sliders for adjusting `u` and `v`.
     */
    constructor(parametric_surface, starting_u=0.5, starting_v=0.5, draw_tangent_space=false, draw_point_plus_tangent_space= false, uv_sliders=true) {
        this.parametric_surface = parametric_surface;
        /**
         * Settings for the visualizer.
         * @type {Object}
         */
        this.settings = {
            u: starting_u,
            v: starting_v,
            draw_tangent_space: draw_tangent_space,
            draw_point_plus_tangent_space: draw_point_plus_tangent_space,
            tangent_space_vector_length: 0.25
        };
        /**
         * GUI for controlling the visualizer.
         * @type {Object}
         */
        let gui = get_default_lil_gui();
        if (uv_sliders) {
            gui.add(this.settings, 'u', 0.0000001, 1).name('u');
            gui.add(this.settings, 'v', 0.0000001, 1).name('v');
        }
        gui.add(this.settings, 'draw_tangent_space').name('Draw Tangent Space');
        gui.add(this.settings, 'draw_point_plus_tangent_space').name('Draw Point + Tangent Space');
        gui.add(this.settings, 'tangent_space_vector_length', 0.1, 1.0).name('Tangent Vec. Size');
        this.gui = gui;
    }

    /**
     * The main loop function for updating the visualization.
     * @param {Object} three_engine - The Three.js engine instance.
     */
    three_loop_function(three_engine) {
        let point = this.parametric_surface.raw_parametric_function(this.settings.u, this.settings.v);
        three_engine.draw_debug_sphere(point, 0.04, 0x00eeff);

        if (this.settings.draw_point_plus_tangent_space) {
            let ss = this.parametric_surface.get_surface_spanning_vectors_at_point(this.settings.u, this.settings.v);
            let t1 = add_matrix_matrix(point, mul_matrix_scalar(ss[0], this.settings.tangent_space_vector_length));
            let t2 = add_matrix_matrix(point, mul_matrix_scalar(ss[1], this.settings.tangent_space_vector_length));
            three_engine.draw_debug_vector(point, t1, 0.012, undefined, 0x777788);
            three_engine.draw_debug_vector(point, t2, 0.012, undefined, 0x777788);
            three_engine.draw_debug_grid_plane(point, ss[0], ss[1], this.settings.tangent_space_vector_length*2, this.settings.tangent_space_vector_length*2, 0x111144, 0.2);
        }

        if (this.settings.draw_tangent_space) {
            let ss = this.parametric_surface.get_surface_spanning_vectors_at_point(this.settings.u, this.settings.v);
            let t1 = add_matrix_matrix([0,0,0], mul_matrix_scalar(ss[0], this.settings.tangent_space_vector_length));
            let t2 = add_matrix_matrix([0,0,0], mul_matrix_scalar(ss[1], this.settings.tangent_space_vector_length));
            three_engine.draw_debug_vector([0,0,0], t1, 0.012, undefined, 0x777788);
            three_engine.draw_debug_vector([0,0,0], t2, 0.012, undefined, 0x777788);
            three_engine.draw_debug_grid_plane([0,0,0], ss[0], ss[1], this.settings.tangent_space_vector_length*2, this.settings.tangent_space_vector_length*2, 0x111144, 0.2);
        }
    }
}

/**
 * A class that utilizes Lie groups and Lie algebras to manipulate and visualize parametric surfaces.
 */
export class ParametricSurfaceLieGroupAndAlgebraUtil {
    /**
     * Creates an instance of ParametricSurfaceLieGroupAndAlgebraUtil.
     * @param {ParametricSurfaceBaseClass} parametric_surface - An instance of a parametric surface.
     * @param {number} [starting_u=0.1] - Initial value for the `u` parameter.
     * @param {number} [starting_v=0.1] - Initial value for the `v` parameter.
     * @param {boolean} [fixed_uv=false] - Whether the `u` and `v` parameters should be fixed.
     */
    constructor(parametric_surface, starting_u=0.1, starting_v=0.1, fixed_uv = false) {
        this.parametric_surface = parametric_surface;
        this.outdated = true;
        this.exp_x_points = [];
        this.actions = {
            compute_exp: () => {
                this.outdated = false;
                this.exp_x_points = get_surface_exp_x_points(this.parametric_surface, this.settings.starting_u, this.settings.starting_v, this.settings.a, this.settings.b);
            }
        }
        /**
         * Settings for the visualizer.
         * @type {Object}
         */
        this.settings = {
            starting_u: starting_u,
            starting_v: starting_v,
            a:0.5,
            b:0.5,
            t:0,
            display_tangent_space: true
        }
        /**
         * GUI for controlling the visualizer.
         * @type {Object}
         */
        let gui = get_default_lil_gui();
        if(!fixed_uv) {
            gui.add(this.settings, 'starting_u', 0.0000001, 1).name('Starting u').onChange(() => { this.outdated = true })
            gui.add(this.settings, 'starting_v', 0.0000001, 1).name('Staring v').onChange(() => { this.outdated = true });
        }
        gui.add(this.settings, 'a', -10.0, 10.0).name('Tangent x').onChange(() => { this.outdated = true });
        gui.add(this.settings, 'b', -10.0, 10.0).name('Tangent y').onChange(() => { this.outdated = true });
        gui.add(this.settings, 't', 0, 1).name('t');
        gui.add(this.settings, 'display_tangent_space').name('Show Tangent Space');
        gui.add(this.actions, 'compute_exp').name('Compute Exponential');
        this.gui = gui;
    }

    /**
     * The main loop function for updating the visualization.
     * @param {Object} three_engine - The Three.js engine instance.
     */
    three_loop_function(three_engine) {
        let origin_point = this.parametric_surface.raw_parametric_function(this.settings.starting_u, this.settings.starting_v);
        three_engine.draw_debug_sphere(origin_point, 0.01, 0x00eeff);
        if (this.settings.display_tangent_space) {
            let ss = this.parametric_surface.get_surface_spanning_vectors_at_point(this.settings.starting_u, this.settings.starting_v);
            let span_vec_1 = ss[0];
            let span_vec_2 = ss[1];

            three_engine.draw_debug_grid_plane(origin_point, span_vec_1, span_vec_2, Math.max(Math.abs(this.settings.a) * 1.5, 1.0), Math.max(Math.abs(this.settings.b) * 1.5, 1.0));
            three_engine.draw_debug_vector(origin_point, add_matrix_matrix(span_vec_1, origin_point), 0.02, undefined, 0xffaaaa);
            three_engine.draw_debug_vector(origin_point, add_matrix_matrix(span_vec_2, origin_point), 0.02, undefined, 0xaaffaa);
        }

        let exp_0_points = get_surface_exp_0_points(this.parametric_surface, this.settings.starting_u, this.settings.starting_v, this.settings.a, this.settings.b);
        for(let i=0; i<exp_0_points.length-1; i++) {
            three_engine.draw_debug_line(exp_0_points[i], exp_0_points[i+1], true, 0.015, 0xffcc00);
        }

        if(!this.outdated) {
            let exp_tx_points = get_surface_exp_tx_points(this.parametric_surface, this.settings.t, exp_0_points, this.exp_x_points);
            for(let i=0; i<exp_0_points.length-1; i++) {
                three_engine.draw_debug_line(exp_tx_points[i], exp_tx_points[i+1], true, 0.025, 0xaaff44);
            }
            three_engine.draw_debug_sphere(exp_tx_points[exp_tx_points.length-1], 0.04, 0x000000);
        }
    }
}

/*
export function get_surface_exp_x_points2(parametric_surface, exp_0_points, segment_length=0.05) {
    let start_point = exp_0_points[0];
    let end_point = exp_0_points[exp_0_points.length-1];

    let dir = sub_matrix_matrix(end_point, start_point);
    let dir_l = frobenius_norm_matrix(dir);
    let dir_n = normalized_matrix(dir);
    let dir_s = mul_matrix_scalar(dir_n, segment_length);
    let curr_u = 0.000001;
    let curr_v = 0.000001;

    let out = [];

    exp_0_points.forEach(exp_0_point => {
        let res = parametric_surface.project_onto_surface(exp_0_point, curr_u, curr_v);
        curr_u = res[0];
        curr_v = res[1];
        out.push(res[2]);
    });

    return out;
}
*/

/*
export function get_surface_exp_x_points(parametric_surface, u, v, segment_length=0.05) {
    let ss = parametric_surface.get_surface_spanning_vectors_at_point(0.0000001, 0.0000001);
    let raw_parametric_function = parametric_surface.raw_parametric_function;
    let start_point = raw_parametric_function(0.000001, 0.000001);
    let end_point = add_matrix_matrix(add_matrix_matrix(mul_matrix_scalar(ss[0], u), mul_matrix_scalar(ss[1], v)), start_point);

    let dir = sub_matrix_matrix(end_point, start_point);
    let dir_l = frobenius_norm_matrix(dir);
    let dir_n = normalized_matrix(dir);
    let dir_s = mul_matrix_scalar(dir_n, segment_length);

    let out = [];
    let curr_point = start_point.slice();
    let curr_length = 0.0;
    // let curr_u = 0.001;
    // let curr_v = 0.001;

    while (true) {
        out.push(curr_point.slice());
        let curr_dir;
        if (curr_length === 0.0) {
            curr_dir = dir_s;
        } else {
            let tmp1 = sub_matrix_matrix(curr_point, out[out.length-2]);
            let tmp2 = normalized_matrix(tmp1);
            curr_dir = mul_matrix_scalar(tmp2, segment_length);
            // curr_dir = dir_s;
        }

        let cast_out_point = add_matrix_matrix(curr_point, curr_dir);
        // let projection_result = parametric_surface.project_onto_surface(cast_out_point, curr_u, curr_v);
        let projected_point = parametric_surface.project_onto_surface_parallel_to_vector(0.0001, 0.0001, curr_dir);
        // curr_u = projection_result[0];
        // curr_v = projection_result[1];
        // let projected_point = projection_result[2];
        curr_point = projected_point;
        // let new_dir = sub_matrix_matrix(projected_point, curr_point);
        // let new_dir_n = normalized_matrix(new_dir);
        // let new_dir_s = mul_matrix_scalar(new_dir_n, segment_length);
        // curr_point = add_matrix_matrix(curr_point, new_dir_s);
        curr_length+=segment_length;
        if (curr_length > dir_l) {
            break;
        }
    }

    return out;
}
*/

/*
export function get_surface_exp_x_point3(parametric_surface, a, b, segment_length=0.07) {
    let ss = parametric_surface.get_surface_spanning_vectors_at_point(0.0000001, 0.0000001);
    let raw_parametric_function = parametric_surface.raw_parametric_function;
    let start_point = raw_parametric_function(0.001, 0.001);
    let end_point = add_matrix_matrix(add_matrix_matrix(mul_matrix_scalar(ss[0], a), mul_matrix_scalar(ss[1], b)), start_point);

    let dir = sub_matrix_matrix(end_point, start_point);
    let dir_l = frobenius_norm_matrix(dir);
    let dir_n = normalized_matrix(dir);
    let dir_s = mul_matrix_scalar(dir_n, segment_length);

    let out = [];
    let curr_point = start_point.slice();
    let curr_length = 0.0;

    let count = 0;
    while(true) {
        out.push(curr_point.slice());
        let curr_dir;
        if (out.length <= 2) {
            curr_dir = dir_s;
        } else {
            let tmp1 = sub_matrix_matrix(curr_point, out[out.length-2]);
            let tmp2 = normalized_matrix(tmp1);
            curr_dir = mul_matrix_scalar(tmp2, segment_length);
        }

        curr_length+=segment_length;
        let parallel_point = parametric_surface.project_onto_surface_parallel_to_vector(curr_point,  curr_dir, 300);
        let new_dir = sub_matrix_matrix(parallel_point, curr_point);
        let new_dir_n = normalized_matrix(new_dir);
        let new_dir_s = mul_matrix_scalar(new_dir_n, segment_length);
        if (curr_length > dir_l) {
            let r = dir_l % segment_length;
            let new_dir_s = mul_matrix_scalar(new_dir_n, r);
            curr_point = add_matrix_matrix(curr_point, new_dir_s);
            out.push(curr_point);
            break;
        }
        curr_point = add_matrix_matrix(curr_point, new_dir_s);
        count += 1;
    }

    return out;
}
*/

/**
 * Generates the points of the exponential map for a given parametric surface and parameter `t`.
 * @param {ParametricSurfaceBaseClass} parametric_surface - The parametric surface object.
 * @param {number} t - The interpolation parameter.
 * @param {number[][]} exp_0_points - The initial exponential points.
 * @param {number[][]} exp_x_points - The exponential points with parameter `x`.
 * @param {number} [segment_length=0.035] - The length of each segment.
 * @returns {number[][]} The list of points representing the exponential map.
 */
export function get_surface_exp_tx_points(parametric_surface, t, exp_0_points, exp_x_points, segment_length=0.035) {
    let out = [exp_0_points[0].slice()];

    let curr_point = exp_0_points[0].slice();
    for(let i = 0; i < exp_0_points.length-1; i++) {
        let v1 = sub_matrix_matrix(exp_0_points[i+1], exp_0_points[i]);
        let v2 = sub_matrix_matrix(exp_x_points[i+1], exp_x_points[i]);

        let v = add_matrix_matrix(  mul_matrix_scalar(v1, 1-t), mul_matrix_scalar(v2, t)  );
        v = mul_matrix_scalar(normalized_matrix(v), segment_length);
        curr_point = add_matrix_matrix(curr_point, v);
        out.push(curr_point.slice());
    }

    return out;
}

/**
 * Generates the points of the exponential map at a given parameter `x` on the parametric surface.
 * @param {ParametricSurfaceBaseClass} parametric_surface - The parametric surface object.
 * @param {number} starting_u - The starting parameter `u`.
 * @param {number} starting_v - The starting parameter `v`.
 * @param {number} a - The change in `u` direction.
 * @param {number} b - The change in `v` direction.
 * @param {number} [segment_length=0.035] - The length of each segment.
 * @returns {number[][]} The list of points representing the exponential map at `x`.
 */
export function get_surface_exp_x_points(parametric_surface, starting_u, starting_v, a, b, segment_length=0.035) {
    let ss = parametric_surface.get_surface_spanning_vectors_at_point(starting_u, starting_v);
    let raw_parametric_function = parametric_surface.raw_parametric_function;
    let start_point = raw_parametric_function(starting_u, starting_v);
    let end_point = add_matrix_matrix(add_matrix_matrix(mul_matrix_scalar(ss[0], a), mul_matrix_scalar(ss[1], b)), start_point);

    let dir = sub_matrix_matrix(end_point, start_point);
    let dir_l = frobenius_norm_matrix(dir);
    let dir_n = normalized_matrix(dir);
    let dir_s = mul_matrix_scalar(dir_n, segment_length);
    let r = dir_l % segment_length;

    let curr_point = start_point.slice();
    let curr_dir = dir_s;
    let curr_length = 0.0;
    let out = [];

    while(true) {
        out.push(curr_point);

        let res = parametric_surface.rotate_vector_onto_surface(curr_point, curr_dir);
        curr_dir = res[0].slice();
        curr_point = res[1].slice();

        curr_length += segment_length;
        if (curr_length > dir_l) { out.push(curr_point); break; }
    }
    // curr_dir = normalized_matrix(curr_dir);
    // curr_dir = mul_matrix_scalar(curr_dir, r);
    // let res = parametric_surface.rotate_vector_onto_surface(curr_point, curr_dir);
    // curr_point = res[1].slice();
    // out.push(curr_point);

    return out;
}

/*
export function get_surface_exp_x_points(parametric_surface, starting_u, starting_v, da, db, segment_length = 0.001) {
    let ss = parametric_surface.get_surface_spanning_vectors_at_point(starting_u, starting_v);
    let raw_parametric_function = parametric_surface.raw_parametric_function;
    let start_point = raw_parametric_function(starting_u, starting_v);
    let end_point = add_matrix_matrix(add_matrix_matrix(mul_matrix_scalar(ss[0], da), mul_matrix_scalar(ss[1], db)), start_point);

    let dir = sub_matrix_matrix(end_point, start_point);
    let dir_l = frobenius_norm_matrix(dir);
    let dir_n = normalized_matrix(dir);
    let r = dir_l % segment_length;

    let curr_length = 0.0;
    let curr_param_u = starting_u;
    let curr_param_v = starting_v;
    let curr_point = parametric_surface.raw_parametric_function(starting_u, starting_v);
    let out = [curr_point.slice()];
    let exit = false;

    while (true) {
        let aa = parametric_surface.raw_parametric_function(curr_param_u, curr_param_v);
        let uu = parametric_surface.raw_parametric_function(curr_param_u+0.0001, curr_param_v);
        let vv = parametric_surface.raw_parametric_function(curr_param_u, curr_param_v+0.0001);
        let distance_u = frobenius_norm_matrix(sub_matrix_matrix(uu, aa));
        let distance_v = frobenius_norm_matrix(sub_matrix_matrix(vv, aa));
        let distance_per_unit_u = distance_u / 0.0001;
        let distance_per_unit_v = distance_v / 0.0001;
        // console.log(distance_per_unit_u, distance_per_unit_v);
        if (curr_length + segment_length > dir_l) { exit = true; segment_length = r; }
        // let tangent_vector = parametric_surface.get_tangent_vector_at_point_in_direction(curr_param_u, curr_param_v, da, db);
        // let scaled_tangent_vector = mul_matrix_scalar(tangent_vector, segment_length);
        // console.log(dir_s);
        // let sss = unroll_matrix_to_list(mul_matrix_scalar(normalized_matrix([da, db]), segment_length));

        let slu = distance_per_unit_u*da*0.003;
        let slv = distance_per_unit_v*db*0.003;
        curr_param_u += slu;
        curr_param_v += slv;
        // let sl = da / distance_per_unit_u
        // curr_param_u += sl[0];
        // curr_param_v += sl[1];

        curr_point = parametric_surface.raw_parametric_function(curr_param_u, curr_param_v);
        // let distance = frobenius_norm_matrix(sub_matrix_matrix(new_point, curr_point));
        curr_length += segment_length;
        out.push(curr_point.slice());
        if (exit) { break; }
    }

    return out;
}
*/

/**
 * Generates the points of the exponential map at the initial parameters `u` and `v` on the parametric surface.
 * @param {ParametricSurfaceBaseClass} parametric_surface - The parametric surface object.
 * @param {number} starting_u - The starting parameter `u`.
 * @param {number} starting_v - The starting parameter `v`.
 * @param {number} a - The change in `u` direction.
 * @param {number} b - The change in `v` direction.
 * @param {number} [segment_length=0.035] - The length of each segment.
 * @returns {number[][]} The list of points representing the exponential map at `u=0`.
 */
export function get_surface_exp_0_points(parametric_surface, starting_u, starting_v, a, b, segment_length=0.035) {
    let ss = parametric_surface.get_surface_spanning_vectors_at_point(starting_u, starting_v);
    let raw_parametric_function = parametric_surface.raw_parametric_function;
    let start_point = raw_parametric_function(starting_u, starting_v);
    let end_point = add_matrix_matrix(add_matrix_matrix(mul_matrix_scalar(ss[0], a), mul_matrix_scalar(ss[1], b)), start_point);

    let dir = sub_matrix_matrix(end_point, start_point);
    let dir_l = frobenius_norm_matrix(dir);
    let dir_n = normalized_matrix(dir);
    let dir_s = mul_matrix_scalar(dir_n, segment_length);

    let out = [];
    let curr_point = start_point.slice();
    let curr_length = 0.0;
    while (true) {
        out.push(curr_point.slice());
        curr_point = add_matrix_matrix(curr_point, dir_s);
        curr_length += segment_length;
        if(curr_length > dir_l) { break; }
    }
    out.push(end_point);

    return out;
}

/**
 * A parametric surface representing a sphere in 3D space.
 * Inherits from ParametricSurfaceBaseClass.
 */
export class ParametricSurfaceSphere extends ParametricSurfaceBaseClass {
    // r1 is distance from center of tube to center of torus
    // r2 is radius of the tube
    /**
     * Creates an instance of ParametricSurfaceSphere.
     * @param {number} [r=1] - The radius of the sphere.
     */
    constructor(r = 1) {
        super();
        this.r = 1;
    }

    /**
     * Returns the raw parametric function for the sphere.
     * @returns {Function} A function that takes parameters `u` and `v` and returns a point [x, y, z] on the sphere.
     */
    get_raw_parametric_function() {
        return (u, v) => {
            u *= Math.PI * 2; // u ranges from 0 to 2π
            v *= Math.PI * 2; // v ranges from 0 to 2π

            const x = this.r*Math.sin(u)*Math.cos(v);
            const y = this.r*Math.sin(u)*Math.sin(v);
            const z = this.r*Math.cos(u);

            return [x, y, z];
        }
    }
}

/**
 * A parametric surface representing a torus in 3D space.
 * Inherits from ParametricSurfaceBaseClass.
 */
export class ParametricSurfaceTorus extends ParametricSurfaceBaseClass {
    // r1 is distance from center of tube to center of torus
    // r2 is radius of the tube
    /**
     * Creates an instance of ParametricSurfaceTorus.
     * @param {number} [r1=2] - The distance from the center of the tube to the center of the torus.
     * @param {number} [r2=0.3] - The radius of the tube.
     */
    constructor(r1 = 2, r2 = 0.3) {
        super();
        this.r1 = r1;
        this.r2 = r2;
    }

    /**
     * Returns the raw parametric function for the torus.
     * @returns {Function} A function that takes parameters `u` and `v` and returns a point [x, y, z] on the torus.
     */
    get_raw_parametric_function() {
        return (u, v) => {
            u *= Math.PI * 2; // u ranges from 0 to 2π
            v *= Math.PI * 2; // v ranges from 0 to 2π

            const x = (this.r1 + this.r2 * Math.cos(v)) * Math.cos(u);
            const y = (this.r1 + this.r2 * Math.cos(v)) * Math.sin(u);
            const z = this.r2 * Math.sin(v);

            return [x, y, z];
        }
    }
}

/**
 * A parametric surface representing wave-like patterns in 3D space.
 * Inherits from ParametricSurfaceBaseClass.
 */
export class ParametricSurfaceWaves extends ParametricSurfaceBaseClass {
    /**
     * Creates an instance of ParametricSurfaceWaves.
     * @param {number} [w=15] - Width parameter for the waves.
     * @param {number} [h=15] - Height parameter for the waves.
     */
    constructor(w=15, h=15) {
        super();
        this.w = w;
        this.h = h;
    }

    /**
     * Returns the raw parametric function for the wave surface.
     * @returns {Function} A function that takes parameters `u` and `v` and returns a point [x, y, z] on the wave surface.
     */
    get_raw_parametric_function() {
        return (u, v) => {
            u -= 0.5;
            v -= 0.5;

            u *= this.h;
            v *= this.w;

            const x = u
            const y = v

            const z = Math.cos(u) + Math.sin(v);

            return [x, y, z];
        }
    }
}

/**
 * A parametric surface representing a Möbius strip in 3D space.
 * Inherits from ParametricSurfaceBaseClass.
 */
export class ParametricSurfaceMobiusStrip extends ParametricSurfaceBaseClass {
    /**
     * Creates an instance of ParametricSurfaceMobiusStrip.
     */
    constructor() {
        super();
    }

    /**
     * Returns the raw parametric function for the Möbius strip.
     * @returns {Function} A function that takes parameters `u` and `v` and returns a point [x, y, z] on the Möbius strip.
     */
    get_raw_parametric_function() {
        return (u, v) => {
            u -= 0.5;
            u *= 2.0;
            u *= Math.PI;
            v -= 0.5;
            v *= 2.0;

            const x = Math.cos(u)*(2 + v*Math.cos(u/2));
            const y = Math.sin(u)*(2 + v*Math.cos(u/2));
            const z = v*Math.sin(u/2);

            return [x, y, z];
        }
    }
}

/**
 * A parametric surface representing a Klein bottle in 3D space.
 * Inherits from ParametricSurfaceBaseClass.
 */
export class ParametricSurfaceKleinBottle extends ParametricSurfaceBaseClass {
    /**
     * Creates an instance of ParametricSurfaceKleinBottle.
     * @param {number} [r1=2] - The radius of the tube of the Klein bottle.
     * @param {number} [r2=0.3] - The radius of the cross-section of the tube.
     */
    constructor(r1 = 2, r2 = 0.3) {
        super();
        this.R = r1; // Radius of the tube of the Klein bottle
        this.r = r2; // Radius of the cross-section of the tube
    }

    /**
     * Returns the raw parametric function for the Klein bottle.
     * @returns {Function} A function that takes parameters `u` and `v` and returns a point [x, y, z] on the Klein bottle.
     */
    get_raw_parametric_function() {
        return (u, v) => {
            u *= Math.PI * 2; // u ranges from 0 to 2π
            v *= Math.PI * 2; // v ranges from 0 to 2π

            const x = (this.R + this.r * Math.cos(u / 2) * Math.sin(v) - this.r * Math.sin(u / 2) * Math.sin(2 * v)) * Math.cos(u);
            const y = (this.R + this.r * Math.cos(u / 2) * Math.sin(v) - this.r * Math.sin(u / 2) * Math.sin(2 * v)) * Math.sin(u);
            const z = this.r * Math.sin(u / 2) * Math.sin(v) + this.r * Math.cos(u / 2) * Math.sin(2 * v);

            return [x, y, z];
        }
    }
}

