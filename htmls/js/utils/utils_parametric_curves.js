/**
 * Author: Danny Rakita
 * Description: For CPSC-487-587 3D Spatial Modeling and Computing at Yale University
 */

import {
    convert_2array_to_3array,
    convert_z_up_array_to_y_up_array,
    get_default_lil_gui,
    spawn_static_parametric_curve
} from "./utils_three.js";
import {
    add_matrix_matrix,
    cross_product, frobenius_norm_matrix, mul_matrix_matrix,
    mul_matrix_scalar,
    normalized_matrix, roll_list_into_column_vec_matrix,
    sub_matrix_matrix,
    unroll_matrix_to_list
} from "./utils_math.js";
import {optimization_gradient_descent, optimization_powell} from "./utils_optimization.js";

/**
 * Base class for creating specific parametric curves in 3D space.
 * This is a template class and should not be instantiated directly.
 */
export class ParametricCurveBaseClass {
    constructor() {
        /**
         * Initializes the raw parametric function.
         * @type {Function}
         */
        if (new.target === ParametricCurveBaseClass) {
            throw new Error("ParametricCurveBaseClass is a template class and cannot be instantiated directly.");
        }

        this.raw_parametric_function = this.get_raw_parametric_function();
    }

    // returns a function that takes in one parameter (u) that will vary on the range 0-1 and returns a
    // point in space [x, y, z].
    /**
     * Abstract method to return a function that represents the parametric curve.
     * Should be implemented by derived classes.
     * @abstract
     * @returns {Function} A function that takes a parameter `u` and returns a point [x, y, z] on the curve.
     */
    get_raw_parametric_function() {
        throw new Error("Method 'get_raw_parametric_function()' must be implemented in the derived class.");
    }

    /**
     * Converts the Z-up coordinate system to Y-up.
     * @returns {Function} A function that takes a parameter `u` and returns a point [x, y, z] on the curve.
     */
    get_raw_parametric_function_z_up() {
        return (u) => {
            return convert_z_up_array_to_y_up_array(this.raw_parametric_function(u));
        }
    }

    /**
     * Returns a normalized vector that spans the surface at a given point on the curve.
     * @param {number} u - The parameter along the curve.
     * @returns {number[][]} A normalized vector at the point `u`.
     */
    get_surface_spanning_vector_at_point(u) {
        let uu = u + 0.0001;

        let point = this.raw_parametric_function(u);
        let uu_point = this.raw_parametric_function(uu);

        return normalized_matrix(sub_matrix_matrix(uu_point, point));
    }

    /**
     * Abstract method to return the ratio between world distance and parametric distance.
     * Should be implemented by derived classes.
     * @abstract
     * @returns {number} The ratio between world distance and parametric distance.
     */
    get_world_dis_to_parametric_dis_ratio() {
        throw new Error("Method 'get_world_dis_to_parametric_dis' must be implemented in the derived class.");
    }

    /**
     * Projects a given point onto the curve using optimization techniques.
     * @param {number[]} point - The 3D point to project onto the curve.
     * @param {number} starting_u - The initial guess for the parameter `u`.
     * @returns {[number, number[]]} A tuple `[u, point_on_curve]` where `u` is the parameter on the curve and `point_on_curve` is the closest point on the curve.
     */
    project_onto_curve(point, starting_u) {
        let f = x => {
            let a = this.raw_parametric_function(x[0]);
            a = roll_list_into_column_vec_matrix(a);
            point = roll_list_into_column_vec_matrix(convert_2array_to_3array(unroll_matrix_to_list(point)));
            let dis = frobenius_norm_matrix(sub_matrix_matrix(a, point));
            return dis*dis;
        }

        // let res = optimization_gradient_descent(f, [starting_u], 0.008, undefined, 0.0001, 30);
        // console.log(res);
        let solution = optimization_gradient_descent(f, [starting_u]);

        let u = solution;

        return [ u, this.raw_parametric_function(u) ];
    }

    /**
     * Rotates a vector onto the curve, optimizing for the closest point on the curve.
     * @param {number[]} starting_point - The starting point on the curve.
     * @param {number[]} direction - The direction vector to rotate.
     * @returns {[number[], number[]]} A tuple `[mapped_direction, new_point]` where `mapped_direction` is the rotated vector and `new_point` is the corresponding point on the curve.
     */
    rotate_vector_onto_curve(starting_point, direction) {
        let f = x => {
            let theta = x[0];
            let mat = [ [Math.cos(theta), -Math.sin(theta)], [Math.sin(theta), Math.cos(theta)] ];
            let d = unroll_matrix_to_list(direction);
            let dd = [[d[0]], [d[1]]];
            let mapped_direction = mul_matrix_matrix(mat, dd);
            let s = unroll_matrix_to_list(starting_point);
            let ss = [[s[0]], [s[1]]];
            let new_point = add_matrix_matrix(ss, mapped_direction);
            new_point = roll_list_into_column_vec_matrix(convert_2array_to_3array(unroll_matrix_to_list(new_point)));
            let proj = this.project_onto_curve(new_point, 0.1)[1];
            let sub = sub_matrix_matrix(proj, new_point);
            let norm = frobenius_norm_matrix(sub);
            return norm*norm;
        }

        let solution = optimization_gradient_descent(f, [0.0]);
        let theta = solution[0];

        let mat = [ [Math.cos(theta), -Math.sin(theta)], [Math.sin(theta), Math.cos(theta)] ];
        let d = unroll_matrix_to_list(direction);
        let dd = [[d[0]], [d[1]]];
        let mapped_direction = mul_matrix_matrix(mat, dd);
        let s = unroll_matrix_to_list(starting_point);
        let ss = [[s[0]], [s[1]]];
        let new_point = add_matrix_matrix(ss, mapped_direction);

        return [mapped_direction, new_point];
    }

    /**
     * Spawns a static visualization of the parametric curve in a Three.js scene.
     * @param {Object} scene - The Three.js scene to add the curve to.
     * @param {number} [num_points=100] - The number of points to render the curve with.
     * @param {boolean} [render_through_other_objects=false] - Whether to render the curve through other objects.
     * @param {number} [width=0.01] - The width of the curve.
     * @param {number} [start_color=0x000000] - The starting color of the curve.
     * @param {number} [end_color=0x000000] - The ending color of the curve.
     * @param {number} [opacity=1.0] - The opacity of the curve.
     */
    spawn_static_parametric_curve(scene, num_points=100, render_through_other_objects=false, width=0.01, start_color=0x000000, end_color= 0x000000, opacity=1.0) {
        spawn_static_parametric_curve(scene, this.raw_parametric_function, num_points, render_through_other_objects, width, start_color, end_color, opacity);
    }
}

/**
 * A class to visualize parametric curves using Three.js.
 */
export class ParametricCurveThreeVisualizer {
    /**
     * Creates an instance of ParametricCurveThreeVisualizer.
     * @param {ParametricCurveBaseClass} parametric_curve - An instance of a parametric curve.
     */
    constructor(parametric_curve) {
        this.parametric_curve = parametric_curve;
        /**
         * Settings for the visualizer.
         * @type {Object}
         */
        this.settings = {
            u: 0.5,
            draw_tangent_space: false,
            tangent_space_vector_length: 0.25
        };
        /**
         * GUI for controlling the visualizer.
         * @type {Object}
         */
        let gui = get_default_lil_gui();
        gui.add(this.settings, 'u', 0, 1).name('u');
        gui.add(this.settings, 'draw_tangent_space').name('Draw Tangent Space');
        gui.add(this.settings, 'tangent_space_vector_length', 0.1, 1.0).name('Tangent Vec. Size');
        this.gui = gui;
    }

    /**
     * The main loop function for updating the visualization.
     * @param {Object} three_engine - The Three.js engine instance.
     */
    three_loop_function(three_engine) {
        let point = this.parametric_curve.raw_parametric_function(this.settings.u);
        three_engine.draw_debug_sphere(point, 0.04, 0x00eeff);

        if (this.settings.draw_tangent_space) {
            let ss = this.parametric_curve.get_surface_spanning_vector_at_point(this.settings.u);
            three_engine.draw_debug_vector(point, add_matrix_matrix(point, mul_matrix_scalar(ss, this.settings.tangent_space_vector_length)), 0.02, undefined, 0x777788);
        }
    }
}

/**
 * A class that utilizes Lie groups and Lie algebras to manipulate and visualize parametric curves.
 */
export class ParametricCurveLieGroupAndAlgebraUtil {
    /**
     * Creates an instance of ParametricCurveLieGroupAndAlgebraUtil.
     * @param {ParametricCurveBaseClass} parametric_curve - An instance of a parametric curve.
     */
    constructor(parametric_curve) {
        this.parametric_curve = parametric_curve;
        /**
         * Settings for the visualizer.
         * @type {Object}
         */
        this.settings = {
            starting_u: 0.1,
            a:0.5,
            t:0,
            display_tangent_space: true
        }
        /**
         * GUI for controlling the visualizer.
         * @type {Object}
         */
        let gui = get_default_lil_gui();
        gui.add(this.settings, 'starting_u', 0.0000001, 1).name('Starting u').onChange(() => { this.outdated = true })
        gui.add(this.settings, 'a', 0.0001, 10.0).name('Tangent x').onChange(() => { this.outdated = true });
        gui.add(this.settings, 't', 0, 1).name('t');
        gui.add(this.settings, 'display_tangent_space').name('Show Tangent Space');
        this.gui = gui;
    }

    /**
     * The main loop function for updating the visualization.
     * @param {Object} three_engine - The Three.js engine instance.
     */
    three_loop_function(three_engine) {
        let origin_point = this.parametric_curve.raw_parametric_function(this.settings.starting_u);
        three_engine.draw_debug_sphere(origin_point, 0.01, 0x00eeff);
        if (this.settings.display_tangent_space) {
            // let ss = this.parametric_curve.get_surface_spanning_vector_at_point(this.settings.starting_u);
            // ss = mul_matrix_scalar(ss, Math.max(Math.abs(this.settings.a)*1.5, 1.0));

            // three_engine.draw_debug_grid_plane(origin_point, span_vec_1, span_vec_2, Math.max(Math.abs(this.settings.a) * 1.5, 1.0), Math.max(Math.abs(this.settings.b) * 1.5, 1.0));
            // three_engine.draw_debug_vector(origin_point, add_matrix_matrix(origin_point, ss), 0.02, undefined, 0x000000);
            // three_engine.draw_debug_vector(origin_point, add_matrix_matrix(origin_point, mul_matrix_scalar(ss, -1)), 0.02, undefined, 0x000000);
        }

        let exp_0_points = get_curve_exp_0_points(this.parametric_curve, this.settings.starting_u, this.settings.a);
        for(let i=0; i<exp_0_points.length-1; i++) {
            three_engine.draw_debug_line(exp_0_points[i], exp_0_points[i+1], true, 0.03, 0xffcc00);
        }

        let exp_x_points = get_curve_exp_x_points(this.parametric_curve, this.settings.starting_u, this.settings.a);

        let exp_tx_points = get_curve_exp_tx_points(this.parametric_curve, this.settings.t, exp_0_points, exp_x_points);
        for(let i=0; i<exp_0_points.length-1; i++) {
            three_engine.draw_debug_line(exp_tx_points[i], exp_tx_points[i+1], true, 0.03, 0xaaff11);
        }
        three_engine.draw_debug_sphere(exp_tx_points[exp_tx_points.length-1], 0.04, 0x000000);
    }
}

/**
 * Generates the points of the exponential map for a given parametric curve and parameter t.
 * @param {ParametricCurveBaseClass} parametric_surface - The parametric curve object.
 * @param {number} t - The interpolation parameter.
 * @param {number[][]} exp_0_points - The initial exponential points.
 * @param {number[][]} exp_x_points - The exponential points with parameter x.
 * @param {number} [segment_length=0.035] - The length of each segment.
 * @returns {number[][]} The list of points representing the exponential map.
 */
export function get_curve_exp_tx_points(parametric_surface, t, exp_0_points, exp_x_points, segment_length= 0.035) {
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

/*
export function get_curve_exp_0_points(parametric_curve, a, segment_length=0.035) {
    let ss = parametric_curve.get_surface_spanning_vector_at_point(0.0000001);
    let raw_parametric_function = parametric_curve.raw_parametric_function;
    let start_point = raw_parametric_function(0.000001);
    let end_point = add_matrix_matrix(start_point, mul_matrix_scalar(ss, a));

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
*/

// da should be in real world distance, not parameter distance
/**
 * Generates the points of the exponential map at the initial parameter u=0.
 * @param {ParametricCurveBaseClass} parametric_curve - The parametric curve object.
 * @param {number} starting_u - The starting parameter u.
 * @param {number} da - The change in parameter distance.
 * @param {number} [segment_length=0.035] - The length of each segment.
 * @returns {number[][]} The list of points representing the exponential map at u=0.
 */
export function get_curve_exp_0_points(parametric_curve, starting_u, da, segment_length=0.035) {
    let ss = parametric_curve.get_surface_spanning_vector_at_point(starting_u);
    let raw_parametric_function = parametric_curve.raw_parametric_function;
    let start_point = raw_parametric_function(starting_u);
    let end_point = add_matrix_matrix(start_point, mul_matrix_scalar(ss, da));

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

/*
export function get_curve_exp_x_points(parametric_curve, a, segment_length=0.035) {
    let ss = parametric_curve.get_surface_spanning_vector_at_point(0.0000001);
    let raw_parametric_function = parametric_curve.raw_parametric_function;
    let start_point = raw_parametric_function(0.000001);
    let end_point = add_matrix_matrix(start_point, mul_matrix_scalar(ss, a));

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

        let res = parametric_curve.rotate_vector_onto_curve(curr_point, curr_dir);
        curr_dir = res[0].slice();
        curr_point = res[1].slice();

        curr_length += segment_length;
        if (curr_length > dir_l) { out.push(curr_point); break; }
    }
    // curr_dir = normalized_matrix(curr_dir);
    // curr_dir = mul_matrix_scalar(curr_dir, r);
    // let res = parametric_curve.rotate_vector_onto_curve(curr_point, curr_dir);
    // curr_point = res[1].slice();
    // out.push(curr_point);

    return out;
}
*/

// da should be in real world distance, not parameter distance
/**
 * Generates the points of the exponential map at a given parameter x.
 * @param {ParametricCurveBaseClass} parametric_curve - The parametric curve object.
 * @param {number} starting_u - The starting parameter u.
 * @param {number} da - The change in parameter distance.
 * @param {number} [segment_length=0.035] - The length of each segment.
 * @returns {number[][]} The list of points representing the exponential map at x.
 */
export function get_curve_exp_x_points(parametric_curve, starting_u, da, segment_length=0.035) {
    let ss = parametric_curve.get_surface_spanning_vector_at_point(starting_u);
    let raw_parametric_function = parametric_curve.raw_parametric_function;
    let start_point = raw_parametric_function(starting_u);
    let end_point = add_matrix_matrix(start_point, mul_matrix_scalar(ss, da));
    let dir = sub_matrix_matrix(end_point, start_point);
    let dir_l = frobenius_norm_matrix(dir);
    let r = dir_l % segment_length;

    let curr_length = 0.0;
    let curr_param = starting_u;
    let curr_point = parametric_curve.raw_parametric_function(starting_u);
    let out = [curr_point.slice()];
    let exit = false;

    while (true) {
        let aa = parametric_curve.raw_parametric_function(curr_param);
        let bb = parametric_curve.raw_parametric_function(curr_param+0.0001);
        let d1 = frobenius_norm_matrix(sub_matrix_matrix(aa,bb));
        let distance_per_unit = d1 / 0.0001;
        if(curr_length + segment_length > dir_l) { exit = true; segment_length = r; }
        let sl = segment_length / distance_per_unit;
        curr_param += sl;
        let new_point = parametric_curve.raw_parametric_function(curr_param);
        // let distance = frobenius_norm_matrix(sub_matrix_matrix(new_point, curr_point));
        curr_length += segment_length;
        curr_point = new_point;
        out.push(curr_point.slice());
        if (exit) { break; }
    }

    return out;
}

/**
 * A simple parametric curve representing a straight line.
 * Inherits from ParametricCurveBaseClass.
 */
export class ParametricCurveLineTest extends ParametricCurveBaseClass {
    constructor() {
        super();
    }

    /**
     * Returns the ratio between world distance and parametric distance.
     * @returns {number} The ratio between world distance and parametric distance.
     */
    get_world_dis_to_parametric_dis_ratio() {
        return 1.0;
    }

    /**
     * Returns the raw parametric function for the line.
     * @returns {Function} A function that takes a parameter `u` and returns a point [x, y, z] on the line.
     */
    get_raw_parametric_function() {
        return u => {
            let start_point = [1,-2,-2.5];
            let end_point = [-1,2,2.5];

            let a = mul_matrix_scalar(start_point, 1-u);
            let b = mul_matrix_scalar(end_point, u);

            let c = add_matrix_matrix(a, b);
            return unroll_matrix_to_list(c);
        }
    }
}

/**
 * A parametric curve representing a unit circle in 3D space.
 * Inherits from ParametricCurveBaseClass.
 */
export class ParametricCurveUnitCircle extends ParametricCurveBaseClass {
    constructor() {
        super();
    }

    /**
     * Returns the ratio between world distance and parametric distance.
     * @returns {number} The ratio between world distance and parametric distance (2π for a unit circle).
     */
    get_world_dis_to_parametric_dis_ratio() {
        return 2.0*Math.PI;
    }

    /**
     * Returns the ratio between world distance and parametric distance.
     * @returns {number} The ratio between world distance and parametric distance (2π for a unit circle).
     */
    get_raw_parametric_function() {
        return u => {
            u *= Math.PI * 2;
            return [Math.cos(u), Math.sin(u), 0];
        }
    }
}