import {
    add_matrix_matrix, frobenius_norm_matrix, identity_matrix,
    mul_matrix_matrix,
    mul_matrix_scalar,
    roll_list_into_column_vec_matrix, sub_matrix_matrix, unroll_matrix_to_list
} from "./utils_math.js";
import {
    vec6_to_se3_mat,
    exp_se3_to_SE3,
    vec3_to_so3_mat,
    exp_so3_to_SO3,
    ln_SE3_to_se3, ln_SO3_and_t_to_so3_and_v, so3_mat_to_vec3
} from "./utils_exp_and_log_obfuscated.js";
import {get_default_lil_gui, refresh_displays} from "./utils_three.js";

/**
 * Base class for polynomial spline segments. Should not be instantiated directly.
 * @abstract
 */
export class PolynomialSplineSegmentBaseClass {
    /**
     * Creates a new polynomial spline segment.
     * @param {number} dim - The dimension of the spline (e.g., 2 for 2D, 3 for 3D).
     * @param {number[][]} [init_control_point_mat=null] - Initial control points for the spline segment.
     */
    constructor(dim, init_control_point_mat=null) {
        if (new.target === PolynomialSplineSegmentBaseClass) {
            throw new Error("PolynomialSplineSegmentBaseClass is a template class and cannot be instantiated directly.");
        }

        this.B = this.get_B_matrix();
        this.num_control_points_per_segment = this.get_num_control_points_per_segment();
        this.segment_type_string = this.get_segment_type_string();
        this.dim = dim;

        this.control_point_mat = [];
        if(init_control_point_mat) {
            this.control_point_mat = init_control_point_mat
        }
        else {
            for(let i = 0; i < this.num_control_points_per_segment; i++) {
                let control_point = []
                for(let j = 0; j < dim; j++) {
                    control_point.push( (Math.random() * 2 - 1)*0.4 );
                }
                this.control_point_mat.push(control_point);
            }
        }
    }

    /**
     * Interpolates the spline at a given parameter t.
     * @param {number} t - The parameter t, between 0 and 1.
     * @returns {number[][]} The interpolated point.
     */
    interpolate(t) {
        if (t < 0 || t > 1) {
            throw new Error('t must be between 0 and 1');
        }

        let a_coeffs = this.get_a_coeffs();

        return this.interpolate_from_a_coeffs(t, a_coeffs);
    }

    /**
     * Interpolates the spline using precomputed coefficients.
     * @param {number} t - The parameter t, between 0 and 1.
     * @param {number[][]} a_coeffs - The precomputed coefficients.
     * @returns {number[][]} The interpolated point.
     */
    interpolate_from_a_coeffs(t, a_coeffs) {
        if (t < 0 || t > 1) {
            throw new Error('t must be between 0 and 1');
        }

        let out = [];
        for(let i = 0; i < this.dim; i++) { out.push( [0] ); }

        for(let i=0; i < a_coeffs.length; i++) {
            out = add_matrix_matrix(out, mul_matrix_scalar(a_coeffs[i], Math.pow(t, i)));
        }

        return out;
    }

    /**
     * Computes the coefficients matrix for the spline segment.
     * @returns {number[][]} The coefficients matrix.
     */
    get_a_coeffs_matrix() {
        return mul_matrix_matrix(this.B, this.control_point_mat);
    }

    /**
     * Computes the coefficients for the spline segment.
     * @returns {number[][]} The coefficients.
     */
    get_a_coeffs() {
        let a_coeffs_matrix = this.get_a_coeffs_matrix();

        let a_coeffs = [];
        for(let i = 0; i < a_coeffs_matrix.length; i++) {
            a_coeffs.push(roll_list_into_column_vec_matrix(a_coeffs_matrix[i]));
        }

        return a_coeffs;
    }

    /**
     * Updates a control point of the spline.
     * @param {number} control_point_idx - The index of the control point to update.
     * @param {number[]} new_control_point - The new control point.
     */
    update_control_point(control_point_idx, new_control_point) {
        if(control_point_idx >= this.control_point_mat.length) {
            throw new Error('control_point_idx is too large');
        }

        new_control_point = unroll_matrix_to_list(new_control_point);
        if(new_control_point.length !== this.dim) {
            throw new Error('new control point is not the right size');
        }

        this.control_point_mat[control_point_idx] = new_control_point;
    }

    /**
     * Returns the B matrix for the spline segment.
     * @abstract
     * @returns {number[][]} The B matrix.
     */
    get_B_matrix() {
        throw new Error("Method 'get_B_matrix()' must be implemented in the derived class.");
    }

    /**
     * Returns the number of control points per segment.
     * @abstract
     * @returns {number} The number of control points per segment.
     */
    get_num_control_points_per_segment() {
        throw new Error("Method 'get_num_control_points_per_segment()' must be implemented in the derived class.");
    }

    /**
     * Returns the type of spline segment.
     * @abstract
     * @returns {string} The type of the spline segment.
     */
    get_segment_type_string() {
        throw new Error("Method 'get_segment_type_string()' must be implemented in the derived class.");
    }
}

/**
 * Represents a linear spline segment.
 * @extends PolynomialSplineSegmentBaseClass
 */
export class LinearSplineSegment extends PolynomialSplineSegmentBaseClass {
    /**
     * @inheritdoc
     */
    constructor(dim, init_control_point_mat=null) {
        super(dim, init_control_point_mat);
    }

    /**
     * @inheritdoc
     */
    interpolate(t) {
        return super.interpolate(t);
    }

    /**
     * @inheritdoc
     */
    interpolate_from_a_coeffs(t, a_coeffs) {
        return super.interpolate_from_a_coeffs(t, a_coeffs);
    }

    /**
     * @inheritdoc
     */
    get_B_matrix() {
        return [ [1, 0], [-1, 1] ];
    }

    /**
     * @inheritdoc
     */
    get_num_control_points_per_segment() {
        return 2;
    }

    /**
     * @inheritdoc
     */
    get_segment_type_string() {
        return 'linear';
    }
}

/**
 * Represents a quadratic spline segment.
 * @extends PolynomialSplineSegmentBaseClass
 */
export class QuadraticSplineSegment extends PolynomialSplineSegmentBaseClass {
    /**
     * @inheritdoc
     */
    constructor(dim, init_control_point_mat=null) {
        super(dim, init_control_point_mat);
    }

    /**
     * @inheritdoc
     */
    interpolate(t) {
        return super.interpolate(t);
    }

    /**
     * @inheritdoc
     */
    interpolate_from_a_coeffs(t, a_coeffs) {
        return super.interpolate_from_a_coeffs(t, a_coeffs);
    }

    /**
     * @inheritdoc
     */
    get_B_matrix() {
        return [ [1, 0, 0], [-3, 4, -1], [2, -4, 2] ];
    }

    /**
     * @inheritdoc
     */
    get_num_control_points_per_segment() {
        return 3;
    }

    /**
     * @inheritdoc
     */
    get_segment_type_string() {
        return 'quadratic';
    }
}

/**
 * Represents a Hermite spline segment.
 * @extends PolynomialSplineSegmentBaseClass
 */
export class HermiteSplineSegment extends PolynomialSplineSegmentBaseClass {
    /**
     * @inheritdoc
     */
    constructor(dim, init_control_point_mat=null) {
        super(dim, init_control_point_mat);
    }

    /**
     * @inheritdoc
     */
    interpolate(t) {
        return super.interpolate(t);
    }

    /**
     * @inheritdoc
     */
    interpolate_from_a_coeffs(t, a_coeffs) {
        return super.interpolate_from_a_coeffs(t, a_coeffs);
    }

    /**
     * @inheritdoc
     */
    get_B_matrix() {
        return [ [1, 0, 0, 0], [0, 1, 0, 0], [-3, -2, 3, -1], [2, 1, -2, 1] ];
    }

    /**
     * @inheritdoc
     */
    get_num_control_points_per_segment() {
        return 4;
    }

    /**
     * @inheritdoc
     */
    get_segment_type_string() {
        return 'hermite';
    }
}

/**
 * Represents a natural spline segment.
 * @extends PolynomialSplineSegmentBaseClass
 */
export class NaturalSplineSegment extends PolynomialSplineSegmentBaseClass {
    /**
     * @inheritdoc
     */
    constructor(dim, init_control_point_mat=null) {
        super(dim, init_control_point_mat);
    }

    /**
     * @inheritdoc
     */
    interpolate(t) {
        return super.interpolate(t);
    }

    /**
     * @inheritdoc
     */
    interpolate_from_a_coeffs(t, a_coeffs) {
        return super.interpolate_from_a_coeffs(t, a_coeffs);
    }

    /**
     * @inheritdoc
     */
    get_B_matrix() {
        return [ [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0.5, 0], [-1, -1, -0.5, 1] ];
    }

    /**
     * @inheritdoc
     */
    get_num_control_points_per_segment() {
        return 4;
    }

    /**
     * @inheritdoc
     */
    get_segment_type_string() {
        return 'natural';
    }
}

/**
 * Represents a Cardinal spline segment.
 * @extends PolynomialSplineSegmentBaseClass
 */
export class CardinalSplineSegment extends PolynomialSplineSegmentBaseClass {
    /**
     * Creates a new Cardinal spline segment.
     * @param {number} dim - The dimension of the spline (e.g., 2 for 2D, 3 for 3D).
     * @param {number} w - The tension parameter for the Cardinal spline.
     * @param {number[][]} [init_control_point_mat=null] - Initial control points for the spline segment.
     */
    constructor(dim, w, init_control_point_mat=null) {
        super(dim, init_control_point_mat);

        this.w = w;
    }

    /**
     * @inheritdoc
     */
    get_B_matrix() {
        return [ [0, 1, 0, 0], [(this.w - 1)/2, 0, (1-this.w)/2, 0], [1 - this.w, 0.5*(-this.w - 5), this.w + 2, (this.w - 1)/2], [(this.w-1)/2, (this.w+3)/2, 0.5*(-this.w - 3), (1-this.w)/2] ];
    }

    /**
     * @inheritdoc
     */
    get_num_control_points_per_segment() {
        return 4;
    }

    /**
     * @inheritdoc
     */
    get_segment_type_string() {
        return 'cardinal';
    }
}

/**
 * Represents a Catmull-Rom spline segment.
 * @extends PolynomialSplineSegmentBaseClass
 */
export class CatmullRomSplineSegment extends PolynomialSplineSegmentBaseClass {
    /**
     * @inheritdoc
     */
    constructor(dim, init_control_point_mat=null) {
        super(dim, init_control_point_mat);
    }

    /**
     * @inheritdoc
     */
    get_B_matrix() {
        return [ [0, 1, 0, 0], [-0.5, 0, 0.5, 0], [1, 5.0/2.0, 2, -0.5], [-0.5, 3/2, -3/2, 0.5] ];
    }

    /**
     * @inheritdoc
     */
    get_num_control_points_per_segment() {
        return 4;
    }

    /**
     * @inheritdoc
     */
    get_segment_type_string() {
        return 'catmull_rom';
    }
}

/**
 * Represents a Bezier spline segment.
 * @extends PolynomialSplineSegmentBaseClass
 */
export class BezierSplineSegment extends PolynomialSplineSegmentBaseClass {
    /**
     * @inheritdoc
     */
    constructor(dim, init_control_point_mat=null) {
        super(dim, init_control_point_mat);
    }

    /**
     * @inheritdoc
     */
    interpolate(t) {
        return super.interpolate(t);
    }

    /**
     * @inheritdoc
     */
    interpolate_from_a_coeffs(t, a_coeffs) {
        return super.interpolate_from_a_coeffs(t, a_coeffs);
    }

    /**
     * @inheritdoc
     */
    get_B_matrix() {
        return [ [1, 0, 0, 0], [-3, 3, 0, 0], [3, -6, 3, 0], [-1,3,-3,1] ];
    }

    /**
     * @inheritdoc
     */
    get_num_control_points_per_segment() {
        return 4;
    }

    /**
     * @inheritdoc
     */
    get_segment_type_string() {
        return 'bezier';
    }
}

/**
 * Helper class for visualizing polynomial spline segments using THREE.js.
 */
export class PolynomialSplineSegmentVisualizerHelper {
    constructor() { }

    /**
     * Creates a new default visualizer helper.
     * @param {PolynomialSplineSegmentBaseClass} polynomial_spline_segment - The polynomial spline segment to visualize.
     * @param {Object} three_engine - The THREE.js engine.
     * @param {Object} transform_gizmo_engine - The engine for managing transform gizmos.
     * @returns {PolynomialSplineSegmentVisualizerHelper} The visualizer helper.
     */
    static new_default(polynomial_spline_segment, three_engine, transform_gizmo_engine) {
        let out = new PolynomialSplineSegmentVisualizerHelper();

        out.polynomial_spline_segment = polynomial_spline_segment;
        out.three_engine = three_engine;
        out.transform_gizmo_engine = transform_gizmo_engine;

        out.transform_gizmo_idxs = [];
        for(let i = 0; i < out.polynomial_spline_segment.num_control_points_per_segment; i++) {
            let control_point = out.polynomial_spline_segment.control_point_mat[i];
            if(control_point.length === 2) {
                let idx = out.transform_gizmo_engine.add_gizmo_SO2_matrix_and_position(three_engine, identity_matrix(2), control_point, 0.3);
                out.transform_gizmo_idxs.push(idx);

                let transform_control = out.transform_gizmo_engine.transform_controls[idx];
                transform_control.addEventListener('change', function(event) {
                    let t = out.transform_gizmo_engine.get_gizmo_pose_as_SO2_matrix_and_position(idx)[1];
                    out.polynomial_spline_segment.update_control_point(i, t);
                });

            } else if(control_point.length === 3) {
                let idx = out.transform_gizmo_engine.add_gizmo_SO3_matrix_and_position(three_engine, identity_matrix(3), control_point, 0.3);
                out.transform_gizmo_idxs.push(idx);

                let transform_control = out.transform_gizmo_engine.transform_controls[idx];
                transform_control.addEventListener('change', function(event) {
                    let t = out.transform_gizmo_engine.get_gizmo_pose_as_SO3_matrix_and_position(idx)[1];
                    out.polynomial_spline_segment.update_control_point(i, t);
                });

            } else if(control_point.length === 6) {
                let se3_mat = vec6_to_se3_mat(control_point);
                let SE3_mat = exp_se3_to_SE3(se3_mat);
                let idx = out.transform_gizmo_engine.add_gizmo_SE3_matrix(three_engine, SE3_mat, 0.3);
                out.transform_gizmo_idxs.push(idx);

                let transform_control = out.transform_gizmo_engine.transform_controls[idx];
                transform_control.addEventListener('change', function(event) {
                    let [SO3_mat, position] = out.transform_gizmo_engine.get_gizmo_pose_as_SO3_matrix_and_position(idx);
                    let [so3_mat, v] = ln_SO3_and_t_to_so3_and_v(S03_mat, position);
                    let s = unroll_matrix_to_list(so3_mat_to_vec3(so3_mat));
                    v = unroll_matrix_to_list(v);

                    let tt = [ v[0], v[1], v[2], s[0], s[1], s[2] ];

                    out.polynomial_spline_segment.update_control_point(i, tt);
                });

            } else {
                throw new Error('cannot handle');
            }
        }

        return out;
    }

    /*
    static new_separated_translation_and_rotation(translation_segment, rotation_segment, three_engine, transform_gizmo_engine) {
        let out = new PolynomialSplineSegmentVisualizerHelper();

        out.translation_segment = translation_segment;
        out.rotation_segment = rotation_segment;
        out.three_engine = three_engine;
        out.transform_gizmo_engine = transform_gizmo_engine;

        out.transform_gizmo_idxs = [];
        for(let i = 0; i < out.polynomial_spline_segment.num_control_points_per_segment; i++) {
            let control_point1 = out.translation_segment.control_point_mat[i];
            let control_point2 = out.rotation_segment.control_point_mat[i];

            let so3_mat = vec3_to_so3_mat(control_point2);
            let SO3_mat = exp_so3_to_SO3(so3_mat);

            let idx = transform_gizmo_engine.add_gizmo_SO3_matrix_and_position(three_engine, SO3_mat, control_point1);
            out.transform_gizmo_idxs.push(idx);
        }

        return out;
    }
    */
}

/**
 * Visualizer for polynomial spline segments using THREE.js.
 */
export class PolynomialSplineSegmentVisualizer {
    /**
     * Creates a new visualizer for polynomial spline segments.
     * @param {PolynomialSplineSegmentVisualizerHelper} helper - The helper for visualizing the spline segment.
     */
    constructor(helper) {
        let gui = get_default_lil_gui();

        this.settings = {
            t:0,
            gizmos_visible: false,
            draw_second_grid: true,
            draw_sum_vecs: false
        }

        this.is_playing = false;
        this.actions = {
            play: () => {
                this.is_playing = true;
            },
            stop: () => {
                this.is_playing = false;
            }
        }

        gui.add(this.settings, 't', 0, 1);
        gui.add(this.settings, 'gizmos_visible').name('Gizmos Visible');
        gui.add(this.settings, 'draw_second_grid').name('Draw Grid');
        gui.add(this.settings, 'draw_sum_vecs').name('Draw Sum Vecs');
        gui.add(this.actions, 'play').name('Play');
        gui.add(this.actions, 'stop').name('Stop');

        this.gui = gui;

        this.helper = helper;
    }

    /**
     * Function to be called in the THREE.js animation loop to render the spline segment.
     * @param {Object} three_engine - The THREE.js engine.
     */
    three_loop_function(three_engine) {
        this.helper.transform_gizmo_engine.set_visibility_of_all_gizmos(three_engine, this.settings.gizmos_visible);

        if(this.is_playing) {
            this.settings.t += 0.005;
            if(this.settings.t > 1.0) { this.settings.t = 0; }
            refresh_displays(this.gui);
        }

        let segment = this.helper.polynomial_spline_segment;
        let type_string = segment.segment_type_string;

        let points = [];
        for(let i = 0; i <= 50; i++) {
            let tt = i/50;
            let int = segment.interpolate(tt);
            points.push(int);
        }
        for(let i = 0; i < points.length-1; i++) {
            three_engine.draw_debug_line(points[i], points[i+1], false, 0.01, 0x555555);
        }

        let s = segment.interpolate(this.settings.t);
        three_engine.draw_debug_sphere(s, 0.06, 0x00eeff);

        let a_coeffs = segment.get_a_coeffs();

        let shift;
        if(a_coeffs[0].length === 2) {
            shift = [3, 0];
        } else {
            shift = [3, 0, 0];
        }

        let shifted_a_coeffs = [];
        let scaled_a_coeffs = [];
        let scaled_and_scaled_a_coeffs = [];
        for(let i=0; i<a_coeffs.length; i++) {
            let a_coeff = a_coeffs[i];
            let shifted_a_coeff = add_matrix_matrix(a_coeff, shift);
            shifted_a_coeffs.push(shifted_a_coeff);
            let scaled_a_coeff = mul_matrix_scalar(a_coeff, Math.pow(this.settings.t, i));
            scaled_a_coeffs.push(scaled_a_coeff);
            let scaled_and_shifted_coeff = add_matrix_matrix(scaled_a_coeff, shift);
            scaled_and_scaled_a_coeffs.push(scaled_and_shifted_coeff);
        }

        if(this.settings.draw_second_grid) {
            three_engine.draw_debug_grid_plane([3, 0, 0], [1, 0, 0], [0, 1, 0], 1, 1, undefined, undefined, undefined, false);
            for (let i = 0; i < a_coeffs.length; i++) {
                three_engine.draw_debug_vector(shift, shifted_a_coeffs[i], 0.01, undefined, 0x777777);
                three_engine.draw_debug_vector(shift, scaled_and_scaled_a_coeffs[i], 0.03, undefined, 0xee00ff);
            }
        }

        if(this.settings.draw_sum_vecs) {
            let curr;
            if(a_coeffs[0].length === 2) {
                curr = [[0], [0]];
            } else {
                curr = [[0], [0], [0]];
            }
            for(let i = 0; i < scaled_a_coeffs.length; i++) {
                let new_curr = add_matrix_matrix(curr, scaled_a_coeffs[i]);
                three_engine.draw_debug_vector(curr, new_curr, 0.02, undefined, 0xee00ff);
                curr = new_curr;
            }
        }


        if(type_string === 'bezier') {
            let control_points = this.helper.polynomial_spline_segment.control_point_mat;
            three_engine.draw_debug_line(control_points[0], control_points[1], false, 0.003, 0x222222);
            three_engine.draw_debug_line(control_points[2], control_points[3], false, 0.003, 0x222222);
        }

        if(type_string === 'hermite') {
            let control_points = this.helper.polynomial_spline_segment.control_point_mat;
            // three_engine.draw_debug_line(control_points[0], control_points[1], false, 0.003, 0x222222);
            // three_engine.draw_debug_line(control_points[2], control_points[3], false, 0.003, 0x222222);
            three_engine.draw_debug_vector([0,0,0], control_points[1], 0.005, undefined, 0x222222);
            three_engine.draw_debug_vector([0,0,0], control_points[3], 0.005, undefined, 0x222222);
        }

        if(type_string === 'natural') {
            let control_points = this.helper.polynomial_spline_segment.control_point_mat;
            three_engine.draw_debug_vector([0,0,0], control_points[1], 0.005, undefined, 0x222222);
            three_engine.draw_debug_vector([0,0,0], control_points[2], 0.005, undefined, 0x882222);
        }
    }
}

/**
 * Represents a polynomial-based spline, which is composed of multiple spline segments.
 */
export class PolynomialBasedSpline {
    /**
     * Creates a new polynomial-based spline.
     * @param {PolynomialSplineSegmentBaseClass[]} segments - An array of spline segments that make up the spline.
     */
    constructor(segments) {
        this.segments = segments;
    }

    /**
     * Interpolates the spline at a given parameter t.
     * @param {number} t - The parameter value for interpolation.
     * @returns {number[]} The interpolated point on the spline.
     */
    interpolate(t) {
        let idx = Math.floor(t);
        let segment = this.segments[idx];
        return segment.interpolate(t - idx);
    }

    /**
     * Returns the maximum interpolation value, which corresponds to the number of segments in the spline.
     * @returns {number} The maximum value of t for interpolation.
     */
    max_interpolation_value() {
        return this.segments.length;
    }
}

/**
 * Helper class for visualizing polynomial splines using THREE.js.
 */
export class PolynomialSplineVisualizerHelper {
    constructor() { }

    /**
     * Creates a new default visualizer helper.
     * @param {PolynomialBasedSpline} spline - The polynomial spline to visualize.
     * @param {Object} three_engine - The THREE.js engine.
     * @param {Object} transform_gizmo_engine - The engine for managing transform gizmos.
     * @returns {PolynomialSplineVisualizerHelper} The visualizer helper.
     */
    static new_default(spline, three_engine, transform_gizmo_engine) {
        let out = new PolynomialSplineVisualizerHelper();

        out.spline = spline;
        out.three_engine = three_engine;
        out.transform_gizmo_engine = transform_gizmo_engine;

        out.helpers = [];
        spline.segments.forEach(segment => {
            out.helpers.push(PolynomialSplineSegmentVisualizerHelper.new_default(segment, three_engine, transform_gizmo_engine));
        });

        return out;
    }
}

/**
 * Visualizer for polynomial splines using THREE.js.
 */
export class PolynomialSplineVisualizer {
    /**
     * Creates a new visualizer for polynomial splines.
     * @param {PolynomialSplineVisualizerHelper} helper - The helper for visualizing the spline.
     * @param {boolean} [arclength_vis=false] - Whether to enable arclength visualization.
     * @param {boolean} [init_display_sphere=true] - Whether to display the interpolation sphere initially.
     * @param {boolean} [init_display_archlength_sphere=false] - Whether to display the arclength sphere initially.
     * @param {boolean} [freeze_display_sphere=false] - Whether to freeze the display of the interpolation sphere.
     * @param {boolean} [freeze_display_arclength_sphere=false] - Whether to freeze the display of the arclength sphere.
     * @param {boolean} [disable_play_stop=false] - Whether to disable the play and stop buttons.
     */
    constructor(helper, arclength_vis=false, init_display_sphere=true, init_display_archlength_sphere=false, freeze_display_sphere=false, freeze_display_arclength_sphere=false, disable_play_stop=false) {
        let gui = get_default_lil_gui();

        this.arclength_vis = arclength_vis;

        this.settings = {
            t:0,
            gizmos_visible: false,
            draw_second_grid: true,
            draw_sum_vecs: false,
            num_arclength_segments:10,
            arclength_slider:0,
            total_distance:0,
            accumulated_distance:0,
            display_sphere:init_display_sphere,
            display_arclength_sphere:init_display_archlength_sphere,
            distance_ratio:0,
            lower_bound_dis: 0,
            upper_bound_dis: 0,
            lower_bound_t: 0,
            upper_bound_t: 0,
            r:0,
            target_t:0,
            playback_speed:1
        }

        this.is_playing = false;
        this.actions = {
            play: () => {
                this.is_playing = true;
            },
            stop: () => {
                this.is_playing = false;
            }
        }

        gui.add(this.settings, 't', 0, helper.spline.max_interpolation_value()-0.00001);
        gui.add(this.settings, 'playback_speed', 0.001, 5).name('Playback speed');
        gui.add(this.settings, 'gizmos_visible').name('Gizmos Visible');
        // gui.add(this.settings, 'draw_second_grid').name('Draw Grid');
        // gui.add(this.settings, 'draw_sum_vecs').name('Draw Sum Vecs');
        if(this.arclength_vis) {
            this.num_arclength_segments_slider = gui.add(this.settings, 'num_arclength_segments', 4, 100, 2).name('Arclength samples');
            this.num_arclength_segments_slider.onChange(value => {
                this.arclength_slider.max(value);
                this.settings.arclength_slider = value;
                refresh_displays(gui);
            });
            this.arclength_slider = gui.add(this.settings, 'arclength_slider', 0, this.settings.num_arclength_segments, 1).name('Arclength slider');
            this.arclength_slider.onChange(() => {
                refresh_displays(gui);
            });
            gui.add(this.settings, 'total_distance').name('Total Dis.').disable();
            gui.add(this.settings, 'accumulated_distance').name('Accumulated Dis.').disable();
            gui.add(this.settings, 'distance_ratio').name('Target distance').disable();
            gui.add(this.settings, 'lower_bound_dis').name('Lower bound dis').disable();
            gui.add(this.settings, 'upper_bound_dis').name('Upper bound dis').disable();
            gui.add(this.settings, 'lower_bound_t').name('Lower bound t').disable();
            gui.add(this.settings, 'upper_bound_t').name('Upper bound t').disable();
            gui.add(this.settings, 'r').name('Ratio between bounds').disable();
            gui.add(this.settings, 'target_t').name('Interpolated t').disable();
        }
        let a = gui.add(this.settings, 'display_sphere').name('Show Sphere');
        if(freeze_display_sphere) { a.disable(); }
        if(this.arclength_vis) {
            let a = gui.add(this.settings, 'display_arclength_sphere').name('Show Arclength sphere');
            if(freeze_display_arclength_sphere) { a.disable(); }
        }

        let b = gui.add(this.actions, 'play').name('Play');
        let c = gui.add(this.actions, 'stop').name('Stop');

        if(disable_play_stop) { b.disable(); c.disable(); }

        this.gui = gui;

        this.helper = helper;
    }

    /**
     * Function to be called in the THREE.js animation loop to render the spline.
     * @param {Object} three_engine - The THREE.js engine.
     */
    three_loop_function(three_engine) {
        this.helper.transform_gizmo_engine.set_visibility_of_all_gizmos(three_engine, this.settings.gizmos_visible);

        let spline = this.helper.spline;

        if(this.is_playing) {
            this.settings.t += this.settings.playback_speed*0.005;
            if(this.settings.t > spline.max_interpolation_value()) { this.settings.t = 0; }
            refresh_displays(this.gui);
        }

        let segments = spline.segments;
        segments.forEach(segment => {
            let type_string = segment.segment_type_string;

            let points = [];
            for(let i = 0; i <= 50; i++) {
                let tt = (i/50);
                let int = segment.interpolate(tt);
                points.push(int);
            }
            for(let i = 0; i < points.length-1; i++) {
                three_engine.draw_debug_line(points[i], points[i+1], false, 0.01, 0x333333);
            }

            if(type_string === 'bezier') {
                let control_points = segment.control_point_mat;
                three_engine.draw_debug_line(control_points[0], control_points[1], false, 0.003, 0x222222);
                three_engine.draw_debug_line(control_points[2], control_points[3], false, 0.003, 0x222222);
            }
        });

        if(this.settings.display_sphere) {
            let s = spline.interpolate(this.settings.t);
            three_engine.draw_debug_sphere(s, 0.06, 0x00eeff);
        }

        if(this.arclength_vis) {

            let prev_point = this.helper.spline.interpolate(0);
            let accumulated_points = [];
            let accumulated_distances = [];
            let accumulated_ts = [];
            let curr_distance = 0;
            for(let i = 0; i <= this.settings.num_arclength_segments; i++) {
                let t = (i / this.settings.num_arclength_segments) * this.helper.spline.max_interpolation_value();
                t = Math.min(t, this.helper.spline.max_interpolation_value()-0.0000001);
                let int = this.helper.spline.interpolate(t);
                let dis = frobenius_norm_matrix(sub_matrix_matrix(int, prev_point));
                curr_distance += dis;
                accumulated_distances.push(curr_distance);
                prev_point = int;
                accumulated_points.push(int);
                accumulated_ts.push(t);
            }

            let total_distance = curr_distance;
            let accumulated_distance = accumulated_distances[this.settings.arclength_slider];

            this.settings.total_distance = total_distance;
            this.settings.accumulated_distance = accumulated_distance;
            refresh_displays(this.gui);

            for(let i = 0; i < this.settings.arclength_slider; i++) {
                let p1 = accumulated_points[i];
                let p2 = accumulated_points[i+1];

                three_engine.draw_debug_line(p1, p2, false, 0.015, 0xffee00);
                three_engine.draw_debug_sphere(p2, 0.02, 0xff00aa);
            }

            // if(this.settings.display_arclength_sphere) {
            let tt = this.settings.t;
            let rr = tt / this.helper.spline.max_interpolation_value();
            rr = Math.min(rr, this.helper.spline.max_interpolation_value() - 0.000001);
            let rd = total_distance * rr;

            this.settings.distance_ratio = rd;

            let lower_bound_t;
            let upper_bound_t;
            let r;

            for (let i = 0; i < accumulated_distances.length - 1; i++) {
                if (accumulated_distances[i] <= rd && rd <= accumulated_distances[i + 1]) {
                    this.settings.lower_bound_dis = accumulated_distances[i];
                    this.settings.upper_bound_dis = accumulated_distances[i+1];
                    lower_bound_t = accumulated_ts[i];
                    upper_bound_t = accumulated_ts[i + 1];
                    r = (rd - accumulated_distances[i]) / (accumulated_distances[i + 1] - accumulated_distances[i]);
                    break;
                }
            }

            this.settings.lower_bound_t = lower_bound_t;
            this.settings.upper_bound_t = upper_bound_t;
            this.settings.r = r;

            let interpolated_t = (1 - r) * lower_bound_t + r * upper_bound_t;
            this.settings.target_t = interpolated_t;
            interpolated_t = Math.min(interpolated_t, this.helper.spline.max_interpolation_value() - 0.000001);
            let s = spline.interpolate(interpolated_t);
            if(this.settings.display_arclength_sphere) {
                three_engine.draw_debug_sphere(s, 0.06, 0xaa44ff);
            }
            refresh_displays(this.gui);
            // }
        }
    }
}

/**
 * Represents a B-spline.
 */
export class BSpline {
    /*
    To make a closed loop of degree d, make sure your control points
    repeat the first d control points at the end.  This is not done automatically
    */
    /**
     * Creates a new B-spline.
     * @param {number[][]} control_points - The control points of the B-spline.
     * @param {number} d - The degree of the spline.
     * @param {boolean} [closed=false] - Whether the spline forms a closed loop.
     */
    constructor(control_points, d, closed=false) {
        this.control_points = control_points.slice();
        // this.control_points_original = control_points.slice();
        this.closed = closed;

        /*
        if(this.closed) {
            for(let i = 0; i < d; i++) {
                this.control_points.push(this.control_points_original[i % this.control_points_original.length]);
            }
        }
        */

        this.control_point_dim = this.control_points[0].length;
        this.n = this.control_points.length;
        this.d = d;
        this.k = d+1;

        this.j = [];
        let curr = -this.k / 2;
        // let curr = 0;
        for(let i = 0; i < this.n + this.k; i++) {
            this.j.push(curr);
            curr += 1;
        }
    }

    /**
     * Generates a new B-spline with random control points.
     * @param {number} num_points - The number of control points.
     * @param {number} dimension_of_control_points - The dimension of each control point.
     * @param {number} d - The degree of the spline.
     * @returns {BSpline} The generated B-spline.
     */
    static new_random_points(num_points, dimension_of_control_points, d) {
        let control_points = [];
        for(let i = 0; i < num_points; i++) {
            let control_point = []
            for(let j = 0; j < dimension_of_control_points; j++) {
                control_point.push( (Math.random() * 2 - 1)*0.4 );
            }
            control_points.push(control_point);
        }
        return new BSpline(control_points, d);
    }

    /**
     * Updates a control point in the B-spline.
     * @param {number} control_point_idx - The index of the control point to update.
     * @param {number[]} new_control_point - The new control point.
     */
    update_control_point(control_point_idx, new_control_point) {
        if(control_point_idx >= this.control_points.length) {
            throw new Error('control_point_idx is too large');
        }

        this.control_points[control_point_idx] = new_control_point;
    }

    /**
     * Updates the degree of the B-spline.
     * @param {number} new_d - The new degree of the spline.
     */
    update_d(new_d) {
        // this.control_points = [];
        // this.control_points = this.control_points_original.slice();

        /*
        if(this.closed) {
            for(let i = 0; i < new_d; i++) {
                this.control_points.push(this.control_points_original[i % this.control_points_original.length]);
            }
        }
        */

        this.d = new_d;
        this.k = new_d+1;
        this.n = this.control_points.length;

        this.j = [];
        let curr = -this.k / 2;
        // let curr = 0;
        for(let i = 0; i < this.n + this.k; i++) {
            this.j.push(curr);
            curr += 1;
        }
    }

    /**
     * The Cox-de Boor recursion formula.
     * @param {number} t - The parameter value.
     * @param {number} i - The index.
     * @param {number} k - The current recursion level.
     * @returns {number} The computed Cox-de Boor value.
     */
    cox_de_boor(t, i, k) {
        if (k === 1) {
            if(this.j[i] <= t && t < this.j[i+1]) { return 1; }
            else { return 0; }
        } else {

            let f1 = (t - this.j[i]) / (this.j[i + k - 1] - this.j[i]);
            let f2 = (this.j[i + k] - t) / (this.j[i + k] - this.j[i+1]);

            return f1 * this.cox_de_boor(t, i, k-1) + f2 * this.cox_de_boor(t, i+1, k-1);
        }
    }

    /**
     * Interpolates the B-spline using the Cox-de Boor vector.
     * @param {number} t - The parameter value.
     * @param {number[]} cox_de_boor_vec - The Cox-de Boor vector.
     * @returns {number[]} The interpolated point.
     */
    interpolate_from_cox_de_boor_vec(t, cox_de_boor_vec) {
        // t = this.remap_t_based_on_range(t);

        let out = [];
        for(let i = 0; i < this.control_point_dim; i++) {out.push([0.0]);}

        for(let i = 0; i < this.n; i++) {
            out = add_matrix_matrix(out, mul_matrix_scalar(this.control_points[i], cox_de_boor_vec[i]));
        }

        return out;
    }

    /**
     * Interpolates the B-spline at a given parameter t.
     * @param {number} t - The parameter value for interpolation.
     * @returns {number[]} The interpolated point.
     */
    interpolate(t) {
        // t = this.remap_t_based_on_range(t);

        if(t < 0 || t > this.n-1) {
            throw new Error('t is out of range');
        }

        let cox_de_boor_vec = this.get_cox_de_boor_vec(t);
        return this.interpolate_from_cox_de_boor_vec(t, cox_de_boor_vec);
    }

    /**
     * Computes the Cox-de Boor vector for a given t.
     * @param {number} t - The parameter value.
     * @returns {number[]} The Cox-de Boor vector.
     */
    get_cox_de_boor_vec(t) {
        t = this.remap_t_based_on_range(t);

        let out = [];
        for(let i = 0; i < this.n; i++) {
            out.push( this.cox_de_boor(t, i, this.k) );
        }
        return out;
    }

    /**
     * Remaps the parameter t based on the current range of the spline.
     * @param {number} t - The parameter value.
     * @returns {number} The remapped value of t.
     */
    remap_t_based_on_range(t) {
        let range = this.get_range();
        let min = range[0];
        let max = range[1];

        let ratio = t / (this.n - 1);
        return (1-ratio)*min + ratio*max;
    }

    /**
     * Gets the current range of the B-spline.
     * @returns {number[]} An array containing the minimum and maximum values of the range.
     */
    get_range() {
        if(this.closed) {
            return [ (this.d-1) * 0.5, this.n-1 - ((this.d-1)*0.5) ];
        } else {
            return [ 0, this.n-1 ];
        }
    }
}

/**
 * Helper class for visualizing B-splines using THREE.js.
 */
export class BSplineVisualizerHelper {
    /**
     * Creates a new visualizer helper for a B-spline.
     * @param {BSpline} spline - The B-spline to visualize.
     * @param {Object} three_engine - The THREE.js engine.
     * @param {Object} transform_gizmo_engine - The engine for managing transform gizmos.
     */
    constructor(spline, three_engine, transform_gizmo_engine) {
        this.spline = spline;
        this.three_engine = three_engine;
        this.transform_gizmo_engine = transform_gizmo_engine;

        this.transform_gizmo_idxs = [];
        for(let i = 0; i < this.spline.n; i++) {
            let control_point = this.spline.control_points[i];
            if(control_point.length === 2) {
                let idx = this.transform_gizmo_engine.add_gizmo_SO2_matrix_and_position(three_engine, identity_matrix(2), control_point, 0.3);
                this.transform_gizmo_idxs.push(idx);

                let transform_control = this.transform_gizmo_engine.transform_controls[idx];

                transform_control.addEventListener('change', () => {
                    let t = this.transform_gizmo_engine.get_gizmo_pose_as_SO2_matrix_and_position(idx)[1];
                    this.spline.update_control_point(i, t);
                });

            } else if(control_point.length === 3) {
                let idx = this.transform_gizmo_engine.add_gizmo_SO3_matrix_and_position(three_engine, identity_matrix(3), control_point, 0.3);
                this.transform_gizmo_idxs.push(idx);

                let transform_control = this.transform_gizmo_engine.transform_controls[idx];
                transform_control.addEventListener('change', () => {
                    let t = this.transform_gizmo_engine.get_gizmo_pose_as_SO3_matrix_and_position(idx)[1];
                    this.spline.update_control_point(i, t);
                });

            } else if(control_point.length === 6) {
                let se3_mat = vec6_to_se3_mat(control_point);
                let SE3_mat = exp_se3_to_SE3(se3_mat);
                let idx = this.transform_gizmo_engine.add_gizmo_SE3_matrix(three_engine, SE3_mat, 0.3);
                this.transform_gizmo_idxs.push(idx);

                let transform_control = this.transform_gizmo_engine.transform_controls[idx];
                transform_control.addEventListener('change', () => {
                    let [SO3_mat, position] = this.transform_gizmo_engine.get_gizmo_pose_as_SO3_matrix_and_position(idx);
                    let [so3_mat, v] = ln_SO3_and_t_to_so3_and_v(S03_mat, position);
                    let s = unroll_matrix_to_list(so3_mat_to_vec3(so3_mat));
                    v = unroll_matrix_to_list(v);

                    let tt = [ v[0], v[1], v[2], s[0], s[1], s[2] ];

                    this.spline.update_control_point(i, tt);
                });

            } else {
                throw new Error('cannot handle');
            }
        }
    }
}

/**
 * Visualizer for B-splines using THREE.js.
 */
export class BSplineVisualizer {
    /**
     * Creates a new visualizer for a B-spline.
     * @param {BSplineVisualizerHelper} helper - The helper for visualizing the spline.
     * @param {number} [init_d=1] - The initial degree of the spline.
     * @param {boolean} [freeze_d=false] - Whether to freeze the degree of the spline.
     */
    constructor(helper, init_d=1, freeze_d=false) {
        this.helper = helper;

        let gui = get_default_lil_gui();

        this.settings = {
            t: 0,
            gizmos_visible: false,
            draw_second_grid: true,
            draw_sum_vecs: false,
            d: init_d
        };

        this.is_playing = false;
        this.actions = {
            play: () => {
                this.is_playing = true;
            },
            stop: () => {
                this.is_playing = false;
            }
        }

        gui.add(this.settings, 't', 0, this.helper.spline.n -1);
        let a = gui.add(this.settings, 'd', 1, 10, 1).name('d');
        if(freeze_d) { a.disable(); }
        gui.add(this.settings, 'gizmos_visible').name('Gizmos Visible');
        gui.add(this.settings, 'draw_second_grid').name('Draw Grid');
        gui.add(this.settings, 'draw_sum_vecs').name('Draw Sum Vecs');
        gui.add(this.actions, 'play').name('Play');
        gui.add(this.actions, 'stop').name('Stop');

        this.gui = gui;
    }

    /**
     * Creates a new visualizer for a B-spline.
     * @param {BSplineVisualizerHelper} helper - The helper for visualizing the spline.
     * @param {number} [init_d=1] - The initial degree of the spline.
     * @param {boolean} [freeze_d=false] - Whether to freeze the degree of the spline.
     */
    three_loop_function(three_engine) {
        this.helper.spline.update_d(this.settings.d);
        this.helper.transform_gizmo_engine.set_visibility_of_all_gizmos(three_engine, this.settings.gizmos_visible);

        if(this.is_playing) {
            this.settings.t += 0.005;
            if(this.settings.t > this.helper.spline.n -1) { this.settings.t = 0; }
            refresh_displays(this.gui);
        }

        let spline = this.helper.spline;

        let points = [];
        for (let i = 0; i <= 100; i++) {
            let t = (i / 100) * (spline.n - 1);
            points.push(spline.interpolate(t));
        }
        for(let i = 0; i < points.length-1; i++) {
            three_engine.draw_debug_line(points[i], points[i+1], false, 0.01, 0x555555);
        }

        let cox_de_boor_vec = spline.get_cox_de_boor_vec(this.settings.t);
        let s = spline.interpolate_from_cox_de_boor_vec(this.settings.t, cox_de_boor_vec);

        three_engine.draw_debug_sphere(s, 0.06, 0x00eeff);

        let control_points = spline.control_points;
        let shift;
        if(control_points[0].length === 2) {
            shift = [3, 0];
        } else {
            shift = [3, 0, 0];
        }

        if(this.settings.draw_second_grid) {
            three_engine.draw_debug_grid_plane([3, 0, 0], [1, 0, 0], [0, 1, 0], 1, 1, undefined, undefined, undefined, false);
            for (let i = 0; i < control_points.length; i++) {
                three_engine.draw_debug_vector(shift, add_matrix_matrix(shift, control_points[i]), 0.01, undefined, 0x777777);
                three_engine.draw_debug_vector(shift, add_matrix_matrix(shift, mul_matrix_scalar(control_points[i], cox_de_boor_vec[i])), 0.015, undefined, 0xee00ff);
            }
        }

        if(this.settings.draw_sum_vecs) {
            let curr;
            if(control_points[0].length === 2) {
                curr = [[0], [0]];
            } else {
                curr = [[0], [0], [0]];
            }
            for(let i = 0; i < control_points.length; i++) {
                let new_curr = add_matrix_matrix(curr, mul_matrix_scalar(control_points[i], cox_de_boor_vec[i]));
                three_engine.draw_debug_vector(curr, new_curr, 0.015, undefined, 0xee00ff);
                curr = new_curr;
            }
        }
     }
}

/**
 * Computes arclength parameterization for a spline and interpolates the spline at the given parameter.
 * @param {number} t - The parameter value.
 * @param {PolynomialBasedSpline} spline - The spline to interpolate.
 * @param {number} spline_max_t - The maximum value of t for the spline.
 * @param {Array} components - The arclength components computed from `get_arclength_components`.
 * @returns {number[]} The interpolated point on the spline.
 */
export function arclength_parameterize_spline_interpolate(t, spline, spline_max_t, components) {
    /*
    let prev_point = spline.interpolate(0);
    let accumulated_points = [];
    let accumulated_distances = [];
    let accumulated_ts = [];
    let curr_distance = 0;
    for(let i = 0; i <= num_samples; i++) {
        let t = (i / num_samples) * spline_max_t;
        t = Math.min(t, spline_max_t-0.0000001);
        let int = spline.interpolate(t);
        let dis = frobenius_norm_matrix(sub_matrix_matrix(int, prev_point));
        curr_distance += dis;
        accumulated_distances.push(curr_distance);
        prev_point = int;
        accumulated_points.push(int);
        accumulated_ts.push(t);
    }
    */

    // let total_distance = curr_distance;
    // let accumulated_distance = accumulated_distances[this.settings.arclength_slider];

    let [ accumulated_points, accumulated_distances, accumulated_ts, total_distance ] = components;

    let tt = t;
    let rr = tt / spline_max_t;
    rr = Math.min(rr, spline_max_t - 0.000001);
    let rd = total_distance * rr;

    let lower_bound_t;
    let upper_bound_t;
    let r;

    for (let i = 0; i < accumulated_distances.length - 1; i++) {
        if (accumulated_distances[i] <= rd && rd <= accumulated_distances[i + 1]) {
            lower_bound_t = accumulated_ts[i];
            upper_bound_t = accumulated_ts[i + 1];
            r = (rd - accumulated_distances[i]) / (accumulated_distances[i + 1] - accumulated_distances[i]);
            break;
        }
    }

    let interpolated_t = (1 - r) * lower_bound_t + r * upper_bound_t;

    return spline.interpolate(interpolated_t);
}

/**
 * Computes arclength components for a given spline.
 * @param {PolynomialBasedSpline} spline - The spline for which to compute the components.
 * @param {number} spline_max_t - The maximum value of t for the spline.
 * @param {number} [num_samples=50] - The number of samples for arclength parameterization.
 * @returns {Array} An array containing the accumulated points, distances, ts, and total distance.
 */
export function get_arclength_components(spline, spline_max_t, num_samples=50) {
    let prev_point = spline.interpolate(0);
    let accumulated_points = [];
    let accumulated_distances = [];
    let accumulated_ts = [];
    let curr_distance = 0;
    for(let i = 0; i <= num_samples; i++) {
        let t = (i / num_samples) * spline_max_t;
        t = Math.min(t, spline_max_t-0.0000001);
        let int = spline.interpolate(t);
        let dis = frobenius_norm_matrix(sub_matrix_matrix(int, prev_point));
        curr_distance += dis;
        accumulated_distances.push(curr_distance);
        prev_point = int;
        accumulated_points.push(int);
        accumulated_ts.push(t);
    }

    let total_distance = curr_distance;
    // let accumulated_distance = accumulated_distances[this.settings.arclength_slider];

    return [ accumulated_points, accumulated_distances, accumulated_ts, total_distance ];
}


