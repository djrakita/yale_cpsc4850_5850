/**
 * Author: Danny Rakita
 * Description: For CPSC-487-587 3D Spatial Modeling and Computing at Yale University
 */

import {
    add_matrix_matrix,
    frobenius_norm_matrix,
    identity_matrix, matrix_inverse_3x3,
    mul_matrix_matrix,
    mul_matrix_scalar, sub_matrix_matrix, transpose, unroll_matrix_to_list
} from "./utils_math.js";
import {
    convert_scalar_vector_to_wxyz_quaternion,
    convert_wxyz_to_scalar_vector_quaternion
} from "./utils_quaternion.js";

/**
 * Exponentiates an h1 quaternion to an H1 quaternion in scalar-vector form.
 *
 * @param {Array} q - The h1 quaternion in scalar-vector form.
 * @returns {Array} - The resulting H1 quaternion in scalar-vector form.
 */
export function exp_h1_to_H1_scalar_vector(q) {
    let v = q[1];

    let vn = frobenius_norm_matrix(v);
    if (vn === 0.0) {
        return [1, [[0], [0], [0]]];
    }
    let new_w = Math.cos(vn);
    let s = Math.sin(vn) / vn;
    let new_v = mul_matrix_scalar(v, s);

    return [new_w, new_v];
}

/**
 * Computes the logarithm of an H1 quaternion to an h1 quaternion in scalar-vector form.
 *
 * @param {Array} q - The H1 quaternion in scalar-vector form.
 * @returns {Array} - The resulting h1 quaternion in scalar-vector form.
 */
export function ln_H1_to_h1_scalar_vector(q) {
    let w = q[0];
    let d = Math.sin(Math.acos(w));
    if (d === 0.0) {
        return [0, [[0], [0], [0]]];
    }
    let s = Math.acos(w) / d;
    return [ 0, mul_matrix_scalar(q[1], s) ];
}

/**
 * Exponentiates an h1 quaternion to an H1 quaternion in wxyz form.
 *
 * @param {Array} q - The h1 quaternion in wxyz form.
 * @returns {Array} - The resulting H1 quaternion in wxyz form.
 */
export function exp_h1_to_H1_wxyz(q) {
    let qq = convert_wxyz_to_scalar_vector_quaternion(q);
    let res = exp_h1_to_H1_scalar_vector(qq);
    return convert_scalar_vector_to_wxyz_quaternion(res);
}

/**
 * Computes the logarithm of an H1 quaternion to an h1 quaternion in wxyz form.
 *
 * @param {Array} q - The H1 quaternion in wxyz form.
 * @returns {Array} - The resulting h1 quaternion in wxyz form.
 */
export function ln_H1_to_h1_wxyz(q) {
    let qq = convert_wxyz_to_scalar_vector_quaternion(q);
    let res = ln_H1_to_h1_scalar_vector(qq);
    return convert_scalar_vector_to_wxyz_quaternion(res);
}

/**
 * Exponentiates an se3 matrix to an SE3 matrix.
 *
 * @param {Array<Array<number>>} se3_mat - The se3 matrix.
 * @returns {Array<Array<number>>} - The resulting SE3 matrix.
 */
export function exp_se3_to_SE3(se3_mat) {
    let A_so3 = [
        [se3_mat[0][0], se3_mat[0][1], se3_mat[0][2]],
        [se3_mat[1][0], se3_mat[1][1], se3_mat[1][2]],
        [se3_mat[2][0], se3_mat[2][1], se3_mat[2][2]]
    ];

    let t = [ [se3_mat[0][3]], [se3_mat[1][3]], [se3_mat[2][3]] ];

    let [R1, t1] = exp_so3_and_v_to_SO3_and_t(A_so3, t);

    return [
        [R1[0][0], R1[0][1], R1[0][2], t1[0][0]],
        [R1[1][0], R1[1][1], R1[1][2], t1[1][0]],
        [R1[2][0], R1[2][1], R1[2][2], t1[2][0]],
        [0,0,0,1]
    ]
}

/**
 * Computes the logarithm of an SE3 matrix to an se3 matrix.
 *
 * @param {Array<Array<number>>} SE3_mat - The SE3 matrix.
 * @returns {Array<Array<number>>} - The resulting se3 matrix.
 */
export function ln_SE3_to_se3(SE3_mat) {
    let SO3_mat = [
        [SE3_mat[0][0], SE3_mat[0][1], SE3_mat[0][2]],
        [SE3_mat[1][0], SE3_mat[1][1], SE3_mat[1][2]],
        [SE3_mat[2][0], SE3_mat[2][1], SE3_mat[2][2]]
    ];

    let t = [ [SE3_mat[0][3]], [SE3_mat[1][3]], [SE3_mat[2][3]] ];

    let [R1, v1] = ln_SO3_and_t_to_so3_and_v(SO3_mat, t);

    return [
        [R1[0][0], R1[0][1], R1[0][2], v1[0][0]],
        [R1[1][0], R1[1][1], R1[1][2], v1[1][0]],
        [R1[2][0], R1[2][1], R1[2][2], v1[2][0]],
        [0,0,0,0]
    ]
}

/**
 * Exponentiates a so3 matrix and a vector to an SO3 matrix and a translation vector.
 *
 * @param {Array<Array<number>>} so3_mat - The so3 matrix.
 * @param {Array<Array<number>>} v - The translation vector.
 * @returns {Array} - The resulting SO3 matrix and translation vector.
 */
export function exp_so3_and_v_to_SO3_and_t(so3_mat, v) {
    let SO3_mat = exp_so3_to_SO3(so3_mat);

    let u = [so3_mat[2][1], so3_mat[0][2], [1][0]];
    let beta = frobenius_norm_matrix(u);

    let p;
    let q;
    if (Math.abs(beta) < 0.0001) {
        p = 0.5 - (Math.pow(beta, 2)/24) + (Math.pow(beta, 4)/720);
        q = (1./6.) - (Math.pow(beta, 2)/120) + (Math.pow(beta, 4)/ 5040);
    } else {
        p = (1 - Math.cos(beta)) / Math.pow(beta, 2);
        q = (beta - Math.sin(beta)) / Math.pow(beta, 3);
    }

    let C = identity_matrix(3);
    C = add_matrix_matrix(C, mul_matrix_scalar(so3_mat, p));
    C = add_matrix_matrix(C, mul_matrix_scalar(mul_matrix_matrix(so3_mat, so3_mat), q));

    let t = mul_matrix_matrix(C, v);

    return [SO3_mat, t];
}

/**
 * Computes the logarithm of an SO3 matrix and a translation vector to a so3 matrix and a vector.
 *
 * @param {Array<Array<number>>} SO3_mat - The SO3 matrix.
 * @param {Array<Array<number>>} t - The translation vector.
 * @returns {Array} - The resulting so3 matrix and vector.
 */
export function ln_SO3_and_t_to_so3_and_v(SO3_mat, t) {
    let so3_mat = ln_SO3_to_so3(SO3_mat);

    let u = [so3_mat[2][1], so3_mat[0][2], [1][0]];
    let beta = frobenius_norm_matrix(u);

    let p;
    let q;
    if (Math.abs(beta) < 0.0001) {
        p = 0.5 - (Math.pow(beta, 2)/24) + (Math.pow(beta, 4)/720);
        q = (1./6.) - (Math.pow(beta, 2)/120) + (Math.pow(beta, 4)/ 5040);
    } else {
        p = (1 - Math.cos(beta)) / Math.pow(beta, 2);
        q = (beta - Math.sin(beta)) / Math.pow(beta, 3);
    }

    let C = identity_matrix(3);
    C = add_matrix_matrix(C, mul_matrix_scalar(so3_mat, p));
    C = add_matrix_matrix(C, mul_matrix_scalar(mul_matrix_matrix(so3_mat, so3_mat), q));

    let C_inv = matrix_inverse_3x3(C);

    let v = mul_matrix_matrix(C_inv, t);

    return [so3_mat, v];
}

/**
 * Exponentiates a so3 matrix to an SO3 matrix.
 *
 * @param {Array<Array<number>>} so3_mat - The so3 matrix.
 * @returns {Array<Array<number>>} - The resulting SO3 matrix.
 */
export function exp_so3_to_SO3(so3_mat) {
    let A2 = mul_matrix_matrix(so3_mat, so3_mat);
    let I = identity_matrix(3);

    // let A = [[0, -ci, bi], [ci, 0, -ai], [-bi, ai, 0]];
    let ai = so3_mat[2][1];
    let bi = so3_mat[0][2];
    let ci = so3_mat[1][0];

    let u = [[ai], [bi], [ci]];
    let beta = frobenius_norm_matrix(u);

    let p;
    let q;

    if (beta < 0.001) {
        p = 1 - (Math.pow(beta, 2) / 6.0) + (Math.pow(beta, 4) / 120.0);
        q = 0.5 - (Math.pow(beta, 2) / 24.0) + (Math.pow(beta, 4) / 720.0);
    } else {
        p = Math.sin(beta) / beta;
        q = (1 - Math.cos(beta)) / Math.pow(beta, 2);
    }

    return add_matrix_matrix(add_matrix_matrix(I, mul_matrix_scalar(so3_mat, p)), mul_matrix_scalar(A2, q));
}

/**
 * Computes the logarithm of an SO3 matrix to a so3 matrix.
 *
 * @param {Array<Array<number>>} SO3_mat - The SO3 matrix.
 * @returns {Array<Array<number>>} - The resulting so3 matrix.
 */
export function ln_SO3_to_so3(SO3_mat) {
    let trace = SO3_mat[0][0] + SO3_mat[1][1] + SO3_mat[2][2];
    let beta = Math.acos( (trace - 1.0) / 2.0 );

    if (Math.abs(beta) < 0.0001) {
        let diff = sub_matrix_matrix(SO3_mat, transpose(SO3_mat));
        let num = 0.5 + Math.pow(beta, 2)/12.0 + 7.0*Math.pow(beta, 4)/720.0;
        return mul_matrix_scalar(diff, num);
    } else if (beta === Math.PI) {
        return [ [ 0.0, -Math.PI*Math.sqrt(0.5 * (SO3_mat[2][2] + 1)), Math.PI*Math.sqrt(0.5 * (SO3_mat[1][1] + 1)) ],
                 [ Math.PI*Math.sqrt(0.5 * (SO3_mat[2][2] + 1)), 0.0, -Math.PI*Math.sqrt(0.5 * (SO3_mat[0][0] + 1)) ],
                 [ -Math.PI*Math.sqrt(0.5 * (SO3_mat[1][1] + 1)) ], Math.PI*Math.sqrt(0.5 * (SO3_mat[0][0] + 1)), 0.0 ]
    } else {
        let diff = sub_matrix_matrix(SO3_mat, transpose(SO3_mat));
        let num = beta / (2.0 * Math.sin(beta));
        return mul_matrix_scalar(diff, num);
    }
}

/**
 * Exponentiates a so2 matrix and a vector to an SO2 matrix and a translation vector.
 *
 * @param {Array<Array<number>>} so2_mat - The so2 matrix.
 * @param {Array<Array<number>>} v - The translation vector.
 * @returns {Array} - The resulting SO2 matrix and translation vector.
 */
export function exp_so2_and_v_to_SO2_and_t(so2_mat, v) {
    let m = so2_mat;
    v = unroll_matrix_to_list(v);
    let se2_mat = [ [ m[0][0], m[0][1], v[0] ], [ m[1][0], m[1][1], v[1] ], [0,0,0] ];
    let r = exp_se2_to_SE2(se2_mat);

    let mm = [ [r[0][0], r[0][1]], [r[1][0], r[1][1]] ];
    let tt = [ [r[0][2]], [r[1][2]] ];

    return [mm, tt];
}

/**
 * Computes the logarithm of an SO2 matrix and a translation vector to a so2 matrix and a vector.
 *
 * @param {Array<Array<number>>} SO2_mat - The SO2 matrix.
 * @param {Array<Array<number>>} t - The translation vector.
 * @returns {Array} - The resulting so2 matrix and vector.
 */
export function ln_SO2_and_t_to_so2_and_v(SO2_mat, t) {
    let m = SO2_mat;
    t = unroll_matrix_to_list(t);
    let SE2_mat = [ [ m[0][0], m[0][1], t[0] ], [ m[1][0], m[1][1], t[1] ], [0,0,1] ];

    let r = ln_SE2_to_se2(SE2_mat);

    let mm = [ [r[0][0], r[0][1]], [r[1][0], r[1][1]] ];
    let tt = [ [r[0][2]], [r[1][2]] ];

    return [mm, tt];
}

/**
 * Exponentiates an se2 matrix to an SE2 matrix.
 *
 * @param {Array<Array<number>>} se2_mat - The se2 matrix.
 * @returns {Array<Array<number>>} - The resulting SE2 matrix.
 */
export function exp_se2_to_SE2(se2_mat) {
    let a1 = se2_mat[1][0];
    let a2 = se2_mat[0][2];
    let a3 = se2_mat[1][2];

    let p;
    let q;

    if(Math.abs(a1) < 0.0001) {
        p = 1 - (Math.pow(a1, 2)/6) + (Math.pow(a1, 4)/120);
        q = (a1/2) - (Math.pow(a1, 3)/24) + (Math.pow(a1, 5)/720);
    } else {
        p = Math.sin(a1) / a1;
        q = (1 - Math.cos(a1)) / a1;
    }

    let mm = [[p, -q], [q, p]];
    let tt = [[a2], [a3]];

    let rr = unroll_matrix_to_list(mul_matrix_matrix(mm, tt));

    return [[Math.cos(a1), -Math.sin(a1), rr[0]], [Math.sin(a1), Math.cos(a1), rr[1]], [0, 0, 1]];
}

/**
 * Computes the logarithm of an SE2 matrix to an se2 matrix.
 *
 * @param {Array<Array<number>>} SE2_mat - The SE2 matrix.
 * @returns {Array<Array<number>>} - The resulting se2 matrix.
 */
export function ln_SE2_to_se2(SE2_mat) {
    let c = SE2_mat[0][0];
    let s = SE2_mat[1][0];
    let a1 = Math.atan2(s, c);

    let p;
    let q;

    if(Math.abs(a1) < 0.0001) {
        p = 1 - (Math.pow(a1, 2)/6) + (Math.pow(a1, 4)/120);
        q = (a1/2) - (Math.pow(a1, 3)/24) + (Math.pow(a1, 5)/720);
    } else {
        p = Math.sin(a1) / a1;
        q = (1 - Math.cos(a1)) / a1;
    }

    let mm = [[p, q], [-q, p]];
    let tt = [ [SE2_mat[0][2]], [SE2_mat[1][2]] ];

    let rr = mul_matrix_matrix(mm, tt);

    rr = mul_matrix_scalar(rr, 1/(Math.pow(p, 2) + Math.pow(q, 2)));
    rr = unroll_matrix_to_list(rr);

    let a2 = rr[0];
    let a3 = rr[1];

    return [ [0, -a1, a2], [a1, 0, a3], [0,0,0] ];
}

/**
 * Exponentiates a so2 matrix to an SO2 matrix.
 *
 * @param {Array<Array<number>>} so2_mat - The so2 matrix.
 * @returns {Array<Array<number>>} - The resulting SO2 matrix.
 */
export function exp_so2_to_SO2(so2_mat) {
    let m = so2_mat;

    return [ [Math.cos(m[0][0]), -Math.sin(m[0][1])], [Math.sin(m[1][0]), Math.cos(m[1][1])] ];
}

/**
 * Computes the logarithm of an SO2 matrix to a so2 matrix.
 *
 * @param {Array<Array<number>>} SO2_mat - The SO2 matrix.
 * @returns {Array<Array<number>>} - The resulting so2 matrix.
 */
export function ln_SO2_to_so2(SO2_mat) {
    let c = SO2_mat[0][0];
    let s = SO2_mat[1][0];
    let a = Math.atan2(s, c);

    return [ [0, -a], [a, 0] ];
}

/**
 * Converts a scalar to a so2 matrix.
 *
 * @param {number} a - The scalar value.
 * @returns {Array<Array<number>>} - The resulting so2 matrix.
 */
export function scalar_to_so2_mat(a) {
    return [ [0, -a], [a, 0] ];
}

/**
 * Converts a so2 matrix to a scalar.
 *
 * @param {Array<Array<number>>} so2_mat - The so2 matrix.
 * @returns {number} - The scalar value.
 */
export function so2_mat_to_scalar(so2_mat) {
    return so2_mat[1][0];
}

/**
 * Converts a 3D vector to an se2 matrix.
 *
 * @param {Array<Array<number>>} v - The 3D vector.
 * @returns {Array<Array<number>>} - The resulting se2 matrix.
 */
export function vec3_to_se2_mat(v) {
    v = unroll_matrix_to_list(v);

    return [[0, -v[0], v[1]], [v[0], 0, v[2]], [0,0,0]];
}

/**
 * Converts an se2 matrix to a 3D vector.
 *
 * @param {Array<Array<number>>} se2_mat - The se2 matrix.
 * @returns {Array<Array<number>>} - The resulting 3D vector.
 */
export function se2_mat_to_vec3(se2_mat) {
    let m = se2_mat;

    return [ [m[1][0]], [m[0][2]], [m[1][2]] ];
}

/**
 * Converts a 3D vector to a so3 matrix.
 *
 * @param {Array<Array<number>>} v - The 3D vector.
 * @returns {Array<Array<number>>} - The resulting so3 matrix.
 */
export function vec3_to_so3_mat(v) {
    v = unroll_matrix_to_list(v);
    return [[ 0, -v[2], v[1] ], [v[2], 0, -v[0]], [-v[1], v[0], 0]];
}

/**
 * Converts a so3 matrix to a 3D vector.
 *
 * @param {Array<Array<number>>} so3_mat - The so3 matrix.
 * @returns {Array<Array<number>>} - The resulting 3D vector.
 */
export function so3_mat_to_vec3(so3_mat) {
    return [ [so3_mat[2][1]], [so3_mat[0][2]], [so3_mat[1][0]] ];
}

/**
 * Converts an se3 matrix to a 6D vector.
 *
 * @param {Array<Array<number>>} se3_mat - The se3 matrix.
 * @returns {Array<Array<number>>} - The resulting 6D vector.
 */
export function se3_mat_to_vec6(se3_mat) {
    return [ [se3_mat[2][1]], [se3_mat[0][2]], [se3_mat[1][0]], [se3_mat[0][1]], [se3_mat[1][1]], [se3_mat[2][1]] ];
}

/**
 * Converts a so3 matrix and a vector to a 6D vector.
 *
 * @param {Array<Array<number>>} so3_mat - The so3 matrix.
 * @param {Array<Array<number>>} v - The vector.
 * @returns {Array<Array<number>>} - The resulting 6D vector.
 */
export function so3_and_v_to_vec6(so3_mat, v) {
    v = unroll_matrix_to_list(v);
    return [ [so3_mat[2][1]], [so3_mat[0][2]], [so3_mat[1][0]], [v[0]], [v[1]], [v[2]] ];
}

/**
 * Converts a 6D vector to an se3 matrix.
 *
 * @param {Array<Array<number>>} v - The 6D vector.
 * @returns {Array<Array<number>>} - The resulting se3 matrix.
 */
export function vec6_to_se3_mat(v) {
    v = unroll_matrix_to_list(v);
    return [
        [ 0, -v[2], v[1], v[3] ],
        [ v[2], 0, -v[0], v[4] ],
        [ -v[1], v[0], 0, v[5] ],
        [0,0,0,1]
    ];
}

/**
 * Converts a 6D vector to a so3 matrix and a vector.
 *
 * @param {Array<Array<number>>} v - The 6D vector.
 * @returns {Array} - The resulting so3 matrix and vector.
 */
export function vec6_to_so3_and_v(v) {
    v = unroll_matrix_to_list(v);
    let so3 = vec3_to_so3_mat( [v[0], v[1], v[2]] );
    let vv = [[v[3]], [v[4]], [v[5]]];
    return [so3, vv];
}

/////////////////////////

/**
 * Inverts a pose represented by an SO3 matrix and a position vector.
 *
 * @param {Array} T - The pose represented by an SO3 matrix and a position vector.
 * @returns {Array} - The inverted pose.
 */
export function inverse_pose_SO3_and_position(T) {
    let R = T[0];
    let t = T[1];
    let RT = transpose(R);
    let tn = mul_matrix_scalar(t, -1.0);

    return [RT, mul_matrix_matrix(RT, tn)];
}

/**
 * Maps a pose represented by an SO3 matrix and a position vector onto another pose.
 *
 * @param {Array} T1 - The first pose represented by an SO3 matrix and a position vector.
 * @param {Array} T2 - The second pose represented by an SO3 matrix and a position vector.
 * @returns {Array} - The resulting mapped pose.
 */
export function map_pose_SO3_and_position(T1, T2) {
    let R1 = T1[0];
    let R2 = T2[0];
    let t1 = T1[1];
    let t2 = T2[1];

    return [mul_matrix_matrix(R1, R2), add_matrix_matrix(mul_matrix_matrix(R1, t2), t1)];
}

/**
 * Computes the displacement between two poses represented by SO3 matrices and position vectors.
 *
 * @param {Array} T1 - The first pose represented by an SO3 matrix and a position vector.
 * @param {Array} T2 - The second pose represented by an SO3 matrix and a position vector.
 * @returns {Array} - The resulting displacement.
 */
export function displacement_pose_SO3_and_position(T1, T2) {
    let inv = inverse_pose_SO3_and_position(T1);
    return map_pose_SO3_and_position(inv, T2);
}

/**
 * Interpolates between two poses represented by SO3 matrices and position vectors.
 *
 * @param {Array} T1 - The first pose represented by an SO3 matrix and a position vector.
 * @param {Array} T2 - The second pose represented by an SO3 matrix and a position vector.
 * @param {number} t - The interpolation factor (0 to 1).
 * @returns {Array} - The interpolated pose.
 */
export function interpolate_poses_SO3_and_position(T1, T2, t) {
    let disp = displacement_pose_SO3_and_position(T1, T2);
    let ln = ln_SO3_and_t_to_so3_and_v(disp[0], disp[1]);

    let v1 = mul_matrix_scalar(so3_mat_to_vec3(ln[0]), t);
    let v2 = mul_matrix_scalar(ln[1], t);

    let exp = exp_so3_and_v_to_SO3_and_t(vec3_to_so3_mat(v1), v2);

    return map_pose_SO3_and_position(T1, exp);
}

/**
 * Inverts a pose represented by an SO2 matrix and a position vector.
 *
 * @param {Array} T - The pose represented by an SO2 matrix and a position vector.
 * @returns {Array} - The inverted pose.
 */
export function inverse_pose_SO2_and_position(T) {
    let R = T[0];
    let t = T[1];
    let RT = transpose(R);
    let tn = mul_matrix_scalar(t, -1.0);

    return [RT, mul_matrix_matrix(RT, tn)];
}

/**
 * Maps a pose represented by an SO2 matrix and a position vector onto another pose.
 *
 * @param {Array} T1 - The first pose represented by an SO2 matrix and a position vector.
 * @param {Array} T2 - The second pose represented by an SO2 matrix and a position vector.
 * @returns {Array} - The resulting mapped pose.
 */
export function map_pose_SO2_and_position(T1, T2) {
    let R1 = T1[0];
    let R2 = T2[0];
    let t1 = T1[1];
    let t2 = T2[1];

    return [mul_matrix_matrix(R1, R2), add_matrix_matrix(mul_matrix_matrix(R1, t2), t1)];
}

/**
 * Computes the displacement between two poses represented by SO2 matrices and position vectors.
 *
 * @param {Array} T1 - The first pose represented by an SO2 matrix and a position vector.
 * @param {Array} T2 - The second pose represented by an SO2 matrix and a position vector.
 * @returns {Array} - The resulting displacement.
 */
export function displacement_pose_SO2_and_position(T1, T2) {
    let inv = inverse_pose_SO2_and_position(T1);
    return map_pose_SO2_and_position(inv, T2);
}

/**
 * Interpolates between two poses represented by SO2 matrices and position vectors.
 *
 * @param {Array} T1 - The first pose represented by an SO2 matrix and a position vector.
 * @param {Array} T2 - The second pose represented by an SO2 matrix and a position vector.
 * @param {number} t - The interpolation factor (0 to 1).
 * @returns {Array} - The interpolated pose.
 */
export function interpolate_poses_SO2_and_position(T1, T2, t) {
    let disp = displacement_pose_SO2_and_position(T1, T2);
    let ln = ln_SO2_and_t_to_so2_and_v(disp[0], disp[1]);

    let ln0 = ln[0];
    let ln1 = ln[1];

    ln0 = mul_matrix_scalar(ln0, t);
    ln1 = mul_matrix_scalar(ln1, t);

    let exp = exp_so2_and_v_to_SO2_and_t(ln0, ln1);

    return map_pose_SO2_and_position(T1, exp);
}