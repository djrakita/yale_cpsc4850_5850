import {
    column_vectors_to_matrix,
    determinant,
    dot_product,
    get_columns,
    gram_determinant,
    gram_schmidt_with_extra_columns, mul_matrix_scalar, roll_list_into_column_vec_matrix, sub_matrix_matrix,
    zeros_matrix
} from "./utils_math.js";

export function k_blade_grade(blade_matrix) {
    if (Array.isArray(blade_matrix) && blade_matrix[0]?.length !== undefined) {
        return blade_matrix[0].length;
    }
    return 0;
}

export function k_blade_dimension(blade_matrix) {
    return blade_matrix.length;
}

/**
 *
 * @param blade_matrix
 * @returns {number}
 */
export function k_blade_weight(blade_matrix) {
    let d = k_blade_dimension(blade_matrix);
    return Math.sqrt(gram_determinant(blade_matrix)) * determinant(gram_schmidt_with_extra_columns(blade_matrix, d));
}

export function k_blade_scalar_product(blade_matrix_a, blade_matrix_b) {
    let grade_a = k_blade_grade(blade_matrix_a);
    let grade_b = k_blade_grade(blade_matrix_b);

    if (grade_a !== grade_b) { return 0.0; }

    let dim_a = k_blade_dimension(blade_matrix_a);
    let dim_b = k_blade_dimension(blade_matrix_b);

    if (dim_a !== dim_b) { return 0.0; }

    let columns_a = get_columns(blade_matrix_a);
    let columns_b = get_columns(blade_matrix_b);

    let m = zeros_matrix(grade_a, grade_a);

    for (let i = 0; i < columns_a.length; i++) {
        for (let j = 0; j < columns_b.length; j++) {
            let jj = grade_a - j - 1;

            let col_a = columns_a[i];
            let col_b = columns_b[jj];

            m[i][j] = dot_product(col_a, col_b);
        }
    }

    return determinant(m);
}

export function k_blade_reversion(blade_matrix) {
    let columns = get_columns(blade_matrix);
    let columns_reversed = columns.reverse();
    return column_vectors_to_matrix(columns_reversed);
}

export function k_blade_squared_norm(blade_matrix) {
    let reversion = k_blade_reversion(blade_matrix);
    return k_blade_scalar_product(blade_matrix, reversion);
}

export function k_blade_norm(blade_matrix) {
    return Math.sqrt(k_blade_squared_norm(blade_matrix));
}

export function k_blade_angle(blade_matrix_a, blade_matrix_b) {
    let reversion_b = k_blade_reversion(blade_matrix_b);
    let n = k_blade_scalar_product(blade_matrix_a, reversion_b);
    let norm_a = k_blade_norm(blade_matrix_a);
    let norm_b = k_blade_norm(blade_matrix_b);
    let d = norm_a * norm_b;
    return Math.acos(n / d);
}

export function contraction_one_blade_and_two_blade(one_blade, two_blade_matrix) {
    let a = roll_list_into_column_vec_matrix(one_blade)
    let tmp = get_columns(two_blade_matrix);
    let b = tmp[0];
    let c = tmp[1];

    let d1 = dot_product(a, b);
    let t1 = mul_matrix_scalar(c, d1);

    let d2 = dot_product(a, c);
    let t2 = mul_matrix_scalar(b, d2);

    return sub_matrix_matrix(t1, t2);
}