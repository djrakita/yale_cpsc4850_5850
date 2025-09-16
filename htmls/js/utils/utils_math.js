/**
 * Author: Danny Rakita
 * Description: For CPSC-487-587 3D Spatial Modeling and Computing at Yale University
 */

// matrices should be an array of arrays in row-major format
// [
// [m00, m01, ..., m0n],
// [m10, m11, ..., m1n],
// ...
// [mm0, mm1, ..., mmn]
// ]
/**
 * Adds two matrices element-wise.
 * @param {Array<Array<number>>} m1 - The first matrix.
 * @param {Array<Array<number>>} m2 - The second matrix.
 * @returns {Array<Array<number>>} The resulting matrix after addition.
 * @throws {Error} If the matrices have different dimensions.
 */
export function add_matrix_matrix(m1, m2) {
    let mm1 = roll_list_into_column_vec_matrix(m1);
    let mm2 = roll_list_into_column_vec_matrix(m2);

    if (mm1.length !== mm2.length || mm1[0].length !== mm2[0].length) {
        throw new Error("Matrices dimensions must be the same.");
    }

    let result = new Array(mm1.length);
    for (let i = 0; i < mm1.length; i++) {
        result[i] = new Array(mm1[i].length);
        for (let j = 0; j < mm1[i].length; j++) {
            result[i][j] = mm1[i][j] + mm2[i][j];
        }
    }
    return result;
}

// matrices should be an array of arrays in row-major format
// [
// [m00, m01, ..., m0n],
// [m10, m11, ..., m1n],
// ...
// [mm0, mm1, ..., mmn]
// ]
/**
 * Subtracts the second matrix from the first matrix element-wise.
 * @param {Array<Array<number>>} m1 - The first matrix.
 * @param {Array<Array<number>>} m2 - The second matrix.
 * @returns {Array<Array<number>>} The resulting matrix after subtraction.
 * @throws {Error} If the matrices have different dimensions.
 */
export function sub_matrix_matrix(m1, m2) {
    m1 = roll_list_into_column_vec_matrix(m1);
    m2 = roll_list_into_column_vec_matrix(m2);

    if (m1.length !== m2.length || m1[0].length !== m2[0].length) {
        throw new Error("Matrices dimensions must be the same.");
    }

    let result = new Array(m1.length);
    for (let i = 0; i < m1.length; i++) {
        result[i] = new Array(m1[i].length);
        for (let j = 0; j < m1[i].length; j++) {
            result[i][j] = m1[i][j] - m2[i][j];
        }
    }
    return result;
}

// matrix should be an array of arrays in row-major format
// [
// [m00, m01, ..., m0n],
// [m10, m11, ..., m1n],
// ...
// [mm0, mm1, ..., mmn]
// ]
/**
 * Calculates the Frobenius norm of a matrix.
 * @param {Array<Array<number>>} m - The input matrix.
 * @returns {number} The Frobenius norm of the matrix.
 */
export function frobenius_norm_matrix(m) {
    m = roll_list_into_column_vec_matrix(m);

    let sum = 0;

    for (let i = 0; i < m.length; i++) {
        for (let j = 0; j < m[i].length; j++) {
            sum += m[i][j] * m[i][j];
        }
    }

    // Return the square root of the sum
    return Math.sqrt(sum);
}

// matrix should be an array of arrays in row-major format
// [
// [m00, m01, ..., m0n],
// [m10, m11, ..., m1n],
// ...
// [mm0, mm1, ..., mmn]
// ]
/**
 * Multiplies a matrix by a scalar.
 * @param {Array<Array<number>>} m - The input matrix.
 * @param {number} scalar - The scalar value.
 * @returns {Array<Array<number>>} The resulting matrix after multiplication.
 */
export function mul_matrix_scalar(m, scalar) {
    m = roll_list_into_column_vec_matrix(m);

    /*
    let result = new Array(m.length);

    for (let i = 0; i < m.length; i++) {
        result[i] = new Array(m[i].length);

        for (let j = 0; j < m[i].length; j++) {
            result[i][j] = m[i][j] * scalar;
        }
    }
    */

    let out = [];
    for(let i = 0; i<m.length; i++) {
        let row = [];
        for(let j = 0; j<m[i].length; j++) {
            row.push(m[i][j] * scalar);
        }
        out.push(row);
    }

    return out;
}

// matrix should be an array of arrays in row-major format
// [
// [m00, m01, ..., m0n],
// [m10, m11, ..., m1n],
// ...
// [mm0, mm1, ..., mmn]
// ]
/**
 * Divides a matrix by a scalar.
 * @param {Array<Array<number>>} m - The input matrix.
 * @param {number} scalar - The scalar value.
 * @returns {Array<Array<number>>} The resulting matrix after division.
 */
export function div_matrix_scalar(m, scalar) {
    m = roll_list_into_column_vec_matrix(m);

    let result = new Array(m.length);

    for (let i = 0; i < m.length; i++) {
        result[i] = new Array(m[i].length);

        for (let j = 0; j < m[i].length; j++) {
            result[i][j] = m[i][j] / scalar;
        }
    }

    return result;
}

// matrix should be an array of arrays in row-major format
// [
// [m00, m01, ..., m0n],
// [m10, m11, ..., m1n],
// ...
// [mm0, mm1, ..., mmn]
// ]
/**
 * Normalizes a matrix using its Frobenius norm.
 * @param {Array<Array<number>>} m - The input matrix.
 * @returns {Array<Array<number>>} The normalized matrix.
 */
export function normalized_matrix(m) {
    m = roll_list_into_column_vec_matrix(m);

    let f = frobenius_norm_matrix(m);
    return div_matrix_scalar(m, f);
}

// matrices should be an array of arrays in row-major format
// [
// [m00, m01, ..., m0n],
// [m10, m11, ..., m1n],
// ...
// [mm0, mm1, ..., mmn]
// ]
/**
 * Multiplies two matrices.
 * @param {Array<Array<number>>} m1 - The first matrix.
 * @param {Array<Array<number>>} m2 - The second matrix.
 * @returns {Array<Array<number>>} The resulting matrix after multiplication.
 * @throws {Error} If the number of columns in the first matrix does not match the number of rows in the second matrix.
 */
export function mul_matrix_matrix(m1, m2) {
    // m1 = roll_list_into_column_vec_matrix(m1);
    // m2 = roll_list_into_column_vec_matrix(m2);
    // console.log(m1);
    // console.log(m2);

    if (m1[0].length !== m2.length) {
        throw new Error('Incompatible matrix dimensions');
    }

    const result = new Array(m1.length).fill(0).map(() => new Array(m2[0].length).fill(0));

    for (let i = 0; i < m1.length; i++) {
        for (let j = 0; j < m2[0].length; j++) {
            for (let k = 0; k < m1[0].length; k++) {
                result[i][j] += m1[i][k] * m2[k][j];
            }
        }
    }

    return result;
}

/*
// matrix should be an array of arrays in row-major format
// [
// [m00, m01],
// [m10, m11]
// ]
// vector should be in format
// [ v0, v1 ]
export function mul_matrix_2x2_vector_2x1(matrix, vector) {
    let res = mul_matrix_matrix( matrix, [[vector[0]], [vector[1]]] );
    return [res[0][0], res[1][0]];
}

// matrix should be an array of arrays in row-major format
// [
// [m00, m01, m02],
// [m10, m11, m12],
// [m20, m21, m22]
// ]
// vector should be in format
// [ v0, v1, v2 ]
export function mul_matrix_3x3_vector_3x1(matrix, vector) {
    let res = mul_matrix_matrix(matrix, [[vector[0]], [vector[1]], [vector[2]]]);
    return [res[0][0], res[1][0], res[2][0]];
}

// matrix should be an array of arrays in row-major format
// [
// [m00, m01, m02],
// [m10, m11, m12],
// [m20, m21, m22]
// ]
// vector should be in format
// [ v0, v1 ]
export function mul_matrix_3x3_vector_2x1(matrix, vector, pad_value_at_end_of_vector=1.0) {
    let res = mul_matrix_matrix(matrix, [[vector[0]], [vector[1]], [pad_value_at_end_of_vector]]);
    return [res[0][0], res[1][0]];
}

// matrix should be an array of arrays in row-major format
// [
// [m00, m01, m02, m03],
// [m10, m11, m12, m13],
// [m20, m21, m22, m23],
// [m30, m31, m32, m33],
// ]
// vector should be in format
// [ v0, v1, v2, v3 ]
export function mul_matrix_4x4_vector_4x1(matrix, vector) {
    let res = mul_matrix_matrix(matrix, [[vector[0]], [vector[1]], [vector[2]], [vector[3]]]);
    return [res[0][0], res[1][0], res[2][0], res[3][0]];
}

// matrix should be an array of arrays in row-major format
// [
// [m00, m01, m02, m03],
// [m10, m11, m12, m13],
// [m20, m21, m22, m23],
// [m30, m31, m32, m33],
// ]
// vector should be in format
// [ v0, v1, v2 ]
export function mul_matrix_4x4_vector_3x1(matrix, vector, pad_value_at_end_of_vector=1.0) {
    let res = mul_matrix_matrix(matrix, [[vector[0]], [vector[1]], [vector[2]], [vector[3]], [pad_value_at_end_of_vector]]);
    return [res[0][0], res[1][0], res[2][0]];
}
*/

// vectors should be in column vector matrix form
// [
// [x],
// [y],
// [z]
// ]
/**
 * Computes the cross product of two vectors.
 * @param {Array<Array<number>>} v1 - The first vector (in column vector matrix form).
 * @param {Array<Array<number>>} v2 - The second vector (in column vector matrix form).
 * @returns {Array<Array<number>>} The resulting vector after the cross product.
 */
export function cross_product(v1, v2) {
    let v1u = unroll_matrix_to_list(v1);
    let v2u = unroll_matrix_to_list(v2);
    let res = cross_product_unrolled(v1u, v2u);
    return [ [res[0]], [res[1]], [res[2]] ]
}

// vectors should be arrays of three values
// [ x, y, z ]
/**
 * Computes the cross product of two 3D vectors.
 * @param {Array<number>} v1 - The first vector [x, y, z].
 * @param {Array<number>} v2 - The second vector [x, y, z].
 * @returns {Array<number>} The resulting vector [x, y, z] after the cross product.
 */
export function cross_product_unrolled(v1, v2) {
    const x = v1[1] * v2[2] - v1[2] * v2[1];
    const y = v1[2] * v2[0] - v1[0] * v2[2];
    const z = v1[0] * v2[1] - v1[1] * v2[0];

    return [x, y, z];
}

// vectors should be in column vector matrix form
// [
// [.],
// [.],
// [.],
// ...,
// [.]
// ]
/**
 * Computes the dot product of two vectors.
 * @param {Array<Array<number>>} v1 - The first vector (in column vector matrix form).
 * @param {Array<Array<number>>} v2 - The second vector (in column vector matrix form).
 * @returns {number} The dot product of the vectors.
 */
export function dot_product(v1, v2) {
    let v1u = unroll_matrix_to_list(v1);
    let v2u = unroll_matrix_to_list(v2);

    return dot_product_unrolled(v1u, v2u);
}

// vectors should be arrays of three values
// [ ., ., ., ..., . ]
/**
 * Computes the dot product of two vectors.
 * @param {Array<number>} v1 - The first vector.
 * @param {Array<number>} v2 - The second vector.
 * @returns {number} The dot product of the vectors.
 * @throws {Error} If the vectors have different dimensions.
 */
export function dot_product_unrolled(v1, v2) {
    if (v1.length !== v2.length) {
        throw new Error("Both vectors must be of the same dimension");
    }

    let dot_product = 0;
    for (let i = 0; i < v1.length; i++) {
        dot_product += (v1[i] * v2[i]);
    }

    return dot_product;
}

/**
 * Creates an identity matrix of size n x n.
 * @param {number} n - The size of the matrix.
 * @returns {Array<Array<number>>} The identity matrix.
 */
export function identity_matrix(n) {
    let out = [];
    for(let i = 0; i < n; i++) {
        let row = [];
        for(let j = 0; j < n; j++) {
            row.push(0.0);
        }
        row[i] = 1.0;
        out.push(row);
    }
    return out;
}

/**
 * Transposes a matrix.
 * @param {Array<Array<number>>} matrix - The input matrix.
 * @returns {Array<Array<number>>} The transposed matrix.
 */
export function transpose(matrix) {
    const numRows = matrix.length;
    const numCols = matrix[0].length;

    // Initialize an empty transposed matrix
    const transposed = new Array(numCols);
    for (let i = 0; i < numCols; i++) {
        transposed[i] = new Array(numRows);
    }

    // Fill the transposed matrix
    for (let i = 0; i < numRows; i++) {
        for (let j = 0; j < numCols; j++) {
            transposed[j][i] = matrix[i][j];
        }
    }

    return transposed;
}

/**
 * Adds two complex numbers.
 * @param {Array<number>} z1 - The first complex number as [real, imaginary].
 * @param {Array<number>} z2 - The second complex number as [real, imaginary].
 * @returns {Array<number>} The resulting complex number after addition.
 */
export function add_complex_numbers(z1, z2) {
    z1 = unroll_matrix_to_list(z1);
    z2 = unroll_matrix_to_list(z2);

    let a1 = z1[0];
    let a2 = z2[0];
    let b1 = z1[1];
    let b2 = z2[1];

    let new_real_part = a1 + a2;
    let new_im_part = b1 + b2;

    return [new_real_part, new_im_part];
}

/**
 * Multiplies two complex numbers.
 * @param {Array<number>} z1 - The first complex number as [real, imaginary].
 * @param {Array<number>} z2 - The second complex number as [real, imaginary].
 * @returns {Array<number>} The resulting complex number after multiplication.
 */
export function mul_complex_numbers(z1, z2) {
    z1 = unroll_matrix_to_list(z1);
    z2 = unroll_matrix_to_list(z2);

    let a1 = z1[0];
    let a2 = z2[0];
    let b1 = z1[1];
    let b2 = z2[1];

    let new_real_part = a1*a2 - b1*b2;
    let new_im_part = a1*b2 + a2*b1;

    return [new_real_part, new_im_part];
}

/**
 * Computes the factorial of a number.
 * @param {number} n - The input number.
 * @returns {number} The factorial of the input number.
 */
export function factorial(n) {
    if (n === 0 || n === 1) {
        return 1;
    } else {
        return n * factorial(n - 1);
    }
}

/**
 * Unrolls a matrix into a list (row-major order).
 * @param {Array<Array<number>>} matrix - The input matrix.
 * @returns {Array<number>} The unrolled list.
 */
export function unroll_matrix_to_list(matrix) {
    if (!Array.isArray(matrix[0])) {
        return matrix;
    }

    let unrolledArray = [];
    for (let i = 0; i < matrix.length; i++) {
        unrolledArray = unrolledArray.concat(matrix[i]);
    }

    return unrolledArray;
}

/**
 * Rolls a list into a matrix of specified dimensions.
 * @param {Array<number>} list - The input list.
 * @param {number} num_rows - The number of rows in the output matrix.
 * @param {number} num_cols - The number of columns in the output matrix.
 * @returns {Array<Array<number>>} The resulting matrix.
 */
export function roll_list_into_matrix(list, num_rows, num_cols) {
    if (Array.isArray(list[0])) {
        return list;
    }

    let matrix = [];
    for (let row = 0; row < num_rows; row++) {
        let rowArray = [];
        for (let col = 0; col < num_cols; col++) {
            let index = row * num_cols + col;
            if (index < list.length) {
                rowArray.push(list[index]);
            } else {
                rowArray.push(null); // or any other default value as needed
            }
        }
        matrix.push(rowArray);
    }
    return matrix;
}

/**
 * Rolls a list into a column vector matrix.
 * @param {Array<number>} list - The input list.
 * @returns {Array<Array<number>>} The resulting column vector matrix.
 */
export function roll_list_into_column_vec_matrix(list) {
    return roll_list_into_matrix(list, list.length, 1);
}

/**
 * Rolls a list into a row vector matrix.
 * @param {Array<number>} list - The input list.
 * @returns {Array<Array<number>>} The resulting row vector matrix.
 */
export function roll_list_into_row_vec_matrix(list) {
    return roll_list_into_matrix(list, 1, list.length);
}

/**
 * Computes the inverse of a 3x3 matrix.
 * @param {Array<Array<number>>} A - The 3x3 matrix.
 * @returns {Array<Array<number>>|null} The inverse matrix or null if the matrix is singular.
 */
export function matrix_inverse_3x3(A) {
    let det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
        A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
        A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (det === 0) {
        return null; // No inverse exists if determinant is 0
    }

    let cofactors = [
        [
            (A[1][1] * A[2][2] - A[2][1] * A[1][2]),
            -(A[1][0] * A[2][2] - A[1][2] * A[2][0]),
            (A[1][0] * A[2][1] - A[2][0] * A[1][1])
        ],
        [
            -(A[0][1] * A[2][2] - A[0][2] * A[2][1]),
            (A[0][0] * A[2][2] - A[0][2] * A[2][0]),
            -(A[0][0] * A[2][1] - A[2][0] * A[0][1])
        ],
        [
            (A[0][1] * A[1][2] - A[0][2] * A[1][1]),
            -(A[0][0] * A[1][2] - A[1][0] * A[0][2]),
            (A[0][0] * A[1][1] - A[1][0] * A[0][1])
        ]
    ];

    let adjugate = [
        [cofactors[0][0] / det, cofactors[1][0] / det, cofactors[2][0] / det],
        [cofactors[0][1] / det, cofactors[1][1] / det, cofactors[2][1] / det],
        [cofactors[0][2] / det, cofactors[1][2] / det, cofactors[2][2] / det]
    ];

    return adjugate;
}

/**
 * Projects a point onto a line segment.
 * @param {Array<number>} pt - The point [x, y, z].
 * @param {Array<number>} a - The starting point of the line segment [x, y, z].
 * @param {Array<number>} b - The ending point of the line segment [x, y, z].
 * @param {boolean} [clamp=false] - If true, clamps the projection onto the segment.
 * @returns {Array<number>} The projected point [x, y, z].
 */
export function proj_pt_onto_line(pt, a, b, clamp=false) {
    let b_minus_a = sub_matrix_matrix(b, a);
    let pt_minus_a = sub_matrix_matrix(pt, a);

    return add_matrix_matrix(proj(pt_minus_a, b_minus_a, clamp), a);
}

/**
 * Computes the distance from a point to a line segment.
 * @param {Array<number>} pt - The point [x, y, z].
 * @param {Array<number>} a - The starting point of the line segment [x, y, z].
 * @param {Array<number>} b - The ending point of the line segment [x, y, z].
 * @param {boolean} [clamp=false] - If true, clamps the projection onto the segment.
 * @returns {number} The distance from the point to the line segment.
 */
export function pt_dis_to_line(pt, a, b, clamp=false) {
    let p = proj_pt_onto_line(pt, a, b, clamp);
    let diff = sub_matrix_matrix(p, pt);

    return frobenius_norm_matrix(diff);
}

/**
 * Projects vector v onto vector u.
 * @param {Array<Array<number>>} v - The vector to project.
 * @param {Array<Array<number>>} u - The vector to project onto.
 * @param {boolean} [clamp=false] - If true, clamps the projection to the magnitude of u.
 * @returns {Array<Array<number>>} The resulting projected vector.
 */
export function proj(v, u, clamp=false) {
    let p = proj_scalar(v, u);
    if(clamp) { p = Math.min(Math.max(p, 0.0), 1.0); }

    return mul_matrix_scalar(u, p);
}

/**
 * Computes the scalar projection of vector v onto vector u.
 * @param {Array<Array<number>>} v - The vector to project.
 * @param {Array<Array<number>>} u - The vector to project onto.
 * @returns {number} The scalar projection.
 */
export function proj_scalar(v, u) {
    let n = dot_product(v, u);
    let d = Math.max(dot_product(u, u), 0.000000001);
    return n/d;
}

/**
 * Computes the determinant of a 3x3 matrix.
 * @param {Array<Array<number>>} matrix - The 3x3 matrix.
 * @returns {number} The determinant of the matrix.
 */
export function determinant3x3(matrix) {
    // Extract rows
    const [row1, row2, row3] = matrix;

    // Extract elements
    const [a, b, c] = row1;
    const [d, e, f] = row2;
    const [g, h, i] = row3;

    // Calculate the determinant using the rule of Sarrus
    const determinant = a * e * i + b * f * g + c * d * h
        - c * e * g - b * d * i - a * f * h;

    return determinant;
}

/**
 * Performs QR decomposition of a matrix using the NumPy algorithm.
 * @param {Array<Array<number>>} A - The input matrix.
 * @param {string} [mode='reduced'] - The mode of decomposition: 'reduced', 'complete', 'r', or 'raw'.
 * @returns {Object} An object containing Q and R matrices, or just R, or raw decomposition, depending on the mode.
 */
export function qr_decomposition(A, mode = 'reduced') {
    // Ensure A is a 2D array
    A = A.map(row => [...row]);
    const m = A.length;
    const n = A[0].length;
    const K = Math.min(m, n);

    // Initialize matrices
    let Q = Array(m).fill().map(() => Array(m).fill(0));
    let R = Array(m).fill().map(() => Array(n).fill(0));
    let tau = Array(K).fill(0);

    // Helper function for Householder reflection
    function householder(x, k) {
        const norm = Math.sqrt(x.slice(k).reduce((sum, val) => sum + val * val, 0));
        const s = (x[k] > 0) ? -1 : 1;
        const u1 = x[k] - s * norm;
        const w = x.slice(k).map(val => val / u1);
        w[0] = 1;
        const tau = -s * u1 / norm;
        return { w, tau };
    }

    // Perform the decomposition
    for (let k = 0; k < K; k++) {
        let x = A.map(row => row[k]);
        let { w, tau: t } = householder(x, k);
        tau[k] = t;

        // Update A
        for (let j = k; j < n; j++) {
            const dot = w.slice(0, m - k).reduce((sum, wi, i) => sum + wi * A[i + k][j], 0);
            for (let i = k; i < m; i++) {
                A[i][j] -= tau[k] * w[i - k] * dot;
            }
        }

        // Store Householder vector
        for (let i = k + 1; i < m; i++) {
            R[i][k] = w[i - k];
        }
    }

    // Extract R
    for (let i = 0; i < m; i++) {
        for (let j = 0; j < n; j++) {
            if (i <= j) {
                R[i][j] = A[i][j];
            }
        }
    }

    // Compute Q if needed
    if (mode !== 'r' && mode !== 'raw') {
        for (let k = K - 1; k >= 0; k--) {
            for (let j = k; j < m; j++) {
                Q[j][k] = (j === k) ? 1 - tau[k] : -tau[k] * R[j][k];
            }
            for (let i = k + 1; i < m; i++) {
                for (let j = k; j < m; j++) {
                    Q[j][i] -= Q[j][k] * R[i][k];
                }
            }
        }
    }

    // Return results based on mode
    switch (mode) {
        case 'reduced':
            return {
                Q: Q.map(row => row.slice(0, K)),
                R: R.slice(0, K)
            };
        case 'complete':
            return { Q, R };
        case 'r':
            return { R: R.slice(0, K) };
        case 'raw':
            return { R, tau };
        default:
            throw new Error("Invalid mode. Use 'reduced', 'complete', 'r', or 'raw'.");
    }
}

/**
 * Computes the determinant of a square matrix.
 * @param {Array<Array<number>>} matrix - The square matrix.
 * @returns {number} The determinant of the matrix.
 */
export function determinant(matrix) {
    const n = matrix.length;

    // Base case for 1x1 matrix
    if (n === 1) {
        return matrix[0][0];
    }

    // Base case for 2x2 matrix
    if (n === 2) {
        return (matrix[0][0] * matrix[1][1]) - (matrix[0][1] * matrix[1][0]);
    }

    let det = 0;
    for (let j = 0; j < n; j++) {
        det += matrix[0][j] * cofactor(matrix, 0, j);
    }

    return det;
}

/**
 * Calculates the cofactor of a matrix element.
 * @param {Array<Array<number>>} matrix - The input matrix.
 * @param {number} row - The row of the element.
 * @param {number} col - The column of the element.
 * @returns {number} The cofactor of the element.
 */
function cofactor(matrix, row, col) {
    const subMatrix = matrix
        .filter((_, i) => i !== row)
        .map(row => row.filter((_, j) => j !== col));
    return Math.pow(-1, row + col) * determinant(subMatrix);
}

/**
 * Calculates the Gram determinant of a matrix A, which is det(A^T A).
 * @param {Array<Array<number>>} matrix - The input matrix.
 * @returns {number} The Gram determinant.
 */
export function gram_determinant(matrix) {
    const AT = transpose(matrix);

    const ATA = mul_matrix_matrix(AT, matrix);

    return determinant(ATA);
}

/**
 *
 * @param matrix
 * @param d - the dimensionality of the vector space
 * @returns {number}
 */
export function k_vector_weight(matrix, d) {
    return Math.sqrt(gram_determinant(matrix)) * determinant(gram_schmidt_with_extra_columns(matrix, d));
}

/**
 * Converts a list of n column m-vectors to an m x n matrix.
 * @param {Array<Array<Array<number>>>} vectors - List of column vectors (each vector is an m-dimensional vector).
 * @returns {Array<Array<number>>} The resulting m x n matrix.
 */
export function column_vectors_to_matrix(vectors) {
    if (vectors.length === 0) {
        throw new Error("Input list is empty.");
    }

    const numRows = vectors[0].length;  // The number of rows (m)
    const numCols = vectors.length;     // The number of vectors (n)

    // Initialize an m x n matrix
    const matrix = new Array(numRows).fill(0).map(() => new Array(numCols).fill(0));

    // Populate the matrix with the column vectors
    for (let j = 0; j < numCols; j++) {
        const vector = roll_list_into_column_vec_matrix(vectors[j]);
        if (vector.length !== numRows) {
            throw new Error("All vectors must have the same number of rows.");
        }

        for (let i = 0; i < numRows; i++) {
            matrix[i][j] = vector[i][0];  // Extract the element from the column vector
        }
    }

    return matrix;
}

export function gram_schmidt_with_extra_columns(matrix, numCols, epsilon = 0.000001) {
    const numRows = matrix.length;
    let currentCols = matrix[0].length;

    if (numCols < currentCols) {
        throw new Error("Number of columns in the output matrix must be greater than or equal to the input matrix.");
    }

    // Initialize Q matrix (m x numCols)
    let Q = new Array(numRows).fill(0).map(() => new Array(numCols).fill(0));

    // Step 1: Copy the input matrix to the first columns of Q
    for (let i = 0; i < numRows; i++) {
        for (let j = 0; j < currentCols; j++) {
            Q[i][j] = matrix[i][j];
        }
    }

    // Gram-Schmidt on the original columns
    for (let j = 0; j < currentCols; j++) {
        // Orthogonalize against previous columns
        for (let k = 0; k < j; k++) {
            let dotProd = dot_product(getColumn(Q, k), getColumn(Q, j));
            let projection = mul_matrix_scalar(getColumn(Q, k), dotProd);

            for (let i = 0; i < numRows; i++) {
                Q[i][j] -= projection[i][0];
            }
        }

        // Normalize the vector
        let norm = Math.sqrt(dot_product(getColumn(Q, j), getColumn(Q, j)));
        for (let i = 0; i < numRows; i++) {
            Q[i][j] /= norm;
        }
    }

    // Step 2: Generate additional columns if needed
    if (numCols > currentCols) {
        for (let j = currentCols; j < numCols; j++) {
            // Create a one-hot vector with small non-zero elements
            for (let i = 0; i < numRows; i++) {
                Q[i][j] = (i === j % numRows) ? 1 : epsilon;
            }

            // Orthogonalize against all previous columns
            for (let k = 0; k < j; k++) {
                let dotProd = dot_product(getColumn(Q, k), getColumn(Q, j));
                let projection = mul_matrix_scalar(getColumn(Q, k), dotProd);

                for (let i = 0; i < numRows; i++) {
                    Q[i][j] -= projection[i][0];
                }
            }

            // Normalize the vector
            let norm = Math.sqrt(dot_product(getColumn(Q, j), getColumn(Q, j)));
            for (let i = 0; i < numRows; i++) {
                Q[i][j] /= norm;
            }
        }
    }

    return Q;
}

/**
 * Gets a specific column from a matrix.
 * @param {Array<Array<number>>} matrix - The input matrix.
 * @param {number} colIndex - The index of the column to retrieve.
 * @returns {Array<Array<number>>} The column vector.
 */
function getColumn(matrix, colIndex) {
    return matrix.map(row => [row[colIndex]]);
}

/**
 * Creates a seeded random number generator for determinism.
 * @param {number} seed - The seed value.
 * @returns {function(): number} A function that returns a pseudo-random number between 0 and 1.
 */
function seededRandom(seed) {
    let x = Math.sin(seed++) * 10000;
    return () => {
        x = Math.sin(seed++) * 10000;
        return x - Math.floor(x);
    };
}

/**
 * Extracts all columns from a matrix.
 * @param {Array<Array<number>>} matrix - The input matrix.
 * @returns {Array<Array<Array<number>>>} An array of column vectors, where each column is represented as a nx1 matrix.
 * @throws {Error} If the matrix is empty or not well-formed.
 */
export function get_columns(matrix) {
    // Check if matrix is empty or malformed
    if (!matrix || matrix.length === 0 || !matrix[0] || !Array.isArray(matrix[0])) {
        throw new Error("Matrix must be a non-empty 2D array");
    }

    const numRows = matrix.length;
    const numCols = matrix[0].length;
    const columns = [];

    // Extract each column as a nx1 matrix
    for (let j = 0; j < numCols; j++) {
        let column = new Array(numRows);
        for (let i = 0; i < numRows; i++) {
            column[i] = [matrix[i][j]];
        }
        columns.push(column);
    }

    return columns;
}

/**
 * Extracts all rows from a matrix.
 * @param {Array<Array<number>>} matrix - The input matrix.
 * @returns {Array<Array<Array<number>>>} An array of row vectors, where each row is represented as a 1xn matrix.
 * @throws {Error} If the matrix is empty or not well-formed.
 */
export function get_rows(matrix) {
    // Check if matrix is empty or malformed
    if (!matrix || matrix.length === 0 || !matrix[0] || !Array.isArray(matrix[0])) {
        throw new Error("Matrix must be a non-empty 2D array");
    }

    // Convert each row into a 1xn matrix format
    return matrix.map(row => [row]);
}

/**
 * Creates an m x n matrix filled with zeros.
 * @param {number} m - The number of rows.
 * @param {number} n - The number of columns.
 * @returns {Array<Array<number>>} The m x n zero matrix.
 */
export function zeros_matrix(m, n) {
    let out = [];
    for (let i = 0; i < m; i++) {
        let row = [];
        for (let j = 0; j < n; j++) {
            row.push(0.0);
        }
        out.push(row);
    }
    return out;
}
