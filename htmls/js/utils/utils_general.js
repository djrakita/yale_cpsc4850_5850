/**
 * Author: Danny Rakita
 * Description: For CPSC-487-587 3D Spatial Modeling and Computing at Yale University
 */

/**
 * Prints a variable's value to the document body by creating a new `div` element.
 *
 * @param {any} v - The variable to print. It can be of any type that can be converted to a string.
 * @param {string} [prefix=""] - An optional prefix to prepend to the variable's value.
 */
export function print_var_to_document(v, prefix="") {
    let newDiv = document.createElement("div");
    newDiv.innerHTML = prefix + v;
    document.body.appendChild(newDiv);
}