import {
    add_matrix_matrix,
    frobenius_norm_matrix,
    mul_matrix_scalar,
    sub_matrix_matrix,
    unroll_matrix_to_list
} from "./utils_math.js";

export function is_edge_feasible(pos1, pos2, feasibility_checker, step_size=0.15) {
    let dis = frobenius_norm_matrix(sub_matrix_matrix(pos1, pos2));
    let num_steps = Math.floor(dis / step_size);
    if(!feasibility_checker.is_feasible(pos1)) { return false; }
    if(!feasibility_checker.is_feasible(pos2)) { return false; }
    for(let i = 0; i < num_steps; i++) {
        let rr = (step_size * i) / dis;
        let interp = add_matrix_matrix(mul_matrix_scalar(pos1, 1.0 - rr), mul_matrix_scalar(pos2, rr));
        let is_feasible = feasibility_checker.is_feasible(interp);
        if(!is_feasible) { return false; }
    }
    return true;
}

export class BaseFeasibilityChecker {
    constructor() {
        if (new.target === BaseFeasibilityChecker) {
            throw new Error("BaseFeasibilityChecker is a template class and cannot be instantiated directly.");
        }
    }

    is_feasible(state_vector) {
        throw new Error("Method 'is_feasible()' must be implemented in the derived class.");
    }

    draw(three_engine) {
        throw new Error("Method 'draw()' must be implemented in the derived class.");
    }
}

export class SimpleBoundsFeasibilityChecker extends BaseFeasibilityChecker {
    constructor() {
        super();

        this.x_min = -4.0;
        this.y_min = -4.0;
        this.x_max = 4.0;
        this.y_max = 4.0;
    }

    is_feasible(state_vector) {
        let s = unroll_matrix_to_list(state_vector);

        return !(s[0] < this.x_min || s[1] < this.y_min || s[0] > this.x_max || s[1] > this.y_max);
    }

    draw(three_engine) {
        three_engine.draw_debug_line([this.x_min, this.y_min], [this.x_min, this.y_max], undefined, 0.06, 0x000000);
        three_engine.draw_debug_line([this.x_min, this.y_min], [this.x_max, this.y_min], undefined, 0.06, 0x000000);
        three_engine.draw_debug_line([this.x_max, this.y_min], [this.x_max, this.y_max], undefined, 0.06, 0x000000);
        three_engine.draw_debug_line([this.x_min, this.y_max], [this.x_max, this.y_max], undefined, 0.06, 0x000000);
    }
}

export class CircleWorldFeasibilityChecker extends SimpleBoundsFeasibilityChecker {
    constructor() {
        super();

        this.cs = [[0.0,0.0], [-2.0,-2.0], [2.0, -2.5], [-2.0, 2.3], [2.1, 2.0]];
        this.rs = [1.0, 0.6, 0.7, 1.1, 0.8];
    }

    is_feasible(state_vector) {
        let res = super.is_feasible(state_vector);
        if (!res) { return false; }

        let s = unroll_matrix_to_list(state_vector);

        for(let i = 0; i < this.cs.length; i++) {
            let dis = frobenius_norm_matrix(sub_matrix_matrix(state_vector, this.cs[i]));
            if(dis < this.rs[i]) { return false; }
        }

        return true;
    }

    draw(three_engine) {
        super.draw(three_engine);

        for(let i = 0; i < this.cs.length; i++) {
            three_engine.draw_debug_sphere(this.cs[i], this.rs[i], 0x333333, 1.0, 20);
        }
    }
}