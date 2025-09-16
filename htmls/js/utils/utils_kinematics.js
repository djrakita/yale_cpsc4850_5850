import {
    exp_se3_to_SE3,
    exp_so3_and_v_to_SO3_and_t,
    exp_so3_to_SO3, ln_SO3_and_t_to_so3_and_v, so3_mat_to_vec3,
    vec3_to_so3_mat,
    vec6_to_se3_mat
} from "./utils_exp_and_log_obfuscated.js";
import {
    add_matrix_matrix, dot_product, frobenius_norm_matrix, identity_matrix,
    mul_matrix_matrix,
    mul_matrix_scalar,
    normalized_matrix, pt_dis_to_line, sub_matrix_matrix,
    unroll_matrix_to_list
} from "./utils_math.js";
import {get_default_lil_gui, refresh_displays} from "./utils_three.js";
import {
    displacement_pose_SO3_and_position,
    interpolate_poses_SO3_and_position,
    map_pose_SO3_and_position
} from "./utils_exp_and_log_obfuscated.js";
import {
    optimization_bfgs,
    optimization_gradient_descent,
    optimization_powell,
    optimization_solve
} from "./utils_optimization.js";
import {TransformGizmoEngine} from "./utils_transform_gizmo.js";

export function forward_kinematics_SE3(robot, state) {
    let num_links = robot.num_links();

    let out = [];
    for(let i = 0; i < num_links; i++) {
        out.push(exp_se3_to_SE3(vec6_to_se3_mat([0,0,0,0,0,0])));
    }

    let kinematic_hierarchy = robot.kinematic_hierarchy;

    for(let i = 1; i < kinematic_hierarchy.length; i++) {
        let layer = kinematic_hierarchy[i];
        layer.forEach(link_idx => {
            let link = robot.links[link_idx];
            let parent_joint_idx = link.parent_joint_idx;
            let parent_link_idx = link.parent_link_idx;
            let joint = robot.joints[parent_joint_idx];

            let joint_type_string = joint.joint_type_string;

            let parent_pose = out[parent_link_idx];

            let constant_transform = joint.xyz_rpy_SE3_matrix;

            let curr_pose = mul_matrix_matrix(parent_pose, constant_transform);

            if(joint_type_string === 'prismatic') {
                let joint_value = state[joint.dof_idx];

                let variable_transform = [
                    [1, 0, 0, joint_value*joint.axis[0][0]],
                    [0, 1, 0, joint_value*joint.axis[1][0]],
                    [0, 0, 1, joint_value*joint.axis[2][0]],
                    [0, 0, 0, 1]
                ];

                curr_pose = mul_matrix_matrix(curr_pose, variable_transform);

            } else if(joint_type_string === 'revolute') {
                let joint_value = state[joint.dof_idx];

                let u = joint.axis;
                let un = normalized_matrix(u);
                let us = mul_matrix_scalar(un, joint_value);
                us = unroll_matrix_to_list(us);
                let se3_mat = vec6_to_se3_mat([us[0], us[1], us[2], 0, 0, 0]);
                let variable_transform = exp_se3_to_SE3(se3_mat);

                curr_pose = mul_matrix_matrix(curr_pose, variable_transform);
            } else if(joint_type_string === 'floating') {
                let rd = joint.rotation_dof_idxs;
                let td = joint.translation_dof_idxs;

                let rv = [ state[rd[0]], state[rd[1]], state[rd[2]] ];
                let tv = [ state[td[0]], state[td[1]], state[td[2]] ];

                let so3_mat = vec3_to_so3_mat(rv);
                let R = exp_so3_to_SO3(so3_mat);

                let variable_transform = [
                    [ R[0][0], R[0][1], R[0][2], tv[0] ],
                    [ R[1][0], R[1][1], R[1][2], tv[1] ],
                    [ R[2][0], R[2][1], R[2][2], tv[2] ],
                    [ 0, 0, 0, 1 ]
                ];

                curr_pose = mul_matrix_matrix(curr_pose, variable_transform);
            }

            out[link_idx] = curr_pose;
        });
    }


    return out;
}

export function forward_kinematics_SO3_and_position(robot, state) {
    let num_links = robot.num_links();

    let out = [];
    for(let i = 0; i < num_links; i++) {
        out.push(exp_so3_and_v_to_SO3_and_t(vec3_to_so3_mat([0,0,0]), [0,0,0]));
    }

    let kinematic_hierarchy = robot.kinematic_hierarchy;

    for(let i = 1; i < kinematic_hierarchy.length; i++) {
        let layer = kinematic_hierarchy[i];
        layer.forEach(link_idx => {
            let link = robot.links[link_idx];
            let parent_joint_idx = link.parent_joint_idx;
            let parent_link_idx = link.parent_link_idx;
            let joint = robot.joints[parent_joint_idx];

            let joint_type_string = joint.joint_type_string;

            let parent_pose = out[parent_link_idx];

            let constant_transform = [joint.rpy_SO3_matrix, joint.xyz_vec];

            // let curr_pose = mul_matrix_matrix(parent_pose, constant_transform);
            let curr_pose = [ mul_matrix_matrix(parent_pose[0], constant_transform[0]), add_matrix_matrix( mul_matrix_matrix(parent_pose[0], constant_transform[1]), parent_pose[1] ) ];

            if(joint_type_string === 'prismatic') {
                let joint_value = state[joint.dof_idx];

                let variable_transform = [ identity_matrix(3), [ [joint_value*joint.axis[0][0]], [joint_value*joint.axis[1][0]], [joint_value*joint.axis[2][0]] ] ];
                // curr_pose = mul_matrix_matrix(curr_pose, variable_transform);
                curr_pose = [ mul_matrix_matrix(curr_pose[0], variable_transform[0]), add_matrix_matrix( mul_matrix_matrix(curr_pose[0], variable_transform[1]), curr_pose[1] ) ];
            } else if(joint_type_string === 'revolute') {
                let joint_value = state[joint.dof_idx];

                let u = joint.axis;
                let un = normalized_matrix(u);
                let us = mul_matrix_scalar(un, joint_value);
                us = unroll_matrix_to_list(us);
                // let se3_mat = vec6_to_se3_mat([us[0], us[1], us[2], 0, 0, 0]);
                // let variable_transform = exp_se3_to_SE3(se3_mat);
                let so3_mat = vec3_to_so3_mat(us);
                let SO3_mat = exp_so3_to_SO3(so3_mat);
                let variable_transform = [ SO3_mat, [[0], [0], [0]] ];

                // curr_pose = mul_matrix_matrix(curr_pose, variable_transform);
                curr_pose = [ mul_matrix_matrix(curr_pose[0], variable_transform[0]), add_matrix_matrix( mul_matrix_matrix(curr_pose[0], variable_transform[1]), curr_pose[1] ) ];

            } else if(joint_type_string === 'floating') {
                let rd = joint.rotation_dof_idxs;
                let td = joint.translation_dof_idxs;

                let rv = [ state[rd[0]], state[rd[1]], state[rd[2]] ];
                let tv = [ state[td[0]], state[td[1]], state[td[2]] ];

                let so3_mat = vec3_to_so3_mat(rv);
                let R = exp_so3_to_SO3(so3_mat);

                // let variable_transform = [
                //     [ R[0][0], R[0][1], R[0][2], tv[0] ],
                //     [ R[1][0], R[1][1], R[1][2], tv[1] ],
                //     [ R[2][0], R[2][1], R[2][2], tv[2] ],
                //     [ 0, 0, 0, 1 ]
                // ];
                let variable_transform = [ R, [ [tv[0]], [tv[1]], [tv[2]] ] ];

                // curr_pose = mul_matrix_matrix(curr_pose, variable_transform);
                curr_pose = [ mul_matrix_matrix(curr_pose[0], variable_transform[0]), add_matrix_matrix( mul_matrix_matrix(curr_pose[0], variable_transform[1]), curr_pose[1] ) ];
            }

            // console.log(curr_pose);
            out[link_idx] = curr_pose;
        });
    }

    return out;
}

export function forward_kinematics_SO3_and_position_all(robot, state) {
    let num_links = robot.num_links();

    let out1 = [];
    let out2 = [];
    let out3 = [];
    for(let i = 0; i < num_links; i++) {
        out1.push(exp_so3_and_v_to_SO3_and_t(vec3_to_so3_mat([0,0,0]), [0,0,0]));
        out2.push(exp_so3_and_v_to_SO3_and_t(vec3_to_so3_mat([0,0,0]), [0,0,0]));
        out3.push(exp_so3_and_v_to_SO3_and_t(vec3_to_so3_mat([0,0,0]), [0,0,0]));
    }

    let kinematic_hierarchy = robot.kinematic_hierarchy;

    for(let i = 1; i < kinematic_hierarchy.length; i++) {
        let layer = kinematic_hierarchy[i];
        layer.forEach(link_idx => {
            let link = robot.links[link_idx];
            let parent_joint_idx = link.parent_joint_idx;
            let parent_link_idx = link.parent_link_idx;
            let joint = robot.joints[parent_joint_idx];

            let joint_type_string = joint.joint_type_string;

            let parent_pose = out3[parent_link_idx];
            out1[link_idx] = parent_pose.slice();

            let constant_transform = [joint.rpy_SO3_matrix, joint.xyz_vec];

            // let curr_pose = mul_matrix_matrix(parent_pose, constant_transform);
            let curr_pose = [ mul_matrix_matrix(parent_pose[0], constant_transform[0]), add_matrix_matrix( mul_matrix_matrix(parent_pose[0], constant_transform[1]), parent_pose[1] ) ];
            out2[link_idx] = curr_pose.slice();

            if(joint_type_string === 'prismatic') {
                let joint_value = state[joint.dof_idx];

                let variable_transform = [ identity_matrix(3), [ [joint_value*joint.axis[0][0]], [joint_value*joint.axis[1][0]], [joint_value*joint.axis[2][0]] ] ];
                // curr_pose = mul_matrix_matrix(curr_pose, variable_transform);
                // curr_pose = [ mul_matrix_matrix(curr_pose[0], variable_transform[0]), add_matrix_matrix( mul_matrix_matrix(curr_pose[0], variable_transform[1]), curr_pose[1] ) ];
                curr_pose = map_pose_SO3_and_position(curr_pose, variable_transform);

            } else if(joint_type_string === 'revolute') {
                let joint_value = state[joint.dof_idx];

                let u = joint.axis;
                let un = normalized_matrix(u);
                let us = mul_matrix_scalar(un, joint_value);
                us = unroll_matrix_to_list(us);
                // let se3_mat = vec6_to_se3_mat([us[0], us[1], us[2], 0, 0, 0]);
                // let variable_transform = exp_se3_to_SE3(se3_mat);
                let so3_mat = vec3_to_so3_mat(us);
                let SO3_mat = exp_so3_to_SO3(so3_mat);
                let variable_transform = [ SO3_mat, [[0], [0], [0]] ];

                // curr_pose = mul_matrix_matrix(curr_pose, variable_transform);
                curr_pose = [ mul_matrix_matrix(curr_pose[0], variable_transform[0]), add_matrix_matrix( mul_matrix_matrix(curr_pose[0], variable_transform[1]), curr_pose[1] ) ];

            } else if(joint_type_string === 'floating') {
                let rd = joint.rotation_dof_idxs;
                let td = joint.translation_dof_idxs;

                let rv = [ state[rd[0]], state[rd[1]], state[rd[2]] ];
                let tv = [ state[td[0]], state[td[1]], state[td[2]] ];

                let so3_mat = vec3_to_so3_mat(rv);
                let R = exp_so3_to_SO3(so3_mat);

                // let variable_transform = [
                //     [ R[0][0], R[0][1], R[0][2], tv[0] ],
                //     [ R[1][0], R[1][1], R[1][2], tv[1] ],
                //     [ R[2][0], R[2][1], R[2][2], tv[2] ],
                //     [ 0, 0, 0, 1 ]
                // ];
                let variable_transform = [ R, [ [tv[0]], [tv[1]], [tv[2]] ] ];

                // curr_pose = mul_matrix_matrix(curr_pose, variable_transform);
                curr_pose = [ mul_matrix_matrix(curr_pose[0], variable_transform[0]), add_matrix_matrix( mul_matrix_matrix(curr_pose[0], variable_transform[1]), curr_pose[1] ) ];
            }

            // console.log(curr_pose);
            out3[link_idx] = curr_pose.slice();
        });
    }

    return [out1, out2, out3];
}

export function set_robot_state(engine, robot, state) {
    let fk = forward_kinematics_SE3(robot, state);
    set_robot_state_from_SE3_fk_result(engine, robot, fk);
}

export function set_robot_state_from_SE3_fk_result(engine, robot, fk_result) {
    for(let i=0; i<fk_result.length; i++) {
        robot.set_link_mesh_pose_from_SE3_matrix(engine, i, fk_result[i]);
    }
}

export function set_robot_state_from_SO3_and_position_fk_result(engine, robot, fk_result) {
    for(let i=0; i<fk_result.length; i++) {
        robot.set_link_mesh_pose_from_SO3_matrix_and_position(engine, i, fk_result[i][0], fk_result[i][1]);
    }
}

export function inverse_kinematics_SO3_and_position(robot, init_state, ik_goals, max_iter = 100) {
    let f = x => {
        let fk_res = forward_kinematics_SO3_and_position(robot, x);

        let out_sum = 0.0;
        ik_goals.forEach(ik_goal => {
            let mode = ik_goal.mode;
            let curr_pose = fk_res[ik_goal.link_idx];
            let goal_pose = ik_goal.goal_pose;

            if(mode === 'combined') {

                let disp = displacement_pose_SO3_and_position(curr_pose, goal_pose);
                let ln = ln_SO3_and_t_to_so3_and_v(disp[0], disp[1]);
                let v1 = unroll_matrix_to_list(so3_mat_to_vec3(ln[0]));
                let v2 = unroll_matrix_to_list(ln[1]);
                let v = [ v1[0], v1[1], v1[2], v2[0], v2[1], v2[2] ];
                let dis = frobenius_norm_matrix(v);

                out_sum += Math.pow(dis, 2);

            } else if(mode === 'separate') {



            } else if(mode === 'position') {

                let curr_position = curr_pose[1];
                let goal_position = goal_pose[1];

                let diff = sub_matrix_matrix(curr_position, goal_position);
                let dis = frobenius_norm_matrix(diff);

                out_sum += Math.pow(dis, 2);
            }
        });

        return out_sum;
    }

    return optimization_powell(f, init_state, max_iter);
}

export function robot_kinematic_opt(robot, init_state, goals, max_iter, solver='bfgs') {
    let f = x => {
        let fk_res = forward_kinematics_SO3_and_position(robot, x);

        let out_sum = 0.0;
        goals.forEach(goal => {
            let mode_string = goal.mode_string;

            if(mode_string === 'pose_match') {
                let curr_pose = fk_res[goal.link_idx];
                let goal_pose = goal.goal_pose;

                let disp = displacement_pose_SO3_and_position(curr_pose, goal_pose);
                let ln = ln_SO3_and_t_to_so3_and_v(disp[0], disp[1]);
                let v1 = unroll_matrix_to_list(so3_mat_to_vec3(ln[0]));
                let v2 = unroll_matrix_to_list(ln[1]);
                let v = [ v1[0], v1[1], v1[2], v2[0], v2[1], v2[2] ];
                let dis = frobenius_norm_matrix(v);

                out_sum += goal.weight*Math.pow(dis, 2);

            } else if(mode_string === 'pose_match_wrong') {
                let curr_pose = fk_res[goal.link_idx];
                let goal_pose = goal.goal_pose;

                let sub1 = sub_matrix_matrix(curr_pose[0], goal_pose[0]);
                let sub2 = sub_matrix_matrix(curr_pose[1], goal_pose[1]);

                let n1 = frobenius_norm_matrix(sub1);
                let n2 = frobenius_norm_matrix(sub2);

                out_sum += Math.pow(n1 + n2, 2);
            } else if(mode_string === 'position_match') {
                let curr_pose = fk_res[goal.link_idx];
                let goal_position = goal.goal_position;

                let curr_position = curr_pose[1];

                let diff = sub_matrix_matrix(curr_position, goal_position);
                let dis = frobenius_norm_matrix(diff);

                out_sum += goal.weight*Math.pow(dis, 2);

            } else if(mode_string === 'look_at') {
                let curr_pose = fk_res[goal.link_idx];
                let forward_axis = goal.forward_axis;
                let mapped_axis = mul_matrix_matrix(curr_pose[0], forward_axis);
                let a = curr_pose[1];
                let b = add_matrix_matrix(mul_matrix_scalar(mapped_axis, 20.0), a);

                let pt = goal.look_at_position;

                let dis = pt_dis_to_line(pt, a, b, true);

                out_sum += goal.weight*Math.pow(dis, 2);
            } else if(mode_string === 'upright') {
                let curr_pose = fk_res[goal.link_idx];
                let side_axis = goal.side_axis;
                let mapped_axis = mul_matrix_matrix(curr_pose[0], side_axis);
                let up_axis = [[0], [0], [1]];
                let dot = dot_product(mapped_axis, up_axis);
                out_sum += goal.weight*Math.pow(dot, 2);
            }
        });

        return out_sum;
    }

    // return optimization_powell(f, init_state, max_iter);
    return optimization_solve(f, init_state, max_iter, solver);
    // return optimization_gradient_descent(f, init_state, max_iter);
}

export class OptGoalSpecPoseMatch {
    constructor(link_idx, weight=1.0) {
        this.mode_string = 'pose_match'
        this.link_idx = link_idx;
        this.weight = weight;
    }
}

export class OptGoalPoseMatch {
    constructor(goal_pose, link_idx, weight=1.0) {
        this.goal_pose = goal_pose;
        this.mode_string = 'pose_match'
        this.link_idx = link_idx;
        this.weight = weight;
    }
}

export class OptGoalSpecPoseMatchWrong {
    constructor(link_idx, weight=1.0) {
        this.mode_string = 'pose_match_wrong'
        this.link_idx = link_idx;
        this.weight = weight;
    }
}

export class OptGoalPoseMatchWrong {
    constructor(goal_pose, link_idx, weight=1.0) {
        this.goal_pose = goal_pose;
        this.mode_string = 'pose_match_wrong'
        this.link_idx = link_idx;
        this.weight = weight;
    }
}

export class OptGoalSpecPositionMatch {
    constructor(link_idx, weight = 1.0) {
        this.mode_string = 'position_match'
        this.link_idx = link_idx;
        this.weight = weight;
    }
}

export class OptGoalPositionMatch {
    constructor(goal_position, link_idx, weight = 1.0) {
        this.goal_position = goal_position;
        this.mode_string = 'position_match'
        this.link_idx = link_idx;
        this.weight = weight;
    }
}

export class OptGoalSpecLookAt {
    constructor(link_idx, forward_axis, weight=1.0) {
        this.mode_string = 'look_at'
        this.link_idx = link_idx;
        this.forward_axis = forward_axis;
        this.weight = weight;
    }
}

export class OptGoalLookAt {
    constructor(look_at_position, link_idx, forward_axis, weight=1.0) {
        this.look_at_position = look_at_position;
        this.mode_string = 'look_at'
        this.link_idx = link_idx;
        this.forward_axis = forward_axis;
        this.weight = weight;
    }
}

export class OptGoalSpecUpright {
    constructor(link_idx, side_axis, weight=1.0) {
        this.mode_string = 'upright'
        this.link_idx = link_idx;
        this.side_axis = side_axis;
        this.weight = weight;
    }
}

export class OptGoalUpright {
    constructor(link_idx, side_axis, weight=1.0) {
        this.mode_string = 'upright'
        this.link_idx = link_idx;
        this.side_axis = side_axis;
        this.weight = weight;
    }
}

export class IKGoal {
    // goal_pose at this point should only be an SO3 matrix and position
    // mode can be 'combined', 'separate', or 'position'
    constructor(link_idx, goal_pose, mode) {
        this.link_idx = link_idx;
        this.goal_pose = goal_pose;
        this.mode = mode;
    }
}

export class RobotFKSlidersVisualizer {
    constructor(robot,
                init_display_mesh=true,
                init_display_wireframe=false,
                init_display_link_mesh_only_with_frame=false,
                init_all_links_selected=false,
                freeze_display_mesh=false,
                freeze_display_wireframe=false,
                freeze_display_link_mesh_only_with_frame=false,
                freeze_dof_sliders=false,
                interpolator=false) {

        this.robot = robot;
        this.interpolator = interpolator;

        this.actions = {};
        this.settings = {
            display_wireframe:init_display_wireframe,
            display_mesh:init_display_mesh,
            display_link_mesh_only_with_frame:init_display_link_mesh_only_with_frame
        };

        this.interpolator_settings = {
            t:0,
            speed:1,
            is_playing:false
        }

        this.interpolator_actions = {
            play: () => {
                this.interpolator_settings.is_playing = true;
            },
            stop: () => {
                this.interpolator_settings.is_playing = false;
            }
        }

        let gui = get_default_lil_gui();
        let a = gui.add(this.settings, 'display_mesh').name('Display Mesh');
        if(freeze_display_mesh) { a.disable(); }
        let b = gui.add(this.settings, 'display_wireframe').name('Display Wireframe');
        if(freeze_display_wireframe) { b.disable(); }
        let c = gui.add(this.settings, 'display_link_mesh_only_with_frame').name('Display Mesh only with Frame');
        if(freeze_display_link_mesh_only_with_frame) { c.disable(); }

        let init_dofs = [];

        let dof_folder = gui.addFolder('DOFs');
        let dof_idx = 0;
        for(let i = 0; i < robot.joints.length; i++) {
            let joint = robot.joints[i];
            if (joint.joint_num_dofs > 0) {
                if(joint.joint_type_string === 'floating') {
                    for(let j=0; j < 3; j++) {
                        this.settings['dof' + dof_idx.toString()] = 0;
                        let slider = dof_folder.add(this.settings, 'dof' + dof_idx.toString(), -3.14, 3.14);
                        if(freeze_dof_sliders) {slider.disable();}
                        this.actions['dof' + dof_idx.toString() + 'reset'] = () => {
                            this.settings['dof' + dof_idx.toString()] = 0;
                            refresh_displays(gui);
                        }
                        init_dofs.push(0);
                        dof_idx++;
                    }
                    for(let j=0; j < 3; j++) {
                        this.settings['dof' + dof_idx.toString()] = 0;
                        let slider = dof_folder.add(this.settings, 'dof' + dof_idx.toString(), -1, 1);
                        if(freeze_dof_sliders) {slider.disable();}
                        this.actions['dof' + dof_idx.toString() + 'reset'] = () => {
                            this.settings['dof' + dof_idx.toString()] = 0;
                            refresh_displays(gui);
                        }
                        init_dofs.push(0);
                        dof_idx++;
                    }
                } else {
                    let joint_lower_bound = joint.lower_bound;
                    let joint_upper_bound = joint.upper_bound;
                    this.settings['dof' + dof_idx.toString()] = Math.max(Math.min(0, joint_upper_bound), joint_lower_bound);
                    init_dofs.push(Math.max(Math.min(0, joint_upper_bound), joint_lower_bound));
                    let slider = dof_folder.add(this.settings, 'dof' + dof_idx.toString(), joint_lower_bound, joint_upper_bound);
                    if(freeze_dof_sliders) {slider.disable();}
                    this.actions['dof' + dof_idx.toString() + 'reset'] = () => {
                        this.settings['dof' + dof_idx.toString()] = 0;
                        refresh_displays(gui);
                    }
                    // dof_folder.add(this.actions, 'dof' + dof_idx.toString() + 'reset').name('Reset DOF');
                    dof_idx += 1;
                }
            }
        }

        let actions_folder =  gui.addFolder('Actions');
        this.actions['reset_all'] = () => {
            for(let i = 0; i < robot.num_dofs(); i++) {
                this.settings['dof' + i.toString()] = init_dofs[i];
            }
            refresh_displays(gui);
        };
        actions_folder.add(this.actions, 'reset_all').name('Reset');

        if (interpolator) {
            let interpolator_folder = gui.addFolder('Interpolator');
            this.interpolator_max = this.robot.links.length * 3 + 3;
            interpolator_folder.add(this.interpolator_settings, 't', 0, this.interpolator_max);
            interpolator_folder.add(this.interpolator_settings, 'speed', 0.0001, 10).name('playback speed');
            interpolator_folder.add(this.interpolator_actions, 'play');
            interpolator_folder.add(this.interpolator_actions, 'stop');
        }

        let links_folder = gui.addFolder('Links');
        for(let i=0; i < robot.links.length; i++) {
            let val = false;
            if(init_all_links_selected) { val = true; }
            this.settings['link' + i.toString() + 'frame'] = val;
            links_folder.add(this.settings, 'link' + i.toString() + 'frame').name('link ' + i.toString() + ': ' + robot.links[i].link_name);
        }

        this.actions['select_all'] = () => {
            for(let i=0; i < robot.links.length; i++) {
                this.settings['link' + i.toString() + 'frame'] = true;
            }
            refresh_displays(gui);
        };

        this.actions['deselect_all'] = () => {
            for(let i=0; i < robot.links.length; i++) {
                this.settings['link' + i.toString() + 'frame'] = false;
            }
            refresh_displays(gui);
        };

        actions_folder.add(this.actions, 'select_all').name('Select All Frames');
        actions_folder.add(this.actions, 'deselect_all').name('Deselect All Frames');

        this.gui = gui;
    }

    three_loop_function(three_engine) {
        if(this.interpolator) {
            if(this.interpolator_settings.is_playing) {
                this.interpolator_settings.t += this.interpolator_settings.speed * 0.01;
                if (this.interpolator_settings.t > this.interpolator_max) {
                    this.interpolator_settings.t = 0;
                }
                refresh_displays(this.gui);
            }
        }

        let state = [];
        for(let i=0; i < this.robot.num_dofs(); i++) {
            let joint_value = this.settings['dof' + i.toString()];
            state.push(joint_value);
        }
        if(!this.interpolator) {
            set_robot_state(three_engine, this.robot, state);
        }

        this.robot.set_wireframe_visibility(three_engine, this.settings.display_wireframe);
        // if(!this.settings.display_link_mesh_only_with_frame) {
        this.robot.set_mesh_visibility(three_engine, this.settings.display_mesh);
        // }

        // let fk = forward_kinematics_SE3(this.robot, state);
        // let fk = forward_kinematics_SO3_and_position(this.robot, state);
        let fk_all = forward_kinematics_SO3_and_position_all(this.robot, state);
        for(let i=0; i < this.robot.links.length; i++) {
            if(this.settings.display_link_mesh_only_with_frame && this.settings.display_mesh) {
                this.robot.set_link_mesh_visibility(three_engine, i, false);
            }
            if(this.settings.display_link_mesh_only_with_frame && this.settings.display_wireframe) {
                this.robot.set_link_wireframe_visibility(three_engine, i, false);
            }

            let frame;
            if(this.interpolator) {
                let t = this.interpolator_settings.t;
                let init_idx = this.robot.link_to_kinematic_hierarchy_order_idx[i] * 3;
                if(t >= init_idx && t <= init_idx + 3) {
                    // frame = [ identity_matrix(3), [ [0], [0], [1] ] ];
                    let v = 3 - ((init_idx + 3) - t);
                    if(0 <= v && v <= 1) {
                        let T1 = [ identity_matrix(3), [ [0], [0], [0] ] ];
                        let T2 = fk_all[0][i];
                        frame = interpolate_poses_SO3_and_position(T1, T2, v);
                    } else if(1 <= v && v <= 2) {
                        let vv = v - 1;
                        let T1 = fk_all[0][i];
                        let T2 = fk_all[1][i];
                        frame = interpolate_poses_SO3_and_position(T1, T2, vv);
                    } else if(2 <= v && v <= 3) {
                        let vv = v - 2;
                        let T1 = fk_all[1][i];
                        let T2 = fk_all[2][i];
                        frame = interpolate_poses_SO3_and_position(T1, T2, vv);
                    }

                } else if(init_idx < t) {
                    frame = fk_all[2][i];
                } else {
                    frame = [ identity_matrix(3), [ [0], [0], [0] ] ];
                }
            } else {
                frame = fk_all[2][i];
            }

            if(this.interpolator) {
                this.robot.set_link_mesh_pose_from_SO3_matrix_and_position(three_engine, i, frame[0], frame[1]);
            }

            if(this.settings['link' + i.toString() + 'frame']) {
                let R = frame[0];
                let t = frame[1];
                let rxv = [ [R[0][0]], [R[1][0]], [R[2][0]] ];
                let ryv = [ [R[0][1]], [R[1][1]], [R[2][1]] ];
                let rzv = [ [R[0][2]], [R[1][2]], [R[2][2]] ];
                // let t = [ [frame[0][3]], [frame[1][3]], [frame[2][3]] ];

                three_engine.draw_debug_line(t, add_matrix_matrix(t, mul_matrix_scalar(rxv, 0.05)), true, 0.002, 0xff3333);
                three_engine.draw_debug_line(t, add_matrix_matrix(t, mul_matrix_scalar(ryv, 0.05)), true, 0.002, 0x33ff33);
                three_engine.draw_debug_line(t, add_matrix_matrix(t, mul_matrix_scalar(rzv, 0.05)), true, 0.002, 0x3333ff);

                if(this.settings.display_link_mesh_only_with_frame && this.settings.display_mesh) {
                    this.robot.set_link_mesh_visibility(three_engine, i, true);
                }
                if(this.settings.display_link_mesh_only_with_frame && this.settings.display_wireframe) {
                    this.robot.set_link_wireframe_visibility(three_engine, i, true);
                }
            }
        }

        return fk_all[2];
    }
}

export class RobotOptVisualizer {
    constructor(three_engine,
                robot,
                init_state,
                goal_specs,
                init_display_mesh=true,
                init_display_wireframe=false,
                init_display_link_mesh_only_with_frame=false,
                init_all_links_selected=false,
                init_continuous_solves= true,
                freeze_display_mesh=false,
                freeze_display_wireframe=false,
                freeze_display_link_mesh_only_with_frame=false,
                freeze_continuous_solves=false,
                disable_solve=false,
                solver='bfgs') {

        this.robot = robot;
        this.init_state = init_state;
        this.curr_state = init_state;
        this.curr_solve = init_state;
        this.prev_solve = init_state;
        this.goal_specs = goal_specs;
        this.first_loop = true;
        this.solver = solver;

        this.solve_now = false;
        this.reset = false;
        this.toggle_widget_mode = false;
        this.actions = {
            solve_now: () => {
                this.solve_now = true;
            },
            reset: () => {
                this.reset = true;
            },
            toggle_widget_mode: () => {
                this.toggle_widget_mode = true;
        }
        };
        this.settings = {
            display_wireframe:init_display_wireframe,
            display_mesh:init_display_mesh,
            display_link_mesh_only_with_frame:init_display_link_mesh_only_with_frame,
            continuous_solves:init_continuous_solves
        };

        let gui = get_default_lil_gui();

        let a = gui.add(this.settings, 'display_mesh').name('Display Mesh');
        if(freeze_display_mesh) { a.disable(); }
        let b = gui.add(this.settings, 'display_wireframe').name('Display Wireframe');
        if(freeze_display_wireframe) { b.disable(); }
        let c = gui.add(this.settings, 'display_link_mesh_only_with_frame').name('Display Mesh only with Frame');
        if(freeze_display_link_mesh_only_with_frame) { c.disable(); }
        let d = gui.add(this.settings, 'continuous_solves').name('Continuous solves');
        if(freeze_continuous_solves) { d.disable(); }
        let e = gui.add(this.actions, 'solve_now').name('Solve now');
        if(disable_solve) { e.disable(); }
        gui.add(this.actions, 'reset').name('Reset');
        gui.add(this.actions, 'toggle_widget_mode').name('Toggle transform gizmo mode');

        let init_dofs = [];

        let dof_folder = gui.addFolder('DOFs');
        let dof_idx = 0;
        for(let i = 0; i < robot.joints.length; i++) {
            let joint = robot.joints[i];
            if (joint.joint_num_dofs > 0) {
                if(joint.joint_type_string === 'floating') {
                    for(let j=0; j < 3; j++) {
                        this.settings['dof' + dof_idx.toString()] = 0;
                        let slider = dof_folder.add(this.settings, 'dof' + dof_idx.toString(), -3.14, 3.14);
                        slider.disable();
                        // this.actions['dof' + dof_idx.toString() + 'reset'] = () => {
                        //     this.settings['dof' + dof_idx.toString()] = 0;
                        //     refresh_displays(gui);
                        // }
                        init_dofs.push(0);
                        dof_idx++;
                    }
                    for(let j=0; j < 3; j++) {
                        this.settings['dof' + dof_idx.toString()] = 0;
                        let slider = dof_folder.add(this.settings, 'dof' + dof_idx.toString(), -1, 1);
                        slider.disable();
                        // this.actions['dof' + dof_idx.toString() + 'reset'] = () => {
                        //     this.settings['dof' + dof_idx.toString()] = 0;
                        //     refresh_displays(gui);
                        // }
                        init_dofs.push(0);
                        dof_idx++;
                    }
                } else {
                    let joint_lower_bound = joint.lower_bound;
                    let joint_upper_bound = joint.upper_bound;
                    this.settings['dof' + dof_idx.toString()] = Math.max(Math.min(0, joint_upper_bound), joint_lower_bound);
                    init_dofs.push(Math.max(Math.min(0, joint_upper_bound), joint_lower_bound));
                    let slider = dof_folder.add(this.settings, 'dof' + dof_idx.toString(), joint_lower_bound, joint_upper_bound);
                    slider.disable();
                    // this.actions['dof' + dof_idx.toString() + 'reset'] = () => {
                    //     this.settings['dof' + dof_idx.toString()] = 0;
                    //     refresh_displays(gui);
                    // }
                    // dof_folder.add(this.actions, 'dof' + dof_idx.toString() + 'reset').name('Reset DOF');
                    dof_idx += 1;
                }
            }
        }

        let actions_folder =  gui.addFolder('Actions');

        let links_folder = gui.addFolder('Links');
        for(let i=0; i < robot.links.length; i++) {
            let val = false;
            if(init_all_links_selected) { val = true; }
            this.settings['link' + i.toString() + 'frame'] = val;
            links_folder.add(this.settings, 'link' + i.toString() + 'frame').name('link ' + i.toString() + ': ' + robot.links[i].link_name);
        }

        this.actions['select_all'] = () => {
            for(let i=0; i < robot.links.length; i++) {
                this.settings['link' + i.toString() + 'frame'] = true;
            }
            refresh_displays(gui);
        };

        this.actions['deselect_all'] = () => {
            for(let i=0; i < robot.links.length; i++) {
                this.settings['link' + i.toString() + 'frame'] = false;
            }
            refresh_displays(gui);
        };

        actions_folder.add(this.actions, 'select_all').name('Select All Frames');
        actions_folder.add(this.actions, 'deselect_all').name('Deselect All Frames');

        let tge = new TransformGizmoEngine(three_engine);

        let fk_res = forward_kinematics_SO3_and_position(robot, init_state);

        goal_specs.forEach(goal_spec => {
            let mode_string = goal_spec.mode_string;

            if (mode_string === 'pose_match' || mode_string === 'pose_match_wrong') {
                tge.add_gizmo_SO3_matrix_and_position(three_engine, fk_res[goal_spec.link_idx][0], fk_res[goal_spec.link_idx][1]);
            } else if (mode_string === 'position_match') {
                tge.add_gizmo_SO3_matrix_and_position(three_engine, fk_res[goal_spec.link_idx][0], fk_res[goal_spec.link_idx][1]);
            } else if (mode_string === 'look_at') {
                let forward_axis = goal_spec.forward_axis;
                let frame = fk_res[goal_spec.link_idx];
                let mapped_axis = mul_matrix_scalar(mul_matrix_matrix(frame[0], forward_axis), 0.3);
                tge.add_gizmo_SO3_matrix_and_position(three_engine, identity_matrix(3), add_matrix_matrix(frame[1], mapped_axis));
            } else if (mode_string === 'upright') {

            }
        });

        // this.time_of_next_scheduled_solve = this.settings.time_between_solves;
        this.time_of_previous_solve = three_engine.get_time_elapsed();

        this.transform_gizmo_engine = tge;
        this.gui = gui;
    }

    three_loop_function(three_engine, max_iter=100) {
        // let now = three_engine.get_time_elapsed();
        // let solve_now = now > this.time_of_next_scheduled_solve;

        if(this.reset) {
            let fk_res = forward_kinematics_SO3_and_position(this.robot, this.init_state);
            for (let i = 0; i < this.robot.num_dofs(); i++) {
                this.settings['dof' + i.toString()] = this.init_state[i];
            }
            refresh_displays(this.gui);

            let gizmo_count = 0;
            this.goal_specs.forEach(goal_spec => {
                let mode_string = goal_spec.mode_string;

                if (mode_string === 'pose_match' || mode_string === 'pose_match_wrong') {
                    this.transform_gizmo_engine.set_pose_of_gizmo_SO3_matrix_and_position(gizmo_count, fk_res[goal_spec.link_idx][0], fk_res[goal_spec.link_idx][1]);
                    gizmo_count += 1;
                    // tge.add_gizmo_SO3_matrix_and_position(three_engine, fk_res[goal_spec.link_idx][0], fk_res[goal_spec.link_idx][1]);
                } else if (mode_string === 'position_match') {
                    this.transform_gizmo_engine.set_pose_of_gizmo_SO3_matrix_and_position(gizmo_count, fk_res[goal_spec.link_idx][0], fk_res[goal_spec.link_idx][1]);
                    gizmo_count += 1;
                    // tge.add_gizmo_SO3_matrix_and_position(three_engine, fk_res[goal_spec.link_idx][0], fk_res[goal_spec.link_idx][1]);
                } else if (mode_string === 'look_at') {
                    let forward_axis = goal_spec.forward_axis;
                    let frame = fk_res[goal_spec.link_idx];
                    let mapped_axis = mul_matrix_scalar(mul_matrix_matrix(frame[0], forward_axis), 0.3);
                    // tge.add_gizmo_SO3_matrix_and_position(three_engine, identity_matrix(3), add_matrix_matrix(frame[1], mapped_axis));
                    this.transform_gizmo_engine.set_pose_of_gizmo_SO3_matrix_and_position(gizmo_count, identity_matrix(3), add_matrix_matrix(frame[1], mapped_axis));
                    gizmo_count += 1;
                } else if (mode_string === 'upright') {

                }
            });
            this.curr_state = this.init_state;
            this.reset = false;
        }

        let solve_now = this.solve_now || this.settings.continuous_solves;
        if(solve_now) {
            let goals = [];
            let gizmo_idx = 0;

            this.goal_specs.forEach(goal_spec => {
                let mode_string = goal_spec.mode_string;

                if (mode_string === 'pose_match') {
                    let pose = this.transform_gizmo_engine.get_gizmo_pose_as_SO3_matrix_and_position(gizmo_idx);
                    gizmo_idx += 1;
                    goals.push(new OptGoalPoseMatch(pose, goal_spec.link_idx, goal_spec.weight));
                } else if (mode_string === 'pose_match_wrong') {
                    let pose = this.transform_gizmo_engine.get_gizmo_pose_as_SO3_matrix_and_position(gizmo_idx);
                    gizmo_idx += 1;
                    goals.push(new OptGoalPoseMatchWrong(pose, goal_spec.link_idx, goal_spec.weight));
                } else if (mode_string === 'position_match') {
                    let pose = this.transform_gizmo_engine.get_gizmo_pose_as_SO3_matrix_and_position(gizmo_idx);
                    gizmo_idx += 1;
                    goals.push(new OptGoalPositionMatch(pose[1], goal_spec.link_idx, goal_spec.weight));
                } else if (mode_string === 'look_at') {
                    let pose = this.transform_gizmo_engine.get_gizmo_pose_as_SO3_matrix_and_position(gizmo_idx);
                    gizmo_idx += 1;
                    goals.push(new OptGoalLookAt(pose[1], goal_spec.link_idx, goal_spec.forward_axis, goal_spec.weight));
                } else if (mode_string === 'upright') {
                    goals.push(new OptGoalUpright(goal_spec.link_idx, goal_spec.side_axis, goal_spec.weight));
                }
            });

            let max_iter_here = max_iter;
            if(!this.settings.continuous_solves) { max_iter_here = 500; }
            let res = robot_kinematic_opt(this.robot, this.curr_state, goals, max_iter_here, this.solver);
            // this.prev_solve = this.curr_solve;
            this.curr_state = res;

            // this.settings['dof' + dof_idx.toString()] = 0;

            // this.time_of_previous_solve = three_engine.get_time_elapsed();
            // this.time_of_next_scheduled_solve = this.time_of_previous_solve + this.settings.time_between_solves;

            this.solve_now = false;
            this.first_loop = false;
        }

        if(this.toggle_widget_mode) {
            this.transform_gizmo_engine.toggle_mode();
            this.toggle_widget_mode = false;
        }

        // let r = (now - this.time_of_previous_solve) / (this.time_of_next_scheduled_solve - this.time_of_previous_solve);
        // r = Math.min(r, 1.0);

        // let curr_state = [];
        // for(let i = 0; i < this.curr_state.length; i++) {
            // curr_state.push(  this.prev_solve[i]*(1-r) + this.curr_solve[i]*r  );
        // }

        // this.curr_state = unroll_matrix_to_list(add_matrix_matrix(mul_matrix_scalar(this.curr_state, 1-r), mul_matrix_scalar(this.curr_solve, r)));
        // console.log(this.curr_state);
        // this.curr_state = curr_state;
        // set_robot_state(three_engine, this.robot, this.curr_state);

        this.robot.set_wireframe_visibility(three_engine, this.settings.display_wireframe);
        this.robot.set_mesh_visibility(three_engine, this.settings.display_mesh);

        let fk_res = forward_kinematics_SO3_and_position(this.robot, this.curr_state);
        // set_robot_state(three_engine, this.robot, this.curr_state);
        set_robot_state_from_SO3_and_position_fk_result(three_engine, this.robot, fk_res);

        for (let i = 0; i < this.robot.num_dofs(); i++) {
            this.settings['dof' + i.toString()] = this.curr_state[i].toFixed(3);
        }
        refresh_displays(this.gui);

        // let fk_all = forward_kinematics_SO3_and_position_all(this.robot, this.curr_state);
        for(let i=0; i < this.robot.links.length; i++) {
            if(this.settings.display_link_mesh_only_with_frame && this.settings.display_mesh) {
                this.robot.set_link_mesh_visibility(three_engine, i, false);
            }
            if(this.settings.display_link_mesh_only_with_frame && this.settings.display_wireframe) {
                this.robot.set_link_wireframe_visibility(three_engine, i, false);
            }

            let frame = fk_res[i];

            if(this.settings['link' + i.toString() + 'frame']) {
                let R = frame[0];
                let t = frame[1];
                let rxv = [ [R[0][0]], [R[1][0]], [R[2][0]] ];
                let ryv = [ [R[0][1]], [R[1][1]], [R[2][1]] ];
                let rzv = [ [R[0][2]], [R[1][2]], [R[2][2]] ];
                // let t = [ [frame[0][3]], [frame[1][3]], [frame[2][3]] ];

                three_engine.draw_debug_line(t, add_matrix_matrix(t, mul_matrix_scalar(rxv, 0.05)), true, 0.002, 0xff3333);
                three_engine.draw_debug_line(t, add_matrix_matrix(t, mul_matrix_scalar(ryv, 0.05)), true, 0.002, 0x33ff33);
                three_engine.draw_debug_line(t, add_matrix_matrix(t, mul_matrix_scalar(rzv, 0.05)), true, 0.002, 0x3333ff);

                if(this.settings.display_link_mesh_only_with_frame && this.settings.display_mesh) {
                    this.robot.set_link_mesh_visibility(three_engine, i, true);
                }
                if(this.settings.display_link_mesh_only_with_frame && this.settings.display_wireframe) {
                    this.robot.set_link_wireframe_visibility(three_engine, i, true);
                }
            }
        }

        this.goal_specs.forEach(goal_spec => {
            let mode_string = goal_spec.mode_string;

            if(mode_string === 'look_at') {
                let forward_axis = goal_spec.forward_axis;
                let mapped_axis = mul_matrix_matrix(fk_res[goal_spec.link_idx][0], forward_axis);
                let pos = fk_res[goal_spec.link_idx][1];

                three_engine.draw_debug_line(pos, add_matrix_matrix(mul_matrix_scalar(mapped_axis, 3.0), pos), true, 0.0015, 0xffee00);
            }
        });

    }
}