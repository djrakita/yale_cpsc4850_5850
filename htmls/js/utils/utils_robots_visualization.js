import { RobotFromPreprocessor } from "./utils_robot.js";
import { RobotFKSlidersVisualizer } from "./utils_kinematics.js";
import { refresh_displays } from "./utils_three.js";

/**
 * Asynchronously loads and visualizes a robot in the 3D engine using configurations from the specified directory.
 *
 * @param {Object} engine - The rendering engine, including the scene, camera, renderer, and animation loop.
 * @param {string} robots_dir - The directory containing the robot configurations.
 * @param {string} robot_name - The name of the robot to be visualized.
 */
export async function visualize_robot(engine, robots_dir, robot_name) {
    const robot_dir = `${robots_dir}/${robot_name}`;

    // Fetch all necessary configuration files in parallel
    const [
        chainConfig,
        urdfConfig,
        meshConfig_stl,
        meshConfig,
        meshConfig_hull,
        meshConfig_convex_decomposition,
        shapesConfig
    ] = await Promise.all([
        fetch(`${robot_dir}/chain_module/module.json`).then(response => response.json()),
        fetch(`${robot_dir}/urdf_module/module.json`).then(response => response.json()),
        fetch(`${robot_dir}/mesh_modules/plain_meshes_module/module.json`).then(response => response.json()),
        fetch(`${robot_dir}/mesh_modules/original_meshes_module/module.json`).then(response => response.json()),
        fetch(`${robot_dir}/mesh_modules/convex_hull_meshes_module/module.json`).then(response => response.json()),
        fetch(`${robot_dir}/mesh_modules/convex_decomposition_meshes_module/module.json`).then(response => response.json()),
        fetch(`${robot_dir}/link_shapes_modules/link_shapes_approximations_module/module.json`).then(response => response.json()),
    ]);

    // Initialize the robot using the fetched configurations
    let robot = new RobotFromPreprocessor(
        chainConfig,
        urdfConfig,
        meshConfig,
        meshConfig_stl,
        'stl',
        meshConfig_hull,
        meshConfig_convex_decomposition,
        shapesConfig,
        `apollo-robots-dir/${robots_dir}`
    );

    // Spawn the robot into the 3D engine's scene
    robot.spawn_robot(engine);

    // Initialize the visualizer for robot forward kinematics (FK) sliders
    let visualizer = new RobotFKSlidersVisualizer(robot);

    // Start the animation loop
    engine.animation_loop(function() {
        visualizer.three_loop_function(engine);
        refresh_displays(visualizer.gui);
    });
}
