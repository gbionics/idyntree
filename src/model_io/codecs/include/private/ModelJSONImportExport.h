// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODEL_JSON_IMPORT_EXPORT_H
#define IDYNTREE_MODEL_JSON_IMPORT_EXPORT_H

#include <iDynTree/Model.h>

#include <string>

namespace iDynTree
{

/**
 * @brief Current version of the idyntree-model-json format.
 *
 * This integer is incremented whenever the format changes in a way that breaks
 * backward or forward compatibility.
 *
 * **No backward compatibility is expected across different versions of the format.**
 * Each iDynTree release documents explicitly which version(s) it supports.
 *
 * Supported version: 1
 *
 * ---
 * **idyntree-model-json format schema (version 1)**
 *
 * Top-level object:
 * ```json
 * {
 *   "idyntree_model_json_version": <int>,
 *   "default_base_link": <string>,
 *   "links": [ <link>, ... ],
 *   "joints": [ <joint>, ... ],
 *   "additional_frames": [ <frame>, ... ],
 *   "visual_solid_shapes": [ <link_shapes>, ... ],
 *   "collision_solid_shapes": [ <link_shapes>, ... ],
 *   "sensors": [ <sensor>, ... ]
 * }
 * ```
 *
 * Link object:
 * ```json
 * {
 *   "name": <string>,
 *   "inertia": {
 *     "mass": <double>,
 *     "center_of_mass": [x, y, z],
 *     "rotational_inertia_wrt_frame_origin": [[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]]
 *   }
 * }
 * ```
 *
 * Joint object (common fields):
 * ```json
 * {
 *   "name": <string>,
 *   "type": "fixed"|"revolute"|"prismatic"|"revolute_so2"|"spherical",
 *   "parent_link": <string>,   // getFirstAttachedLink()
 *   "child_link":  <string>,   // getSecondAttachedLink()
 *   "rest_transform": <transform>  // link1_X_link2 at rest position
 * }
 * ```
 * Plus type-specific fields for non-fixed joints (see below).
 *
 * Transform object:
 * ```json
 * {
 *   "position": [x, y, z],
 *   "rotation_matrix": [[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]]
 * }
 * ```
 *
 * Axis object:
 * ```json
 * {
 *   "direction": [x, y, z],
 *   "origin":    [x, y, z]
 * }
 * ```
 *
 * Limits object (1-DOF joints):
 * ```json
 * {
 *   "has_position_limits": <bool>,
 *   "min_position": <double>,    // present only if has_position_limits == true
 *   "max_position": <double>,    // present only if has_position_limits == true
 *   "has_effort_limits": <bool>,
 *   "effort_limit": <double>,    // present only if has_effort_limits == true
 *   "has_velocity_limits": <bool>,
 *   "velocity_limit": <double>   // present only if has_velocity_limits == true
 * }
 * ```
 *
 * Dynamics object (1-DOF joints):
 * ```json
 * {
 *   "type": "NoJointDynamics"|"URDFJointDynamics",
 *   "damping": <double>,
 *   "static_friction": <double>
 * }
 * ```
 *
 * Revolute / Prismatic / RevoluteSO2 extra fields:
 * ```json
 * {
 *   "axis": <axis>,
 *   "limits": <limits>,
 *   "dynamics": <dynamics>
 * }
 * ```
 *
 * Spherical extra fields:
 * ```json
 * {
 *   "joint_center_wrt_parent": [x, y, z],
 *   "limits": {
 *     "has_position_limits": <bool>,
 *     "min_position": [min0, min1, min2],  // present only if has_position_limits == true
 *     "max_position": [max0, max1, max2],  // present only if has_position_limits == true
 *     "has_effort_limits": <bool>,
 *     "effort_limit": [e0, e1, e2],        // present only if has_effort_limits == true
 *     "has_velocity_limits": <bool>,
 *     "velocity_limit": [v0, v1, v2]       // present only if has_velocity_limits == true
 *   },
 *   "dynamics": {
 *     "type": "NoJointDynamics"|"URDFJointDynamics",
 *     "damping": [d0, d1, d2],
 *     "static_friction": [sf0, sf1, sf2]
 *   }
 * }
 * ```
 *
 * Additional frame object:
 * ```json
 * {
 *   "name": <string>,
 *   "parent_link": <string>,
 *   "link_H_frame": <transform>
 * }
 * ```
 *
 * Link shapes object:
 * ```json
 * {
 *   "link_name": <string>,
 *   "shapes": [ <shape>, ... ]
 * }
 * ```
 *
 * Shape object:
 * ```json
 * {
 *   "type": "sphere"|"box"|"cylinder"|"external_mesh",
 *   "has_name": <bool>,
 *   "name": <string>,              // present only if has_name == true
 *   "link_H_geometry": <transform>,
 *   "has_material": <bool>,
 *   "material": <material>,        // present only if has_material == true
 *   ... type-specific fields ...
 * }
 * ```
 *
 * For "external_mesh" shapes, only the mesh filename is serialized. The
 * package_dirs information is intentionally omitted to keep the JSON artifact
 * self-contained and independent from local package search paths.
 *
 * Sensor object:
 * ```json
 * {
 *   "name": <string>,
 *   "type": "six_axis_force_torque"|"accelerometer"|"gyroscope"|
 *           "three_axis_angular_accelerometer"|"three_axis_force_torque_contact",
 *   ... type-specific fields ...
 * }
 * ```
 */
static constexpr int IDYNTREE_MODEL_JSON_FORMAT_VERSION = 1;

/**
 * Export an iDynTree::Model to a JSON string in the idyntree-model-json format.
 *
 * @param model the model to export
 * @param jsonString the output JSON string
 * @return true on success, false on failure
 */
bool modelToJSONString(const Model& model, std::string& jsonString);

/**
 * Export an iDynTree::Model to a JSON file in the idyntree-model-json format.
 *
 * @param model the model to export
 * @param filename the output file path
 * @return true on success, false on failure
 */
bool modelToJSONFile(const Model& model, const std::string& filename);

/**
 * Load an iDynTree::Model from a JSON string in the idyntree-model-json format.
 *
 * @param jsonString the input JSON string
 * @param model the output model
 * @return true on success, false on failure
 */
bool modelFromJSONString(const std::string& jsonString, Model& model);

/**
 * Load an iDynTree::Model from a JSON file in the idyntree-model-json format.
 *
 * @param filename the input file path
 * @param model the output model
 * @return true on success, false on failure
 */
bool modelFromJSONFile(const std::string& filename, Model& model);

} // namespace iDynTree

#endif // IDYNTREE_MODEL_JSON_IMPORT_EXPORT_H