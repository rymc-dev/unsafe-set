import numpy as np
from colav_unsafe_set.objects import DynamicObject
import math
from typing import Tuple


def calc_cpa(
    agent_object: DynamicObject, target_object: DynamicObject
) -> Tuple[float, float]:
    """
    calculate DCPA and TCPA between agent and target
    CPA based on this code: https://github.com/MelihAkdag/ships_collision_risk_calculations/blob/main/DCPA_TCPA_with_Python.ipynb

    """
    # # distance between agent and targetship
    # distance = math.sqrt((target_object.configuration.pose.position.x - agent_object.configuration.pose.position.x)**2 + (target_object.configuration.pose.position.y - agent_object.configuration.pose.position.y) ** 2)

    # # relative speed
    # # target ship against agent vessel
    # relative_speed = target_object.configuration.velocity - agent_object.configuration.velocity

    # # alpha_r (True bearing of the targetship)
    # if (target_object.configuration.pose.position.y - agent_object.configuration.pose.position.y >= 0) and (target_object.configuration.pose.x - agent_object.configuration.pose.position.x >= 0):
    #     delta_alpha = 0
    # elif (target_object.configuration.pose.position.y - agent_object.configuration.pose.position.y >= 0) and (target_object.configuration.pose.position.x - agent_object.configuration.pose.position.x < 0):
    #     delta_alpha = 0
    # elif (target_object.configuration.pose.position.y - agent_object.configuration.pose.position.y < 0) and (target_object.configuration.pose.position.x - agent_object.configuration.pose.position.x < 0):
    #     delta_alpha = 2 * math.pi
    # elif (target_object.configuration.pose.position.y - agent_object.configuration.pose.position.y < 0) and (target_object.configuration.pose.position.x - agent_object.configuration.pose.position.x >= 0):
    #     delta_alpha = 2 * math.pi
    # alpha_r = math.atan2((target_object.configuration.pose.position.y - agent_object.configuration.pose.position.y), (target_object.configuration.pose.position.x - agent_object.configuration.pose.position.x)) + delta_alpha

    # # chi_r (Relative course of TS (from 0 to U_r))
    # if (target_object.configuration.velocity - agent_object.configuration.velocity >= 0):
    #     delta_chi = 0
    # elif (target_object.configuration.velocity - agent_object.configuration.velocity >= 0):
    #     delta_chi = 0
    # elif (target_object.configuration.velocity - agent_object.configuration.velocity < 0):
    #     delta_chi = 2 * math.pi
    # elif (target_object.configuration.velocity - agent_object.configuration.velocity < 0):
    #     delta_chi = 2 * math.pi
    # chi_r = math.atan2((target_object.configuration.velocity - .v), (targetship.u - self.u)) + delta_chi

    # # beta
    # beta = chi_r - alpha_r - math.pi

    # # DCPA and TCPA
    # dcpa = abs(round(D_r * math.sin(beta), 2))
    # tcpa = round((D_r * math.cos(beta)) / abs(U_r), 2)
    # print("DCPA:", dcpa, " TCPA:", tcpa)

    # # Collision Risk Index (CRI)
    # cons_d = 0.8
    # cons_t = 0.2
    # d_safe = 1000
    # t_safe = 1000
    # if dcpa >= 1000 or tcpa >= 1000 or tcpa < 0:
    #     cri = 0.0
    # elif dcpa < 1000:
    #     cri = 0.001 * (cons_d * (d_safe - dcpa) * cons_t * (t_safe - tcpa))
    # print("CRI: ", cri)

    # return dcpa, tcpa, cri

    # Extract positions
    p1 = np.array(
        [
            agent_object.configuration.pose.position.x,
            agent_object.configuration.pose.position.y,
        ]
    )

    p2 = np.array(
        [
            target_object.configuration.pose.position.x,
            target_object.configuration.pose.position.y,
        ]
    )

    # Extract velocities
    theta1 = 2 * np.arctan2(
        agent_object.configuration.pose.orientation.z,
        agent_object.configuration.pose.orientation.w,
    )
    theta2 = 2 * np.arctan2(
        target_object.configuration.pose.orientation.z,
        target_object.configuration.pose.orientation.w,
    )

    v1 = np.array(
        [
            agent_object.configuration.velocity * np.cos(theta1),
            agent_object.configuration.velocity * np.sin(theta1),
        ]
    )

    v2 = np.array(
        [
            target_object.configuration.velocity * np.cos(theta2),
            target_object.configuration.velocity * np.sin(theta2),
        ]
    )

    # Relative velocity and position
    v_rel = v1 - v2
    p_rel = p1 - p2

    # Check for perpendicular velocities (dot product should be near zero)
    if np.abs(np.dot(v_rel, p_rel)) < 1e-6:  # Perpendicular if dot product is near zero
        return np.inf, np.inf  # Return infinite TCPA if moving perpendicular

    # Compute TCPA
    tcpa = -np.dot(p_rel, v_rel) / (
        np.linalg.norm(v_rel) ** 2 + 1e-6
    )  # Avoid division by zero

    # Ensure TCPA is positive (future time only)
    if tcpa < 0:
        tcpa = np.inf  # If negative, no future approach

    # Compute DCPA
    if np.isinf(tcpa):
        # If TCPA is infinite, we compute DCPA based on the current state
        dcpa = np.linalg.norm(
            p1 - p2
        )  # Distance between the two objects at current time
    else:
        # If TCPA is valid, calculate the closest points at TCPA
        closest_p1 = p1 + v1 * tcpa
        closest_p2 = p2 + v2 * tcpa
        dcpa = np.linalg.norm(closest_p1 - closest_p2)

    return dcpa, tcpa
