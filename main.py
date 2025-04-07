#!/usr/bin/env python3

from frankapy import FrankaArm
import pour_beads
from traj_opt import solve_bead_pour

import numpy as np
from weighing_scale import WeighingScale
import utils

def main():
    fa = FrankaArm()
    scale = WeighingScale()

    target_weight, current_weight = utils.get_weights(scale)
    X, U, success, t_vec, dt = solve_bead_pour(start_mass=current_weight, end_mass=target_weight, verbose=True)

    if not success: raise Exception("No Feasible trajectory found")

    # TODO: pickup cup and bring to pre-pour position

    utils.pour_beads(fa, Xm=X, Um=U, t_vec=t_vec, scale=scale, at_pre_pour=False, dt=dt, verbose=False)

    final_weight = scale.weight_averaged()
    print(f"Error = {target_weight - final_weight} g")

if __name__ == "__main__":
    main()