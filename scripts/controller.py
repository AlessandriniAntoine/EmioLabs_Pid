import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.sparse import data
from tf_pid_tools import auto_tune_app
import control as ct

####################################################################################
# Note: This is where you design the state feedback controller and compute G.
####################################################################################
def design_controller(tf):
    """
    Compute the PID controller gains.

    Parameters:
    - tf: Transfer function of the system

    Returns:
    - Kp: np.ndarray, shape (nu, nx), proportional gain
    - Ki: np.ndarray, shape (nu, nx), integral gain
    - Kd: np.ndarray, shape (nu, ny), derivative gain for reference tracking
    """

    bounds = [(1e-8, None), (0., None), (0., None)]
    integrator = ct.TransferFunction([1], [1])
    pid = auto_tune_app(integrator * tf, bounds=bounds)

    return pid.Kp, pid.Ki, pid.Kd

####################################################################################
# Note: The following code loads data, handles paths, and runs the controller logic.
####################################################################################

lab_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
data_path = os.path.join(lab_path, "data")

def get_parser_args():
    parser = argparse.ArgumentParser(description='Design a PID controller.')
    parser.add_argument('--nb_zeros', type=int, default=0,
        help="Number of zeros in the transfer function", dest="nb_zeros")
    parser.add_argument('--nb_poles', type=int, default=2,
        help="Number of poles in the transfer function", dest="nb_poles")
    try:
        args = parser.parse_args()
    except SystemExit:
        args = parser.parse_args([])
    return args

def perform_controller_design():
    args = get_parser_args()

    # Load identified model
    tf_path = os.path.join(data_path, f"tf_{args.nb_zeros}zeros_{args.nb_poles}poles.npz")
    if not os.path.exists(tf_path):
        raise FileNotFoundError(f"Model file not found: {tf_path}. Please run identification first.")
    tf_data = np.load(tf_path)
    num = tf_data["numerator"]
    den = tf_data["denominator"]
    tf = ct.TransferFunction(num, den)

    # Design controller
    Kp, Ki, Kd = design_controller(tf)

    # Save controller
    np.savez(os.path.join(data_path, f"pid_{args.nb_zeros}zeros_{args.nb_poles}poles.npz"),
            proportionalGain=Kp,
            integralGain=Ki,
            derivativeGain=Kd)

# Optional SOFA interface
def createScene(root):
    perform_controller_design()

# Entry point
if __name__ == "__main__":
    perform_controller_design()
