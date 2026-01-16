import os
import numpy as np
import control as ct
import matplotlib.pyplot as plt
from tf_pid_tools import auto_estimate_app


####################################################################################
# Note: This is where you implement the identification of the reduced linear model.
####################################################################################
def identify_function(inputs, outputs, dt):
    """
    Identify the transfert function from data.

    Parameters:
    - inputs: np.ndarray, shape (nu, L), inputs over time
    - outputs: np.ndarray, shape (ny, L), outputs over time

    Returns:
    - tf: ct.TransferFunction, identified transfer function of the system.
    """

    tf = auto_estimate_app(inputs, outputs, dt)

    plt.show()
    return tf

####################################################################################
# Note: The following code is provided and loads data, handles paths, and runs logic.
####################################################################################

lab_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
data_path = os.path.join(lab_path, "data")

def perform_identification():

    # Load recorded data
    openloop_path = os.path.join(data_path, "openLoop.npz")
    if not os.path.exists(openloop_path):
        raise FileNotFoundError(f"Data file not found at: {openloop_path}. Please run the open-loop simulation first.")
    data = np.load(openloop_path)
    outputs = data["markersPos"].T
    inputs = data["motorPos"].T
    dt = 1 / data["fps"]

    inputs = inputs[:, :-1]
    outputs = outputs[1, 1:]

    # Identify the model
    tf = identify_function(inputs, outputs, dt)
    nb_zeros = len(tf.num[0][0]) - 1
    nb_poles = len(tf.den[0][0]) - 1

    # Save model
    np.savez(os.path.join(data_path, f"tf_{nb_zeros}zeros_{nb_poles}poles.npz"),
             numerator=tf.num[0][0],
             denominator=tf.den[0][0],
             )

# Optional SOFA interface
def createScene(root):
    perform_identification()

# Entry point
if __name__ == "__main__":
    perform_identification()
