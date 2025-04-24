import matplotlib.pyplot as plt
import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 5
        self.integral = 0

    def compute(self, process_variable, dt):
            # Calculate error
            error = self.setpoint - process_variable
            
            # Proportional term
            P_out = self.Kp * error
            
            # Integral term
            self.integral += error * dt
            I_out = self.Ki * self.integral
            
            # Derivative term
            derivative = (error - self.previous_error) / dt
            D_out = self.Kd * derivative
            
            output = P_out + I_out + D_out
            
            self.previous_error = error
            
            return output

# %% Initialize PID controller
setpoint_in = 1050  # Desired airflow in
setpoint_out = 900 # Desired airflow out
setpoint_safe = 800 # Desired airflow safety cabinet

# %% Simulation time settings:

dt = 1.0  # [s]
t_start = 0  # [s]
t_stop = 25  # [s]
N_sim = int((t_stop - t_start)/dt) + 1  # Number of time-steps
t0 = 5.0 # [s] Delay parameter
Tf = 2.0 # [s]

# %% Preallocation of arrays for plotting:

t_array = np.zeros(N_sim)
y_in = np.zeros(N_sim)
y_out = np.zeros(N_sim)
y_safe = np.zeros(N_sim)
p_array = np.zeros(N_sim)
p_ref_array = np.zeros(N_sim)

# %% Preallocation of array for time-delay:

Nf = int(round(Tf/dt)) + 1
delay_array = np.zeros(Nf) + t0

# %% Simulation parameters for airflow
process_variable_out = 850  # Initial extraction airflow
process_values_out = []

process_variable_safe = 200  # Initial safety cabinet airflow
process_values_safe = []

process_variable_in = process_variable_out + process_variable_safe  # Initial intake airflow
process_values_in = []

pid_in = PIDController(Kp=0.01, Ki=1.5, Kd=0.008, setpoint=setpoint_in)
pid_out = PIDController(Kp=0.3, Ki=5.0, Kd=0.005, setpoint=setpoint_out)
pid_safe = PIDController(Kp=0.5, Ki=1.5, Kd=0.1, setpoint=setpoint_safe)

# %% Simulate the process
for t in range(0,N_sim):
    
    t_k = t * dt
    
    # Selecting inputs:
    if (t_k >= t_start and t_k < t0):
        y_safe = 200
    elif (t_k >= t0):
        y_safe = 1050

    # # Moving array elements one step:
    # y_in = delay_array[-1]
    # delay_array[1:] = delay_array[0:-1]
    # delay_array[0] = y_safe
    
        control_output_in = pid_in.compute(process_variable_in, dt)
        # process_variable_in += control_output_in * dt - 0.01 * (process_variable_in - 10) * dt
        process_values_in.append(process_variable_in)

        control_output_out = pid_out.compute(process_variable_out, dt)
        # process_variable_out += control_output_out * dt - 0.01 * (process_variable_out - 10) * dt
        process_values_out.append(process_variable_out)

        control_output_safe = pid_safe.compute(process_variable_safe, dt)
        # process_variable_safe += control_output_safe * dt - 0.1 * (process_variable_safe - 10) * dt
        process_values_safe.append(process_variable_safe)
    
    t_array[t] = t_k
    # y_in[t] = process_values_in
    # y_out[t] = process_values_out
    # y_safe[t] = process_values_safe

# %% Plot results
plt.figure(figsize=(16, 10))
plt.plot(t_array, process_values_out, label='Process Variable, outflow (Airflow [m3/h])')
plt.plot(t_array, process_values_in, label='Process Variable, inflow (Airflow [m3/h])')
plt.plot(t_array, process_values_safe, label='Process Variable, safety cabinet (Airflow [m3/h])')
plt.axhline(y=setpoint_safe, color='r', linestyle='--', label='Setpoint safety cabinet')
plt.axhline(y=setpoint_in, color='b', linestyle='--', label='Setpoint intake')
plt.axhline(y=setpoint_out, color='y', linestyle='--', label='Setpoint extract')
plt.xlabel('Time (s)')
plt.ylabel('Airflow [m3/h]')
plt.title('PID Controller Simulation')
plt.legend()
plt.grid()
plt.show()