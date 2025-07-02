# %% Imports
import matplotlib.pyplot as plt
import numpy as np

# %% Class definition
class PIDController: 
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 45
        self.integral = 15

    def compute_safe(self, process_variable_safe, dt):
            # Calculate error
            error = self.setpoint - process_variable_safe
            
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

    def compute_in(self, process_variable_in, dt):
            # Calculate error
            error = self.setpoint - process_variable_in
            
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
        
    def compute_out(self, process_variable_out, dt):
            # Calculate error
            error = self.setpoint - process_variable_out
            
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
setpoint_in = 1050                                                             # Desired airflow in
setpoint_out = 850                                                             # Desired airflow out
setpoint_safe = 200                                                            # Desired airflow safety cabinet, min. airflow
setpoint_safe_high = 950                                                       # Desired airflow safety cabinet, max. airflow

# %% Simulation time settings:

dt = 0.2                                                                       # [s] time-step
t_start = 0                                                                    # [s] starting time
t_stop = 12                                                                    # [s] end time
N_sim = int((t_stop - t_start)/dt) + 1                                         # [-] number of time-steps
t0 = 2.0                                                                       # [s] Delay parameter
Tf = 2.0                                                                       # [s]
p0 = 20                                                                        # [Pa] reference pressure
V = 54                                                                         # [m3] volume of room
n_P = 0.5                                                                      # [-] pressure exponent

# %% Pressure constants
P_0 = 101.325
P_supply = 101.425
P_exhaust = 101.225
P_infiltration = 101.500
P_room = 101.300

# %% Resistance parameters
C_supply = 0.7
C_exhaust = 0.7
C_infiltration = 0.7

# %% Air flow resistance vs. damper stroke parameters
X_supply = 0.7
X_exhaust = 0.7

# %% Room pressure calculations
def room_P():

    P_dot_room = P_0 / V * (P_supply + P_infiltration - P_exhaust)
    
    return P_dot_room

# %% Preallocation of arrays for plotting:

t_array = np.zeros(N_sim)
y_in = np.zeros(N_sim)
y_out = np.zeros(N_sim)
y_safe = np.zeros(N_sim)
p_array = np.zeros(N_sim)
p_ref_array = np.zeros(N_sim)
y_safe_ref = np.zeros(N_sim)

# %% Preallocation of array for time-delay:

Nf = int(round(Tf/dt)) + 1
delay_array = np.zeros(Nf) + t0

# %% Simulation parameters for airflow
process_variable_out = 850                                                     # Initial exhaust airflow
process_values_out = []

process_variable_safe = 950                                                    # Initial safety cabinet airflow
process_values_safe = []

process_variable_in = process_variable_out + process_variable_safe             # Initial supply airflow
process_values_in = []

pressure_variable = 50
pressure_value = []

pid_in = PIDController(Kp=0.9, Ki=1.3, Kd=0.7, setpoint=setpoint_in)           # PID settings, supply air
pid_out = PIDController(Kp=0.9, Ki=1.2, Kd=0.75, setpoint=setpoint_out)        # PID settings, exhaust air
pid_safe = PIDController(Kp=0.5, Ki=2.0, Kd=0.5, setpoint=setpoint_safe)       # PID settings, safety cabinet

# %% Simulate the process
for t in range(0,N_sim):
    
    t_k = t * dt       

    # Moving array elements one step:
    # setpoint_safe_high = delay_array[-1]
    # delay_array[1:] = delay_array[0:-1]
    # delay_array[0] = setpoint_safe

    control_output_safe = pid_safe.compute_safe(process_variable_safe, dt)
    process_variable_safe += control_output_safe * dt - 0.01 * process_variable_safe * t_k
    process_values_safe.append(process_variable_safe)
    
    control_output_in = pid_in.compute_in(process_variable_in, dt)
    process_variable_in += control_output_in * dt - 0.01 * process_variable_in * t_k
    process_values_in.append(process_variable_in + process_variable_safe)

    control_output_out = pid_out.compute_out(process_variable_out, dt)
    process_variable_out += control_output_out * dt - 0.01 * process_variable_out * t_k
    process_values_out.append(process_variable_out + process_variable_safe)
        
    P_supply = (setpoint_in / (C_supply * X_supply))**(1/n_P) + P_room
    P_exhaust = P_room - (setpoint_out + setpoint_safe / (C_exhaust * X_exhaust))**(1/n_P)
    P_infiltration = (setpoint_safe / (C_infiltration))**(1/n_P) + P_room
   
    P_room = P_0 / V * (P_supply + P_infiltration - P_exhaust)
    
    pressure_output = P_0 / V * (P_supply + P_infiltration - P_exhaust)
    pressure_variable += pressure_output * dt - 0.5 * pressure_variable * t_k
    pressure_value.append(pressure_variable)

    t_array[t] = t_k
    p_array[t] = pressure_value[t]
    y_in[t] = process_values_in[t]
    y_out[t] = process_values_out[t]
    y_safe[t] = process_values_safe[t]
    y_safe_ref[t] = setpoint_safe

# %% Plot results
fig, ax1 = plt.subplots(figsize = (16, 10))
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Airflow [m3/h]', color='blue')

plot_1 = ax1.plot(t_array, y_out, color='gold', label='Process Variable, extract (Airflow [m3/h])')
plot_2 = ax1.plot(t_array, y_in, color='deepskyblue', label='Process Variable, inflow (Airflow [m3/h])')
plot_3 = ax1.plot(t_array, y_safe, color='salmon', label='Process Variable, safety cabinet (Airflow [m3/h])')
ax1.tick_params(axis ='y', labelcolor = 'blue')

plot_4 = ax1.plot(t_array, y_safe_ref, color='m', linestyle='--', label='Setpoint safety cabinet [m3/h]')

ax2 = ax1.twinx()
ax2.set_ylabel('Pressure [Pa]', color='green')
plot_5 = ax2.hlines(y=p0, xmin=t_start, xmax=t_stop, color='seagreen', label='Reference pressure [Pa]')
plot_6 = ax2.plot(t_array, p_array, color='darkgreen', label='Room pressure [Pa]')
ax2.tick_params(axis ='y', labelcolor = 'green')

fig.legend()
plt.title('PID Controller Simulation')
plt.grid()
plt.show()