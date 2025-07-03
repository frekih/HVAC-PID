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
        self.previous_error = 100
        self.integral = 50

    def compute_safe(self, process_variable_safe, dt):
            # Calculate error
            error = self.setpoint - process_variable_safe
            
            # Proportional term
            P_safe = self.Kp * error
            
            # Integral term
            self.integral += error * dt
            I_safe = self.Ki * self.integral
            
            # Derivative term
            derivative = (error - self.previous_error) / dt
            D_safe = self.Kd * derivative
            
            output = P_safe + I_safe + D_safe
            
            self.previous_error = error
            
            return output

    def compute_in(self, process_variable_in, dt):
            # Calculate error
            error = self.setpoint - process_variable_in
            
            # Proportional term
            P_in = self.Kp * error
            
            # Integral term
            self.integral += error * dt
            I_in = self.Ki * self.integral
            
            # Derivative term
            derivative = (error - self.previous_error) / dt
            D_in = self.Kd * derivative
            
            output = P_in + I_in + D_in
            
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
setpoint_safe = 950                                                            # Desired airflow safety cabinet, min. airflow
setpoint_inf = 5

# %% Simulation time settings:

time = np.linspace(0, 12, 100)
dt = time[1] - time[0]                                                         # [s] time-step
t_start = 0                                                                    # [s] starting time
t_stop = 14                                                                    # [s] end time
N_sim = int((t_stop - t_start)/dt) + 1                                         # [-] number of time-steps
p0 = 20                                                                        # [Pa] reference pressure
V = 54                                                                         # [m3] volume of room
n_P = 0.7                                                                      # [-] pressure exponent

# %% Pressure constants

P_0 = 101.325
P_supply = 101.425
P_exhaust = 101.225
P_infiltration = 101.375
P_room = P_0 / V * (P_supply + P_infiltration - P_exhaust)

# %% Resistance parameters

C_supply = 0.5
C_exhaust = 0.5
C_infiltration = 0.7

# %% Air flow resistance vs. damper stroke parameters

X_supply = 0.5
X_exhaust = 0.5

# %% Preallocation of arrays for plotting:

t_array = np.zeros(N_sim)
y_in = np.zeros(N_sim)
y_out = np.zeros(N_sim)
y_safe = np.zeros(N_sim)
p_array = np.zeros(N_sim)
y_safe_ref = np.zeros(N_sim)

# %% Simulation parameters for airflow

process_variable_out = 850   
process_values_out = []

process_variable_safe = 200
process_values_safe = []

process_variable_in = process_variable_out + process_variable_safe
process_values_in = []

pressure_variable = 50
pressure_value = []

pid_in = PIDController(Kp=0.8, Ki=0.9, Kd=0.8, setpoint=setpoint_in)           # PID settings, supply air
pid_out = PIDController(Kp=0.5, Ki=0.9, Kd=0.9, setpoint=setpoint_out)        # PID settings, exhaust air
pid_safe = PIDController(Kp=0.8, Ki=0.7, Kd=0.75, setpoint=setpoint_safe)      # PID settings, safety cabinet

# %% Simulate the process

for t in range(0,N_sim):
    
    t_k = t * dt       
    
    if t_k <= 1:
    
        setpoint_safe = 200
    
    else:
        
        setpoint_safe = 950
        
    control_output_safe = pid_safe.compute_safe(process_variable_safe, dt)
    process_variable_safe += control_output_safe * dt - 0.01 * process_variable_safe * dt
    process_values_safe.append(process_variable_safe)

    control_output_out = pid_out.compute_out(process_variable_out, dt)
    process_variable_out += control_output_out * dt - 0.01 * process_variable_out * dt
    process_values_out.append(process_variable_out)
      
    control_output_in = pid_in.compute_in(process_variable_in, dt)
    process_variable_in += control_output_in * dt - 0.01 * process_variable_in * dt
    process_values_in.append(process_variable_in + process_variable_safe)
    
    P_supply = (setpoint_in / (C_supply * X_supply))**(n_P) + P_room
    P_exhaust = P_room - (setpoint_out + setpoint_safe / (C_exhaust * X_exhaust))**(n_P)
    P_infiltration = (setpoint_inf / (C_infiltration))**(n_P) + P_room
    
    pressure_output = P_room
    pressure_variable += pressure_output * dt - 0.9 * pressure_variable * dt
    pressure_value.append(pressure_variable)

    t_array[t] = t_k
    p_array[t] = pressure_value[t]
    y_in[t] = process_values_in[t]
    y_out[t] = process_values_out[t]
    y_safe[t] = process_values_safe[t]
    y_safe_ref[t] = setpoint_safe

# %% Plot results

fig, ax1 = plt.subplots(figsize = (20, 13))
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