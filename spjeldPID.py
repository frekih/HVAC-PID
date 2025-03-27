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

    # Initialize PID controller
setpoint = 1050  # Desired airflow
pid_out = PIDController(Kp=0.5, Ki=5.0, Kd=0.005, setpoint=setpoint)
pid_safe = PIDController(Kp=0.9, Ki=2.5, Kd=0.01, setpoint=setpoint)

# Simulation parameters for outflow
time = np.linspace(0, 10, 100)  # 10 seconds, 100 steps
dt = time[1] - time[0]
process_variable_out = 850  # Initial airflow
process_values_out = []

# Simulation parameters for safety cabinet
process_variable_safe = 200  # Initial airflow
process_values_safe = []

# Simulation parameters for total airflow
process_flow = 0
total_flow = []


# Simulate the process
for t in time:

    control_output_out = pid_out.compute(process_variable_out, dt)
    process_variable_out += control_output_out * dt - 0.001 * (process_variable_out - 10) * dt 
    process_values_out.append(process_variable_out)

    control_output_safe = pid_safe.compute(process_variable_safe, dt)
    process_variable_safe += control_output_safe * dt - 0.1 * (process_variable_safe - 10) * dt 
    process_values_safe.append(process_variable_safe)
     
    process_flow = np.add(process_variable_safe, process_variable_out)
    total_flow.append(process_flow)


# Plot results
plt.figure(figsize=(10, 6))
plt.plot(time, process_values_out, label='Process Variable, outflow (Airflow [m3/h])')
plt.plot(time, process_values_safe, label='Process Variable, safety cabinet (Airflow [m3/h])')
plt.plot(time, total_flow, label='Total flow [m3/h]')
plt.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
plt.xlabel('Time (s)')
plt.ylabel('Airflow [m3/h]')
plt.title('PID Controller Simulation')
plt.legend()
plt.grid()
plt.show()