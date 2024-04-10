import pandas as pd
import matplotlib.pyplot as plt
import Simulation

# Define the CSV file and field names
output_dir = 'Gen_Data'
output_file = f'{output_dir}/Observer.csv'
output_file2 = f'{output_dir}/RPM_PID.csv'
fieldnames = ["time", "motor_angle", "motor_setpoint", "pendulum_angle", "pendulum_setpoint", "rpm", "voltage",
              "current"]

# Read data using pandas
data = pd.read_csv(output_file)
data2 = pd.read_csv(output_file2)
# Select the columns for plotting
time_values = data['time']
rpm_values = data['rpm']
m_angle_values = data['motor_angle']
volt_values = data['voltage']
time_values2 = data2['time']
rpm_values2 = data2['rpm']
m_angle_values2 = data2['motor_angle']
volt_values2 = data2['voltage']
# Plotting
# plt.figure(figsize=(10, 5))
plt.grid()
plt.style.use('ggplot')
t, y = Simulation.get_sim_math(1160)
t1, y1 = Simulation.get_sim_estim()
#plt.plot(t+5, y, label='Observer', color='purple')
#plt.plot(t1+5, y1, label='Sim_estim', color='red')
plt.plot(time_values, m_angle_values, label='Observer', color='blue')
plt.xlabel('time')
plt.ylabel('angle')
plt.xlim((4.9, 10))
plt.ylim((-10, 70))
plt.legend()
plt.title('sim vs step-respons')
plt.tight_layout()
plt.show()
