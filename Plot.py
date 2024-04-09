import pandas as pd
import matplotlib.pyplot as plt
import Simulation

# Define the CSV file and field names
output_dir = 'Gen_Data'
output_file = f'{output_dir}/Phys.csv'
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
t, y = Simulation.get_sim(1160)
plt.plot(t, y, label='Sim', color='red')
#plt.plot(time_values, rpm_values, label='Pysical', color='blue')
plt.xlabel('time')
plt.ylabel('angle')
plt.xlim((-0.1, 1.5))
plt.ylim((-10, 1750))
plt.legend()
plt.title('sim')
plt.tight_layout()
plt.show()
