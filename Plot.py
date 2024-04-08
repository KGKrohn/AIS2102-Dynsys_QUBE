import pandas as pd
import matplotlib.pyplot as plt
import Simulation

# Define the CSV file and field names
output_dir = 'Gen_Data'
output_file = f'{output_dir}/log13.csv'
fieldnames = ["time", "motor_angle", "motor_setpoint", "pendulum_angle", "pendulum_setpoint", "rpm", "voltage",
              "current"]

# Read data using pandas
data = pd.read_csv(output_file)

# Select the columns for plotting
time_values = data['time']
rpm_values = data['rpm']
m_angle_values = data['motor_angle']
volt_values = data['voltage']

# Plotting
# plt.figure(figsize=(10, 5))
plt.grid()
plt.style.use('ggplot')

plt.plot(time_values, m_angle_values, label='physical', color='blue')
t, y = Simulation.get_sim_val()
t = t+5
plt.plot(t, y, label='simulation', color='green')
plt.xlabel('time')
plt.ylabel('rpm')
plt.xlim((4.8, 8))
plt.ylim((-100, 1500))
plt.legend()
plt.title('Simulation vs logdata')
plt.tight_layout()
plt.show()
