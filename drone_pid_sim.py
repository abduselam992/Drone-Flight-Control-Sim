import matplotlib.pyplot as plt
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional: "How far are we from target?"
        self.ki = ki  # Integral: "How much error has built up?"
        self.kd = kd  # Derivative: "How fast are we moving toward target?"
        self.prev_error = 0
        self.integral = 0

    def calculate(self, setpoint, current_value, dt):
        error = setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output

# --- SIMULATION ---
def run_flight_simulation():
    pid = PIDController(kp=1.5, ki=0.1, kd=0.5)
    target_altitude = 10.0  # 10 Meters
    current_altitude = 0.0  # Start on ground
    
    # Physics constants
    gravity = -9.8
    mass = 1.0 # 1kg drone
    dt = 0.1 # 100ms time steps
    velocity = 0
    
    altitudes = []
    times = []

    print(f"Takeoff! Target: {target_altitude}m")
    
    for t in range(100):
        # 1. Calculate Motor Power needed from PID
        thrust = pid.calculate(target_altitude, current_altitude, dt)
        
        # 2. Apply Physics (Acceleration = Force/Mass)
        acceleration = (thrust + (gravity * mass)) / mass
        velocity += acceleration * dt
        current_altitude += velocity * dt
        
        # Keep drone above ground
        if current_altitude < 0:
            current_altitude = 0
            velocity = 0
            
        altitudes.append(current_altitude)
        times.append(t * dt)

    # Plot the results
    plt.plot(times, altitudes, label='Drone Altitude')
    plt.axhline(y=target_altitude, color='r', linestyle='--', label='Target')
    plt.title("Drone Altitude PID Control Simulation")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    run_flight_simulation()
