import setup_path 
import airsim

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# add new vehicle
vehicle_name = "Drone2"
client.simAddVehicle(vehicle_name, "simpleflight", "", 0, 0, 0)
client.enableApiControl(True, vehicle_name)
client.armDisarm(True, vehicle_name)
client.takeoffAsync(10.0, vehicle_name)
