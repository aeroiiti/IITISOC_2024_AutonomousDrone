import json
import os
import numpy as np
import matplotlib.pyplot as plt

class EKFSLAM:
    @staticmethod
    def readJson(jsonFile):
        if not os.path.isfile(jsonFile):
            raise FileNotFoundError(f"File {jsonFile} not found.")
        sensor_data = {}
        with open(jsonFile, 'r') as f:
            data = json.load(f)
            sensor_data = data.get('map', {})
        
        return sensor_data

    def __init__(self, ogParameters, smParameters):
        self.ogParameters = ogParameters
        self.smParameters = smParameters
        
        # Initialize state (position and map)
        self.state = np.zeros((3 + 2 * ogParameters[0] * ogParameters[1], 1))
        self.covariance = np.eye(3 + 2 * ogParameters[0] * ogParameters[1])
        self.unitGridSize = ogParameters[3]
        self.lidarFOV = ogParameters[4]
        self.lidarMaxRange = ogParameters[5]
        self.numSamplesPerRev = ogParameters[6]
        self.wallThickness = ogParameters[7]
        
        # Additional parameters for EKF
        self.Q = np.diag([0.1, 0.1, np.deg2rad(5)]) ** 2  # Process noise
        self.R = np.diag([0.1, np.deg2rad(1)]) ** 2  # Measurement noise

    def predict(self, control):
        """
        Predict the next state based on control input (movement and rotation).
        """
        theta = self.state[2, 0]
        delta_trans = control[0]
        delta_rot = control[1]
        
        # State transition
        self.state[0, 0] += delta_trans * np.cos(theta)
        self.state[1, 0] += delta_trans * np.sin(theta)
        self.state[2, 0] += delta_rot
        
        # Jacobian of the motion model
        G = np.eye(len(self.state))
        G[0, 2] = -delta_trans * np.sin(theta)
        G[1, 2] = delta_trans * np.cos(theta)
        
        # Update the covariance
        self.covariance = G @ self.covariance @ G.T + self.Q
    
    def update(self, measurements):
        """
        Update the state based on measurements.
        """
        for measurement in measurements:
            z = measurement[:2]
            landmark_id = int(measurement[2])
            if landmark_id not in self.landmarks:
                # Initialize landmark position in the state
                self.state[3 + 2 * landmark_id: 3 + 2 * landmark_id + 2, 0] = self.state[:2, 0] + z
                self.landmarks.add(landmark_id)
            
            # Extract the predicted landmark position
            lx = self.state[3 + 2 * landmark_id, 0]
            ly = self.state[3 + 2 * landmark_id + 1, 0]
            
            # Measurement model
            dx = lx - self.state[0, 0]
            dy = ly - self.state[1, 0]
            q = dx**2 + dy**2
            z_pred = np.array([np.sqrt(q), np.arctan2(dy, dx) - self.state[2, 0]])
            
            # Jacobian of the measurement model
            H = np.zeros((2, len(self.state)))
            H[0, 0] = -dx / np.sqrt(q)
            H[0, 1] = -dy / np.sqrt(q)
            H[0, 2] = 0
            H[1, 0] = dy / q
            H[1, 1] = -dx / q
            H[1, 2] = -1
            H[0, 3 + 2 * landmark_id] = dx / np.sqrt(q)
            H[0, 3 + 2 * landmark_id + 1] = dy / np.sqrt(q)
            H[1, 3 + 2 * landmark_id] = -dy / q
            H[1, 3 + 2 * landmark_id + 1] = dx / q
            
            # Kalman gain
            S = H @ self.covariance @ H.T + self.R
            K = self.covariance @ H.T @ np.linalg.inv(S)
            
            # Update state
            self.state += K @ (z - z_pred)
            
            # Update covariance
            I = np.eye(len(self.state))
            self.covariance = (I - K @ H) @ self.covariance
    
    def processSensorData(self, sensorData):
        for key, data in sensorData.items():
            range_data = data['range']
            theta = data['theta']
            x = data['x']
            y = data['y']
            
            # Use the sensor data to create control and measurement vectors
            control = [x, theta]  # This is a simplified version
            measurements = [[r, theta, i] for i, r in enumerate(range_data)]
            
            # Predict step
            self.predict(control)
            
            # Update step
            self.update(measurements)
           
    def plotMap(self):
        """
        Plot the map with state estimates and landmarks.
        """
        plt.figure(figsize=(10, 8))
        
        # Plot state estimates
        plt.plot(self.state[0, 0], self.state[1, 0], 'bo', markersize=10, label='Robot Position')
        
        # Plot landmarks
        for landmark_id in self.landmarks:
            lx = self.state[3 + 2 * landmark_id, 0]
            ly = self.state[3 + 2 * landmark_id + 1, 0]
            plt.plot(lx, ly, 'rx', markersize=10, label=f'Landmark {landmark_id}')
        
        # Plot settings
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('EKF SLAM: Map and Landmarks')
        plt.legend()
        plt.grid(True)
        plt.show()

def main():
    initMapXLength = 50
    initMapYLength = 50
    unitGridSize = 0.02  # in Meters
    lidarFOV = 3.141592653589793  # np.pi
    lidarMaxRange = 10.0  # in Meters
    scanMatchSearchRadius = 1.4
    scanMatchSearchHalfRad = 0.25
    scanSigmaInNumGrid = 2
    wallThickness = 0.1  # 5 * unitGridSize
    moveRSigma = 0.1
    maxMoveDeviation = 0.25
    turnSigma = 0.3
    missMatchProbAtCoarse = 0.15
    coarseFactor = 5
    
    try:
        sensorData = EKFSLAM.readJson("/home/rohan/SLAM-2D-LIDAR-SCAN/DataSet/PreprocessedData/intel_gfs")
        print("sensorData:", sensorData)  # Print sensorData to debug
        
        # Ensure sensorData has at least one key
        if not sensorData:
            raise ValueError("sensorData is empty or not in expected format.")
        
        numSamplesPerRev = len(sensorData[list(sensorData.keys())[0]]['range'])  # Get how many points per revolution
        initXY = sensorData[sorted(sensorData.keys())[0]]
        ogParameters = [initMapXLength, initMapYLength, initXY, unitGridSize, lidarFOV, lidarMaxRange, numSamplesPerRev, wallThickness]
        smParameters = [scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid, moveRSigma, maxMoveDeviation, turnSigma, missMatchProbAtCoarse, coarseFactor]
        
        ekf_slam = EKFSLAM(ogParameters, smParameters)
        ekf_slam.processSensorData(sensorData)
        ekf_slam.plotMap()
    
    except Exception as e:
        print(e)
        return

if __name__ == '__main__':
    main()

