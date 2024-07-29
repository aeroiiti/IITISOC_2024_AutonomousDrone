import gtsam
import numpy as np
import json

import json
import logging

def readJson(jsonFile):
    try:
        with open(jsonFile, 'r') as f:
            data = json.load(f)
        return data
    except IOError as e:
        logging.error(f"Error reading file {jsonFile}: {e}")
    except json.JSONDecodeError as e:
        logging.error(f"JSON decoding error in file {jsonFile}: {e}")
    return None


def main():
    sensorData = readJson("../DataSet/PreprocessedData/intel_gfs")

    graph = gtsam.NonlinearFactorGraph()
    initialEstimate = gtsam.Values()

    odomNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
    measurementNoise = gtsam.noiseModel.Isotropic.Sigma(2, 0.1)

    initialPose = gtsam.Pose2(0.0, 0.0, 0.0)
    initialEstimate.insert(0, initialPose)

    previousPose = initialPose
    count = 0
    for key in sorted(sensorData.keys()):
        count += 1
        currentReading = sensorData[key]
        
        # Ensure currentReading has 'x', 'y', 'theta' keys before accessing
        if 'x' in currentReading and 'y' in currentReading and 'theta' in currentReading:
            currentPose = gtsam.Pose2(currentReading['x'], currentReading['y'], currentReading['theta'])
            odometryFactor = gtsam.BetweenFactorPose2(count - 1, count, previousPose.between(currentPose), odomNoise)
            graph.add(odometryFactor)
            previousPose = currentPose
        else:
            print(f"Skipping invalid data at timestamp {key}. Missing keys in currentReading: {currentReading.keys()}")

    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate)
    result = optimizer.optimize()

    optimizedPoses = []
    for i in range(count):
        optimizedPose = result.atPose2(i)
        optimizedPoses.append((optimizedPose.x(), optimizedPose.y(), optimizedPose.theta()))

    print("Optimized Poses:", optimizedPoses)

if __name__ == '__main__':
    main()

