# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Demonstration of vehicle over SCM deformable terrain
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left. All units SI.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m


# =============================================================================

class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay):
        veh.ChDriver.__init__(self, vehicle)
        self.delay = delay

    def Synchronize(self, time):
        eff_time = time - self.delay
        if (eff_time < 0):
            return

        if (eff_time > 0.2):
            self.SetThrottle(0.7)
        else:
            self.SetThrottle(3.5 * eff_time)

        if (eff_time < 2):
            self.SetSteering(0.0)
        else:
            self.SetSteering(0.6 * m.sin(2.0 * m.pi * (eff_time - 2) / 6))

        self.SetBraking(0.0)


def main():
    # print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the HMMWV vehicle, set parameters, and initialize
    # my_hmmwv = veh.HMMWV_Full()
    # my_hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    # my_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(-5, -2, 0.6), chrono.ChQuaternionD(1, 0, 0, 0)))
    # my_hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    # my_hmmwv.SetTransmissionType(veh.TransmissionModelType_SHAFTS)
    # my_hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    # my_hmmwv.SetTireType(veh.TireModelType_RIGID)
    # my_hmmwv.Initialize()
    #
    # my_hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    # my_hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    # my_hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    # my_hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
    # my_hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    my_hmmwv = veh.HMMWV_Full()
    my_hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    my_hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    my_hmmwv.SetChassisFixed(False)
    my_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(-5, -2, 0.6), chrono.ChQuaternionD(1, 0, 0, 0)))
    my_hmmwv.SetEngineType( veh.EngineModelType_SHAFTS)
    my_hmmwv.SetTransmissionType(veh.TransmissionModelType_SHAFTS)
    my_hmmwv.SetDriveType( veh.DrivelineTypeWV_AWD)
    my_hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    my_hmmwv.SetTireType(veh.TireModelType_TMEASY)
    my_hmmwv.SetTireStepSize(tire_step_size)
    my_hmmwv.Initialize()

    my_hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create the (custom) driver
    driver = MyDriver(my_hmmwv.GetVehicle(), 0.5)
    driver.Initialize()

    soil_conditions = [
        {"Kphi": 1.2e6, "Kc": 0.1e3, "n": 1.1, "cohesion": 0.05, "friction": 25.0, "shear": 0.01},  # 高含水量
        {"Kphi": 1.5e6, "Kc": 0.2e3, "n": 1.3, "cohesion": 0.02, "friction": 30.0, "shear": 0.01},  # 低含水量
        {"Kphi": 1.8e6, "Kc": 0.3e3, "n": 1.2, "cohesion": 0.03, "friction": 35.0, "shear": 0.01},  # 高含沙量
        {"Kphi": 1.4e6, "Kc": 0.15e3, "n": 1.0, "cohesion": 0.04, "friction": 28.0, "shear": 0.01}  # 低含沙量
    ]
    results = []
    # 设置 SMC 地形
    for condition in soil_conditions:
        # 设置 SMC 地形
        terrain = veh.SCMTerrain(my_hmmwv.GetSystem())
        terrain.SetSoilParameters(condition["Kphi"], condition["Kc"], condition["n"],
                                  condition["cohesion"], condition["friction"],
                                  condition["shear"], elastic_K=4e4, damping_R=3e3)

        # Create the SCM deformable terrain patch
        terrain = veh.SCMTerrain(my_hmmwv.GetSystem())

        # Optionally, enable moving patch feature (single patch around vehicle chassis)
        terrain.AddMovingPatch(my_hmmwv.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))

        # Set plot type for SCM (false color plotting)
        terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1);

        # Initialize the SCM terrain, specifying the initial mesh grid
        terrain.Initialize(terrainLength, terrainWidth, delta);

        # Create the vehicle Irrlicht interface
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle('HMMWV Deformable Soil Demo')
        vis.SetWindowSize(1280, 1024)
        vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
        vis.AddLightDirectional()
        vis.AddSkyBox()
        vis.AttachVehicle(my_hmmwv.GetVehicle())

        speeds = []
        # Simulation loop
        while vis.Run():
            time = my_hmmwv.GetSystem().GetChTime()

            # End simulation
            if (time >= 2):
                # 记录当前速度
                speed = my_hmmwv.GetVehicle().GetSpeed()
                speeds.append(speed)

                break

            # Draw scene
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            # Get driver inputs
            driver_inputs = driver.GetInputs()

            # Update modules (process inputs from other modules)
            driver.Synchronize(time)
            terrain.Synchronize(time)
            my_hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            # Advance simulation for one timestep for all modules
            driver.Advance(step_size)
            terrain.Advance(step_size)
            my_hmmwv.Advance(step_size)
            vis.Advance(step_size)
        # 计算平均速度并保存结果
        avg_speed = sum(speeds) / len(speeds)
        print("结果记录")
        results.append({"condition": condition, "avg_speed": avg_speed})
        print(results)

    # 输出结果
    for result in results:
        print(f"土壤条件: {result['condition']} 平均速度: {result['avg_speed']} m/s")

    return 0
# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# SCM patch dimensions
terrainHeight = 0
terrainLength = 50.0  # size in X direction
terrainWidth = 80.0  # size in Y direction

# SCM grid spacing
delta = 0.05

# Simulation step sizes
# step_size = 2e-3;
step_size = 1e-3;
tire_step_size = 1e-3;

main()
