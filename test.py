import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math as m
import pandas as pd

# 初始化摩擦系数和恢复系数的值列表
friction_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
mellowness_values = [0.1]


# // =============================================================================

def main():
    results = []  # 结果列表，用于存储所有仿真结果

    for friction in friction_values:
        for mellowness in mellowness_values:
            # 创建 HMMWV 车辆
            my_hmmwv = veh.HMMWV_Full()
            my_hmmwv.SetContactMethod(contact_method)
            my_hmmwv.SetChassisCollisionType(chassis_collision_type)
            my_hmmwv.SetChassisFixed(False)
            my_hmmwv.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
            my_hmmwv.SetEngineType(engine_model)
            my_hmmwv.SetTransmissionType(transmission_model)
            my_hmmwv.SetDriveType(drive_type)
            my_hmmwv.SetSteeringType(steering_type)
            my_hmmwv.SetTireType(tire_model)
            my_hmmwv.SetTireStepSize(tire_step_size)
            my_hmmwv.Initialize()

            my_hmmwv.SetChassisVisualizationType(chassis_vis_type)
            my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
            my_hmmwv.SetSteeringVisualizationType(steering_vis_type)
            my_hmmwv.SetWheelVisualizationType(wheel_vis_type)
            my_hmmwv.SetTireVisualizationType(tire_vis_type)

            # 创建地形
            terrain = veh.SCMTerrain(my_hmmwv.GetSystem())

            # 设置SCM地形参数，模拟土壤的松软度
            terrain.SetSoilParameters(1200,  # 土壤密度（单位：kg/m³）
                                      1.0e5,  # 土壤弹性模量（单位：Pa）
                                      0.01,  # 屈服强度（单位：Pa）
                                      1.0,  # 剪切强度（单位：Pa）
                                      5.0e3)  # 压实模量（单位：Pa）

            # 初始化SCM地形
            terrain.Initialize()

            # 创建车辆可视化接口
            vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
            vis.SetWindowTitle('HMMWV')
            vis.SetWindowSize(1280, 1024)
            vis.SetChaseCamera(trackPoint, 6.0, 0.5)
            vis.Initialize()
            vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
            vis.AddLightDirectional()
            vis.AddSkyBox()
            vis.AttachVehicle(my_hmmwv.GetVehicle())

            # 创建交互式驾驶系统
            driver = veh.ChInteractiveDriverIRR(vis)
            driver.SetSteeringDelta(render_step_size / 1.0)
            driver.SetThrottleDelta(render_step_size / 1.0)
            driver.SetBrakingDelta(render_step_size / 0.3)
            driver.Initialize()

            my_hmmwv.GetVehicle().EnableRealtime(True)

            while vis.Run():
                time = my_hmmwv.GetSystem().GetChTime()

                # 仿真结束时间
                if time >= t_end:
                    speed = my_hmmwv.GetVehicle().GetSpeed()
                    position = my_hmmwv.GetChassis().GetPos()

                    # 记录每次仿真的最终状态
                    results.append([time, speed, position.x, position.y, position.z, friction, mellowness])
                    break

                # 设置油门和方向
                driver_inputs = driver.GetInputs()
                driver_inputs.m_throttle = 1.0  # 全油门
                driver_inputs.m_steering = 0.0  # 保持直行

                # 更新各个模块
                driver.Synchronize(time)
                terrain.Synchronize(time)
                my_hmmwv.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                # 进步仿真
                driver.Advance(step_size)
                terrain.Advance(step_size)
                my_hmmwv.Advance(step_size)
                vis.Advance(step_size)

    # 将结果保存到Excel
    df = pd.DataFrame(results, columns=['Time', 'Speed', 'PosX', 'PosY', 'PosZ', 'Friction', 'Mellowness'])
    df.to_excel('vehicle_simulation_results.xlsx', index=False)
    print("仿真结果已保存到 'vehicle_simulation_results.xlsx' 文件中。")

    return 0


# 设置其他仿真参数
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
initLoc = chrono.ChVectorD(0, 0, 1.6)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
engine_model = veh.EngineModelType_SHAFTS
transmission_model = veh.TransmissionModelType_SHAFTS
drive_type = veh.DrivelineTypeWV_AWD
steering_type = veh.SteeringTypeWV_PITMAN_ARM
tire_model = veh.TireModelType_TMEASY
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)
contact_method = chrono.ChContactMethod_SMC
contact_vis = False
step_size = 3e-3
tire_step_size = 1e-3
t_end = 20
render_step_size = 1.0 / 50
out_dir = os.path.join(os.path.dirname(__file__), "HMMWV_demo")
debug_output = True
debug_step_size = 1.0 / 1
povray_output = False

# 运行主程序
main()
