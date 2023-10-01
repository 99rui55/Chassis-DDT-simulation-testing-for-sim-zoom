// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Simulation.SimulationGyro;
import frc.robot.Simulation.SimulatorMotor;

public class Chassis extends SubsystemBase {
  DifferentialDriveOdometry odometry;
  /** Creates a new Chassis. */
  private DifferentialDriveWheelSpeeds wheelSpeeds;
  private SimulatorMotor m1;
  private SimulatorMotor m2;
  private SimulationGyro gyro;
  private Field2d field;

  public Chassis() {
    m1 = new SimulatorMotor();
    m2 = new SimulatorMotor();
    gyro = new SimulationGyro();
    field = new Field2d();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);
    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(gyro.getSelectedSensorPosition()),
     m1.getSelectedSensorPosition()/Constants.PULSES_PER_METER,
     m2.getSelectedSensorPosition()/Constants.PULSES_PER_METER);
     field.setRobotPose(odometry.getPoseMeters());
  }

  public void setVelocity(ChassisSpeeds chassisSpeeds){
    wheelSpeeds = Constants.KINEMATICS.toWheelSpeeds(chassisSpeeds);
    m1.setVelocity(wheelSpeeds.leftMetersPerSecond*Constants.PULSES_PER_METER);
    m2.setVelocity(wheelSpeeds.rightMetersPerSecond*Constants.PULSES_PER_METER);
    gyro.setVelocity(Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
  }
  
  @Override
  public void simulationPeriodic(){
    m1.update();
    m2.update();
    gyro.update();
  }
  
}
