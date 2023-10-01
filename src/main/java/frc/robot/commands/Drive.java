// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  XboxController controller;
  Chassis chassis;
  public Drive(XboxController controller, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.chassis = chassis;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vx = controller.getLeftY();
    double vOmega = controller.getLeftX();
    ChassisSpeeds speeds = new ChassisSpeeds(vx, 0, vOmega);
    chassis.setVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
