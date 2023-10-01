// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Simulation;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SimulationGyro {
    private double velocityDS = 0;
    private double angle = 0;

    public SimulationGyro(){

    }

    public void update(){
        angle += velocityDS*(0.02);
    }

    public double getSelectedSensorPosition(){
        return angle;
    }

    //Degrees/SECOND
    public void setVelocity(double velocityDS){
        this.velocityDS = velocityDS;
    }
}
