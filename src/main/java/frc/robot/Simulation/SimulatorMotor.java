// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Simulation;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SimulatorMotor {

    private double velocityPS = 0;
    private double encoder = 0;

    public SimulatorMotor(){}

    public void update(){
        encoder += velocityPS*(0.02);
    }

    public double getSelectedSensorPosition(){
        return encoder;
    }

    //PULSES/SECOND
    public void setVelocity(double velocityPS){
        this.velocityPS = velocityPS;
    }

}
