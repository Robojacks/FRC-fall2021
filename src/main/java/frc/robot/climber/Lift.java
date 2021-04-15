// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  private WPI_TalonSRX lift = new WPI_TalonSRX(kLiftPort);

  public void move(double speed) {
    lift.set(speed);
  }

  public void stop() {
    lift.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
