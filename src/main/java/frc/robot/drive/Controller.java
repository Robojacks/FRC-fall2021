// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Controller extends SubsystemBase {

  XboxController controller;

  public Controller(XboxController xbox) {
    controller = xbox;
  }

  public void startRumble() {
      controller.setRumble(RumbleType.kLeftRumble, .7);
      controller.setRumble(RumbleType.kRightRumble, .7);
  }

  public void startRumbleCalm() {
    controller.setRumble(RumbleType.kLeftRumble, .4);
    controller.setRumble(RumbleType.kRightRumble, .4);
}

  public void stopRumble() {
    controller.setRumble(RumbleType.kLeftRumble, 0);
    controller.setRumble(RumbleType.kRightRumble, 0);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
