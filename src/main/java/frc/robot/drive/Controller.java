// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Controller extends SubsystemBase {
  /** Creates a new Controller. */

  XboxController xbox;
  
  public Controller(XboxController controller) {
    xbox = controller;
  }

  public void commenceRumble() {
    xbox.setRumble(RumbleType.kLeftRumble, .7);
    xbox.setRumble(RumbleType.kRightRumble, .7);
  }

  public void stopRumble() {
    xbox.setRumble(RumbleType.kLeftRumble, 0);
    xbox.setRumble(RumbleType.kRightRumble, .0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
