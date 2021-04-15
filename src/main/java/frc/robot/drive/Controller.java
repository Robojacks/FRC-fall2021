package frc.robot.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import jdk.javadoc.internal.doclets.toolkit.resources.doclets;

public class Controller extends SubsystemBase {
    XboxController xbox;
    
    public Controller(XboxController controller){
        xbox = controller;
    }

public void startRumble() {

    xbox.setRumble(RumbleType.kRightRumble, 0.7);
    xbox.setRumble(RumbleType.kLeftRumble, 0.7);
}

public void stopRumble() {

    xbox.setRumble(RumbleType.kRightRumble, 0);
    xbox.setRumble(RumbleType.kLeftRumble, 0);
}

@Override
public void periodic(){
}

}