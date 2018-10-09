package org.ghrobotics.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

    // Motor IDs
    private final int kLeftMasterId = 1;
    private final int kLeftSlaveId = 2;
    private final int kRightMasterId = 3;
    private final int kRightSlaveId = 4;

    // Xbox Controller Port
    private final int kXboxControllerPort = 0;

    // Motor Controllers
    private final TalonSRX leftMaster = new TalonSRX(kLeftMasterId);
    private final TalonSRX leftSlave = new TalonSRX(kLeftSlaveId);
    private final TalonSRX rightMaster = new TalonSRX(kRightMasterId);
    private final TalonSRX rightSlave = new TalonSRX(kRightSlaveId);

    // Xbox Controller
    private final XboxController controller = new XboxController(kXboxControllerPort);

    @Override
    public void robotInit() {
        // Followers
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        // Motor Inversion
        leftMaster.setInverted(false);
        leftSlave.setInverted(false);
        rightMaster.setInverted(true);
        rightSlave.setInverted(true);
    }

    @Override
    public void autonomousPeriodic() {
        // Add your autonomous code here
    }

    @Override
    public void teleopPeriodic() {
        // Control robot using two sticks on Xbox controller
        drive(-controller.getY(GenericHID.Hand.kLeft), -controller.getY(GenericHID.Hand.kRight));
    }

    /**
     * Sends an output to the drive motors
     * @param left The output to the left side of the drivetrain. Should be a value between -1 and 1
     * @param right The output to the right side of the drivetrain. Should be a value between -1 and 1
     */
    private void drive(double left, double right) {
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }
}
