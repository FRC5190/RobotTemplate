/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.robot.subsytems.drive

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kX
import org.ghrobotics.robot.Controls

/**
 * Command for manually operating the drivetrain
 */
class ManualDriveCommand : FalconCommand(DriveSubsystem) {

    companion object {
        // Get sources
        private const val kDeadband = 0.02
        private val speedSource = Controls.mainXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband)
        private val rotationSource = Controls.mainXbox.getX(GenericHID.Hand.kLeft).withDeadband(kDeadband)
        private val quickTurnSource = Controls.mainXbox.getRawButton(kX)
    }

    /**
     * Runs periodically and sets motor outputs.
     */
    override suspend fun execute() {
        DriveSubsystem.curvatureDrive(
            -speedSource(),
            rotationSource(),
            quickTurnSource()
        )
    }
}
