/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.robot

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.GlobalScope
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryFollower
import org.ghrobotics.lib.utils.launchFrequency
import org.ghrobotics.lib.wrappers.FalconRobotBase
import org.ghrobotics.lib.wrappers.networktables.FalconNetworkTable
import org.ghrobotics.lib.wrappers.networktables.get
import org.ghrobotics.robot.auto.AutoMode
import org.ghrobotics.robot.auto.StartingPositions
import org.ghrobotics.robot.subsytems.drive.DriveSubsystem

/**
 * Manages network communications
 */
object Network {

    // LiveDashboard NT instance
    val INSTANCE = FalconNetworkTable.getTable("Live Dashboard")

    // Auto sendable choosers
    val startingPositionChooser = SendableChooser<StartingPositions>()
    val autoModeChooser = SendableChooser<AutoMode>()

    // Network table entries
    private val robotX = INSTANCE["Robot X"]
    private val robotY = INSTANCE["Robot Y"]
    private val robotHdg = INSTANCE["Robot Heading"]

    private val pathX = INSTANCE["Path X"]
    private val pathY = INSTANCE["Path Y"]
    private val pathHdg = INSTANCE["Path Heading"]

    private val isEnabled = INSTANCE["Is Enabled"]

    init {
        // Initialize sendable choosers
        AutoMode.values().forEach {
            autoModeChooser.addDefault(it.name.toLowerCase().capitalize(), it)
        }
        StartingPositions.values().forEach {
            startingPositionChooser.addDefault(it.name.toLowerCase().capitalize(), it)
        }

        // Put choosers on dashboard
        SmartDashboard.putData("Starting Position", startingPositionChooser)
        SmartDashboard.putData("Auto Mode", autoModeChooser)

        // Launch periodic thread
        launchPeriodicNetworkCoroutine()
    }

    /**
     * Periodic thread that sends data to dashboards.
     */
    private fun launchPeriodicNetworkCoroutine() {
        GlobalScope.launchFrequency(50) {
            val robotPosition = DriveSubsystem.localization.robotPosition

            val x = robotPosition.translation.x.feet
            val y = robotPosition.translation.y.feet
            val a = robotPosition.rotation.radian

            // LiveDashboard plot data for robot
            robotX.setDouble(x)
            robotY.setDouble(y)
            robotHdg.setDouble(a)

            val trajectoryFollower: TrajectoryFollower = DriveSubsystem.trajectoryFollower

            // LiveDashboard plot data for paths
            pathX.setDouble(trajectoryFollower.referencePose.translation.x.feet)
            pathY.setDouble(trajectoryFollower.referencePose.translation.y.feet)
            pathHdg.setDouble(trajectoryFollower.referencePose.rotation.degree)

            isEnabled.setString(if (FalconRobotBase.INSTANCE.isEnabled) "Enabled" else "Disabled")

            // Shuffleboard data
            SmartDashboard.putNumber("Robot X", x)
            SmartDashboard.putNumber("Robot Y", y)
            SmartDashboard.putNumber("Robot Angle", robotPosition.rotation.degree)
        }
    }
}