/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.robot

import org.ghrobotics.lib.wrappers.FalconRobotBase
import org.ghrobotics.robot.auto.Autonomous
import org.ghrobotics.robot.sensors.AHRS
import org.ghrobotics.robot.sensors.AHRSSensorType
import org.ghrobotics.robot.sensors.ahrsSensorType
import org.ghrobotics.robot.subsytems.drive.DriveSubsystem


/**
 * Main robot class.
 */
class Robot : FalconRobotBase() {

    /**
     * Initialize all systems.
     */
    override suspend fun initialize() {
        // Set AHRS type
        ahrsSensorType = AHRSSensorType.Pigeon

        // Add controllers
        +Controls.mainXbox

        // Add subsystems
        +DriveSubsystem

        // Misc singleton initialization
        Network
        Autonomous
        AHRS
    }
}