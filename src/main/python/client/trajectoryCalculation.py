#!/usr/bin/env python3

import ntcore
import time

def main() -> None:
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("shootingClientCommunications")
    distanceToTargetSub = table.getDoubleTopic("distanceToTarget").subscribe(0)
    necesarySpeedSubscriber = table.getDoubleTopic("noteVelocity").getEntry(0)
    flywheelSpeedsSuppliter = table.getDoubleTopic("flywheelSpeed").getEntry(0)
    inst.startClient4("Shooter Calculations")
    inst.setServerTeam(7540)
    inst.startDSClient()

    while True:
        time.sleep(0.02)
        
        print("polling")

        distanceToTarget = distanceToTargetSub.get()

        noteVelocity: float = calculateShooterPowerFromDistance(distanceToTarget)

        flywheelVelocity: float = calculateShooterPowerFromDistance(noteVelocity)

        necesarySpeedSubscriber.set(noteVelocity)
        flywheelSpeedsSuppliter.set(flywheelVelocity)
        



def calculateShooterPowerFromDistance(distance: float) -> float:
    return distance

def calculateFlywheelSpeedsFromShooterPower(noteVelocity: float) -> float:
    return noteVelocity

if __name__ == "__main__":
    main()
