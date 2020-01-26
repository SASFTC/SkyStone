package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Extension {

    /* Fields */
    private HardwareMap hardwareMap;



    /* Methods */
    /**
     * Constructor for the Drivetrain class. Every drivetrain implemented must extend this class.
     * @param hardwareMap: A refetrence to the hardwareMap.
     */
    public Extension(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Get method for the HardwareMap
     * @param deviceClass: The device's class
     * @param deviceName: The device's name.
     * @return Return the reference to the first device that matches th type and name specified.
     */
    public <T> T get(Class<T> deviceClass, String deviceName) {

        return this.hardwareMap.get(deviceClass, deviceName);
    }
}
