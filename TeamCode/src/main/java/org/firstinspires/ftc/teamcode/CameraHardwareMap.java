package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;

public class CameraHardwareMap {
    public Camera webcam        = null;

    HardwareMap hwMap           = null;
    private ElapsedTime period  = new ElapsedTime();

    public CameraHardwareMap() { }

    public void init(HardwareMap hwMap) {
        hwMap = this.hwMap;

        webcam =hwMap.get(Camera.class, "webcam");
    }

}
