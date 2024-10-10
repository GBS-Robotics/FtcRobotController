package org.firstinspires.ftc.teamcode;

import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class EulerAngle
{
    public float w, x, y, z;
    public float roll, pitch, yaw;

    public EulerAngle(Quaternion q)
    {
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        roll = (float) (atan2(sinr_cosp, cosr_cosp));

        // pitch (y-axis rotation)
        double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
        double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
        pitch = (float) (2 * atan2(sinp, cosp) - Math.PI / 2);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        yaw = (float) (atan2(siny_cosp, cosy_cosp));
    }

    public String toString()
    {
        return "PRY: " + pitch + " - " + yaw + " - " + roll;
    }
}