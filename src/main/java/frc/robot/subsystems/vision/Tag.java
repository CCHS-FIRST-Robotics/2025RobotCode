package frc.robot.subsystems.vision;

public class Tag {
    long m_ID;
    double m_distance;

    //angle 0 deg = centered, -180 deg = left, 180 deg = right
    double m_angle;

    public Tag(long ID, double distance, double angle) {
        m_ID = ID;
        m_distance = distance;
        m_angle = angle;
    }

    public long getID() {
        return m_ID;
    }

    public double getDistance() {
        return m_distance;
    }

    public double getAngle() {
        return m_angle;
    }

    public double getAngleRad() {
        return m_angle * Math.PI/180;
    }

}
