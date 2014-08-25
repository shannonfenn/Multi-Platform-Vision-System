#ifndef ORIENTATIONUKF_H
#define ORIENTATIONUKF_H

#include "Tools/Math/depUKF.h"

class OrientationUKF : public depUKF
{
public:
    OrientationUKF();
    enum State
    {
        pitchAngle,
        pitchGyroOffset,
        rollAngle,
        rollGyroOffset,
        numStates
    };
    void initialise(double time, const std::vector<float>& gyroReadings, const std::vector<float>& accelerations, bool validkinematics, const std::vector<float> kinematicorientation);
    void TimeUpdate(const std::vector<float>& gyroReadings, double timestamp);
    void MeasurementUpdate(const std::vector<float>& accelerations, bool validKinematics, const std::vector<float>& kinematicsOrientation);
    bool Initialised(){return m_initialised;}

private:
    void AccelerationFromOrientation(const Matrix& orientation, Matrix& accelerations);
    void OrientationFromAcceleration(const std::vector<float>& accelerations, std::vector<float>& orientation);

private:
    static constexpr float g = 980.7;                //!< Standard gravity
    double m_timeOfLastUpdate;
    Matrix m_updateSigmaPoints;
    Matrix m_processNoise;
    bool m_initialised;
    int m_initialised_count;
    float m_scale;                               //!< the scalar to make the observations have a magnitude of g
    std::vector<float> m_offset;                 //!< the offset between the accelerometer and the gravity vector
};

#endif // ORIENTATIONUKF_H
