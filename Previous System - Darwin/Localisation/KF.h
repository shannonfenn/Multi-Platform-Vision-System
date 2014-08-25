#ifndef _KF_h_DEFINED
#define _KF_h_DEFINED

#define PLAYING_STATE_RESETTING 1

#include <math.h>
#include "Tools/Math/Matrix.h"
#include "odometryMotionModel.h"
#include <string>
enum KfUpdateResult
{
    KF_OUTLIER = 0,
    KF_OK = 1,
};

using namespace std;

class KF {
    public:
  	    // Constructor
        KF();
    
        enum State
        {
            selfX,
            selfY,
            selfTheta,
            ballX,
            ballY,
            ballXVelocity,
            ballYVelocity,
            numStates
        };

        // Functions

        // Update functions
        void timeUpdate(double deltaTime);
        void timeUpdate(float odom_x, float odom_y, float odom_theta, double deltaTime);
        KfUpdateResult odometeryUpdate(double odom_X, double odom_Y, double odom_Theta, double R_X, double R_Y, double R_Theta);
        KfUpdateResult ballmeas(double Ballmeas, double theta_Ballmeas);
        KfUpdateResult fieldObjectmeas(double distance, double bearing,double objX,double objY, double distanceErrorOffset, double distanceErrorRelative, double bearingError);
        KfUpdateResult MultiFieldObs(const Matrix& locations, const Matrix& measurements, const Matrix& R_Measurement);
        void linear2MeasurementUpdate(double Y1,double Y2, double SR11, double SR12, double SR22, int index1, int index2);
        KfUpdateResult updateAngleBetween(double angle, double x1, double y1, double x2, double y2, double sd_angle);
        static unsigned int GenerateId();
        void setAlpha(double new_alpha);

        // Data retrieval
        double sd(int Xi) const;
        double variance(int Xi) const;
        double state(int stateID) const;
        Matrix GetBallSR() const;
        double getDistanceToPosition(double posX, double posY) const;
        double getBearingToPosition(double posX, double posY) const;
        double alpha() const;
        bool active() const;
        void setActive(bool active=true);
        unsigned int id() const;
        unsigned int parentId() const;
        unsigned int spawnFromModel(const KF& parent);

        // Utility
        void init();
        void Reset();
        bool clipState(int stateIndex, double minValue, double maxValue);
        bool isVarianceOutOfBounds();

        /*!
        @brief Output streaming operation.
        @param output The output stream.
        @param p_kf The source kalman filter to be streamed.
        */
        friend std::ostream& operator<< (std::ostream& output, const KF& p_kf);

        /*!
        @brief Input streaming operation.
        @param input The input stream.
        @param p_kf The destination kalman filter to be streamed to.
        */
        friend std::istream& operator>> (std::istream& input, KF& p_kf);

        Matrix CalculateSigmaPoints() const;
        float CalculateAlphaWeighting(const Matrix& innovation, const Matrix& innovationVariance, float outlierLikelyhood) const;
        // Variables

        // Multiple Models - Model state Description.
        bool m_toBeActivated;

        Matrix updateUncertainties; // Update Uncertainty. (A matrix)
        Matrix stateEstimates; // State estimates. (Xhat Matrix)
        Matrix stateStandardDeviations; // Standard Deviation Matrix. (S Matrix)

        int nStates; // Number of states. (Constant)
        Matrix sqrtOfTestWeightings; // Square root of W (Constant)
        Matrix sqrtOfProcessNoise; // Square root of Process Noise (Q matrix). (Constant)
        Matrix sqrtOfProcessNoiseReset; // Square root of Q when resetting. (Conastant) 
	
	// Motion Model
	OdometryMotionModel odom_Model;
        // Tuning Values (Constants) -- Values assigned in KF.cpp
        static const float c_Kappa;
        static const float c_ballDecayRate;
        static const float c_threshold2;

        // Ball distance measurement error weightings (Constant) -- Values assigned in KF.cpp
        static const float c_R_ball_theta;
        static const float c_R_ball_range_offset;
        static const float c_R_ball_range_relative;

        static const float c_outlierLikelyhood;
	
	void performFiltering(double odometeryForward, double odometeryLeft, double odometeryTurn);
        std::string summary(bool brief=true) const;

private:
        double m_alpha;
        bool m_isActive;
        unsigned int m_id;
        unsigned int m_parentId;
        double m_creationTime;
};

#endif
