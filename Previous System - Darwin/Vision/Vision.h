/*!
  @file Vision.h
  @brief Declaration of NUbots Vision class.
*/

#ifndef VISION_H
#define VISION_H

#include "Infrastructure/NUImage/ClassifiedImage.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Kinematics/Horizon.h"
#include "ClassifiedSection.h"
#include "ScanLine.h"
#include "TransitionSegment.h"
#include "RobotCandidate.h"
#include "LineDetection.h"
#include "ObjectCandidate.h"
#include "NUPlatform/NUCamera.h"
#include "Tools/Math/Vector2.h"
#include "Tools/FileFormats/LUTTools.h"

#include <vector>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <fstream>
//#include <QImage>

class NUSensorsData;
class NUActionatorsData;
class SaveImagesThread;
class NUPlatform;
class NUCameraData;

#define ORANGE_BALL_DIAMETER 6.5 //IN CM for NEW BALL

class Circle;
class NUImage;
class JobList;
class NUIO;
//! Contains vision processing tools and functions.
class Vision
{
    private:
    const NUImage* currentImage;                //!< Storage of a pointer to the raw colour image.
    const unsigned char* currentLookupTable;    //!< Storage of the current colour lookup table.
    unsigned char* LUTBuffer;                   //!< Storage of the current colour lookup table.
    unsigned char* testLUTBuffer;
    int spacings;

    NUSensorsData* m_sensor_data;               //!< pointer to shared sensor data object
    NUActionatorsData* m_actions;               //!< pointer to shared actionators data object
    friend class SaveImagesThread;
    SaveImagesThread* m_saveimages_thread;      //!< an external thread to do saving images in parallel with vision processing
    
    int findYFromX(const std::vector<Vector2<int> >&points, int x);
    bool checkIfBufferSame(boost::circular_buffer<unsigned char> cb);

    //! SavingImages:
    bool isSavingImages;
    bool isSavingImagesWithVaryingSettings;
    int numSavedImages;
    ofstream imagefile;
    ofstream sensorfile;
    int ImageFrameNumber;
    int numFramesDropped;               //!< the number of frames dropped since the last call to getNumFramesDropped()
    int numFramesProcessed;             //!< the number of frames processed since the last call to getNumFramesProcessed()
    CameraSettings currentSettings;
    NUCameraData* m_camera_specs;

    void SaveAnImage();

    public:

    static const int OBSTACLE_HEIGH_THRESH = 5;
    static const int OBSTACLE_WIDTH_MIN = 2;

    //! FieldObjects Container
    FieldObjects* AllFieldObjects;
    Horizon m_horizonLine;
    int classifiedCounter;
    //! Default constructor.
    Vision();
    //! Destructor.
    ~Vision();
    double m_timestamp;

    std::ofstream m_profiling_stream;


    double CalculateBearing(double cx) const;
    double CalculateElevation(double cy) const;

    double EFFECTIVE_CAMERA_DISTANCE_IN_PIXELS();


    void process (JobList* jobs);

    void ProcessFrame(NUImage* image, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects);

    void setFieldObjects(FieldObjects* fieldobjects);

    void setSensorsData(NUSensorsData* data);

    void setActionatorsData(NUActionatorsData* actions);

    void setLUT(unsigned char* newLUT);
    void loadLUTFromFile(const std::string& fileName);

    void setImage(const NUImage* sourceImage);
    int getNumFramesDropped();
    int getNumFramesProcessed();

    
    /*!
      @brief perform an edge filter to extract prominent features for further analysis
      */
    /*QImage getEdgeFilter();
    QImage getEdgeFilter(int x, int y, int width, int height);
    QImage getEdgeFilter(int x, int y, int width, int height, int decimation_spacing);
    QImage getEdgeFilter(QImage image);*/

    /*!
      @brief perform an fft to extract prominent features for further analysis
      */
    /*QImage getFFT();
    QImage getFFT(int x, int y, int width, int height);
    QImage getFFT(int x, int y, int width, int height, int decimation_spacing);
    QImage getFFT(QImage image);*/

    void classifyPreviewImage(ClassifiedImage &target,unsigned char* tempLut);
    /*!
      @brief Produce a classified.

      Primarily used for display when debugging and constructing a lookup table.
      @param targetImage The target classification image that will be written to.
      @param sourceImage The raw image to be classified.
      @param lookUpTable The colour classification lookup table. This table maps colours
      from the raw source image into the classified colour space.
      */
    void classifyImage(ClassifiedImage &targetImage);
    /*!
      @brief Classifies an individual pixel.
      @param x The x coordinate of the pixel to be classified.
      @param y The y coordinate of the pixel to be classified.
      @return Returns the classfied colour index for the given pixel.
      */
    inline unsigned char classifyPixel(int x, int y)
    {
        classifiedCounter++;
        const Pixel& temp = (*currentImage)(x,y);
        //return  currentLookupTable[(temp->y<<16) + (temp->cb<<8) + temp->cr]; //8 bit LUT
        return  currentLookupTable[LUTTools::getLUTIndex(temp)]; // 7bit LUT
    }

    enum tCLASSIFY_METHOD
    {
        PRIMS,
        DBSCAN
    };

    /*!
      @brief Joins segments to create a joined segment clusters that represent candidate robots
      @param segList The segList is a vector of TransitionSegments after field lines have been rejected
      @returns A list of ObjectCanidates
    */

    std::vector<ObjectCandidate> classifyCandidates(std::vector< TransitionSegment > &segments,
                                                    const std::vector<Vector2<int> >&fieldBorders,
                                                    const std::vector<unsigned char> &validColours,
                                                    int spacing,
                                                    float min_aspect, float max_aspect, int min_segments,
                                                    tCLASSIFY_METHOD method);

    std::vector<ObjectCandidate> classifyCandidates(std::vector< TransitionSegment > &segments,
                                                    const std::vector<Vector2<int> >&fieldBorders,
                                                    const std::vector<unsigned char> &validColours,
                                                    int spacing,
                                                    float min_aspect, float max_aspect, int min_segments,
                                                    std::vector< TransitionSegment >& leftover);

    std::vector<ObjectCandidate> classifyCandidatesPrims(std::vector< TransitionSegment > &segments,
                                                         const std::vector<Vector2<int> >&fieldBorders,
                                                         const std::vector<unsigned char> &validColours,
                                                         int spacing,
                                                         float min_aspect, float max_aspect, int min_segments);

    std::vector<ObjectCandidate> classifyCandidatesPrims(std::vector< TransitionSegment > &segments,
                                                         const std::vector<Vector2<int> >&fieldBorders,
                                                         const std::vector<unsigned char> &validColours,
                                                         int spacing,
                                                         float min_aspect, float max_aspect, int min_segments,
                                                         std::vector< TransitionSegment >& leftover);

    std::vector<ObjectCandidate> classifyCandidatesDBSCAN(std::vector< TransitionSegment > &segments,
                                                          const std::vector<Vector2<int> >&fieldBorders,
                                                          const std::vector<unsigned char> &validColours,
                                                          int spacing,
                                                          float min_aspect, float max_aspect, int min_segments);


    /*!
      @brief Returns true when the colour passed in is a valid colour from the list passed in
      @param colour The colour value that needs to be checked if it is a robot colour
      @param colourList The vector of valid colours to match against
      @return bool True when the colour passed in is an assigned robot colour
    */
    bool isValidColour(unsigned char colour, const std::vector<unsigned char> &colourList);

    int findInterceptFromPerspectiveFrustum(const std::vector<Vector2<int> >&points, int current_x, int target_x, int spacing);
    static bool sortTransitionSegments(TransitionSegment a, TransitionSegment b);

    std::vector<Vector2<int> > findGreenBorderPoints(int scanSpacing, Horizon* horizonLine);
    std::vector<Vector2<int> > getConvexFieldBorders(const std::vector<Vector2<int> >& fieldBorders);
    std::vector<Vector2<int> > interpolateBorders(const std::vector<Vector2<int> >& fieldBorders, int scanSpacing);


    ClassifiedSection horizontalScan(const std::vector<Vector2<int> >&fieldBoarders, int scanSpacing);
    ClassifiedSection verticalScan(const std::vector<Vector2<int> >&fieldBoarders, int scanSpacing);
    void ClassifyScanArea(ClassifiedSection* scanArea);
    void CloselyClassifyScanline(ScanLine* tempLine, TransitionSegment* tempSeg, int spacing, int direction, const std::vector<unsigned char> &colourList,int bufferSize);

    void DetectLineOrRobotPoints(ClassifiedSection* scanArea, LineDetection* LineDetector);

    void DetectLines(LineDetection* LineDetector);
    void DetectLines(LineDetection* LineDetector, vector<ObjectCandidate>& candidates, vector< TransitionSegment >& leftover);

     std::vector< ObjectCandidate > ClassifyCandidatesAboveTheHorizon(std::vector< TransitionSegment > &segments,
                                                                      const std::vector<unsigned char> &validColours,
                                                                      int spacing, int min_segments);

     std::vector< ObjectCandidate > ClassifyCandidatesAboveTheHorizon(std::vector< TransitionSegment > &segments,
                                                                      const std::vector<unsigned char> &validColours,
                                                                      int spacing, int min_segments,
                                                                      std::vector< TransitionSegment > &leftover);

    Circle DetectBall(const std::vector<ObjectCandidate> &FO_Candidates);

    void DetectGoals(std::vector<ObjectCandidate>& FO_Candidates,
                     std::vector<ObjectCandidate>& FO_AboveHorizonCandidates,
                     const std::vector< TransitionSegment > horizontalSegments);

    void PostProcessGoals();

    void DetectRobots(std::vector<ObjectCandidate> &RobotCandidates);

    bool isPixelOnScreen(int x, int y);
    int getImageHeight(){ return currentImage->getHeight();}
    int getImageWidth(){return currentImage->getWidth();}

    int getScanSpacings(){return spacings;}

    NUSensorsData* getSensorsData() {return m_sensor_data;}
    bool checkIfBufferContains(boost::circular_buffer<unsigned char> cb, const std::vector<unsigned char> &colourList);

    int CalculateSkipSpacing(int currentPosition, int lineLength, bool greenSeen);

    //ADDED BY SHANNON 23-11-11
    /*!
      @brief Creates a vector of AmbiguousObjects matching the candidate vector given.
      @param candidates A vector of ObjectCandidates to make the AmbiguousObjects from.
      @return vector<AmbiguousObject> Vector of objects for later use.
    */
    vector<AmbiguousObject> getObjectsFromCandidates(vector<ObjectCandidate> candidates);
    //ADDED BY SHANNON 08-12-11
    /*!
      @brief Calculates the vertical differences between the green border points and the hull made from them.
      @param prehull The original green border points (should be the same length as hull).
      @param hull The upper convex hull of the border points (should be the same length as prehull).
      @return vector<int> Vector of the pairwise differences between the inputs (will be null if the inputs do not match).
    */
    vector<int> getVerticalDifferences(const vector< Vector2<int> >& prehull, const vector< Vector2<int> >& hull) const;
    /*!
      @brief Generates a vector of ObjectCandidates corresponding to seen obstacles.
      @param prehull The original green border points (should be the same length as hull).
      @param hull The upper convex hull of the border points (should be the same length as prehull).
      @param height_thresh The minimum height difference between points for an obstacle.
      @param width_min  The minimum number of consecutive breaks for an obstacle.
      @return vector<ObjectCandidate> The obstacles found.
    */
    vector<ObjectCandidate> getObstacleCandidates(const vector< Vector2<int> >& prehull, const vector< Vector2<int> >& hull,
                                                  int height_thresh, int width_min) const;
/**ADDED BY SHANNON**/

};
#endif // VISION_H
