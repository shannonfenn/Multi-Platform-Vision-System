/*
 * Author: Shannon Fenn
 * Last Modified: 25/02/11
 * Description:
 *      - A static class of method implementing the split and merge line extraction
        algorithm for us on the NAO robot platform

        - Parameters are static member variable set by initRules(), rather than
        #define macros. Make sure to call this method with reasonable values
        before calling splitAndMergeLS() or splitAndMergeLSClusters()

        - the only method to call (besides initRules()) is either
         + splitAndMergeLS() - for non-clustered input
         + splitAndMergeLSClusters() - for clustered input

        - If things need to be changed the main decisions are in:
         + splitLSIterative() - decisions on whether to split, keep or throw away lines
         + findFurthestPoint() - finds the number of points distant from the line, and the furthest point
         + separateLS() - divides point set into two subsets based on the line equation and the furthest point
                uses a transform to axis defined by the line itself, and the normal from the furthest point
                to the line.
         + shouldMergeLines() - decisions on whether two lines should be merged

         - All other methods are trivial or only make simple decisions based on the parameters set
            by initRules()
 */

#ifndef LINEDETECTORSAM_H
#define LINEDETECTORSAM_H

#include <vector>
#include <math.h>
#include "Tools/Math/LSFittedLine.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Vector3.h"
#include "Vision/Modules/linedetector.h"

using std::vector;

class LineDetectorSAM : public LineDetector
{
public:
    LineDetectorSAM();
    ~LineDetectorSAM();

    virtual std::vector<FieldLine> run(const std::vector<NUPoint> &points);

private:
    std::vector< std::pair<LSFittedLine, LSFittedLine> > fitLines(const std::vector<NUPoint>& points, bool noise=true);
    //RULES
    //maximum field objects rules
    //unsigned int MAX_POINTS; //500
    unsigned int MAX_LINES; //15
    //splitting rules
    double SPLIT_DISTANCE; //1.0
    unsigned int MIN_POINTS_OVER; //2
    unsigned int MIN_POINTS_TO_LINE; //3
    //merging rules
    double MAX_ANGLE_DIFF_TO_MERGE;
    double MAX_DISTANCE_TO_MERGE;
    //Line keeping rules
    unsigned int MIN_POINTS_TO_LINE_FINAL; //5
    double MIN_LINE_R2_FIT; //0.90
    double MAX_LINE_MSD; //50 set at constructor
    //clearing rules
    bool CLEAR_SMALL;
    bool CLEAR_DIRTY;

    std::vector<NUPoint> noisePoints;

    //LEAST-SQUARES FITTING
    void split(std::vector<std::pair<LSFittedLine, LSFittedLine> > &lines, const std::vector<NUPoint> &points);
    //void splitIterative(std::vector<LSFittedLine>& lines, std::vector<Point>& points);
    void splitNoise(std::vector<std::pair<LSFittedLine, LSFittedLine> >& lines);
    void merge(std::vector<LSFittedLine>& lines);
    void generateLines(std::pair<LSFittedLine, LSFittedLine>& lines, const std::vector<NUPoint>& points);
    bool separate(std::vector<NUPoint>& left, std::vector<NUPoint>& right, NUPoint split_point, const std::vector<NUPoint>& points, const LSFittedLine& line);
    //static void sortLinesLS(std::vector<LSFittedLine*>& lines);


    //GENERIC
    void findPointsOver(LSFittedLine& line, unsigned int &points_over, int& furthest_point);
    void addToNoise(const NUPoint& point);
    void addToNoise(const std::vector<NUPoint>& points);
    void clearSmallLines(std::vector<std::pair<LSFittedLine, LSFittedLine> >& lines);
    void clearDirtyLines(std::vector<std::pair<LSFittedLine, LSFittedLine> >& lines);

};

#endif // LINEDETECTORSAM_H
