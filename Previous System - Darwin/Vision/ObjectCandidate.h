#ifndef OBJECTCANDIDATE_H
#define OBJECTCANDIDATE_H

#include <vector>
#include "Tools/Math/Vector2.h"
#include "TransitionSegment.h"


class ObjectCandidate
{
public:
    Vector2<int> getTopLeft() const;
    Vector2<int> getBottomRight() const;
    void setTopLeft(Vector2<int> point);
    void setBottomRight(Vector2<int> point);
    int width() const;
    int height() const;
    int getCentreX() const;
    int getCentreY() const;
    float aspect() const;
    unsigned char getColour()  const;
    void setColour(unsigned char c);
    std::vector<TransitionSegment> getSegments() const;
    void addSegments(const std::vector<TransitionSegment> &new_segments);
    void addSegment(const TransitionSegment &new_segment);

    ObjectCandidate();
    ObjectCandidate(int left, int top, int right, int bottom);
    ObjectCandidate(int left, int top, int right, int bottom, unsigned char colour);
    ObjectCandidate(int left, int top, int right, int bottom, unsigned char colour, std::vector<TransitionSegment> candidate_segments);
    ~ObjectCandidate();


protected:
    Vector2<int> topLeft;
    Vector2<int> bottomRight;
    std::vector<TransitionSegment> segments;
    unsigned char colour;


};

#endif // OBJECTCANDIDATE_H
