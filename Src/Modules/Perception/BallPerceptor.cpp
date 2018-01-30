/**
 * @file BallPerceptor.cpp
 * This file uese the perception results of BlackAndWhiteBallPercept to form ball percept
 * TODO try to use full size of the image i.e. 1280*960
 * @author Li Shu
 */ 
#define NIGHT
//#define STRICKT
#define INDOOR
//#define SINGLE_CHANNEL
#define GRASS_LAND
#define COMBINE_WITH_FIT
#define DAY_LIGHT
//#define SIMPLE_LOWER // in this case balls in lower camera can be treated as real balls
#define DARK
#include "BallPerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Math/Eigen.h"
#include <iostream>
  
//using namespace std;
using namespace std::chrono;

MAKE_MODULE(BallPerceptor, perception)

bool BallPerceptor::isWhite(const Image::Pixel* p) const
{
	if (theColorTable[*p].is(ColorClasses::white))
	        return true;
		else
			return false;
}

ColorTable::Colors BallPerceptor::determineColor(
    const Image::Pixel & p) const
{
	if (theColorTable[p].is(ColorClasses::green))
	        return ColorClasses::green;
	if (theColorTable[p].is(ColorClasses::white))
	        return ColorClasses::white;
	if (theColorTable[p].is(ColorClasses::black))
	        return ColorClasses::black;
  return ColorClasses::none;
}

bool BallPerceptor::possibleGreen(const Image::Pixel& p) const
{
	if (theColorTable[p].is(ColorClasses::green))
			return true;
	return false;
}

bool BallPerceptor::possibleGrayScale(const Image::Pixel& p) const
{
	if (theColorTable[p].is(ColorClasses::white)||theColorTable[p].is(ColorClasses::black)||theColorTable[p].is(ColorClasses::none))
		        return true;
    return false;
}

void BallPerceptorScaler::scaleInput()
{
  typedef BallPerceptorBase B;
  theCameraInfo = B::theCameraInfo;
  theBallSpots = B::theBallSpots;

  // do not copy "table"
  theImageCoordinateSystem.rotation = B::theImageCoordinateSystem.rotation;
  theImageCoordinateSystem.invRotation =
      B::theImageCoordinateSystem.invRotation;
  theImageCoordinateSystem.origin = B::theImageCoordinateSystem.origin;
  theImageCoordinateSystem.offset = B::theImageCoordinateSystem.offset;
  theImageCoordinateSystem.a = B::theImageCoordinateSystem.a;
  theImageCoordinateSystem.b = B::theImageCoordinateSystem.b;

  if (theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
  {
    theCameraInfo.width *= 1;  // 1280
    theCameraInfo.height *= 1;  // 960
    theCameraInfo.opticalCenter *= 1.f;
    theCameraInfo.focalLength *= 1.f;
    theCameraInfo.focalLengthInv /= 1.f;
    theCameraInfo.focalLenPow2 *= 1.f;

    theImageCoordinateSystem.origin *= 1.f;
    theImageCoordinateSystem.b *= 1.f;

    for (BallSpot& b : theBallSpots.ballSpots)
    {
      b.position *= 1;
//      ++b.position.x();  // original y channel was the second one
    }
  }
  theImageCoordinateSystem.setCameraInfo(theCameraInfo);
}

void BallPerceptorScaler::scaleOutput(BallPercept& ballPercept) const
{
  if (theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
  {
    ballPercept.positionInImage *= 0.5f;
    ballPercept.radiusInImage *= 0.5f;
  }
}

// Alternate drawing macros that scale down if required
#define LINE2(n, x1, y1, x2, y2, w, s, c) LINE(n, scale(x1), scale(y1), scale(x2), scale(y2), w, s, c)
#define CROSS2(n, x, y, r, w, s, c) CROSS(n, scale(x), scale(y), scale(r), w, s, c)
#define CIRCLE2(n, x, y, r, w, s1, c1, s2, c2) CIRCLE(n, scale(x), scale(y), scale(r), w, s1, c1, s2, c2)
#define RECTANGLE3(n, x1, y1, x2, y2, w, s, c) RECTANGLE(n, scale(x1), scale(y1), scale(x2), scale(y2), w, s, c)
#define DOT2(n, x, y, c1, c2) DOT(n, scale(x), scale(y), c1, c2)
#define DRAWTEXT2(n, x, y, w, s, c) DRAWTEXT(n, scale(x), scale(y), w, s, c)

BallPerceptor::BallPerceptor()
    : right(0),
      horizon(0),
      height(0)
{
  sqrMaxBallDistance = Vector2f(
      theFieldDimensions.xPosOpponentFieldBorder
          - theFieldDimensions.xPosOwnFieldBorder,
      theFieldDimensions.yPosLeftFieldBorder
          - theFieldDimensions.yPosRightFieldBorder).squaredNorm();
}


void BallPerceptor::update(BallPercept& ballPercept)
{
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:ballSpot", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:ballSpotScanLines",
                        "drawingOnImage");
  DECLARE_PLOT("module:BallPerceptor:angle");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:ballRectangle", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:simpleCheck", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:possibleBalls", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:edgePoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:spotsMonitor", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:searchWindow", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:RadiusOnField", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:errors", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:colorCounter", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:colorCounterAll", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:thre", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:visioin", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:filterballspot", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:HOGScore", "drawingOnImage");

  scaleInput();
  possibleBalls.clear();
  startT = high_resolution_clock::now();

  ballPercept.status = BallPercept::notSeen;
  if (!theCameraMatrix.isValid)
  {
    return;
  }
  if(theBallSpots.ballSpots.empty())
      return;
  float bestError = 10.f;
  int bestIdx = 0;
  float maxScore = 0;
  for(int i=0;i<static_cast<int>(theBallSpots.ballSpots.size());++i)
  {
      BallPerceptor::Ball simpleBall;
      simpleBall.radius = theBallSpots.ballSpots[i].shouldR;
      simpleBall.x = theBallSpots.ballSpots[i].position.x();
      simpleBall.y = theBallSpots.ballSpots[i].position.y();
      candidateBall simpleCb;
        simpleCb.ball = simpleBall;
        simpleCb.disError = 0;
      Vector2f spotPosition = Vector2f(theBallSpots.ballSpots[i].position.x(),theBallSpots.ballSpots[i].position.y());
      if (theBallSpots.ballSpots[i].distance > 7000.f)
          continue;
      DOT("module:BallPerceptor:filterballspot", theBallSpots.ballSpots[i].position.x(), theBallSpots.ballSpots[i].position.y(),
              ColorRGBA::red, ColorRGBA::red);
      Vector2f ballColor = Vector2f(theBallSpots.ballSpots[i].colorH,theBallSpots.ballSpots[i].colorS);
      BallPerceptor::Ball b = fitBall(spotPosition,theBallSpots.ballSpots[i].shouldR, ballColor);
      if (b.found == true)
      {
        Vector2f ballPosition = Vector2f(b.x,b.y);

        float disError =  (ballPosition-spotPosition).norm()/theBallSpots.ballSpots[i].shouldR;
        float radError = fabs(b.radius-theBallSpots.ballSpots[i].shouldR)/theBallSpots.ballSpots[i].shouldR;
        if ((disError<0.8f && theCameraInfo.camera == CameraInfo::upper)
        || (disError<1.f && theCameraInfo.camera == CameraInfo::lower))
            if ((radError>-1.5f && radError<1.5f && theCameraInfo.camera == CameraInfo::upper)
            || (radError>-1.5f && radError<1.5f && theCameraInfo.camera == CameraInfo::lower))
        {
        DRAWTEXT("module:BallPerceptor:errors", b.x, b.y-10,10,ColorRGBA::orange,"disError="<<disError);
        DRAWTEXT("module:BallPerceptor:errors", b.x, b.y,10,ColorRGBA::magenta,"radError="<<radError);
        DRAWTEXT("module:BallPerceptor:errors", b.x, b.y+10,10,ColorRGBA::cyan,"Quality="<<b.q);
        DRAWTEXT("module:BallPerceptor:errors", b.x, b.y+20,10,ColorRGBA::magenta,"Distance="<<theBallSpots.ballSpots[i].distance);

        if(b.radius < 6 || b.radius > 65)
        	continue;

        CIRCLE("module:BallPerceptor:possibleBalls", b.x, b.y, b.radius, 2,
                    	                        Drawings::solidPen, ColorRGBA::blue, Drawings::solidBrush,
                    	                        ColorRGBA(255, 0, 0, 150));
                DRAWTEXT("module:BallPerceptor:possibleBalls", b.x, b.y+10, 10,
                                           ColorRGBA::orange, b.radius);

        candidateBall cb;
        cb.ball = b;
        cb.disError = disError;
        if (classifyBalls2(cb.ball))
        {
        	possibleBalls.push_back(cb);
        }
        }
      }
  }

  for(int i=0;i<static_cast<int>(possibleBalls.size());++i)
  {
      if (possibleBalls[i].disError < bestError)
      {

    	  bestError = possibleBalls[i].disError;
          bestIdx = i;
      }
  }
  if (bestError<10.f)
  {
      ballPercept.status = BallPercept::seen;
      ballPercept.positionInImage.x() = possibleBalls[bestIdx].ball.x;
      ballPercept.positionInImage.y() = possibleBalls[bestIdx].ball.y;
      ballPercept.radiusInImage = possibleBalls[bestIdx].ball.radius;
      calculateBallOnField(ballPercept);
      ballPercept.radiusOnField = 50;
  }
//  endT = high_resolution_clock::now();
//  duration<double> dTotal = duration_cast<duration<double>>(endT-startT);
//  std::cout << "BallPercept=" << dTotal.count()*1000 <<"ms"<< std::endl;

    PLOT("module:BallPerceptor:angle",
       toDegrees(ballPercept.relativePositionOnField.angle()));
}

bool BallPerceptor::checkBallSpot(const BallSpot& ballSpot)
{
  // calculate an approximation of the radius based on bearing distance of the ball spot
  const Vector2i& spot = ballSpot.position;
  Vector2f correctedStart = theImageCoordinateSystem.toCorrected(spot);
  Vector3f cameraToStart(theCameraInfo.focalLength,
                         theCameraInfo.opticalCenter.x() - correctedStart.x(),
                         theCameraInfo.opticalCenter.y() - correctedStart.y());
  Vector3f unscaledField = theCameraMatrix.rotation * cameraToStart;
  if (unscaledField.z() >= 0.f)
  {
    return false;  // above horizon
  }
  const float scaleFactor = (theCameraMatrix.translation.z()
      - theFieldDimensions.ballRadius) / unscaledField.z();
  cameraToStart *= scaleFactor;
  unscaledField *= scaleFactor;
  if (unscaledField.topRows(2).squaredNorm() > sqrMaxBallDistance)
  {
    return false;  // too far away
  }
  cameraToStart.y() +=
      cameraToStart.y() > 0 ?
          -theFieldDimensions.ballRadius : theFieldDimensions.ballRadius;
  cameraToStart /= scaleFactor;
  approxRadius1 = std::abs(
      theCameraInfo.opticalCenter.x() - cameraToStart.y() - correctedStart.x());

  return true;
}

bool BallPerceptor::searchBallPoints(const BallSpot& ballSpot)
{
  Vector2i start = ballSpot.position;
  const float approxDiameter = approxRadius1 * clippingApproxRadiusScale
      + clippingApproxRadiusPixelBonus;
  int halfApproxRadius = int(approxRadius1 * 0.5f);
  COMPLEX_DRAWING("module:BallPerceptor:image")drawBall(start, approxDiameter, 0x6a);
  // try to improve the start point
//  int maxColorDistance = scanMaxColorDistance;
  int resolutionWidth = theCameraInfo.width;
  int resolutionHeight = theCameraInfo.height;
  startPixel = getPixel(start.y(), start.x());
  Vector2i preScanPoints[4] =
  { start + Vector2i(halfApproxRadius, halfApproxRadius), start
      + Vector2i(-halfApproxRadius, -halfApproxRadius), start
      + Vector2i(halfApproxRadius, -halfApproxRadius), start
      + Vector2i(-halfApproxRadius, halfApproxRadius) };
  bool preScanResults[4];
  for (int i = 0; i < 4; ++i)
  {
    if (preScanPoints[i].x() < 0 || preScanPoints[i].x() >= resolutionWidth
        || preScanPoints[i].y() < 0 || preScanPoints[i].y() >= resolutionHeight)
    {
      i -= i % 2;
      preScanResults[i++] = false;
      ASSERT(i < 4);
      preScanResults[i] = false;
    }
    else
    {
      const Image::Pixel pixel = getPixel(preScanPoints[i].y(),
                                          preScanPoints[i].x());
      ColorTable::Colors pColor2 = determineColor(getPixel(preScanPoints[i].y(), preScanPoints[i].x()));

      preScanResults[i] = (pColor2.is(ColorClasses::black)
          || pColor2.is(ColorClasses::white))
          && !pColor2.is(ColorClasses::green);
    }
  }
  if (preScanResults[0] != preScanResults[1]
      && preScanResults[2] != preScanResults[3])
  {
    start = Vector2i::Zero();
    if (preScanResults[0])
    {
      start += preScanPoints[0];
    }
    else
    {
      start += preScanPoints[1];
    }
    if (preScanResults[2])
    {
      start += preScanPoints[2];
    }
    else
    {
      start += preScanPoints[3];
    }
    start /= 2;
  }
  else if (preScanResults[0] != preScanResults[1])
  {
    start = preScanResults[0] ? preScanPoints[0] : preScanPoints[1];
  }
  else if (preScanResults[2] != preScanResults[3])
  {
    start = preScanResults[2] ? preScanPoints[2] : preScanPoints[3];
  }
  COMPLEX_DRAWING("module:BallPerceptor:image")drawBall(start, approxDiameter, 0xff);
  // prepare scans
  totalPixelCount = 0;
  // vertical scan
  if (!searchBallPoint(start, startPixel, Vector2i(0, 1), approxDiameter,
                       ballPoints[0])
      || !searchBallPoint(start, startPixel, Vector2i(0, -1), approxDiameter,
                          ballPoints[4]))
  {
    return false;
  }
  if (ballPoints[0].atBorder && ballPoints[4].atBorder)
  {
    return false;  // too large
  }
  else if (ballPoints[0].atBorder)
  {
    start.y() = ballPoints[4].point.y() + int(approxRadius1);
    if (start.y() > ballPoints[0].point.y() - 1)
    {
      start.y() = ballPoints[0].point.y() - 1;
    }
  }
  else if (ballPoints[4].atBorder)
  {
    start.y() = ballPoints[0].point.y() - int(approxRadius1);
    if (start.y() < ballPoints[4].point.y() + 1)
    {
      start.y() = ballPoints[4].point.y() + 1;
    }
  }
  else
  {
    start.y() = (ballPoints[0].point.y() + ballPoints[4].point.y()) / 2;
  }
  // horizontal scan
  if (!searchBallPoint(start, startPixel, Vector2i(1, 0), approxDiameter,
                       ballPoints[2])
      || !searchBallPoint(start, startPixel, Vector2i(-1, 0), approxDiameter,
                          ballPoints[6]))
  {
    return false;
  }
  if (ballPoints[2].atBorder && ballPoints[6].atBorder)
  {
    return false;  // too large
  }
  else if (ballPoints[2].atBorder)
  {
    start.x() = ballPoints[6].point.x() + int(approxRadius1);
    if (start.x() > ballPoints[2].point.x() - 1)
    {
      start.x() = ballPoints[2].point.x() - 1;
    }
  }
  else if (ballPoints[6].atBorder)
  {
    start.x() = ballPoints[2].point.x() - int(approxRadius1);
    if (start.x() < ballPoints[6].point.x() + 1)
    {
      start.x() = ballPoints[6].point.x() + 1;
    }
  }
  else
  {
    start.x() = (ballPoints[2].point.x() + ballPoints[6].point.x()) / 2;
  }
  approxCenter2 = start;
  // maybe repeat vertical and horizontal scans
  int skipArea = std::min(ballPoints[0].point.y() - ballPoints[4].point.y(),
                          ballPoints[2].point.x() - ballPoints[6].point.x())
      / 4;
  float maxLength = approxDiameter - skipArea;
  if (abs(start.x() - ballPoints[0].start.x()) > halfApproxRadius)
  {
    if (!searchBallPoint(start + Vector2i(0, skipArea), startPixel,
                         Vector2i(0, 1), maxLength, ballPoints[0])
        || !searchBallPoint(start + Vector2i(0, -skipArea), startPixel,
                            Vector2i(0, -1), maxLength, ballPoints[4]))
    {
      return false;
    }
  }
  if (abs(start.y() - ballPoints[2].start.y()) > halfApproxRadius)
  {
    if (!searchBallPoint(start + Vector2i(skipArea, 0), startPixel,
                         Vector2i(1, 0), maxLength, ballPoints[2])
        || !searchBallPoint(start + Vector2i(-skipArea, 0), startPixel,
                            Vector2i(-1, 0), maxLength, ballPoints[6]))
    {
      return false;
    }
  }
  // diagonal scans
  skipArea = std::min(ballPoints[0].point.y() - ballPoints[4].point.y(),
                      ballPoints[2].point.x() - ballPoints[6].point.x()) / 4;
  maxLength = approxDiameter - 1.41421356f * skipArea;
  if (!searchBallPoint(start + Vector2i(skipArea, skipArea), startPixel,
                       Vector2i(1, 1), maxLength, ballPoints[1])
      || !searchBallPoint(start + Vector2i(-skipArea, -skipArea), startPixel,
                          Vector2i(-1, -1), maxLength, ballPoints[5])
      || !searchBallPoint(start + Vector2i(skipArea, -skipArea), startPixel,
                          Vector2i(1, -1), maxLength, ballPoints[3])
      || !searchBallPoint(start + Vector2i(-skipArea, skipArea), startPixel,
                          Vector2i(-1, 1), maxLength, ballPoints[7]))
  {
    return false;
  }
  // improve ball points
  if (totalPixelCount == 0)
  {
    return false;
  }
  for (unsigned j = 0; j < sizeof(ballPoints) / sizeof(*ballPoints); ++j)
  {
    BallPoint& ballPoint = ballPoints[j];
    const Vector2i& step = ballPoint.step;
    Vector2i pos = ballPoint.point;
    int i = 0;
    for (; i < (int) refineMaxPixelCount; ++i)
    {
      pos += step;
      if (pos.x() < 0 || pos.x() >= resolutionWidth || pos.y() < 0
          || pos.y() >= resolutionHeight)
      {
        break;
      }
      const Image::Pixel pixel = getPixel(pos.y(), pos.x());
      ColorTable::Colors pColor3 = determineColor(getPixel(pos.y(), pos.x()));
      if (!((pColor3.is(ColorClasses::black) || pColor3.is(ColorClasses::white))
          && !pColor3.is(ColorClasses::green)))
      {
        break;
      }
    }
    if (i)
    {
      ballPoint.point = pos - step;
    }
    ballPoint.pointf = ballPoint.point.cast<float>();
  }
  return true;
}

void BallPerceptor::drawBall(const Vector2i& pos, float approxDiameter,
                             unsigned char opacity) const
{
  CROSS2("module:BallPerceptor:image", pos.x(), pos.y(), 2, 1,
         Drawings::solidPen, ColorRGBA(0xff, 0, 0, opacity));
  CIRCLE2("module:BallPerceptor:image", pos.x(), pos.y(), approxDiameter, 1,
          Drawings::solidPen, ColorRGBA(0xff, 0, 0, opacity), Drawings::noBrush,
          ColorRGBA());
}

bool BallPerceptor::searchBallPoint(const Vector2i& start,
                                    Image::Pixel startPixel,
                                    const Vector2i& step, float maxLength,
                                    BallPoint& ballPoint)
{
  ASSERT(step.x() == 0 || step.x() == 1 || step.x() == -1);
  ASSERT(step.y() == 0 || step.y() == 1 || step.y() == -1);
  Vector2i pos = start;
  Vector2i lastValidPos = pos;
  unsigned int overtime = 0;
  int pixelCount = 1;
  ballPoint.step = step;
  ballPoint.atBorder = false;
  ballPoint.start = start;
  unsigned int maxOvertime = scanPixelTolerance;
  int resolutionWidth = theCameraInfo.width;
  int resolutionHeight = theCameraInfo.height;
  int stepAbs1024 = (step.x() == 0 || step.y() == 0) ? 1024 : 1448;  // 1448 = sqrt(2) * 1024
  int maxAbs1024 = int(maxLength * 1024.f);  // + 1024;
  int length1024 = 0;
  for (;;)
  {
    pos += step;
    length1024 += stepAbs1024;
    if (length1024 > maxAbs1024)
    {
      if (overtime > 0)
      {
        break;
      }
      LINE2("module:BallPerceptor:image", start.x(), start.y(), pos.x(),
            pos.y(), 1, Drawings::solidPen, ColorRGBA(0, 0, 0xff, 0x70));
      return false;
    }

    if (pos.x() < 0 || pos.x() >= resolutionWidth || pos.y() < 0
        || pos.y() >= resolutionHeight)
    {
      ballPoint.atBorder = true;
      break;
    }

    const Image::Pixel pixel = getPixel(pos.y(), pos.x());
    ColorTable::Colors pColor4 = determineColor(getPixel(pos.y(), pos.x()));
    if (!((pColor4.is(ColorClasses::black) || pColor4.is(ColorClasses::white))
        && !pColor4.is(ColorClasses::green)))
    {
      if (overtime == 0)
      {
        lastValidPos = pos - step;
      }
      ++overtime;
      if (overtime <= maxOvertime)
      {
        continue;
      }
      break;
    }
    ++pixelCount;
    overtime = 0;
  }

  ballPoint.point = lastValidPos;

  if (pixelCount > 0)
  {
    --pixelCount;
    totalPixelCount += pixelCount;
  }
  LINE2("module:BallPerceptor:image", ballPoint.start.x(), ballPoint.start.y(),
        ballPoint.point.x(), ballPoint.point.y(), 1, Drawings::solidPen,
        ColorRGBA(0, 0, 0xff));
  return true;
}

bool BallPerceptor::checkBallPoints()
{
  // find "valid" ball points
  validBallPoints = 0;
  static const int countOfBallPoints = sizeof(ballPoints) / sizeof(*ballPoints);
  for (int i = 0; i < countOfBallPoints; ++i)
  {
    BallPoint& ballPoint(ballPoints[i]);
    ballPoint.isValid = !ballPoint.atBorder
        && ballPoint.point != ballPoint.start;
    if (ballPoint.isValid)
    {
      ++validBallPoints;
      DOT2("module:BallPerceptor:image", ballPoint.point.x(),
           ballPoint.point.y(), ColorRGBA(0, 0xff, 0, 0x70),
           ColorRGBA(0, 0xff, 0, 0x70));
    }
  }
  if (validBallPoints < 4)
  {
    return false;
  }

  // find duplicated ball points (may occur in small balls)
  for (int i = 0; i < countOfBallPoints; ++i)
  {
    BallPoint& ballPoint(ballPoints[i]);
    if (ballPoint.isValid)
    {
      const BallPoint& nextBallPoint(ballPoints[(i + 1) % countOfBallPoints]);
      if (nextBallPoint.isValid && ballPoint.point == nextBallPoint.point)
      {
        ballPoint.isValid = false;
        --validBallPoints;
      }
    }
  }
  if (validBallPoints < 4)
    return false;

  // drop mismatching ball points
  while (validBallPoints > 4)
  {
    Vector2f preCenter;
    float preRadius;
    if (!getBallFromBallPoints(preCenter, preRadius))
    {
      return false;
    }

    float minDist = 0;
    BallPoint* minDistBallPoint = 0;
    for (int i = 0; i < countOfBallPoints; ++i)
    {
      BallPoint& ballPoint(ballPoints[i]);
      if (ballPoint.isValid)
      {
        float dist = (ballPoint.pointf - preCenter).squaredNorm();
        if (!minDistBallPoint || dist < minDist)
        {
          minDist = dist;
          minDistBallPoint = &ballPoint;
        }
      }
    }
    minDistBallPoint->isValid = false;
    --validBallPoints;

    if ((preRadius - (sqrt(minDist) + 2.f)) / preRadius < 0.1f)
    {
      break;
    }
  }

  COMPLEX_DRAWING("module:BallPerceptor:image"){
  for(BallPoint* ballPoint = ballPoints, *end = ballPoints +
      sizeof(ballPoints) / sizeof(*ballPoints); ballPoint < end; ++ballPoint)
  if(ballPoint->isValid)
  {
    DOT2("module:BallPerceptor:image", ballPoint->point.x(), ballPoint->point.y(),
        ColorRGBA(0, 0xff, 0), ColorRGBA(0, 0, 0));
  }
}

  return true;
}

bool BallPerceptor::getBallFromBallPoints(Vector2f& center, float& radius) const
{
  float Mx = 0, My = 0, Mxx = 0, Myy = 0, Mxy = 0, Mz = 0, Mxz = 0, Myz = 0;

  for (const BallPoint* ballPoint = ballPoints, *end = ballPoints
      + sizeof(ballPoints) / sizeof(*ballPoints); ballPoint < end; ++ballPoint)
    if (ballPoint->isValid)
    {
      float x = static_cast<float>(ballPoint->point.x());
      float y = static_cast<float>(ballPoint->point.y());
      float xx = x * x;
      float yy = y * y;
      float z = xx + yy;
      Mx += x;
      My += y;
      Mxx += xx;
      Myy += yy;
      Mxy += x * y;
      Mz += z;
      Mxz += x * z;
      Myz += y * z;
    }

  // Construct and solve matrix
  // Result will be center and radius of ball in theImage.
  Eigen::Matrix3d M;
  M << Mxx, Mxy, Mx, Mxy, Myy, My, Mx, My, validBallPoints;

  Eigen::Matrix3d Minv;
  bool invertible;
  M.computeInverseWithCheck(Minv, invertible);
  if (!invertible)
  {
    return false;
  }
  Eigen::Vector3d BCD = Minv * Eigen::Vector3d(-Mxz, -Myz, -Mz);

  center.x() = static_cast<float>(BCD.x() * -0.5);
  center.y() = static_cast<float>(BCD.y() * -0.5);
  float radicand = static_cast<float>(BCD.x() * BCD.x() / 4.0
      + BCD.y() * BCD.y() / 4.0 - BCD.z());
  if (radicand <= 0.0f)
  {
    return false;
  }
  radius = std::sqrt(radicand);
  return true;
}

bool BallPerceptor::isOnRobots(int x, int y) const
{
  for (const PlayersPercept::Player& player : thePlayersPercept.players)
  {
    if (x > player.x1FeetOnly && x < player.x2FeetOnly && y > player.y1
        && y < player.y2)
    {
      return true;
    }
  }
  return false;
}

bool BallPerceptor::calculateBallOnField(BallPercept& ballPercept) const
{
  const Vector2f correctedCenter = theImageCoordinateSystem.toCorrected(
      ballPercept.positionInImage);
  Vector3f cameraToBall(theCameraInfo.focalLength,
                        theCameraInfo.opticalCenter.x() - correctedCenter.x(),
                        theCameraInfo.opticalCenter.y() - correctedCenter.y());
  cameraToBall.normalize(
      theFieldDimensions.ballRadius * theCameraInfo.focalLength
          / ballPercept.radiusInImage);
  Vector3f rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
  const Vector3f sizeBasedCenterOnField = theCameraMatrix.translation
      + rotatedCameraToBall;
  const Vector3f bearingBasedCenterOnField = theCameraMatrix.translation
      - rotatedCameraToBall
          * ((theCameraMatrix.translation.z() - theFieldDimensions.ballRadius)
              / rotatedCameraToBall.z());

  CIRCLE2("module:BallPerceptor:field", sizeBasedCenterOnField.x(),
          sizeBasedCenterOnField.y(), theFieldDimensions.ballRadius, 1,
          Drawings::solidPen, ColorRGBA(0, 0, 0xff), Drawings::noBrush,
          ColorRGBA());
  CIRCLE2("module:BallPerceptor:field", bearingBasedCenterOnField.x(),
          bearingBasedCenterOnField.y(), theFieldDimensions.ballRadius, 1,
          Drawings::solidPen, ColorRGBA(0xff, 0, 0), Drawings::noBrush,
          ColorRGBA());

  if (rotatedCameraToBall.z() < 0)
  {
    ballPercept.relativePositionOnField.x() = bearingBasedCenterOnField.x();
    ballPercept.relativePositionOnField.y() = bearingBasedCenterOnField.y();
  }
  else
  {
    ballPercept.relativePositionOnField.x() = sizeBasedCenterOnField.x();
    ballPercept.relativePositionOnField.y() = sizeBasedCenterOnField.y();
  }
  return true;
}

bool BallPerceptor::raytraceCircle(const Image& img, Vector2f pos,
                                   Vector2f points[NUM_STAR_SCANLINES],
                                   Vector2f& ballColor,float searchR)
{
  int rayCnt = 0;
  int skipMax = 10;
  int step  = 2;
  int skip = 0;
  skipMax = skipMax<1?1:skipMax;
  skipMax = skipMax>5?5:skipMax;
  skipMax = 5;
  
  for (int dx = -2; dx <= 2; dx++)
  {
    for (int dy = -2; dy <= 2; dy++)
    {
      if (abs(dx) == 2 || abs(dy) == 2)
      {
        int bx = static_cast<int>(pos.x());
        int by = static_cast<int>(pos.y());
        int vx = dx;
        int vy = dy;
        int dist = 0;
        bool borderFound = false;
//        for (dist = 0; dist < searchR; dist++)
        for (dist = 0; dist < 120; dist++)
        {
          bx += vx;
          by += vy;
          if (bx < 2 || bx >= theImage.width - 1)
          {
            vx = -vx;
            continue;
          }
          if (by < 2 || by >= theImage.height - 1)
          {
            vy = -vy;
            continue;
          }
          if (by<theFieldBoundary.getBoundaryY(bx))
              break;
          if (!theColorTable[img[by][bx]].is(ColorClasses::green))
          {
            skip--;
            if (skip<0)
            skip = 0;
            continue;
          }
          else
          {
              skip ++;
              if (skip<=skipMax)
                  continue;
              bx -= skipMax*vx;
              by -= skipMax*vy;
          }
          borderFound = true;
          break;
        }
        if (borderFound)
        {
            if (bx<1 || by<1 || bx>(img.width-1) || by>(img.height))
               points[rayCnt] = Vector2f(-1, -1);
            else 
               points[rayCnt] = Vector2f(bx, by);
        }
        else
        {
          points[rayCnt] = Vector2f(-1, -1);
        }
        rayCnt++;
      }
    }
  }
  return true;
}

Vector2f BallPerceptor::getInnerPoint(Vector2f points[NUM_STAR_SCANLINES],
                                      Vector2f guess)
{
  float maxDist = 0;
  int maxIdx = 0;
  for (int i = 0; i < NUM_STAR_SCANLINES; i++)
  {
    int dx = guess.x() - points[i].x();
    int dy = guess.y() - points[i].y();
    int dist = dx * dx + dy * dy;
    if (dist > maxDist)
    {
      maxDist = dist;
      maxIdx = i;
    }
  }
  return Vector2f((guess.x() + points[maxIdx].x()) / 2,
                    (guess.y() + points[maxIdx].y()) / 2);
}

void BallPerceptor::updateBallColor(const Image& img, Vector2f& ballColor,
                                    Vector2f *points, int n)
{

}

float BallPerceptor::guessDiameter(Vector2f points[NUM_STAR_SCANLINES])
{
  int sum = 0;
  for (int i = 0; i < 8; i++)
  {
    int dx = points[i].x() - points[i + 8].x();
    int dy = points[i].y() - points[i + 8].y();
    int dist = dx * dx + dy * dy;
    sum += dist;
  }
  return sqrtf(sum >> 3);
}

BallPerceptor::circle BallPerceptor::ransacCircle(
    Vector2f points[NUM_STAR_SCANLINE_CANDIDATES], float maxDistEdge)
{
  float max = 0;
  BallPerceptor::circle bestCircle = newCircle(0, 0, 0, 0);
  for (int i = 0; i < NUM_STAR_SCANLINE_CANDIDATES; i++)
  {
    //modell:
    int p1 = i;
    if (points[p1].x() == -1)
      continue;
    int p2 = (i + NUM_STAR_SCANLINES / 4) % (NUM_STAR_SCANLINES * 2);
    if (points[p2].x() == -1)
      continue;
    int p3 = (i + NUM_STAR_SCANLINES / 2) % (NUM_STAR_SCANLINES * 2);
    if (points[p3].x() == -1)
      continue;
    BallPerceptor::circle c = getCircle(points[p1].x(), points[p1].y(),
                                        points[p2].x(), points[p2].y(),
                                        points[p3].x(), points[p3].y());

    //bewerten:
    float sum = 0;
    for (int j = 0; j < NUM_STAR_SCANLINE_CANDIDATES; j++)
    {
      float distx = (c.x - points[j].x());
      float disty = (c.y - points[j].y());
      float r = sqrtf(distx * distx + disty * disty);
      float distR = fabsf(r - c.r);
      if (distR < maxDistEdge)
        sum++;
    }
    if (sum > max)
    {
      max = sum;
      bestCircle = c;
    }
  }
  bestCircle.q = max;
  return bestCircle;
}

BallPerceptor::circle BallPerceptor::getCircle(float x1, float y1, float x2,
                                               float y2, float x3, float y3)
{
  float f2 = x3 * x3 - x3 * x2 - x1 * x3 + x1 * x2 + y3 * y3 - y3 * y2 - y1 * y3
      + y1 * y2;
  float g2 = x3 * y1 - x3 * y2 + x1 * y2 - x1 * y3 + x2 * y3 - x2 * y1;
  float m = 0;
  if (g2 != 0)
    m = (f2 / g2);

  float c = (m * y2) - x2 - x1 - (m * y1);
  float d = (m * x1) - y1 - y2 - (x2 * m);
  float e = (x1 * x2) + (y1 * y2) - (m * x1 * y2) + (m * x2 * y1);

  float x = (c / 2);
  float y = (d / 2);
  float s = (((x) * (x)) + ((y) * (y)) - e);
  float r = pow(s, .5f);
  return newCircle(-x, -y, r, 0);
}

BallPerceptor::circle BallPerceptor::newCircle(float x, float y, float r,
                                               float q)
{
  BallPerceptor::circle c;
  c.x = x;
  c.y = y;
  c.r = r;
  c.q = q;
  return c;
}

BallPerceptor::Ball BallPerceptor::newBall(float x, float y, float radius,
                                           bool found)
{
  BallPerceptor::Ball b;
  b.x = x;
  b.y = y;
  b.radius = radius;
  b.found = found;
  return b;
}
BallPerceptor::Ball BallPerceptor::newBall(float x, float y, float radius,
                                           bool found, float q)
{
  BallPerceptor::Ball b;
  b.x = x;
  b.y = y;
  b.radius = radius;
  b.found = found;
  b.q = q;
  return b;
}

BallPerceptor::Ball BallPerceptor::fitBall(const Vector2f& center, const float searchR, Vector2f& ballColor)
{
//  color Ball;
  BallPerceptor::Ball pBall = newBall(5000, 5000, 0, false, 0);
  float sr = 1.f*searchR;

  Vector2f points1[NUM_STAR_SCANLINES];
  Vector2f points2[NUM_STAR_SCANLINES];
  Vector2f guess = Vector2f(center.x(), center.y());
  Vector2f guess1 = Vector2f(center.x(), center.y()+0.5f*searchR);
  Vector2f guess2 = Vector2f(center.x()-1.732f*0.25f*searchR, center.y()-0.25f*searchR);
  Vector2f guess3 = Vector2f(center.x()+1.732f*0.25f*searchR, center.y()-0.25f*searchR);

  if (theCameraInfo.camera == CameraInfo::lower)
  {
   if (!raytraceCircle(theImage, guess1, points1, ballColor, sr))
    return pBall;
//  Vector2f innerPos = getInnerPoint(points1, guess);
//  innerPos.x*=2;
//  innerPos.y*=2;
//  updateBallColor(theImage, ballColor, points1, NUM_STAR_SCANLINES);

//  Vector2f points2[NUM_STAR_SCANLINES];
  if (!raytraceCircle(theImage, guess2, points2, ballColor, sr))//innerPos
    return pBall;

//  Vector2f innerPos2 = getInnerPoint(points2, innerPos);
//  innerPos2.x*=2;
//  innerPos2.y*=2;
  if (!raytraceCircle(theImage, guess3, points1, ballColor, sr))//innerPos2
    return pBall;
  }
  else
  {
  if (!raytraceCircle(theImage, guess, points1, ballColor, sr))
    return pBall;
  Vector2f innerPos = getInnerPoint(points1, guess);
//  innerPos.x*=2;
//  innerPos.y*=2;
//  updateBallColor(theImage, ballColor, points1, NUM_STAR_SCANLINES);

//  Vector2f points2[NUM_STAR_SCANLINES];
  if (!raytraceCircle(theImage, innerPos, points2, ballColor, sr))//innerPos
    return pBall;

  Vector2f innerPos2 = getInnerPoint(points2, innerPos);
//  innerPos2.x*=2;
//  innerPos2.y*=2;
  if (!raytraceCircle(theImage, innerPos2, points1, ballColor, sr))//innerPos2
    return pBall;
  }
  float diameterGuess = guessDiameter(points2);
//  CIRCLE2("module:BallPerceptor:possibleBalls", innerPos.x, innerPos.y, 10, 1,
//                      Drawings::solidPen, ColorRGBA::blue, Drawings::solidBrush,
//                      ColorRGBA(255, 255, 1, 50));




  Vector2f allPoints[NUM_STAR_SCANLINE_CANDIDATES];
  memcpy(allPoints, points1, sizeof(Vector2f) * NUM_STAR_SCANLINES);
  memcpy(allPoints + NUM_STAR_SCANLINES, points2,
         sizeof(Vector2f) * NUM_STAR_SCANLINES);


  for (int i = 0; i < NUM_STAR_SCANLINE_CANDIDATES; ++i)
  {
    DOT("module:BallPerceptor:edgePoints", allPoints[i].x(), allPoints[i].y(),
        ColorRGBA::yellow, ColorRGBA::yellow);
  }
  #ifdef GRASS_LAND
  BallPerceptor::circle circle = ransacCircle(allPoints,
                                              0.2f + diameterGuess * 0.05f);
  #else
  BallPerceptor::circle circle = ransacCircle(allPoints,
                                              0.2f + diameterGuess * 0.05f);
  #endif

  if (circle.q < 12)
    return pBall;
  if (circle.r > 120 || circle.r < 1)
    return pBall;
  return newBall(circle.x, circle.y, circle.r, true, circle.q);

}

color BallPerceptor::getBallColor(const Image& img, BallSpot guess)
{
  int ballCy = img[(int) guess.position.y()][(int) guess.position.x()].y;
  int ballCb = img[(int) guess.position.y()][(int) guess.position.x()].cb;
  int ballCr = img[(int) guess.position.y()][(int) guess.position.x()].cr;
  color ball =
  { ballCy, ballCb, ballCr };
  return ball;
}

color BallPerceptor::getBallColor(const Image& img, const Vector2f& center)
{
  int ballCy = img[(int) center.y()][(int) center.x()].y;
  int ballCb = img[(int) center.y()][(int) center.x()].cb;
  int ballCr = img[(int) center.y()][(int) center.x()].cr;
  color ball =
  { ballCy, ballCb, ballCr };
  return ball;
}

bool BallPerceptor::Classifier(BallPerceptor::Ball &b,float * val)
{
  //create box of ball
  int left = b.x - b.radius;
  left < 0 ? 0 : left;
  int right = b.x + b.radius;
  right > theImage.width ? theImage.width : right;
  int top = b.y - b.radius;
  top < 0 ? 0 : top;
  int bottom = b.y + b.radius;
  bottom > theImage.height ? theImage.height : height;
  float thre = 0.8f;
 if (theCameraInfo.camera == CameraInfo::lower)
      thre = 0.8f;
  int black_cnt = 0;
  int white_cnt = 0;
  int black_cnt1 = 0;
  int white_cnt1 = 0;
  int black_cnt2 = 0;
  int white_cnt2 = 0;
  int black_cnt3 = 0;
  int white_cnt3 = 0;
  int black_cnt4 = 0;
  int white_cnt4 = 0;
  int non_cnt = 0;
  int total_cnt = 0;
  for (int i = left; i <= right; ++i)
  {
    for (int j = top; j <= bottom; ++j)
    {
      ColorTable::Colors pColor = determineColor(theImage[j][i]);
      if (pColor.is(ColorClasses::white))
      {
        white_cnt++;
//        if (i<b.x)
//        {
//          if (j<b.y)
//            white_cnt1++;
//          else
//            white_cnt3++;
//        }
//        else
//        {
//          if (j<b.y)
//            white_cnt2++;
//          else
//            white_cnt4++;
//        }
      }
      if (pColor.is(ColorClasses::black))
      {
        black_cnt++;
//        if (i<b.x)
//        {
//          if (j<b.y)
//            black_cnt1++;
//          else
//            black_cnt3++;
//        }
//        else
//        {
//          if (j<b.y)
//            black_cnt2++;
//          else
//            black_cnt4++;
//        }
      }
      if (!pColor.is(ColorClasses::green))
      {
        non_cnt++;
      }
      total_cnt++;
    }
  }
  float score1 = float(white_cnt) / float(total_cnt);
  float score2 = float(black_cnt) / float(total_cnt);
  float score3 = float(non_cnt) / float(total_cnt);
//  float score4 = float(std::min((white_cnt1+white_cnt3),(black_cnt2+black_cnt4)))/float(std::max((white_cnt1+white_cnt3),(black_cnt2+black_cnt4)));
//  float score5 = float(std::min((white_cnt1+white_cnt2),(black_cnt3+black_cnt4)))/float(std::max((white_cnt1+white_cnt2),(black_cnt3+black_cnt4)));
//  float score6 = float(std::min((white_cnt1+white_cnt4),(black_cnt2+black_cnt3)))/float(std::max((white_cnt1+white_cnt4),(black_cnt2+black_cnt3)));
//  float score7 = float(std::min((white_cnt2+white_cnt4),(black_cnt1+black_cnt3)))/float(std::max((white_cnt2+white_cnt4),(black_cnt1+black_cnt3)));
//  float score8 = float(std::min((white_cnt3+white_cnt4),(black_cnt1+black_cnt2)))/float(std::max((white_cnt3+white_cnt4),(black_cnt1+black_cnt2)));
//  float score9 = float(std::min((white_cnt2+white_cnt3),(black_cnt1+black_cnt4)))/float(std::max((white_cnt2+white_cnt3),(black_cnt1+black_cnt4)));

  val[0] = score1;
  val[1] = score2;
  val[2] = score3;
  if (score1 < 0.1f || score2 < 0.1f || (val[0]+val[1])/val[2]<thre)
    return false;
  if (score3<0.5f || score3>0.79f)
    return false;
  return true;
}

bool BallPerceptor::distanceClassifier(BallPerceptor::Ball &b, float& v)
{
  int bx = b.x;
  int by = b.y;
  int negThre = -0.1999f;
  if (theCameraInfo.camera == CameraInfo::lower)
      negThre = -0.5f;
  Vector2i PosImg(bx, by);
  Vector2f corrected = theImageCoordinateSystem.toCorrected(PosImg);
  Vector2i RealPos;
  RealPos.x() = static_cast<int>(corrected.x());
  RealPos.y() = static_cast<int>(corrected.y());
  Vector2f position;
  if (!Transformation::imageToRobotWithCameraRotation(RealPos, theCameraMatrix,
                                                      theCameraInfo, position))
  {
    return false;
  }
  else
  {
    float distance = position.norm();
    const int shouldR = static_cast<int>(Geometry::getSizeByDistance(
        theCameraInfo, 50, distance));
    float val = double(b.radius - shouldR) / double(shouldR);
    v = val;
    if (fabs(val) > 1.6f)
      return false;
  }
  return true;
}

float BallPerceptor::ballSpotsClassifier(BallSpot & bs,float * t)
{
    int windowSize = 8;
    int green_cnt = 0;
    int white_cnt = 0;
    int black_cnt = 0;
    Vector2i PosImg(bs.position.x(), bs.position.y());
  Vector2f corrected = theImageCoordinateSystem.toCorrected(PosImg);
  Vector2i RealPos;
  RealPos.x() = static_cast<int>(corrected.x());
  RealPos.y() = static_cast<int>(corrected.y());
  Vector2f position;
  if (!Transformation::imageToRobotWithCameraRotation(RealPos, theCameraMatrix,
                                                      theCameraInfo, position))
  {
    return 1.f;
  }
    float distance = position.norm();
    const int shouldR = static_cast<int>(Geometry::getSizeByDistance(
        theCameraInfo, 50, distance));
    windowSize = shouldR;    
    
    for (int i = bs.position.x()-(windowSize>>1);i<=bs.position.x()+(windowSize>>1);++i)
    {
        for (int j = bs.position.y()-(windowSize>>1);j<=bs.position.y()+(windowSize>>1);++j)
        {
            ColorTable::Colors c = determineColor(theImage[j][i]);
            if (c.is(ColorClasses::green))
                green_cnt ++;
            else if (c.is(ColorClasses::white))
                white_cnt ++;
            else if (c.is(ColorClasses::black))
                black_cnt ++;
        }
    }
    t[0] = float(green_cnt)/float((windowSize+1)*(windowSize+1));
    t[1] = float(white_cnt)/float((windowSize+1)*(windowSize+1));
    t[2] = float(black_cnt)/float((windowSize+1)*(windowSize+1));
    float pen = t[0];
    return pen;
}

float BallPerceptor::otsuThreshold(int * grayImg)
{
  std::vector<int> histogram(256,0);
  for (int y = 0; y < 30; y++)
  {
    for (int x = 0; x < 30; x++)
    {
      int value = grayImg[30*x+y];
      ASSERT(value > 0);
      ASSERT(histogram.size() > static_cast<unsigned int>(value));
      histogram[value]++;
    }
  }

  float meanThreshold = 0;
  int threshold = 0;
  int background = 0;
  int foreground = 0;
  int sumBackground = 0;
  float max = 0;

  for (unsigned int i = 1; i < histogram.size(); i++)
    meanThreshold += (i * histogram[i - 1]);

  for (unsigned int t = 0; t < histogram.size(); t++)
  {
    background += histogram[t];
    if (background == 0)
      continue;

    foreground = (900) - background;
    if (foreground == 0)
      break;

    sumBackground += t * histogram[t];

    float meanDiff = (sumBackground / background - (meanThreshold - sumBackground) / foreground);
    float tmp = meanDiff * meanDiff * static_cast<float>(background * foreground);

    if (tmp > max)
    {
      max = tmp;
      threshold = t;
    }
  }
  
  return threshold;
}

bool BallPerceptor::classifyUsingOtsu(int * grayImg, float thre, float* val)
{
  int black = 0;
  int white = 0;
  int avgBlack = 0;
  int avgWhite = 0;
  for (int i=0;i<30;++i)
  {
      for (int j=0;j<30;++j)
      {
          if(grayImg[30*i+j]>thre)
          {
              white++;
              avgWhite+=grayImg[30*i+j];
          }
          else
          {
              black++;
              avgBlack+=grayImg[30*i+j];
          }
      }
  }
  float b = float(black)/900.f;
  float w = float(white)/900.f;
  if (black == 0)
  {
      avgBlack = 0;
      avgWhite = 0;
  }
  else if (white == 0)
  {
      avgBlack = 0;
      avgWhite = 0;
  }
  else
  {
  avgBlack /= black;
  avgWhite /= white;
  }
  val[0] = b;
  val[1] = w;
  val[2] = avgBlack;
  val[3] = avgWhite;

  return true;
}

bool BallPerceptor::classifyBalls2(BallPerceptor::Ball & b)
{
    //FIXME calculate green pixels inside circle, using average Y of green outside the cicle as BW thre
  int meanY = 0;
  int meanBlack = 0;
  int meanWhite = 0;
  int blackCnt = 0;
  int whiteCnt = 0;
  int greenCnt = 0;
  int total = 0;
  int totalOutCircle = 0;
  int totalInCircle = 0;
  int varY = 0;
  float ratio = 0;
  float ratioGreen = 0;
  float ratioTotal = 0;
  float whitePercent = 0;
  Vector2f cen(b.x,b.y); 
  if (b.x<2 || b.y<2)
      return false;
  if (b.radius<2)
      return false;
  ASSERT(b.radius>1);
  ASSERT(b.x>0&&b.y>0);
    for (int x = std::max(0,b.x-b.radius); x<std::min(theImage.width,b.x+b.radius); ++x)
  {
      for (int y = std::max(0,b.y-b.radius); y<std::min(theImage.height,b.y+b.radius); ++y)
      {
          
          Vector2f cur(x,y);
          if ((cur-cen).norm()>=b.radius)
          {
              totalOutCircle ++;
              if (possibleGreen(theImage[y][x]) || y<theFieldBoundary.getBoundaryY(x))
                  greenCnt ++;
              continue;
          }
          total ++;
          if (!possibleGrayScale(theImage[y][x]))
              continue;
          totalInCircle ++;
          meanY += theImage[y][x].y;
          if (!theColorTable[theImage[y][x]].is(ColorClasses::white))
          {
              blackCnt ++;
              meanBlack += theImage[y][x].y;
          }
          else
          {
              whiteCnt ++;
              meanWhite += theImage[y][x].y;
          }
      }
  }
  if (totalInCircle==0)
      return false;
  ASSERT(totalInCircle!=0);
  ASSERT(totalOutCircle!=0);
  ASSERT(total!=0);
  meanY /= totalInCircle;
  ratioGreen = float(greenCnt)/float(totalOutCircle);
  ratioTotal = float(totalInCircle)/float(total);
  if (blackCnt == 0 || whiteCnt == 0)
  {
      meanBlack = 0;
      meanWhite = 0;
      ratio = 0;
      return false;
  }
  else
  {
      meanBlack /= blackCnt;
      meanWhite /= whiteCnt;
      ratio = float(std::min(blackCnt,whiteCnt))/float(std::max(blackCnt,whiteCnt));
      whitePercent = float(whiteCnt)/float(totalInCircle);
  }
  for (int x = std::max(0,b.x-b.radius); x<std::min(theImage.width,b.x+b.radius); ++x)
  {
      for (int y = std::max(0,b.y-b.radius); y<std::min(theImage.height,b.y+b.radius); ++y)
      {
          if (possibleGreen(theImage[y][x]))
              continue;
          Vector2f cur(x,y);
          if ((cur-cen).norm()>b.radius)
              continue;
          varY += (theImage[y][x].y-meanY)*(theImage[y][x].y-meanY);
      }
  }
  varY /= totalInCircle;

    DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y-30, 10,
                ColorRGBA::gray, "(meanWhite-meanBlack)"<<(meanWhite-meanBlack));
    DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y-20, 10,
                ColorRGBA::cyan, "varY="<<varY);
    DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y-10, 10,
                ColorRGBA::orange, "GrayP="<<ratioTotal);
    DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y-0, 10,
                ColorRGBA::white, "ratio="<<ratio);
    DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y+20, 10,
                ColorRGBA::white, "whitePercent="<<whitePercent);


  if ((meanWhite-meanBlack)<30)
    return false;
  if (varY<300)
    return false;
  if (ratio<0.09f)
    return false;
  if (whitePercent<0.39f)
    return false;
  if (ratioTotal<0.2f)
    return false;

    
  #ifdef GRASS_LAND
//  float Score = tansig(ratio,0.3f)*0.2f+tansig(ratioTotal,0.95f)*0.4f+tansig(ratioGreen,0.5f)*0.4f;//0625good
  float Score = tansig(ratio,0.3f)*0.2f+tansig(ratioTotal,0.95f)*0.4f+tansig(ratioGreen,0.5f)*0.4f;//0625good
  #else
  float Score = tansig(ratio,0.3f)*0.2f+tansig(ratioTotal,1.2f)*0.4f+tansig(ratioGreen,0.5f)*0.4f;
  #endif
    int varYThre = (meanWhite-meanBlack)*10+100;//originally 100
  if (theCameraInfo.camera == CameraInfo::lower)
  {
      varYThre += 200;
  }
  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y+10, 10,
              ColorRGBA::gray, "Score="<<Score);
//  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y-20, 10,
//              ColorRGBA::cyan, "Green="<<ratioGreen);
//  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y-10, 10,
//              ColorRGBA::orange, "GrayP="<<ratioTotal);
//  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y-0, 10,
//              ColorRGBA::white, "White="<<ratio<<"  Perce="<<whitePercent);
//  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y+10, 10,
//              ColorRGBA::white, "MeanB="<<meanBlack-theTJArkVision.seedY<<"  MeanW="<<meanWhite);
//  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y+20, 10,
//              ColorRGBA::white, "MeanY="<<meanY<<"  DvarY="<<varY-varYThre);

// just for test
   float th = otsuThreshold(b);
  DRAWTEXT("module:BallPerceptor:thre", b.x, b.y+15, 10,
              ColorRGBA::cyan, "thre="<<th); 
  if (varY<varYThre)
    return false;
    if ((meanWhite-meanBlack)<30)
    return false;
  if (varY<300)
    return false;
  if (whitePercent<0.39f)
    return false;
  if ((Score<0.75f && theCameraInfo.camera == CameraInfo::upper)
        || (Score<0.85f && theCameraInfo.camera == CameraInfo::lower))
    return false;  


#ifdef DAY_LIGHT
  #ifdef INDOOR
  if (ratioTotal<0.75f)//0.68
  #else
  if (ratioTotal<0.68f)
  #endif
    return false;
  if (Score<0.5f)
    return false;
  if (ratio<0.06f)//originally 0.03
    return false;
  if ((meanBlack-127)>10) //From ColorTable
      return false;
#else
  if (ratioTotal<0.88f)
    return false;
  if (Score<0.5f)
    return false;  
  if (ratio<0.09f)
    return false;  
  if ((meanBlack-127)>4)  //From ColorTable
      return false;
#endif 

    return true;
}

float BallPerceptor::tansig(float per,float ref)
{
    float x = (per)/ref;
    x = x<0?-x:x;
    x = x>1?1:x;
    return 1.f/(1+exp(-4*(2*x-1)));
}

float BallPerceptor::otsuThreshold(BallPerceptor::Ball & ball)
{
  std::vector<int> histogram(256,0);
  int l = static_cast<int>(ball.x-ball.radius);
  l = l<0?0:l;
  int r = static_cast<int>(ball.x+ball.radius);
  r = r>theImage.width?theImage.width:r;
  int t = static_cast<int>(ball.y-ball.radius);
  t = t<0?0:t;
  int b = static_cast<int>(ball.y+ball.radius);
  b = b>theImage.height?theImage.height:b;
  Vector2f cen(ball.x,ball.y);
  int cnt=0;
  for (int x = l; x < r; ++x)
  {
    for (int y = t; y < b; ++y)
    {
      Vector2f pre(x,y);
      if ((pre-cen).norm()>ball.radius)
          continue;
          cnt++;
      int value = theImage[y][x].y;
      ASSERT(value >= 0);
      ASSERT(histogram.size() > static_cast<unsigned int>(value));
      histogram[value]++;
    }
  }

  float meanThreshold = 0;
  int threshold = 0;
  int background = 0;
  int foreground = 0;
  int sumBackground = 0;
  float max = 0;
  float total = std::accumulate(std::begin(histogram),std::end(histogram),0.f);
  for (unsigned int i = 1; i < histogram.size(); i++)
    meanThreshold += (i * histogram[i - 1]);

  for (unsigned int t = 0; t < histogram.size(); t++)
  {
    background += histogram[t];
    if (background == 0)
      continue;

    foreground = (total) - background;
    if (foreground == 0)
      break;

    sumBackground += t * histogram[t];

    float meanDiff = (sumBackground / background - (meanThreshold - sumBackground) / foreground);
    float tmp = meanDiff * meanDiff * static_cast<float>(background * foreground);

    if (tmp > max)
    {
      max = tmp;
      threshold = t;
    }
  }
  
  return threshold;
}
