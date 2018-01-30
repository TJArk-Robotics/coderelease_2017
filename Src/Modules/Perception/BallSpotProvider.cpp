#include "BallSpotProvider.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <chrono>
#include <iostream> 
//#define SUNSHINE
using namespace std::chrono;

bool BallSpotProvider::isWhite(const Image::Pixel* p) const
{
	if (theColorTable[*p].is(ColorClasses::white))
        return true;
	else
		return false;
}

ColorTable::Colors BallSpotProvider::determineColor(
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

bool BallSpotProvider::possibleGreen(const Image::Pixel& p) const
{
	if (theColorTable[p].is(ColorClasses::green))
		return true;
    return false;
}

void BallSpotProvider::update(BallSpots& ballSpots)
{

  high_resolution_clock::time_point startT,endT;
  DECLARE_DEBUG_DRAWING("module:BallSpotProvider:ballSpotScanLines",
                        "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallSpotProvider:isInRobot", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallSpotProvider:ballspot", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallSpotProvider:ballspotcolor",
                          "drawingOnImage");
  ballSpots.ballSpots.clear();
//  startT = high_resolution_clock::now();
  searchScanLines(ballSpots);
//  endT = high_resolution_clock::now();
//  duration<double> dTotal = duration_cast<duration<double>>(endT-startT);
//  std::cout << "BallSpot=" << dTotal.count()*1000 <<"ms"<< std::endl;
}

bool BallSpotProvider::searchScanLines(BallSpots& ballSpots) const
{
  bool found = false;
  for (const Scanline& line : theScanlineRegionsClipped.scanlines)
  {
    for (const Region& region : line.regions)
    {
      ASSERT(region.upper >= 0 && region.upper < theImage.height);
      ASSERT(region.lower > 0 && region.lower <= theImage.height);

      /** not green and (is none or white) */
      if (!region.color.is(ColorClasses::green)
          && (region.color.is(ColorClasses::none)
              || region.color.is(ColorClasses::white)
			  || region.color.is(ColorClasses::black)
              ))
      {
        const int y = static_cast<int>((region.upper + region.lower) / 2);
        ASSERT(static_cast<int>(line.x) < theImage.width);
        ASSERT(y >= 0 && y < theImage.height);
        if (isOnRobots(line.x, y))
        {
          continue;
        }
        BallSpot bs(line.x, y);
        Vector2i PosImg(line.x, y);
        Vector2f corrected = theImageCoordinateSystem.toCorrected(PosImg);
        Vector2i RealPos;
        RealPos.x() = static_cast<int>(corrected.x());
        RealPos.y() = static_cast<int>(corrected.y());
        Vector2f position;
        if (!Transformation::imageToRobotWithCameraRotation(RealPos, theCameraMatrix,
                                                          theCameraInfo, position))
        {
          continue;
        }
        float dis = position.norm();
        const int shouldR = static_cast<int>(Geometry::getSizeByDistance(
            theCameraInfo, 50, dis));
        bs.shouldR = shouldR;
        bs.distance = dis;
        DOT("module:BallSpotProvider:ballspot", bs.position.x(), bs.position.y(),
                            ColorRGBA::yellow, ColorRGBA::yellow);
        if (getBallSpot(bs) && !isOnRobots(bs.position.x(), bs.position.y()))
        {
          found = true;
          //if (bs.position.y()>theFieldBoundary.getBoundaryY(bs.position.x()))
          if(1)
          {
            getBallSpotColor(bs);
            ballSpots.ballSpots.emplace_back(bs);
            DOT("module:BallSpotProvider:ballspot", bs.position.x(), bs.position.y(),
                    ColorRGBA::yellow, ColorRGBA::yellow);
          }
        }
      }
    }
  }
  return found;
}

bool BallSpotProvider::getBallSpot(BallSpot& ballSpot) const
{
  const int height = theImage.height - 3;
  int horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
  horizon = std::min(horizon, height);
  const int leftLimit = 2;
//  const unsigned int orangeSkipping = 5;
  int orangeSkipping = ballSpot.shouldR/5;
  orangeSkipping = orangeSkipping<1?1:orangeSkipping;
  orangeSkipping = orangeSkipping>6?6:orangeSkipping;
  const int rightLimit = theImage.width - 3;
  float factor = 1.f;
  float minR = factor*ballSpot.shouldR;
  float maxR = 1.f/factor*ballSpot.shouldR;
  
  unsigned int skipped = 0;
  int lower, upper;
  int left = 0;
  int right = 0;
  // find upper/lower => middle vertical
  lower = upper = ballSpot.position.y();
  skipped = 0;
  while (lower <= height && skipped < orangeSkipping)
  {
    if((theColorTable[theImage[lower][ballSpot.position.x()]].is(ColorClasses::none) ||
    		theColorTable[theImage[lower][ballSpot.position.x()]].is(ColorClasses::black) ||
        theColorTable[theImage[lower][ballSpot.position.x()]].is(ColorClasses::white)) &&
        !theColorTable[theImage[lower][ballSpot.position.x()]].is(ColorClasses::green))
    {
      skipped = 0;
    }
    else
    {
      skipped++;
    }
    lower++;
  }
  lower -= skipped;

  skipped = 0;
  while (upper >= horizon && skipped < orangeSkipping)
  {
    if((theColorTable[theImage[upper][ballSpot.position.x()]].is(ColorClasses::none) ||
    		theColorTable[theImage[lower][ballSpot.position.x()]].is(ColorClasses::black) ||
        theColorTable[theImage[upper][ballSpot.position.x()]].is(ColorClasses::white)) &&
        !theColorTable[theImage[upper][ballSpot.position.x()]].is(ColorClasses::green))

    {
      skipped = 0;
    }
    else
    {
      skipped++;
    }
    upper--;
  }
  upper += skipped + 1;
  ballSpot.position.y() = (lower + upper) / 2;
  LINE("module:BallSpotProvider:ballSpotScanLines", ballSpot.position.x(),
       lower, ballSpot.position.x(), upper, 1, Drawings::solidPen,
       ColorRGBA::blue);
  // find left/right => middle horizontal
  left = right = ballSpot.position.x();
  skipped = 0;
  while (left >= leftLimit && skipped < orangeSkipping)
  {
    if((theColorTable[theImage[ballSpot.position.y()][left]].is(ColorClasses::none) ||
    		theColorTable[theImage[lower][ballSpot.position.x()]].is(ColorClasses::black) ||
        theColorTable[theImage[ballSpot.position.y()][left]].is(ColorClasses::white)) &&
        !theColorTable[theImage[ballSpot.position.y()][left]].is(ColorClasses::green))
    {
      skipped = 0;
    }
    else
    {
      skipped++;
    }
    left--;
  }
  left += skipped + 1;
  skipped = 0;
  while (right <= rightLimit && skipped < orangeSkipping)
  {
    if((theColorTable[theImage[ballSpot.position.y()][right]].is(ColorClasses::none) ||
    		theColorTable[theImage[lower][ballSpot.position.x()]].is(ColorClasses::black) ||
        theColorTable[theImage[ballSpot.position.y()][right]].is(ColorClasses::white)) &&
        !theColorTable[theImage[ballSpot.position.y()][right]].is(ColorClasses::green))
    {
      skipped = 0;
    }
    else
    {
      skipped++;
    }
    right++;
  }
  right -= skipped;
  ballSpot.position.x() = (left + right) / 2;
  LINE("module:BallSpotProvider:ballSpotScanLines", left, ballSpot.position.y(),
       right, ballSpot.position.y(), 1, Drawings::solidPen, ColorRGBA::blue);
  const int minBallSpotRadius = 12;
  return true;
  return (right - left) >= 0.1f*minR && (right - left) <= 2.5f*maxR
      && (lower - upper) >= 0.1f*minR && (lower - upper) <= 2.5f*maxR;
}

void BallSpotProvider::getBallSpotColor(BallSpot& bs) const
{
    std::vector<int> colorH;
    std::vector<int> colorS;
    std::vector<int> colorY;
    colorH.clear();
    colorS.clear();
    colorY.clear();
    float searchR = 0.71f*bs.shouldR;
    int l = static_cast<int>(bs.position.x()-searchR);
    l = l<1?1:l;
    int r = static_cast<int>(bs.position.x()+searchR);
    r = r>(theImage.width-1)?(theImage.width-1):r;
    int t = static_cast<int>(bs.position.y()-searchR);
    t = t<1?1:t;
    int b = static_cast<int>(bs.position.y()+searchR);
    b = b>(theImage.height-1)?(theImage.height-1):b;
    int step = 2;

    if (bs.distance>4000.f)
        step = 1;
    for (int x=l; x<r; x += step)
    {
        for (int y=t; y<b; y += step)
        {
        	if((x - bs.position.x()) * (x - bs.position.x()) + (y - bs.position.y()) * (y - bs.position.y()) > searchR * searchR)
        		continue;
            if (possibleGreen(theImage[y][x]))
                continue;
            Image::Pixel ss;
            //theColorTable.getHSV(theImage[y][x],ss);
            colorH.push_back(0);
            colorS.push_back(0);
            colorY.push_back(0);
        }
    }
    float avgH = std::accumulate(std::begin(colorH),std::end(colorH),0.f)/(static_cast<float>(colorH.size()));
    float avgS = std::accumulate(std::begin(colorS),std::end(colorS),0.f)/(static_cast<float>(colorS.size())); 
    float avgY = std::accumulate(std::begin(colorY),std::end(colorY),0.f)/(static_cast<float>(colorY.size()));
    bs.colorH = static_cast<int>(avgH);
    bs.colorS = static_cast<int>(avgS);
    bs.colorY = static_cast<int>(avgY);
}

bool BallSpotProvider::isOnRobots(int x, int y) const
{
	return false;
    #ifdef SUNSHINE
    return false;
    #else
  /** @TODO if robot is fallen? */
  const float feetPercent = 0.f;
  for (const PlayersPercept::Player& player : thePlayersPercept.players)
  {
    if ((x > player.x1FeetOnly && x < player.x2FeetOnly)
        && (y > player.y1
            && y < (player.y2 - (float) (player.y2 - player.y1) * feetPercent)))
    {
      return true;
    }
  }
  return false;
  #endif
}

MAKE_MODULE(BallSpotProvider, perception)
