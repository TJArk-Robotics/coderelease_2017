#pragma once

#include <iostream>

#include "Tools/Module/Module.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ScanlineRegions.h"
#include "Representations/Perception/PlayersPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/LinePercept.h"

MODULE(BallSpotProvider,
{,
  REQUIRES(ColorTable),
  REQUIRES(Image),
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(CameraInfo),
  REQUIRES(FieldBoundary),
  REQUIRES(ScanlineRegionsClipped),
  REQUIRES(LinePercept),
  REQUIRES(PlayersPercept),
  PROVIDES(BallSpots),
});

/**
* @class BallSpotProvider
* A module that provides spots that might be inside balls
*/
class BallSpotProvider: public BallSpotProviderBase
{
private:
  void update(BallSpots& ballSpots);
  using Scanline = ScanlineRegionsClipped::Scanline;
  using Region = ScanlineRegionsClipped::Region;

  /**searches the scanlines for ball spots*/
  bool searchScanLines(BallSpots& ballSpots) const;

  /**Tries to find the center of a Ball*/
  bool getBallSpot(BallSpot& ballSpot) const;

  /**Check point is on Robots' body*/
  bool isOnRobots(int x, int y) const;
  
  /**Get ball Color */
  void getBallSpotColor(BallSpot& bs) const;

  ColorTable::Colors determineColor(const Image::Pixel & p) const;
  bool possibleGreen(const Image::Pixel& p) const;

  bool isWhite(const Image::Pixel* p) const;

};
