#ifndef _GRIDWORLD_H_
#define _GRIDWORLD_H_

#include <common/Random.h>

#include <iostream>
#include <vector>

/** \file
    Declarations supporting GridWorld */

/** A representation of a rectangular grid with interior walls that
    allows users easily to determine available directions of travel in
    each cell. NOTE: There are no walls around the grid. */
class GridWorld {
public:
  /** Creates a gridworld using the given wall occupancy matrices.
      \param height Height of the gridworld.
      \param width  Width of the gridworld.
      \param northsouth Whether each interior wall blocking NS
                        movement exists, organized first by [w]
                        columns and then by [h-1] rows.
      \param eastwest Whether each interior wall blocking EW movement
                      exists, organized first by [h] rows and then by
                      [w-1] columns. */
  GridWorld(const unsigned height, const unsigned width,
            const std::vector<std::vector<bool> > &northsouth,
            const std::vector<std::vector<bool> > &eastwest);

  /** Creates a random gridworld with the desired dimensions.  */
  GridWorld(const unsigned height, const unsigned width, Random &rng);

  unsigned height() const { return h; }
  unsigned width() const { return w; }

  /** Checks if a wall blocks movement in a given direction from a
      given coordinate.
      \param nsCoord The coordinate along the NS direction.
      \param ewCoord The coordinate along the EW direction.
      \param dir The direction in which to check movement.  0 is
                 north, 1 is south, 2 is east, 3 is west.  */
  bool wall(unsigned nsCoord, unsigned ewCoord, unsigned dir) const;

  friend std::ostream &operator<<(std::ostream &out, const GridWorld &g);

protected:
  /** Attempts to add a random wall that must not touch any other
      interior wall. */
  void addObstacle(Random &rng);

  /** Given a segment of wall that could be built, builds a subset of
      it of random length.  Always builds from one of the two ends.  */
  void chooseSegment(unsigned first,
                     unsigned last,
                     unsigned j,
                     std::vector<std::vector<bool> > &parallel,
                     Random &rng);

private:
  /** Determines if the "smaller" endpoint of the given line segment
      is clear: none of the four possible walls using that endpoint
      exist.
      \param i An index into parallel.
      \param j An index into parallel[i].
      \param parallel Occupancy matrix for walls parallel to the wall
                      under consiration.
      \param perpendicular Occupancy matrix for walls perpendicular to
                           the wall under consiration.  */
  bool isClear(const unsigned i, const unsigned j,
               const std::vector<std::vector<bool> > &parallel,
               const std::vector<std::vector<bool> > &perpendicular) const;

  const unsigned h;
  const unsigned w;

  /** The occupancy matrix for the walls that obstruct NS
      movement. Element i,j is true if in the ith column the jth
      east-west wall from the bottom is present. */
  std::vector<std::vector<bool> > ns;

  /** The same as for ns but different. */
  std::vector<std::vector<bool> > ew;
};

#endif
