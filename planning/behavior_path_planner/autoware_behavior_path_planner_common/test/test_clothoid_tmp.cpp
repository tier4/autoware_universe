#include <boost/geometry/geometry.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

int main() {
  std::ofstream svg("/tmp/debug.svg");
  boost::geometry::svg_mapper<lanelet::BasicPoint2d> mapper(svg, 800, 800);


  const auto in = lanelets.front().polygon2d().basicPolygon();
  lanelet::BasicPolygons2d out;
  namespace bg = boost::geometry;
  constexpr auto buffer_distance = 2.5;
  bg::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance);
  bg::strategy::buffer::join_miter join_strategy;
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::point_square point_strategy;
  bg::strategy::buffer::side_straight side_strategy;
  boost::geometry::buffer(in, out, distance_strategy, side_strategy,
            join_strategy, end_strategy, point_strategy);
  mapper.add(in);
  mapper.add(out.front());
  mapper.map(in, "opacity:0.5;fill-opacity:0.0;fill:red;stroke:red;stroke-width:1");
  mapper.map(out.front(), "opacity:0.5;fill-opacity:0.0;fill:blue;stroke:blue;stroke-width:1");
}