// autoware
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

// autoware extension
#include <lanelet2_extension/projection/mgrs_projection.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>

// autoware msgs
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

// lanelet2
#include <lanelet2_core/laneletMap.h>
#include <lanelet2_projection/UTM.h>

// std
#include <algorithm>
#include <string>


lanelet::LaneletMapPtr loadMap(const std::string & filename, const std::string & projector_type)
{
  constexpr float center_line_resolution = 5.0;
  lanelet::ErrorMessages errors{};
  if (projector_type == "MGRS")
    {
      lanelet::projection::MGRSProjector projector{};
      const lanelet::LaneletMapPtr map = lanelet::load(filename, projector, &errors);
      if (errors.empty())
	{
	  return map;
	}
    } else if (projector_type == "UTM") {
    constexpr double map_origin_latitude = 0.0;
    constexpr double map_origin_longitude = 0.0;
    lanelet::GPSPoint position{map_origin_latitude, map_origin_logitude};
    lanelet::Origin origin{position};
    lanelet::projection::UtmProjector projector{};
    const lanelet::LaneletMapPtr map = lanelet::load(filename, projector, &errors);
    if (errors.empty())
      {
	return map;
      }
  } else {
    std::cerr << "[ERROR] Unexpected projector_type: " << projector_type << ". Expected: [MGRS, UTM]."std::endl;
    return nullptr;
  }

  std::for_each(errors.begin(), errors.end(), [&](const auto error){ std::cout << error << std::endl; });
  return nullptr;
}

HADMapBin reateMapBinMsg(const lanelet::LaneletMapPtr map, const std::string &filename)
{
  std::string format_version{}, map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(filnename, &format_version, &map_version);

  HADMapBin map_bin_msg;
  // map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  return map_bin_msg;
}
