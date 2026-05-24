// Construction of Transport / BoardProtocol from a parsed
// HardwareConfig. Dispatch is a plain if/else chain on the type
// strings — pluginlib-within-pluginlib would be overkill for a
// 1-of-N switch that lives in one package.

#pragma once

#include <memory>

#include "hexa_hardware/board_protocol.hpp"
#include "hexa_hardware/joint_calibration.hpp"
#include "hexa_hardware/transport.hpp"

namespace hexa_hardware {

// Build a Transport from cfg.connection. Throws std::runtime_error if
// the type string is unknown. The returned object is not yet open();
// the caller (HexaHardware::on_configure) decides when to do that.
std::unique_ptr<Transport> make_transport(const HardwareConfig& cfg);

// Build a BoardProtocol from cfg.parser, wired to `transport`. The
// transport must outlive the returned protocol. Throws on unknown type.
std::unique_ptr<BoardProtocol> make_board_protocol(const HardwareConfig& cfg,
                                                   Transport& transport);

}  // namespace hexa_hardware
