const std = @import("std");
const bfe = @import("bfe");

//-----------------------------------------------------------------------------//
//   Init / DeInit
//-----------------------------------------------------------------------------//

//-----------------------------------------------------------------------------//
//   Getter / Setter
//-----------------------------------------------------------------------------//

//-----------------------------------------------------------------------------//
//   Processing
//-----------------------------------------------------------------------------//

//-----------------------------------------------------------------------------//
//   Internal
//-----------------------------------------------------------------------------//

const log_road = std.log.scoped(.road);

var gpa = if (bfe.cfg.debug_allocator == true) std.heap.GeneralPurposeAllocator(.{ .verbose_log = true }){} else std.heap.GeneralPurposeAllocator(.{}){};
const allocator = gpa.allocator();

const RoadgraphIdType = u64;

const connections_max = 4;

const Nodes = struct {
    id: RoadgraphIdType = 0,
    connections: [connections_max]RoadgraphIdType = .{0},
    // position
}; 

const Edges = struct {
    id: RoadgraphIdType = 0,
    neighbor_left: RoadgraphIdType = 0,
    neighbor_right: RoadgraphIdType = 0,
};

//-----------------------------------------------------------------------------//
//   Tests
//-----------------------------------------------------------------------------//
