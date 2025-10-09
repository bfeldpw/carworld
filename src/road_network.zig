const std = @import("std");

const bfe = @import("bfe");
const gfx_base = bfe.gfx.base;
const idt = bfe.util.id_type;

const id_t = idt.IdType();


//-----------------------------------------------------------------------------//
//   Init / DeInit
//-----------------------------------------------------------------------------//

pub fn init() !void {
    var n0 = Node{.id = id_t.create(),
                  .pos = .{-100.0, -10.0}
                 };

    const id_root: id_t = n0.id; 
    var n1 = Node{.id = id_t.create(),
                  .pos = .{100.0, -30.0}
                 };
    const n2 = Node{.id = id_t.create(),
                    .pos = .{120.0, 0.0}
                   };
    const e0 = Edge{.id = id_t.create(),
                    .id_node_from = n0.id,
                    .id_node_to = n1.id,
                    .angle = std.math.atan2(n1.pos[1] - n0.pos[1],
                                            n1.pos[0] - n0.pos[0])
                   };
    const e1 = Edge{.id = id_t.create(),
                    .id_node_from = n1.id,
                    .id_node_to = n2.id,
                    .angle = std.math.atan2(n2.pos[1] - n1.pos[1],
                                            n2.pos[0] - n1.pos[0])
                   };
    n0.id_edges[0] = e0.id;
    n1.id_edges[0] = e1.id;
    n1.angles[0] = e1.angle - e0.angle;
    log_road.debug("edge2edge angle = {d:.1}", .{std.math.radiansToDegrees(n1.angles[0])});
    try nodes.put(n0.id.v, n0);
    try nodes.put(n1.id.v, n1);
    try nodes.put(n2.id.v, n2);
    try edges.put(e0.id.v, e0);
    try edges.put(e1.id.v, e1);

    // Graphics
    id_buf_edges = try gfx_base.addBuffer(12, .PxyCrgbaF32, null);

    const grey = 0.025;

    const width = 3.0; // m
    var id_next: id_t = id_root;
    {
        const data = try gfx_base.getBufferToAddVertexData(id_buf_edges, 2 * 6);
        const n_next = nodes.get(id_next.v).?;
        data[0] = n_next.pos[0] + width * 0.5 * @sin(edges.get(n_next.id_edges[0].v).?.angle);
        data[1] = n_next.pos[1] - width * 0.5 * @cos(edges.get(n_next.id_edges[0].v).?.angle);
        data[2] = grey;
        data[3] = grey;
        data[4] = grey;
        data[5] = 1.0;
        data[6] = n_next.pos[0] - width * 0.5 * @sin(edges.get(n_next.id_edges[0].v).?.angle);
        data[7] = n_next.pos[1] + width * 0.5 * @cos(edges.get(n_next.id_edges[0].v).?.angle);
        data[8] = grey;
        data[9] = grey;
        data[10] = grey;
        data[11] = 1.0;
        id_next = n_next.id_edges[0];
    }
    while (id_next.is_valid()) {
        id_next = edges.get(id_next.v).?.id_node_to;
        const data = try gfx_base.getBufferToAddVertexData(id_buf_edges, 2 * 6);
        const n_next = nodes.get(id_next.v).?;
        data[0] = n_next.pos[0] - width * 0.5;
        data[1] = n_next.pos[1];
        data[2] = grey;
        data[3] = grey;
        data[4] = grey;
        data[5] = 1.0;
        data[6] = n_next.pos[0] + width * 0.5;
        data[7] = n_next.pos[1];
        data[8] = grey;
        data[9] = grey;
        data[10] = grey;
        data[11] = 1.0;
        id_next = n_next.id_edges[0];
    }
}

pub fn deinit() void {
    edges.deinit();
    nodes.deinit();

    const leaked = gpa.deinit();
    if (leaked == .leak) log_road.err("Memory leaked in GeneralPurposeAllocator", .{});
}

//-----------------------------------------------------------------------------//
//   Getter / Setter
//-----------------------------------------------------------------------------//

//-----------------------------------------------------------------------------//
//   Processing
//-----------------------------------------------------------------------------//

pub fn render() !void {
    try gfx_base.renderBatch(id_buf_edges, .TriangleStrip, .Keep);
}

//-----------------------------------------------------------------------------//
//   Internal
//-----------------------------------------------------------------------------//

const log_road = std.log.scoped(.road);

var gpa = if (bfe.cfg.debug_allocator == true) std.heap.GeneralPurposeAllocator(.{ .verbose_log = true }){} else std.heap.GeneralPurposeAllocator(.{}){};
const allocator = gpa.allocator();

var edges: std.AutoArrayHashMap(idt.ValueType, Edge) =
    std.AutoArrayHashMap(idt.ValueType, Edge).init(allocator);
var nodes: std.AutoArrayHashMap(idt.ValueType, Node) =
    std.AutoArrayHashMap(idt.ValueType, Node).init(allocator);

const Vec2d = @Vector(2, f32);
const connections_max = 4;

const Node = struct {
    id: id_t = .{},
    id_edges: [connections_max]id_t = [_]id_t{.{}} ** connections_max,
    angles: [connections_max]f32 = [_]f32{0} ** connections_max,
    pos: Vec2d = .{0, 0},
}; 

const Edge = struct {
    id: id_t = .{},
    id_node_from: id_t = .{},
    id_node_to: id_t = .{},
    id_neighbor_left: id_t = .{},
    id_neighbor_right: id_t = .{},
    angle: f32 = 0.0
};

var id_buf_edges: u32 = 0;

//-----------------------------------------------------------------------------//
//   Tests
//-----------------------------------------------------------------------------//
