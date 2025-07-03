const std = @import("std");
const bfe = @import("bfe");
const cfg = bfe.cfg;
const cfg_gfx = bfe.gfx.cfg;
const gfx_core = bfe.gfx.core;
const gfx_base = bfe.gfx.base;

//-----------------------------------------------------------------------------//
//   Error Sets / Enums / Types
//-----------------------------------------------------------------------------//
pub const GraphicsDataType = struct {
    render_mode: gfx_base.RenderMode = .PxyCuniF32,
    render_mode_user_data: ?gfx_base.RenderModeUserData = null,
    primitive_mode: gfx_core.PrimitiveMode = .LineLoop,
    draw_mode: gfx_core.DrawMode = .Dynamic,
    attrib_size: u32 = 2,
    vao: u32 = 0,
    vbo: u32 = 0,
    ebo: u32 = 0,
    ebo_capacity: u32 = 0,
    vbo_capacity: u32 = 0,
    data0_buf: std.ArrayList(f32) = std.ArrayList(f32).init(allocator),
    data_buf: std.ArrayList(f32) = std.ArrayList(f32).init(allocator),
    ebo_buf: std.ArrayList(u32) = std.ArrayList(u32).init(allocator),
    data_free: std.ArrayList(IndexRange) = std.ArrayList(IndexRange).init(allocator),
    ebo_free: std.ArrayList(IndexRange) = std.ArrayList(IndexRange).init(allocator)
};

pub const ObjectDataType = struct {
    pub const GfxSource = struct {
        data: *GraphicsDataType = undefined,
        i_data: IndexRange,
        i_ebo: IndexRange
    };
    n_src: u32 = 0,
    src: [GfxSourcesMax]GfxSource = undefined,
    age: f32 = 0.0,
    is_free: bool = true
};

//-----------------------------------------------------------------------------//
//   Init / DeInit
//-----------------------------------------------------------------------------//

pub fn init() !void {
    // It is crucial to reserve enough memory to avoid resizing, i.e.
    // changing the memory addresses, since Box2Ds userData points to
    // it.
    // There's plenty of room with 1_000_000 objects (they are reused as well)
    // which account fors 20MiB of memory. If exceeding the limit nonetheless
    // no new objects will be created if using the helper function
    // "gfx_ext.appendObject" consistently.
    try objects.ensureTotalCapacity(1_000_000);
    try gfx_core.enablePrimitiveRestart();
}

pub fn deinit() void {
    log_gfx.info("objects new: {}", .{objects_nr_new});
    log_gfx.info("objects removed: {}", .{objects_nr_removed});
    log_gfx.info("objects reused: {}", .{objects_nr_reused});
    log_gfx.info("reusable object slots: {}", .{objects_free.items.len});
    log_gfx.info("data new: {}", .{data_nr_new});
    log_gfx.info("data reused: {}", .{data_nr_reused});
    log_gfx.info("ebo new: {}", .{ebo_nr_new});
    log_gfx.info("ebo reused: {}", .{ebo_nr_reused});

    objects.deinit();
    objects_free.deinit();

    const leaked = gpa.deinit();
    if (leaked == .leak) log_gfx.err("Memory leaked in GeneralPurposeAllocator", .{});
}

//-----------------------------------------------------------------------------//
//   Create
//-----------------------------------------------------------------------------//

pub fn createObjectData() !*ObjectDataType {
    var obj: *ObjectDataType = undefined;
    if (objects_free.items.len > 0) {
        const obj_ptr = objects_free.pop();
        if (obj_ptr != null) {
            obj = obj_ptr.?;
            objects_nr_reused += 1;
        } else {
            log_gfx.warn("Cannot get free object although list is not empty, something is wrong here. Creating new object", .{});
            obj = try appendObject(.{});
            objects_nr_new += 1;
        }
    } else {
        obj = try appendObject(.{});
        objects_nr_new += 1;
    }
    obj.age = 0.0;
    obj.n_src = 0;
    obj.is_free = false;
    return obj;
}

pub fn createObjectDataAddSrc(obj: *ObjectDataType, g: *GraphicsDataType, n: u32) !void {
    const i_src = obj.n_src;

    const n_data = n * g.attrib_size;
    const n_ebo = n + 1;

    var i_free: u32 = 0;
    var is_reused: bool = false;
    while (i_free < g.data_free.items.len and is_reused == false) : (i_free += 1) {
        if (g.data_free.items[i_free].r1 - g.data_free.items[i_free].r0 + 1 == n_data) {
            obj.src[i_src].i_data = g.data_free.items[i_free];
            _ = g.data_free.swapRemove(i_free);
            is_reused = true;
            data_nr_reused += 1;
        }
    }
    if (!is_reused) {
        obj.src[i_src].i_data.r0 = @intCast(g.data0_buf.items.len);
        obj.src[i_src].i_data.r1 = @intCast(g.data0_buf.items.len + n_data - 1);
        try g.data0_buf.resize(g.data0_buf.items.len + n_data);
        try g.data_buf.resize(g.data_buf.items.len + n_data);
        data_nr_new += 1;
    }

    i_free = 0;
    is_reused = false;
    while (i_free < g.ebo_free.items.len and is_reused == false) : (i_free += 1) {
        if (g.ebo_free.items[i_free].r1 - g.ebo_free.items[i_free].r0 + 1 == n_ebo) {
            obj.src[i_src].i_ebo = g.ebo_free.items[i_free];
            _ = g.ebo_free.swapRemove(i_free);
            is_reused = true;
            ebo_nr_reused += 1;
        }
    }
    if (!is_reused) {
        obj.src[i_src].i_ebo.r0 = @intCast(g.ebo_buf.items.len);
        obj.src[i_src].i_ebo.r1 = @intCast(g.ebo_buf.items.len + n_ebo - 1);
        try g.ebo_buf.resize(g.ebo_buf.items.len + n_ebo);
        ebo_nr_new += 1;
    }

    obj.src[i_src].data = g;
    obj.n_src += 1;
}

pub fn createObjectCircleGfx(obj: *ObjectDataType, i_src: u32, c_x: f32, c_y: f32, r: f32, n: u32) void {
    const gfx = obj.src[i_src].data;

    var i: u32 = 0;
    var a: f32 = 0.0;
    while (a < 2.0 * std.math.pi * 0.99) : (a += 2.0 * std.math.pi / @as(f32, @floatFromInt(n))) {
        gfx.data0_buf.items[obj.src[i_src].i_data.r0 + i]     = c_x + r * @cos(a);
        gfx.data0_buf.items[obj.src[i_src].i_data.r0 + i + 1] = c_y + r * @sin(a);

        i += gfx.attrib_size;
    }
    const r0 = obj.src[i_src].i_data.r0;
    const r1 = obj.src[i_src].i_data.r1 + 1;
    @memcpy(gfx.data_buf.items[r0..r1], gfx.data0_buf.items[r0..r1]);

    const i_f: u32 = obj.src[i_src].i_data.r0 / gfx.attrib_size;
    i = 0;
    while (i < n) : (i += 1) {
        gfx.ebo_buf.items[obj.src[i_src].i_ebo.r0 + i] = i_f + i;
    }
    gfx.ebo_buf.items[obj.src[i_src].i_ebo.r0 + i] = gfx_core.PRI;
}

pub fn createObjectPolygonGfx(obj: *ObjectDataType, i_src: u32, v: []f32) void {
    const n = v.len;

    const g = obj.src[i_src].data;
    const n_v = n / g.attrib_size;

    var i: u32 = 0;
    while (i < n) : (i += 1) {
        g.data0_buf.items[obj.src[i_src].i_data.r0 + i] = v[i];
        g.data_buf.items[obj.src[i_src].i_data.r0 + i] = v[i];
    }

    const i_e: u32 = obj.src[i_src].i_data.r0 / g.attrib_size;
    i = 0;
    while (i < n_v) : (i += 1) {
        g.ebo_buf.items[obj.src[i_src].i_ebo.r0 + i] = i_e + i;
    }
    g.ebo_buf.items[obj.src[i_src].i_ebo.r0 + i] = gfx_core.PRI;
}

pub fn initBatch(g: *GraphicsDataType, cap_data: u32, cap_ebo: u32) !void {
    // Generate buffers
    g.vao = try gfx_core.genVAO();
    g.vbo = try gfx_core.genBuffer();
    try gfx_core.bindVAO(g.vao);
    try gfx_core.bindVBO(g.vbo);
    _ = try gfx_base.setVertexAttributes(g.render_mode, g.render_mode_user_data);
    g.ebo = try gfx_core.genBuffer();
    // If cap is 0, just append when initialising, do not expect increase in
    // simulation loop
    if (cap_data != 0) {
        try g.data0_buf.ensureTotalCapacity(cap_data);
        try g.data_buf.ensureTotalCapacity(cap_data);
        log_gfx.debug("Reserving {} * f32 for data", .{cap_data});
    }
    if (cap_ebo != 0) {
        try g.ebo_buf.ensureTotalCapacity(cap_ebo);
        log_gfx.debug("Reserving {} * u32 for indexing", .{cap_ebo});
    }
    try objects_free.ensureTotalCapacity(100);

    // Upload data
    g.vbo_capacity = @intCast(g.data_buf.capacity);
    try gfx_core.bindVBOAndReserveBuffer(f32, .Array, g.vbo, g.vbo_capacity, g.draw_mode);
    try gfx_core.bindVBOAndBufferSubData(f32, 0, g.vbo, @intCast(g.data_buf.items.len),
                                         g.data_buf.items);
    g.ebo_capacity = @intCast(g.ebo_buf.capacity);
    try gfx_core.bindEBOAndReserveBuffer(g.ebo, g.ebo_capacity, g.draw_mode);
    try gfx_core.bindEBOAndBufferSubData(0, g.ebo, @intCast(g.ebo_buf.items.len), g.ebo_buf.items);
}

pub fn deinitBatch(g: *GraphicsDataType) void {
    g.data0_buf.deinit();
    g.data_buf.deinit();
    g.ebo_buf.deinit();
    g.data_free.deinit();
    g.ebo_free.deinit();
}

//-----------------------------------------------------------------------------//
//   Processing
//-----------------------------------------------------------------------------//

pub fn modifyData(obj: *ObjectDataType, i_src: u32, v: []f32) !void {
    std.debug.assert(i_src < obj.n_src);
    std.debug.assert(obj.src[i_src].i_data.r1 - obj.src[i_src].i_data.r0 + 1 == v.len);

    const i_data = obj.src[i_src].i_data;
    var i: u32 = i_data.r0;
    var j: u32 = 0;
    while (i < i_data.r1 + 1) : (i += 1) {
        obj.src[i_src].data.data_buf.items[i] = v[j];
        j += 1;
    }
}

pub fn modifyDataSimd8(obj: *ObjectDataType, i_src: u32, v: *@Vector(8, f32)) !void {
    std.debug.assert(i_src < obj.n_src);
    // std.debug.assert(obj.src[i_src].i_data.r1 - obj.src[i_src].i_data.r0 + 1 == v.len);

    const i_data = obj.src[i_src].i_data;
    var i: u32 = i_data.r0;
    var j: u32 = 0;
    while (i < i_data.r1 + 1) : (i += 1) {
        obj.src[i_src].data.data_buf.items[i] = v[j];
        j += 1;
    }
}

pub fn drawBatch(g: *GraphicsDataType) !void {
    try gfx_core.bindVAO(g.vao);
    if (g.draw_mode != .Static) {
        const l_data = @as(u32, @intCast(g.data_buf.items.len));
        if (g.vbo_capacity < l_data) {
            log_gfx.debug("Increasing vbo buffer size", .{});

            // Make sure to not multiply by zero endlessly
            g.vbo_capacity = @max(1, g.vbo_capacity);

            while (g.vbo_capacity < l_data) g.vbo_capacity *= 2;
            try gfx_core.bindVBOAndReserveBuffer(f32, .Array, g.vbo, g.vbo_capacity, g.draw_mode);
        }
        try gfx_core.bindVBOAndBufferSubData(f32, 0, g.vbo, l_data, g.data_buf.items);

        const l_ebo = @as(u32, @intCast(g.ebo_buf.items.len));
        if (g.ebo_capacity < l_ebo) {
            log_gfx.debug("Increasing ebo buffer size", .{});

            // Make sure to not multiply by zero endlessly
            g.ebo_capacity = @max(1, g.ebo_capacity);

            while (g.ebo_capacity < l_ebo) g.ebo_capacity *= 2;
            try gfx_core.bindEBOAndReserveBuffer(g.ebo, g.ebo_capacity, g.draw_mode);
        }
        try gfx_core.bindEBOAndBufferSubData(0, g.ebo, l_ebo, g.ebo_buf.items);
    } else {
        try gfx_core.bindVBO(g.vbo);
    }
    // _ = try gfx_base.setVertexAttributes(g.render_mode, g.render_mode_user_data);
    const sp = gfx_base.getShaderProgramFromRenderMode(g.render_mode, g.render_mode_user_data);
    try gfx_core.useShaderProgram(sp);
    try gfx_core.drawElements(g.primitive_mode, @intCast(g.ebo_buf.items.len));
}

//-----------------------------------------------------------------------------//
//   Internal
//-----------------------------------------------------------------------------//
const log_gfx = std.log.scoped(.gfx);

var gpa = if (cfg.debug_allocator) std.heap.GeneralPurposeAllocator(.{ .verbose_log = true }){} else std.heap.GeneralPurposeAllocator(.{}){};
const allocator = gpa.allocator();

const IndexRange = struct {
    r0: u32 = 0,
    r1: u32 = 0
};

//------------------------------------------------------------------------------
//   Object data
//------------------------------------------------------------------------------

const GfxSourcesMax = 10;

var objects: std.ArrayList(ObjectDataType) = std.ArrayList(ObjectDataType).init(allocator);
var objects_free: std.ArrayList(*ObjectDataType) = std.ArrayList(*ObjectDataType).init(allocator);
var objects_nr_removed: u32 = 0;
var objects_nr_reused: u32 = 0;
var objects_nr_new: u32 = 0;
var data_nr_new: u32 = 0;
var data_nr_reused: u32 = 0;
var ebo_nr_new: u32 = 0;
var ebo_nr_reused: u32 = 0;

//------------------------------------------------------------------------------
//   Object input data
//------------------------------------------------------------------------------

fn setupObjectDataBegin(i_src: u32, obj: *ObjectDataType, g: *GraphicsDataType) void {
    obj.src[i_src].data = g;
    obj.src[i_src].i_data.r0 = @intCast(g.data0_buf.items.len);
    obj.src[i_src].i_ebo.r0 = @intCast(g.ebo_buf.items.len);
}

fn setupObjectDataEnd(i_src: u32, obj: *ObjectDataType) void {
    const l_d = obj.src[i_src].data.data0_buf.items.len - 1;
    const l_e = obj.src[i_src].data.ebo_buf.items.len - 1;
    obj.src[i_src].i_data.r1 = @intCast(l_d);
    obj.src[i_src].i_ebo.r1 = @intCast(l_e);
}

//-----------------------------------------------------------------------------//
//   Processing
//-----------------------------------------------------------------------------//

pub fn destroyObject(obj: *ObjectDataType) !void {
    if (!obj.is_free) {
        objects_nr_removed += 1;
        var i: u32 = 0;
        while (i < obj.n_src) : (i += 1) {
            const s = &obj.src[i];
            var i_ebo: u32 = s.i_ebo.r0;
            while (i_ebo <= s.i_ebo.r1) : (i_ebo += 1) {
                s.data.ebo_buf.items[i_ebo] = gfx_core.PRI;
            }
            try s.data.data_free.append(s.i_data);
            try s.data.ebo_free.append(s.i_ebo);
        }
        obj.is_free = true;
        try objects_free.append(obj);
    }
}

//-----------------------------------------------------------------------------//
//   Helper
//-----------------------------------------------------------------------------//

pub fn appendObject(obj: ObjectDataType) !*ObjectDataType {
    std.debug.assert(objects.items.len < objects.capacity);
    try objects.append(obj);
    return &objects.items[objects.items.len - 1];
}

//-----------------------------------------------------------------------------//
//   Tests
//-----------------------------------------------------------------------------//
