const std = @import("std");
const bfe = @import("bfe");
const cam = bfe.gfx.cam;
const cfg = bfe.cfg;
const id_type = bfe.util.id_type;
const car = @import("car.zig");
const input = @import("input_plugin.zig");

pub const std_options: std.Options = .{
    .log_scope_levels = &[_]std.log.ScopeLevel{
        // .{ .scope = .gfx, .level = .debug },
        .{ .scope = .input, .level = .info },
        // .{ .scope = .stats, .level = .info },
    }
};

pub fn main() !void {

    // Init
    prng.seed(23041979 * @as(u64, @intCast(std.time.timestamp())));

    bfe.gfx.core.setFpsTarget(60.0);
    try bfe.gfx.core.init();
    try bfe.gfx.base.init(.{}, "./src/bfe/gfx/shader/");
    try bfe.input.init(bfe.gfx.core.getWindow());
    try input.init();
    defer bfe.gfx.core.deinit();
    defer bfe.gfx.base.deinit();
    defer bfe.input.deinit();

    try cam.init(.{.w = 200, .x_lim = .{-1000, 1000}, .y_lim = .{-1000, 1000}});
    defer cam.deinit();
    try cam.addCameraEventCallback(&handleCameraEvent);

    try bfe.gfx.core.setPointSize(5);
    try bfe.gfx.base.setColor(.PxyCuniF32,1,1,1,1);

    try car.init();
    defer car.deinit();

    var counter: u64 = 0;

    // Run
    while (bfe.gfx.core.isWindowOpen()) {
        bfe.input.process();

        cam.update();

        if (!pause) try car.update();
        try car.render();

        try bfe.gfx.core.finishFrame();

        counter += 1;
    }
}

pub fn togglePause() void {
    pause = !pause;
}

//-----------------------------------------------------------------------------//
//   Internal
//-----------------------------------------------------------------------------//
var gpa = if (cfg.debug_allocator) std.heap.GeneralPurposeAllocator(.{ .verbose_log = true }){} else std.heap.GeneralPurposeAllocator(.{}){};
const allocator = gpa.allocator();

var prng = std.Random.DefaultPrng.init(0);
const rand = prng.random();

var pause: bool = true;

fn handleCameraEvent(w: f32, p: [4]f32) void {
    // const w = cam.getParam().w;
    const h = w / bfe.gfx.core.getAspect();
    bfe.gfx.base.updateProjection(.PxyCuniF32, -w + p[0] + p[2], w + p[0] + p[2],
                                  -h + p[1] + p[3], h + p[1] + p[3]);
}

//-----------------------------------------------------------------------------//
//   Tests
//-----------------------------------------------------------------------------//

test "main test" {
    const bfe_test = bfe;
    _ = bfe_test;
}
