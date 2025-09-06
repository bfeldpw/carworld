const std = @import("std");
const bfe = @import("bfe");
const cam = bfe.gfx.cam;
const cfg = bfe.cfg;
const gui = bfe.gfx.gui;
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

    var arena_allocator = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena_allocator.deinit();
    const arena = arena_allocator.allocator();

    // Init
    prng.seed(23041979 * @as(u64, @intCast(std.time.timestamp())));

    bfe.gfx.core.setFpsTarget(60.0);
    try bfe.gfx.core.init();
    try bfe.gfx.base.init(.{}, "./src/bfe/gfx/shader/");
    try bfe.gfx.gui.init();
    try bfe.input.init(bfe.gfx.core.getWindow());
    try input.init();
    defer bfe.gfx.core.deinit();
    defer bfe.gfx.base.deinit();
    defer bfe.gfx.gui.deinit();
    defer bfe.input.deinit();

    try cam.init(.{.w = 50, .x_lim = .{-1000, 1000}, .y_lim = .{-1000, 1000}});
    defer cam.deinit();
    try cam.addCameraEventCallback(&handleCameraEvent);

    try bfe.gfx.core.setPointSize(5);
    try bfe.gfx.base.setColor(.PxyCuniF32,1,1,1,1);

    try setupGui();

    try car.init();
    defer car.deinit();

    var counter: u64 = 0;

    // Run
    while (bfe.gfx.core.isWindowOpen()) {
        bfe.input.process();

        cam.update();

        if (!pause) try car.update();
        try car.render();
        const wgt_lg = try gui.getTextWidget("wgt_lg");
        wgt_lg.text = try car.getCarData(arena, 0);
        try bfe.gfx.gui.update();

        try bfe.gfx.core.finishFrame();

        if (arena_allocator.reset(.retain_capacity) == false) {
            std.log.warn("Arena allocator reset not succesful", .{});
        }
        counter += 1;
    }

    const leaked = gpa.deinit();
    if (leaked == .leak) std.log.err("Memory leaked in GeneralPurposeAllocator", .{});
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

var pause: bool = false;

fn handleCameraEvent(w: f32, p: [4]f32) void {
    // const w = cam.getParam().w;
    const h = w / bfe.gfx.core.getAspect();
    bfe.gfx.base.updateProjection(.PxyCuniF32, -w + p[0] + p[2], w + p[0] + p[2],
                                               -h + p[1] + p[3], h + p[1] + p[3]);
}

fn setupGui() !void {
    try gui.addColor("lg_bg", .{0.1, 0.3, 0.1, 0.6});
    try gui.addColor("light_blue", .{0.4, 0.8, 1.0, 1.0});
    try gui.addColor("light_green", .{0.6, 1.0, 0.6, 1.0});
    try gui.addColor("orange", .{1.0, 0.7, 0.0, 1.0});
    const ovl_lg: gui.Overlay = .{
        .col = gui.getColor("lg_bg"),
        .title = .{.text = "Car Data",
                   .font_size = 24,
                   .separator_thickness = 3.0},
        .width = 400.0,
        .align_h = .left,
        .align_v = .top,
        // .resize_mode = .auto_vertical
    };
    const wgt_lg: gui.TextWidget = .{
        .text = "invalid",
        .font_size = 20,
        .col = gui.getColor("light_green"),
    };
    try gui.addOverlay("ovl_lg", ovl_lg);
    try gui.addTextWidget("ovl_lg", "wgt_lg", wgt_lg);
}

//-----------------------------------------------------------------------------//
//   Tests
//-----------------------------------------------------------------------------//

test "main test" {
    const bfe_test = bfe;
    _ = bfe_test;
}
