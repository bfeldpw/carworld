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
        // .{ .scope = .input, .level = .info },
        // .{ .scope = .stats, .level = .info },
    }
};

pub fn main() !void {
    var gpa = if (cfg.debug_allocator) std.heap.GeneralPurposeAllocator(.{ .verbose_log = true }){} else std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();
    defer {
        const leaked = gpa.deinit();
        if (leaked == .leak) std.log.err("Memory leaked in GeneralPurposeAllocator", .{});
    }

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

    try car.init(allocator);
    defer car.deinit(allocator);

    var pf_gui: bfe.util.stats.PerFrameTimerBuffered(20) = bfe.util.stats.PerFrameTimerBuffered(20).init();
    var counter: u64 = 0;

    // Run
    while (bfe.gfx.core.isWindowOpen()) {
        bfe.input.process();
        cam.update();

        if (!pause) try car.update();

        try car.render();

        pf_gui.start();
        
            const wgt_cd = try gui.getTextWidget("wgt_cd");
            wgt_cd.text = try car.getCarData(arena, 0);
            try bfe.gfx.gui.update();

        pf_gui.stop();

        try bfe.gfx.core.finishFrame();

        if (arena_allocator.reset(.retain_capacity) == false) {
            std.log.warn("Arena allocator reset not succesful", .{});
        }
        counter += 1;
    }

    std.log.info("Gui update   {d:.4}ms", .{pf_gui.getAvgAllMs()});
}

pub fn togglePause() void {
    pause = !pause;
}

//-----------------------------------------------------------------------------//
//   Internal
//-----------------------------------------------------------------------------//
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
    try gui.addColor("bg", .{0.1, 0.3, 0.1, 0.6});
    try gui.addColor("light_blue", .{0.4, 0.8, 1.0, 1.0});
    try gui.addColor("light_green", .{0.6, 1.0, 0.6, 1.0});
    try gui.addColor("orange", .{1.0, 0.7, 0.0, 1.0});

    const ovl_car_data: gui.Overlay = .{
        .col = gui.getColor("bg"),
        .title = .{.text = "Car Data",
                   .font_size = 24,
                   .separator_thickness = 3.0},
        .width = 400.0,
        .align_h = .left,
        .align_v = .top,
        .is_enabled = true,
        // .resize_mode = .auto_vertical
    };
    const wgt_car_data: gui.TextWidget = .{
        .text = "invalid",
        .font_size = 20,
        .col = gui.getColor("light_green"),
    };
    try gui.addOverlay("ovl_cd", ovl_car_data);
    try gui.addTextWidget("ovl_cd", "wgt_cd", wgt_car_data);

    const ovl_input: gui.Overlay = .{
        .col = gui.getColor("bg"),
        .title = .{.text = "Input",
                   .font_size = 24,
                   .separator_thickness = 3.0},
        .width = 400.0,
        .align_h = .right,
        .align_v = .top,
        .is_enabled = true,
    };
    const wgt_input: gui.TextWidget = .{
        .text = "no controllers",
        .font_size = 20,
        .col = gui.getColor("light_green"),
    };
    try gui.addOverlay("ovl_input", ovl_input);
    try gui.addTextWidget("ovl_input", "wgt_input", wgt_input);
}

// fn formatInputData(a: std.mem.Allocator) ![]u8 {
    // const str1 = try std.fmt.allocPrint(
    //     a,
    //     "GENERIC\n" ++
    //     "  Velocity = {d:.0} km/h\n" ++
    //     "  Drive train layout: {s}\n" ++
    //     "\n" ++
    //     "TIRE MODEL\n" ++
    //     "  Linear   = {d:.0} %\n" ++
    //     "  Pacejka  = {d:.0} %\n" ++
    //     "\n" ++
    //     "TIRE DATA\n" ++
    //     "  slip angle [degree]   {d:5.1} {d:5.1}\n" ++
    //     "                        {d:5.1} {d:5.1}\n" ++
    //     "\n" ++
    //     "  slip ratio [%]        {d:4.0} {d:4.0}\n" ++
    //     "                        {d:4.0} {d:4.0}\n" ++
    //     "\n" ++
    //     "  load [kN]             {d:4.1} {d:4.1}\n" ++
    //     "                        {d:4.1} {d:4.1}\n" ++
    //     "\n" ++
    //     "  force lat. [kN]       {d:4.1} {d:4.1}\n" ++
    //     "                        {d:4.1} {d:4.1}\n" ++
    //     "\n" ++
    //     "  force lon. [kN]       {d:4.1} {d:4.1}\n" ++
    //     "                        {d:4.1} {d:4.1}\n" ++
    //     "\n" ++
    //     "  velocity lon. [km/h]  {d:4.1} {d:4.1}\n" ++
    //     "                        {d:4.1} {d:4.1}\n" ++
    //     "\n" ++
    //     "  velocity tan. [km/h]  {d:4.1} {d:4.1}\n" ++
    //     "                        {d:4.1} {d:4.1}\n"
    //         ,
    //     .{getLength2(cars.items(.body)[idx].vel) * 3.6,
    //       mapDriveTrainLayout(cars.items(.drive_train)[0].layout),
    //       buf_tire_model_lin.getAvg(),
    //       buf_tire_model_paj.getAvg(),
    //       buf_tire_slip_a[2].getAvg(),
    //       buf_tire_slip_a[1].getAvg(),
    //       buf_tire_slip_a[3].getAvg(),
    //       buf_tire_slip_a[0].getAvg(),
    //       buf_tire_slip_r[2].getAvg(),
    //       buf_tire_slip_r[1].getAvg(),
    //       buf_tire_slip_r[3].getAvg(),
    //       buf_tire_slip_r[0].getAvg(),
    //       buf_tire_load[2].getAvg() * 0.001,
    //       buf_tire_load[1].getAvg() * 0.001,
    //       buf_tire_load[3].getAvg() * 0.001,
    //       buf_tire_load[0].getAvg() * 0.001,
    //       buf_tire_fy[2].getAvg() * 0.001,
    //       buf_tire_fy[1].getAvg() * 0.001,
    //       buf_tire_fy[3].getAvg() * 0.001,
    //       buf_tire_fy[0].getAvg() * 0.001,
    //       buf_tire_fx[2].getAvg() * 0.001,
    //       buf_tire_fx[1].getAvg() * 0.001,
    //       buf_tire_fx[3].getAvg() * 0.001,
    //       buf_tire_fx[0].getAvg() * 0.001,
    //       buf_tire_lon[2].getAvg() * 3.6,
    //       buf_tire_lon[1].getAvg() * 3.6,
    //       buf_tire_lon[3].getAvg() * 3.6,
    //       buf_tire_lon[0].getAvg() * 3.6,
    //       buf_wheel_vel[2].getAvg() * 3.6,
    //       buf_wheel_vel[1].getAvg() * 3.6,
    //       buf_wheel_vel[3].getAvg() * 3.6,
    //       buf_wheel_vel[0].getAvg() * 3.6}
    // );

    // const str2 = try std.fmt.allocPrint(
    //     a,
    //     "\nAERODYNAMICS\n" ++
    //     "    drag (front) = {d:.0} N\n" ++
    //     "    drag (side)  = {d:.0} N\n"
    //         ,
    //     .{buf_drag[0].getAvg(),
    //       buf_drag[1].getAvg()}
    // );
    // return try std.fmt.allocPrint(a, "{s}{s}", .{str1, str2});
// }

//-----------------------------------------------------------------------------//
//   Tests
//-----------------------------------------------------------------------------//

test "main test" {
    const bfe_test = bfe;
    _ = bfe_test;
}
