const std = @import("std");

const bfe = @import("bfe");
const c = bfe.gfx.c.c;
const gfx_cam = bfe.gfx.cam;
const ipt = bfe.input;

const car = @import("car.zig");
const main = @import("main.zig");

//-----------------------------------------------------------------------------//
//   Error Sets / Enums / Types / Structs
//-----------------------------------------------------------------------------//

const CarControlSetup = struct {
    steer_jid: i32 = 0,
    steer_axis: i32 = 0,
    steer_invert: f32 = 1.0,
    throttle_jid: i32 = 0,
    throttle_axis: i32 = 1,
    throttle_invert: f32 = 1.0,
    brake_jid: i32 = 0,
    brake_axis: i32 = 2,
    brake_invert: f32 = 1.0,
    handbrake_jid: i32 = 0,
    handbrake_bt: i32 = 0
};

const GameControlSetup = struct {
    cam_x_jid: i32 = 0,
    cam_x_axis: i32 = 3,
    cam_x_invert: f32 = 1.0,
    cam_x_deadzone: f32 = 1.0,
    cam_y_jid: i32 = 0,
    cam_y_axis: i32 = 4,
    cam_y_invert: f32 = 1.0,
    cam_y_deadzone: f32 = 1.0,
    pause_jid: i32 = 0,
    pause_bt: i32 = 1,
};

const ControlSetup = struct {
    car: CarControlSetup = .{},
    game: GameControlSetup = .{},
};

//-----------------------------------------------------------------------------//
//   Init / DeInit
//-----------------------------------------------------------------------------//

pub fn init() !void {
    try ipt.addPlugin(&process);
}

//-----------------------------------------------------------------------------//
//   Getter/Setter
//-----------------------------------------------------------------------------//

pub inline fn getSteering() f32 {
    return ipt.getAxisState(control_setup.car.steer_jid, control_setup.car.steer_axis);
}

pub inline fn getSteeringInvert() f32 {
    return control_setup.car.steer_invert;
}

pub inline fn getThrottle() f32 {
    return ipt.getAxisState(control_setup.car.throttle_jid, control_setup.car.throttle_axis);
}

pub inline fn getThrottleInvert() f32 {
    return control_setup.car.throttle_invert;
}

pub inline fn getBrake() f32 {
    return ipt.getAxisState(control_setup.car.brake_jid, control_setup.car.brake_axis);
}

pub inline fn getBrakeInvert() f32 {
    return control_setup.car.brake_invert;
}

pub inline fn getHandbrake() bool {
    return ipt.getButtonState(control_setup.car.handbrake_jid, control_setup.car.handbrake_bt);
}

pub fn setupControls(ctl: ControlSetup) void {
    control_setup = ctl;
}

//-----------------------------------------------------------------------------//
//   Getter/Setter
//-----------------------------------------------------------------------------//

pub fn loadControlConfig(allocator: std.mem.Allocator, f: []const u8) !void {
    log_input.info("Loading controller config: {s}", .{f});
    
    // Read the JSON file into memory
    const json_data = std.fs.cwd().readFileAlloc(f,allocator, std.Io.Limit.unlimited) catch |err| {
        log_input.warn("Unable to read file {s}: {}", .{f, err});
        return;
    };
    defer allocator.free(json_data);

    // Parse the JSON data into your struct
    const parsed = std.json.parseFromSlice(ControlSetup, allocator, json_data, .{}) catch |err| {
        log_input.warn("Unable to parse json {s}: {}", .{json_data, err});
        return;
    };
    defer parsed.deinit();

    log_input.debug("Controller config loaded: \n{s}", .{json_data});

    control_setup = parsed.value;
}

//-----------------------------------------------------------------------------//
//   Internal
//-----------------------------------------------------------------------------//

const log_input = std.log.scoped(.input);

var control_setup: ControlSetup = .{};

//-----------------------------------------------------------------------------//
//   Processing
//-----------------------------------------------------------------------------//

fn process() void {
    if (ipt.getKeyState(.key_ctrl_left)) {
        if (ipt.getKeyPressEvent(.key_q)) ipt.closeWindow();
        if (ipt.getKeyPressEvent(.key_r)) {
            gfx_cam.reset();
            gfx_cam.zoomToWidth(150.0);
        }
        if (ipt.getKeyPressEvent(.key_p)) main.togglePause();
        if (ipt.getKeyPressEvent(.key_e)) bfe.gfx.gui.toggleEditMode();
    }
    if (ipt.getKeyPressEvent(.key_h)) car.toggleHook();
    if (ipt.getKeyPressEvent(.key_j)) car.setHook();
    if (ipt.getKeyPressEvent(.key_t)) car.rotateDriveTrainLayout();

    // const shift = ipt.getKeyState(.key_shift_left);
    if (ipt.getKeyState(.key_left)) gfx_cam.moveVelRel(-0.5, 0);
    if (ipt.getKeyState(.key_right)) gfx_cam.moveVelRel(0.5, 0);
    if (ipt.getKeyState(.key_up)) gfx_cam.moveVelRel(0, 0.5);
    if (ipt.getKeyState(.key_down)) gfx_cam.moveVelRel(0, -0.5);
    if (ipt.getKeyState(.key_page_up)) gfx_cam.zoomBy(-0.02);
    if (ipt.getKeyState(.key_page_down)) gfx_cam.zoomBy(0.02);
    if (ipt.getKeyState(.key_space)) car.useHandbrake();
    if (ipt.getKeyState(.key_e)) car.increaseThrottle();
    if (ipt.getKeyState(.key_q)) car.decreaseThrottle();
    if (ipt.getKeyState(.key_w)) car.accelerate();
    if (ipt.getKeyState(.key_s)) car.decelerate();
    if (ipt.getKeyState(.key_a)) car.steerLeft();
    if (ipt.getKeyState(.key_d)) car.steerRight();

    if (ipt.getJoysticks().contains(control_setup.car.throttle_jid)) {
        car.steer(getSteering(), getSteeringInvert());
        car.brake(getBrake(), getBrakeInvert());
        if (getHandbrake()) car.useHandbrake();
        car.throttle(getThrottle(), getThrottleInvert());

        var cam_x: f32 = ipt.getAxisState(control_setup.game.cam_x_jid, control_setup.game.cam_x_axis) *
                        control_setup.game.cam_x_invert;
        var cam_y: f32 = ipt.getAxisState(control_setup.game.cam_y_jid, control_setup.game.cam_y_axis) *
                        control_setup.game.cam_y_invert;
        const dz_x = control_setup.game.cam_x_deadzone;
        const dz_y = control_setup.game.cam_y_deadzone;
        if (@abs(cam_x) < dz_x) cam_x = dz_x;
        if (@abs(cam_y) < dz_y) cam_y = dz_y;
        gfx_cam.moveVelRel(2.0 * (cam_x - dz_x) / (1.0 - dz_x), 2.0 * (cam_y - dz_y) / (1.0 - dz_y));

        if (ipt.getButtonState(control_setup.game.pause_jid, control_setup.game.pause_bt)) pause = true;
        if (pause and !ipt.getButtonState(control_setup.game.pause_jid, control_setup.game.pause_bt)) {
            main.togglePause();
            pause = false;
        }
    }
}

var pause: bool = false;
