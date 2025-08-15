const std = @import("std");
const bfe = @import("bfe");
const cfg = bfe.cfg;
const id_type = bfe.util.id_type;
const stats = bfe.util.stats;
const gfx = @import("gfx_ext.zig");


//-----------------------------------------------------------------------------//
//   Init / DeInit
//-----------------------------------------------------------------------------//

pub fn init() !void {
    prng.seed(23041979 * @as(u64, @intCast(std.time.timestamp())));

    try gfx.init();
    try initCars();
    try initGfx();
}

pub fn deinit() void {
    deinitGfx();
    gfx.deinit();

    log_car.info("  Performance Stats", .{});
    log_car.info("  ============================", .{});
    log_car.info("  gfx update   |   {d:.2}ms", .{pf.gfx.getAvgAllMs()});
    log_car.info("  physics      |   {d:.2}ms", .{pf.phy.getAvgAllMs()});
    log_car.info("   - dynamics  |   {d:.2}ms", .{pf.phy_dyn.getAvgAllMs()});
    log_car.info("   - tires     |   {d:.2}ms", .{pf.phy_tire.getAvgAllMs()});
    log_car.info("  ============================", .{});
}

//-----------------------------------------------------------------------------//
//   Process
//-----------------------------------------------------------------------------//
var is_accelerating: bool = false;
var is_decelerating: bool = false;
var acc_ext_int: Vec2d = .{0.0, 0.0};

pub fn accelerate() void {
    is_accelerating = true;
}

pub fn decelerate() void {
    is_decelerating = true;
}

fn accelerateInternal() void {
    if (is_accelerating) {
        const b = &cars.items(.body)[0];
        const a: f32 = 0.05;
        const a_max: f32 = 1.5;
        if (getLength2(acc_ext_int) < a_max) {
            acc_ext_int += Vec2d{a * @cos(b.angle), a * @sin(b.angle)};
            acc_ext = acc_ext_int;
        } else {
            acc_ext = Vec2d{a_max * @cos(b.angle), a_max * @sin(b.angle)};
        }
    } else if (!is_decelerating){
        acc_ext_int *= .{0.8, 0.8};
        acc_ext = acc_ext_int;
    }
}

fn decelerateInternal() void {
    if (is_decelerating) {
        const b = &cars.items(.body)[0];
        const a: f32 = -0.05;
        const a_max: f32 = -2.0;
        if (getLength2(acc_ext_int) < -a_max) {
            acc_ext_int += Vec2d{a * @cos(b.angle), a * @sin(b.angle)};
            acc_ext = acc_ext_int;
        } else {
            acc_ext = Vec2d{a_max * @cos(b.angle), a_max * @sin(b.angle)};
        }
    } else if (!is_accelerating) {
        acc_ext_int *= .{0.8, 0.8};
        acc_ext = acc_ext_int;
    }
}

pub fn steerLeft() void {
    cars.items(.steering)[0].target += 0.01;
}

pub fn steerRight() void {
    cars.items(.steering)[0].target -= 0.01;
}

pub fn render() !void {
    try gfx.drawBatch(&gfx_car);
    try gfx.drawBatch(&gfx_tires);
    try bfe.gfx.base.setColor(.PxyCuniF32, 0.0, 0.0, 1.0, 1.0);
    try bfe.gfx.core.setLineWidth(3);
    try gfx.drawBatch(&gfx_dbg_vec);
    try bfe.gfx.base.setColor(.PxyCuniF32, 1.0, 1.0, 0.0, 1.0);
    try bfe.gfx.core.setPointSize(3);
    try gfx.drawBatch(&gfx_dbg_com);
    try bfe.gfx.core.setLineWidth(1);
    try bfe.gfx.base.setColor(.PxyCuniF32, 1.0, 1.0, 1.0, 1.0);
}

pub fn update() !void {
    pf.phy.start();

    updateTireFn();
    accelerateInternal();
    decelerateInternal();
    is_accelerating = false;
    is_decelerating = false;
    updateForces();
    applyForces();
    try updateCarDynamics();
    clearForces();

    pf.phy.stop();

    if (is_hooked) bfe.gfx.cam.updateHook(cars.items(.body)[0].pos[0], cars.items(.body)[0].pos[1], 0, 0);
    try updateCarGfx();
}

var is_hooked: bool = false;

pub fn toggleHook() void {
    is_hooked = !is_hooked;
}

//-----------------------------------------------------------------------------//
//   Internal
//-----------------------------------------------------------------------------//
const log_car = std.log.scoped(.gfx_car);

var gpa = if (cfg.debug_allocator) std.heap.GeneralPurposeAllocator(.{ .verbose_log = true }){} else std.heap.GeneralPurposeAllocator(.{}){};
const allocator = gpa.allocator();

var prng = std.Random.DefaultPrng.init(0);
const rand = prng.random();

var cars: std.MultiArrayList(Car) = .{};

const gravity = 9.81;

const Car = struct {
    body: BodyDef,
    tires: [n_tires_max]TireDef,
    steering: SteeringDef,
    id: id_type.IdType()
};

const BodyDef = struct {
    force: Vec2d = .{0.0, 0.0}, // Force w.r.t. to COM
    acc: Vec2d = .{0.0, 0.0},
    vel_p: Vec2d = .{0.0, 0.0}, // Previous velocity
    vel: Vec2d = .{0.0, 0.0},
    pos: Vec2d = .{0.0, 0.0},
    torque: f32 = 0.0,          // Torque w.r.t COM
    angle_acc: f32 = 0.0,
    angle_vel: f32 = 0.0,
    angle: f32 = 0.0,
    com0: Vec2d = .{0.0, 0.0},
    com: Vec2d = .{0.0, 0.0},   // center of mass
    com_vel: Vec2d = .{0.0, 0.0},
    com_z: f32 = 0.5,           // height above axle plane
    mass: f32 = 1.0,
    // Graphics
    obj: *gfx.ObjectDataType,
};

const SteeringDef = struct {
    is_steering: bool = false,
    speed: f32 = 0.5, // rad/s
    angle: f32 = 0.0,
    target: f32 = 0.0
};

const TireDef = struct {
    // Local parameters
    pos: Vec2d = .{0.0, 0.0},
    angle: f32 = 0.0,
    f_x: f32 = 0.0, // longitudinal force
    f_y: f32 = 0.0, // lateral force
    f_z: f32 = 1.0, // normal force
    // Misc information
    is_steered: bool = false,
    is_powered: bool = false,
    id: id_type.IdType() = .{},
    // Graphics
    obj: *gfx.ObjectDataType
};

var tire_prm: TirePrm = .{};

const TirePrm = struct {
    lat: struct {
        stiffness: f32 = 0.1,
        shape: f32 = 2.4,
        peak: f32 = 1.0,
        curvature: f32 = 0.97,
    } = .{},
    lon: struct {
        stiffness: f32 = 10.0,
        shape: f32 = 2.4,
        peak: f32 = 1.0,
        curvature: f32 = 0.97,
    } = .{},

    // 195/55 R16
    r: f32 = 0.62 / 2.0,
    w_half: f32 = 0.195 / 2.0,
};

const n_cars = 1;
const Vec2d = @Vector(2, f32);
const Vec4d = @Vector(4, f32);
const Vec8d = @Vector(8, f32);
const car_body_segments = 4;
const car_tire_segments = 4;
const n_tires_max = 4;

var acc_ext: Vec2d = .{0.0, 0.0};

var gfx_car: gfx.GraphicsDataType = .{};
var gfx_tires: gfx.GraphicsDataType = .{};
var gfx_dbg_vec: gfx.GraphicsDataType = .{.primitive_mode = .LineStrip};
var gfx_dbg_com: gfx.GraphicsDataType = .{.primitive_mode = .Points};

var pf: struct {
    gfx: stats.PerFrameTimerBuffered(20) = stats.PerFrameTimerBuffered(20).init(),
    phy: stats.PerFrameTimerBuffered(20) = stats.PerFrameTimerBuffered(20).init(),
    phy_dyn: stats.PerFrameTimerBuffered(20) = stats.PerFrameTimerBuffered(20).init(),
    phy_tire: stats.PerFrameTimerBuffered(20) = stats.PerFrameTimerBuffered(20).init()
} = .{};

fn initCars() !void {
    var i_car: u32 = 0;
    while (i_car < n_cars) : (i_car += 1) {
        var car: Car = undefined;
        car.id.init();
        // car.body0.pos = .{0.0, 0.0};//.{rand.float(f32) * 200 - 100, rand.float(f32) * 200 - 100};
        car.body.pos = .{rand.float(f32) * 200 - 100, rand.float(f32) * 200 - 100};
        car.body.angle = rand.float(f32) * 2.0 * std.math.pi;
        // car.body.angle = 0.0;//rand.float(f32) * 2.0 * std.math.pi;
        car.body.mass = 2000.0;

        const a: f32 = 0.1 * rand.float(f32);
        car.body.acc = Vec2d{a * @cos(car.body.angle), a * @sin(car.body.angle)};
        car.body.vel_p = @splat(0.0);
        car.body.vel = @splat(0.0);
        car.body.com0 = .{ 0.3, 0.0};
        car.body.com_z = 0.2;

        car.tires[0].pos = .{-1.5, -0.8};
        car.tires[1].pos = .{ 1.5, -0.8};
        car.tires[2].pos = .{ 1.5,  0.8};
        car.tires[3].pos = .{-1.5,  0.8};
        car.tires[0].is_steered = false;
        car.tires[1].is_steered = true;
        car.tires[2].is_steered = true;
        car.tires[3].is_steered = false;
        // car.tires[0].is_powered = false;
        // car.tires[1].is_powered = true;
        // car.tires[2].is_powered = true;
        // car.tires[3].is_powered = false;
        car.tires[0].is_powered = true;
        car.tires[1].is_powered = false;
        car.tires[2].is_powered = false;
        car.tires[3].is_powered = true;

        car.tires[0].id.init();
        car.tires[1].id.init();
        car.tires[2].id.init();
        car.tires[3].id.init();

        car.steering.is_steering = false;
        car.steering.angle = 0.25 * std.math.pi;
        car.steering.target = 0.0;
        car.steering.speed = 0.5;

        try cars.append(allocator, car);
    }
    for (cars.items(.body), cars.items(.tires)) |*b, *t| {
        b.obj = try gfx.createObjectData();
        try gfx.createObjectDataAddSrc(b.obj, &gfx_car, 4);

        var v: [8]f32 = .{b.pos[0] - 2, b.pos[1] - 1,
                          b.pos[0] + 2, b.pos[1] - 1,
                          b.pos[0] + 2, b.pos[1] + 1,
                          b.pos[0] - 2, b.pos[1] + 1,
                          };
        gfx.createObjectPolygonGfx(b.obj, 0, &v);
        {
            // Debug vector for forces
            try gfx.createObjectDataAddSrc(b.obj, &gfx_dbg_vec, 2);
            var v_dbg: [4]f32 = .{b.pos[0], b.pos[1], b.pos[0] + b.acc[0], b.pos[1] + b.acc[1]};
            gfx.createObjectPolygonGfx(b.obj, 1, &v_dbg);
        }


        for (t) |*tire| {
            tire.obj = try gfx.createObjectData();
            try gfx.createObjectDataAddSrc(tire.obj, &gfx_tires, 4);
            var v1: [8]f32 = .{tire.pos[0] + b.pos[0] - tire_prm.w_half, tire.pos[1] + b.pos[1] - tire_prm.r,
                               tire.pos[0] + b.pos[0] + tire_prm.w_half, tire.pos[1] + b.pos[1] - tire_prm.r,
                               tire.pos[0] + b.pos[0] + tire_prm.w_half, tire.pos[1] + b.pos[1] + tire_prm.r,
                               tire.pos[0] + b.pos[0] - tire_prm.w_half, tire.pos[1] + b.pos[1] + tire_prm.r,
                               };
            gfx.createObjectPolygonGfx(tire.obj, 0, &v1);

            {
                // Debug vector for forces
                try gfx.createObjectDataAddSrc(tire.obj, &gfx_dbg_vec, 3);
                var v_dbg: [6]f32 = .{tire.pos[0] + 1, tire.pos[1], tire.pos[0], tire.pos[1], tire.pos[0], tire.pos[1] + 1};
                gfx.createObjectPolygonGfx(tire.obj, 1, &v_dbg);
            }
        }
        {
            // Debug vector for forces
            try gfx.createObjectDataAddSrc(b.obj, &gfx_dbg_com, 1);
            var com_dbg: [2]f32 = .{b.com[0], b.com[1]};
            gfx.createObjectPointGfx(b.obj, 2, &com_dbg);
        }
    }
    // for (cars.items(.body)) |*b| {
    // }
}

fn initGfx() !void {
    try gfx.initBatch(&gfx_car, 8 * n_cars, (4 + 1) * n_cars);
    try gfx.initBatch(&gfx_tires, 4 * 8 * n_cars, 4 * (4 + 1) * n_cars);
    try gfx.initBatch(&gfx_dbg_vec, 2 * n_cars, (2 + 1) * n_cars);
    try gfx.initBatch(&gfx_dbg_com, 2 * n_cars, (2 + 1) * n_cars);
}

fn deinitGfx() void {
    gfx.deinitBatch(&gfx_car);
    gfx.deinitBatch(&gfx_tires);
    gfx.deinitBatch(&gfx_dbg_vec);
    gfx.deinitBatch(&gfx_dbg_com);
}

fn updateCarDynamics() !void {
    pf.phy_dyn.start();

    cars.items(.body)[0].acc = Vec2d{0.0, 0.0};
    // cars.items(.body)[0].acc = acc_ext;

    // Rough estimation distance to fromt axle *
    // distance to rear axle * mass
    const Inertia = 1900;

    const dt = 1.0 / 60.0;
    const dt2 = @as(Vec2d, @splat(dt));
    for (cars.items(.body)) |*b| {
        b.acc += b.force / @as(Vec2d, @splat(b.mass));
        b.vel_p = b.vel;
        b.vel += b.acc * dt2;
        b.pos += b.vel * dt2;
        b.acc += (b.vel - b.vel_p) / dt2;

        // "Suspension"
        const f_acc = b.acc / (Vec2d{2.0, 2.0} * @abs(b.acc) + @as(Vec2d, @splat(1)));
        const com = transform(-b.angle, -f_acc) * Vec2d{b.com_z, b.com_z * 0.5} + b.com0;// * @as(Vec2d, @splat(b.com_z * 0.2))));
        const dx = com - b.com;
        const c = Vec2d{50, 40};
        const d = Vec2d{4, 4};
        const acc = c * dx - d * b.com_vel;
        b.com_vel += acc * dt2;
        b.com += b.com_vel * dt2;

        b.angle_acc = b.torque / Inertia;
        b.angle_vel += b.angle_acc * dt;
        b.angle += b.angle_vel * dt;
        clampAngleInline(&b.angle);
    }
    acc_ext = .{0.0, 0.0};
    for (cars.items(.steering), cars.items(.body), cars.items(.tires)) |*s, *b, *t| {
        if (s.target < 0) {
            if (s.angle > s.target) {
                s.angle -= s.speed / 60.0;
            } else {
                // b.angle = clampAngle(b.angle + s.target);
                // s.target = 0;
                s.angle = s.target;
            }
        } else if (s.target > 0) {
            if (s.angle < s.target) {
                s.angle += s.speed / 60.0;
            } else {
                // b.angle = clampAngle(b.angle + s.target);
                // s.target = 0;
                s.angle = s.target;
            }
        }
        for (t) |*t_i| {
            if (t_i.is_steered) t_i.angle = clampAngle(b.angle + s.angle)
            else t_i.angle = clampAngle(b.angle);
            // std.log.debug("TireAngle (dyn) = {d:.2}", .{std.math.radiansToDegrees(t_i.angle)});
        }
    }

    pf.phy_dyn.stop();
}

fn updateCarGfx() !void {
    pf.gfx.start();

    for (cars.items(.body), cars.items(.tires), cars.items(.steering)) |*b, *ts, s| {
        {
            const f = 0.5;
            const f2 = 0.75;
            const com = b.com - b.com0;
            const p0 = transformOffset(b.angle, Vec2d{-2, -1} - Vec2d{ f2 * com[1],  f * com[0]}, b.pos);
            const p1 = transformOffset(b.angle, Vec2d{ 2, -1} - Vec2d{-f2 * com[1], -f * com[0]}, b.pos);
            const p2 = transformOffset(b.angle, Vec2d{ 2,  1} - Vec2d{ f2 * com[1],  f * com[0]}, b.pos);
            const p3 = transformOffset(b.angle, Vec2d{-2,  1} - Vec2d{-f2 * com[1], -f * com[0]}, b.pos);
            var v: [8]f32 = .{p0[0], p0[1],
                              p1[0], p1[1],
                              p2[0], p2[1],
                              p3[0], p3[1],
                            };
            try gfx.modifyData(b.obj, 0, &v);
        }
        {
            for (ts) |*t| {
                if (t.is_steered) {
                    const t0 = transformOffset(s.angle, Vec2d{-tire_prm.r, -tire_prm.w_half}, t.pos);
                    const t1 = transformOffset(s.angle, Vec2d{ tire_prm.r, -tire_prm.w_half}, t.pos);
                    const t2 = transformOffset(s.angle, Vec2d{ tire_prm.r,  tire_prm.w_half}, t.pos);
                    const t3 = transformOffset(s.angle, Vec2d{-tire_prm.r,  tire_prm.w_half}, t.pos);
                    const p0 = transformOffset(b.angle, t0, b.pos);
                    const p1 = transformOffset(b.angle, t1, b.pos);
                    const p2 = transformOffset(b.angle, t2, b.pos);
                    const p3 = transformOffset(b.angle, t3, b.pos);
                    var v: [8]f32 = .{p0[0], p0[1],
                                    p1[0], p1[1],
                                    p2[0], p2[1],
                                    p3[0], p3[1],
                                    };
                    try gfx.modifyData(t.obj, 0, &v);
                } else {
                    const p0 = transformOffset(b.angle, t.pos + Vec2d{-tire_prm.r, -tire_prm.w_half}, b.pos);
                    const p1 = transformOffset(b.angle, t.pos + Vec2d{ tire_prm.r, -tire_prm.w_half}, b.pos);
                    const p2 = transformOffset(b.angle, t.pos + Vec2d{ tire_prm.r,  tire_prm.w_half}, b.pos);
                    const p3 = transformOffset(b.angle, t.pos + Vec2d{-tire_prm.r,  tire_prm.w_half}, b.pos);
                    var v: [8]f32 = .{p0[0], p0[1],
                                    p1[0], p1[1],
                                    p2[0], p2[1],
                                    p3[0], p3[1],
                                    };
                    try gfx.modifyData(t.obj, 0, &v);
                }
                {
                    const p0 = transformOffset(b.angle, t.pos, b.pos);

                    // Direction is 90° (-sin, cos, switch coordinates and negate x)
                    // but the force is oriented agains the cars movement, therefore
                    // the vector has to be negated
                    const p1 = transformOffset(b.angle, t.pos, b.pos + scaleDbg2(getVec2Lateral(t.f_y, t.angle)));
                    const p2 = transformOffset(b.angle, t.pos, b.pos + scaleDbg2(getVec2(t.f_x, t.angle)));// getVec2Lateral(t.f_x, t.angle + std.math.pi * 0.5));
                    // Debug vector for forces
                    var v_dbg: [6]f32 = .{p2[0], p2[1], p0[0], p0[1], p1[0], p1[1]};
                    try gfx.modifyData(t.obj, 1, &v_dbg);
                }
            }
        }
        {
            // b.force = b.vel * @as(Vec2d, @splat(60.0 * b.mass));
            // std.log.debug("Force to stop = {d:.1}", .{b.force});
            var v: [4]f32 = .{b.pos[0], b.pos[1],
                              b.pos[0] + scaleDbg(b.mass * b.vel[0]),
                              b.pos[1] + scaleDbg(b.mass * b.vel[1])};
            try gfx.modifyData(b.obj, 1, &v);

            var v_com: [2]f32 = transformOffset(b.angle, b.com, b.pos);
            try gfx.modifyData(b.obj, 2, &v_com);
        }
    }

    pf.gfx.stop();
}

fn updateForces() void {
    pf.phy_tire.start();
    // const mu = 0.8;

    for (cars.items(.body), cars.items(.tires)) |*b, *t| {

        for (t) |*t_i| {
            t_i.f_y = 0.0;

            const r = transform(b.angle, t_i.pos - b.com);
            const vel_r = @as(Vec2d, @splat(b.angle_vel)) * Vec2d{-r[1], r[0]};
            const vel = vel_r + b.vel;

            // Slip angle
            const a_vel = std.math.atan2(vel[1], vel[0]);
            var a_slip = a_vel - t_i.angle;
            while (a_slip > 0.5 * std.math.pi) a_slip = std.math.pi - a_slip;
            while (a_slip < -0.5 * std.math.pi) a_slip = -std.math.pi - a_slip;

            const v_abs = getLength2(vel);
            const v_lon = v_abs * @cos(a_slip);
            std.log.debug("v_lon = {d:.2}", .{v_lon * 3.6});

            a_slip = std.math.radiansToDegrees(a_slip);

            // const r_slip = @max((v_lon - getLength2(acc_ext)) / (getLength2(acc_ext) + 0.2), 0.0);
            // const r_slip = @max((v_lon - 1.0) / 1.0, 0.0);
            const r_slip = (v_lon - 20.0) / 20.0;
            // const r_slip = v_lon * 0.0001 + 99;

            if (v_abs > 1.0) {

                // Pacejka
                // F = Fz · D · sin(C · arctan(B·slip – E · (B·slip – arctan(B·slip))))
                var v_low: f32 = 1.0;
                if (v_abs < 3.0) {
                    v_low = v_abs / 3.0;
                }
                const slip = tire_prm.lat.stiffness * a_slip;
                // const load = getLength2(t_i.pos) / getLength2(t_i.pos - b.com);
                // t_i.f_y = t_i.f_z * v_low * load * tire_prm.lat.peak * @sin(tire_prm.lat.shape * std.math.atan(slip - tire_prm.lat.curvature * slip - std.math.atan(slip)));
                t_i.f_y = t_i.f_z * v_low * tire_prm.lat.peak * @sin(tire_prm.lat.shape * std.math.atan(slip - tire_prm.lat.curvature * slip - std.math.atan(slip)));
            } else {
                const C_a = 100.0 * v_abs;
                t_i.f_y = -C_a * a_slip;
            }
            var power: f32 = 0.0;
            // if (t_i.is_powered) power = getLength2(acc_ext) * 1000;
            if (t_i.is_powered) {
                power = t_i.f_z * tire_prm.lon.peak * @sin(tire_prm.lon.shape * std.math.atan(r_slip - tire_prm.lon.curvature * r_slip - std.math.atan(r_slip)));
            }
            if (v_abs > 0.01) {
                t_i.f_x = -t_i.f_z * 0.01 + power;
            } else {
                t_i.f_x = 0.0 + power;
            }

            const t_max = t_i.f_z;
            t_i.f_x = @min(t_i.f_x, t_max);
            t_i.f_y = t_i.f_y * @sqrt(1 - (t_i.f_x / t_max) * (t_i.f_x / t_max));
        }
    }

    pf.phy_tire.stop();
}

fn updateTireFn() void {
    for (cars.items(.body), cars.items(.tires)) |b, *t| {
        var sum: f32 = 0.0;
        for (t) |*t_i| {
            sum += getLength2(t_i.pos - b.com);
        }
        for (t) |*t_i| {
            t_i.f_z = b.mass * gravity  * getLength2(t_i.pos - b.com) / sum;
        }
    }
}

fn applyForces() void {
    for (cars.items(.body), cars.items(.tires), cars.items(.steering), 0..) |*b, t, *s, idx| {
        if (rand.float(f32) < 0.01 and idx > 0) {
            s.is_steering = !s.is_steering;
            s.target = rand.float(f32) * std.math.pi * 0.4 - std.math.pi * 0.2;
            const a: f32 = 100.0 * rand.float(f32);
            b.acc = Vec2d{a * @cos(b.angle), a * @sin(b.angle)};
        }
        for (t) |t_i| {
            // acc_ext -= Vec2d{t_i.f_y * @sin(t_i.angle), t_i.f_y * -@cos(t_i.angle)} / @as(Vec2d, @splat(b.mass * 100));
            // acc_ext -= getVec2Lateral(t_i.f_y, t_i.angle) / @as(Vec2d, @splat(b.mass * 100));
            const r = transform(b.angle, t_i.pos - b.com);
            const f = getVec2(t_i.f_x, t_i.angle) + getVec2Lateral(t_i.f_y, t_i.angle);
            b.force += f;
            b.torque += cross2(r, f);
        }
    }
}

fn clearForces() void {
    for (cars.items(.body)) |*b| {
        b.acc = .{0.0, 0.0};
        b.force = .{0.0, 0.0};
        b.torque = 0.0;
    }
}


fn clampAngle(a: f32) f32 {
    // if (a < 0) return a + 2.0 * std.math.pi
    // else if (a > 2.0 * std.math.pi) return a - 2.0 * std.math.pi
    // else return a;
    if (a < -std.math.pi) return a + 2.0 * std.math.pi
    else if (a > std.math.pi) return a - 2.0 * std.math.pi
    else return a;
}

fn clampAngleInline(a: *f32) void {
    // if (a < 0) return a + 2.0 * std.math.pi
    // else if (a > 2.0 * std.math.pi) return a - 2.0 * std.math.pi
    // else return a;
    if (a.* < -std.math.pi) a.* += 2.0 * std.math.pi
    else if (a.* > std.math.pi) a.* -= 2.0 * std.math.pi;
}

inline fn scaleDbg(v: f32) f32 {
    return v * 0.0001;
}

inline fn scaleDbg2 (v: Vec2d) Vec2d {
    return v * @as(Vec2d, @splat(0.0001));
}

inline fn cross2(v1: Vec2d, v2: Vec2d) f32 {
    return v1[0] * v2[1] - v2[0] * v1[1];
}

inline fn getLength2(v: Vec2d) f32 {
    return @sqrt(@reduce(.Add, v * v));
}

inline fn getVec2(v: f32, angle: f32) Vec2d {
    return .{v * @cos(angle), v * @sin(angle)};
}

inline fn getVec2Lateral(lat: f32, angle: f32) Vec2d {
    return .{-lat * @sin(angle), lat * @cos(angle)};
}

fn rotate(alpha: f32, v: Vec2d) Vec2d {
    // const cs: Vec2d = .{@cos(alpha), -@sin(alpha)};
    // const sc: Vec2d = .{@sin(alpha), @cos(alpha)};
    // const x = @reduce(.Add, v * cs);
    // const y = @reduce(.Add, v * sc);
    // return .{x, y};
    return .{v[0] * @cos(alpha) - v[1] * @sin(alpha),
             v[0] * @sin(alpha) + v[1] * @cos(alpha)};
}

fn transform(alpha: f32, v: Vec2d) Vec2d {
    // const cs: Vec2d = .{@cos(alpha), -@sin(alpha)};
    // const sc: Vec2d = .{@sin(alpha), @cos(alpha)};
    // const x = @reduce(.Add, v * cs);
    // const y = @reduce(.Add, v * sc);
    // return .{x, y};
    return .{v[0] * @cos(alpha) - v[1] * @sin(alpha),
             v[0] * @sin(alpha) + v[1] * @cos(alpha)};
}

fn transformOffset(alpha: f32, v: Vec2d, o: Vec2d) Vec2d {
    // const cs: Vec2d = .{@cos(alpha), -@sin(alpha)};
    // const sc: Vec2d = .{@sin(alpha), @cos(alpha)};
    // const x = @reduce(.Add, v * cs);
    // const y = @reduce(.Add, v * sc);
    // return Vec2d{x, y} + o;
    return .{v[0] * @cos(alpha) - v[1] * @sin(alpha) + o[0],
             v[0] * @sin(alpha) + v[1] * @cos(alpha) + o[1]};
}

fn transform4(alpha: f32, x: Vec4d, y: Vec4d) Vec8d {
    const sin_alpha = @as(Vec4d, @splat(@sin(alpha)));
    const cos_alpha = @as(Vec4d, @splat(@cos(alpha)));
    const x_n = cos_alpha * x - sin_alpha * y;
    const y_n = sin_alpha * x + cos_alpha * y;

    return .{x_n[0], y_n[0], x_n[1], y_n[1], x_n[2], y_n[2], x_n[3], y_n[3]};
}
