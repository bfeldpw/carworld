const std = @import("std");
const bfe = @import("bfe");
const cfg = bfe.cfg;
const buf = bfe.util.value_buffer;
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
    cars.deinit(allocator);

    deinitGfx();
    gfx.deinit();

    log_car.info("  Performance Stats", .{});
    log_car.info("  ============================", .{});
    log_car.info("  gfx update   |   {d:.2}ms", .{pf.gfx.getAvgAllMs()});
    log_car.info("  physics      |   {d:.2}ms", .{pf.phy.getAvgAllMs()});
    log_car.info("   - dynamics  |   {d:.2}ms", .{pf.phy_dyn.getAvgAllMs()});
    log_car.info("   - tires     |   {d:.2}ms", .{pf.phy_tire.getAvgAllMs()});
    log_car.info("  ============================", .{});

    const leaked = gpa.deinit();
    if (leaked == .leak) log_car.err("Memory leaked in GeneralPurposeAllocator", .{});
}

//-----------------------------------------------------------------------------//
//   Getter / Setter
//-----------------------------------------------------------------------------//
pub fn getCarData(a: std.mem.Allocator, idx: usize) ![]u8 {
    std.debug.assert(idx >= 0);
    std.debug.assert(idx < n_cars);

    return try std.fmt.allocPrint(
        a,
        "GENERIC\n" ++
        "  Velocity = {d:.0} km/h\n" ++
        "\n" ++
        "TIRE MODEL\n" ++
        "  Linear   = {d:.0} %\n" ++
        "  Pacejka  = {d:.0} %\n" ++
        "\n" ++
        "  SLIP ANGLE [degree]\n" ++
        "    front (l / r) = {d:.0}, {d:.0}\n" ++
        "    rear  (l / r) = {d:.0}, {d:.0}\n" ++
        "\n" ++
        "  SLIP RATIO [%]\n" ++
        "    front (l / r) = {d:.0}, {d:.0}\n" ++
        "    rear  (l / r) = {d:.0}, {d:.0}\n" ++
        "\n" ++
        "  TIRE LOAD [kN]\n" ++
        "    front (l / r) = {d:.1}, {d:.1}\n" ++
        "    rear  (l / r) = {d:.1}, {d:.1}\n" ++
        "\n" ++
        "AERODYNAMICS\n" ++
        "    drag (front) = {d:.0} N\n" ++
        "    drag (side)  = {d:.0} N\n"
            ,
        .{getLength2(cars.items(.body)[idx].vel) * 3.6,
          buf_tire_model_lin.getAvg(),
          buf_tire_model_paj.getAvg(),
          buf_tire_slip_a[2].getAvg(),
          buf_tire_slip_a[1].getAvg(),
          buf_tire_slip_a[3].getAvg(),
          buf_tire_slip_a[0].getAvg(),
          buf_tire_slip_r[2].getAvg(),
          buf_tire_slip_r[1].getAvg(),
          buf_tire_slip_r[3].getAvg(),
          buf_tire_slip_r[0].getAvg(),
          buf_tire_load[2].getAvg() * 0.001,
          buf_tire_load[1].getAvg() * 0.001,
          buf_tire_load[3].getAvg() * 0.001,
          buf_tire_load[0].getAvg() * 0.001,
          buf_drag[0].getAvg(),
          buf_drag[1].getAvg()}
    );
}

//-----------------------------------------------------------------------------//
//   Process
//-----------------------------------------------------------------------------//
var is_accelerating: bool = false;
var is_decelerating: bool = false;

pub fn increaseThrottle() void {
    const thr = &cars.items(.engine)[0].throttle;
    if (thr.* < cars.items(.engine)[0].torque_max) thr.* += 10;
    log_car.debug("thr = {d:.2}Nm", .{thr.*});
}

pub fn decreaseThrottle() void {
    const thr = &cars.items(.engine)[0].throttle;
    thr.* -= 10;
    if (thr.* < 0.0) thr.* = 0.0;
    log_car.debug("thr = {d:.2}Nm", .{thr.*});
}

pub fn accelerate() void {
    is_accelerating = true;
}

pub fn decelerate() void {
    is_decelerating = true;
}

pub fn steerLeft() void {
    const car = &cars.items(.steering)[0];
    car.target += car.speed * 1.0 / 60.0;
    if (car.target > car.max) car.target = car.max;
}

pub fn steerRight() void {
    const car = &cars.items(.steering)[0];
    car.target -= car.speed * 1.0 / 60.0;
    if (car.target < -car.max) car.target = -car.max;
}

pub fn render() !void {
    try bfe.gfx.base.setColor(.PxyCuniF32, 1.0, 1.0, 1.0, 1.0);
    try bfe.gfx.core.setLineWidth(1);
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
    updateForces();
    is_accelerating = false;
    is_decelerating = false;
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
    if (is_hooked) bfe.gfx.cam.resetPosition();
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
    steering: SteeringDef = .{},
    engine: EngineDef = .{},
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
    // Aero dynamics
    cw_x: f32 = 0.3,
    area_x: f32 = 2.2,
    cw_y: f32 = 0.9,
    area_y: f32 = 6.5,
    // Graphics
    obj: *gfx.ObjectDataType,
};

const EngineDef = struct {
    torque_max: f32 = 300.0, // Nm
    throttle: f32 = 0.0
};

const SteeringDef = struct {
    is_steering: bool = false,
    speed: f32 = 0.5, // rad/s
    angle: f32 = 0.0,
    max: f32 = std.math.pi * 0.25, // 45°
    target: f32 = 0.0
};

const TireDef = struct {
    // Local parameters
    pos: Vec2d = .{0.0, 0.0},
    angle: f32 = 0.0,
    f_x: f32 = 0.0,  // longitudinal force
    f_y: f32 = 0.0,  // lateral force
    f_y0: f32 = 0.0, // lateral force of previous frame
    f_z: f32 = 1.0,  // normal force
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
        stiffness: f32 = 10.0,
        shape: f32 = 1.3,
        peak: f32 = 1.0,
        curvature: f32 = 0.97,
    } = .{},
    lon: struct {
        stiffness: f32 = 10.0,
        shape: f32 = 1.65,
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
        car.body.com0 = .{ 0.2, 0.0};
        car.body.com_z = 0.2;
        car.body.cw_x = 0.3;
        car.body.area_x = 2.2;
        car.body.cw_y = 0.9;
        car.body.area_y = 6.5;

        car.engine.throttle = 0.0;
        car.engine.torque_max = 300.0;

        car.tires[0].pos = .{-1.5, -0.8};
        car.tires[1].pos = .{ 1.5, -0.8};
        car.tires[2].pos = .{ 1.5,  0.8};
        car.tires[3].pos = .{-1.5,  0.8};
        car.tires[0].is_steered = false;
        car.tires[1].is_steered = true;
        car.tires[2].is_steered = true;
        car.tires[3].is_steered = false;
        // FWD
        // car.tires[0].is_powered = false;
        // car.tires[1].is_powered = true;
        // car.tires[2].is_powered = true;
        // car.tires[3].is_powered = false;
        // RWD
        car.tires[0].is_powered = true;
        car.tires[1].is_powered = false;
        car.tires[2].is_powered = false;
        car.tires[3].is_powered = true;
        // AWD
        // car.tires[0].is_powered = true;
        // car.tires[1].is_powered = true;
        // car.tires[2].is_powered = true;
        // car.tires[3].is_powered = true;

        car.tires[0].id.init();
        car.tires[1].id.init();
        car.tires[2].id.init();
        car.tires[3].id.init();

        car.steering.is_steering = false;
        car.steering.angle = 0.0;
        car.steering.max = std.math.degreesToRadians(35.0);
        car.steering.target = 0.0;
        car.steering.speed = 1.5;

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
        // const f_acc = b.acc / (Vec2d{2.0, 2.0} * @abs(b.acc) + @as(Vec2d, @splat(1)));
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
                s.angle = s.target;
            }
        } else if (s.target > 0) {
            if (s.angle < s.target) {
                s.angle += s.speed / 60.0;
            } else {
                s.angle = s.target;
            }
        }
        for (t) |*t_i| {
            if (t_i.is_steered) t_i.angle = clampAngle(b.angle + s.angle)
            else t_i.angle = clampAngle(b.angle);
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

var buf_drag: [2]buf.Buffer(f32, 60) = .{
    buf.Buffer(f32, 60).init(),
    buf.Buffer(f32, 60).init()
};
var buf_tire_load: [4]buf.Buffer(f32, 60) = .{
    buf.Buffer(f32, 60).init(),
    buf.Buffer(f32, 60).init(),
    buf.Buffer(f32, 60).init(),
    buf.Buffer(f32, 60).init()
};
var buf_tire_slip_a: [4]buf.Buffer(f32, 60) = .{
    buf.Buffer(f32, 60).init(),
    buf.Buffer(f32, 60).init(),
    buf.Buffer(f32, 60).init(),
    buf.Buffer(f32, 60).init()
};
var buf_tire_slip_r: [4]buf.Buffer(f32, 60) = .{
    buf.Buffer(f32, 60).init(),
    buf.Buffer(f32, 60).init(),
    buf.Buffer(f32, 60).init(),
    buf.Buffer(f32, 60).init()
};
var buf_tire_model_lin: buf.Buffer(f32, 60) = buf.Buffer(f32, 60).init();
var buf_tire_model_paj: buf.Buffer(f32, 60) = buf.Buffer(f32, 60).init();

fn updateForces() void {
    pf.phy_tire.start();
    // const mu = 0.8;

    for (cars.items(.body), cars.items(.engine), cars.items(.tires)) |*b, e, *t| {

        var count: u32 = 0;
        for (t) |*t_i| {
            t_i.f_y = 0.0;

            // Calculate velocity of single tire, including the bodies angular velocity
            // ---
            // p_l: local tire position
            // v_l: local tire velocity
            // v:   tire velocity (vector)
            // v_abs: absolute tire velocity (scalar)
            const p_l = transform(b.angle, t_i.pos - b.com);
            const v_l = @as(Vec2d, @splat(b.angle_vel)) * Vec2d{-p_l[1], p_l[0]};
            const v = v_l + b.vel;
            const v_abs = getLength2(v);

            // Calculate the slip angle
            // ---
            // a_v: angle of tires velocity vector
            // a_slip: slip angle
            // v_lon: longitudinal tire veolocity (scalar)
            const a_v = std.math.atan2(v[1], v[0]);
            var a_slip = a_v - t_i.angle;
            const v_lon = v_abs * @cos(a_slip);

            // Clamp slip angle to +/-90° for Pacejka
            while (a_slip > 0.5 * std.math.pi) a_slip = std.math.pi - a_slip;
            while (a_slip < -0.5 * std.math.pi) a_slip = -std.math.pi - a_slip;
            a_slip = std.math.radiansToDegrees(a_slip);

            buf_tire_slip_a[count].add(a_slip);
            buf_tire_load[count].add(t_i.f_z);

            // Hacking throttle behaviour, has to be refined
            var v_thr: f32 = 0.0;
            var r_slip: f32 = 0.0;
            var dir: f32 = 1.0;
            if (is_accelerating or is_decelerating) v_thr = (e.throttle / e.torque_max + 1) * v_lon;
            if (v_lon > 0.1) {
                if (is_accelerating and v_thr > 0.0) r_slip = @min(100.0, (v_thr / v_lon - 1) * 100);
                // else r_slip = v_abs / v_lon - 1; // This is just a hack, adapt
                // else r_slip = -100.0;
            } else if (v_lon > -0.01) {
                if (is_accelerating) r_slip = 0.1;
            } else {
                // if (is_accelerating and v_thr > 0.0) r_slip = @min(100.0, (v_thr / -v_lon - 1) * 100);
                r_slip = -100.0;//v_abs / v_lon - 1;
                dir = -1.0;
            }
            buf_tire_slip_r[count].add(r_slip);
            count += 1;
            // log_car.debug("e_thr = {d:.2}", .{e.throttle});
            // log_car.debug("v_thr = {d:.2}", .{v_thr});
            // log_car.debug("v_abs = {d:.2}", .{v_abs});
            // log_car.debug("v_lon = {d:.2}", .{v_lon});
            // log_car.debug("r_slip = {d:.2}", .{r_slip});

            // Calculate lateral forces
            // ---
            //   Calculate Pacejka model
            //   ---
            //   F = Fz · D · sin(C · arctan(B·slip – E · (B·slip – arctan(B·slip))))
            const slip = tire_prm.lat.stiffness * a_slip;
            const model_paj = -t_i.f_z * tire_prm.lat.peak * @sin(tire_prm.lat.shape * std.math.atan(slip - tire_prm.lat.curvature * (slip - std.math.atan(slip))));

            // Calculate simplified (linear) model
            // ---
            //   In the simplified model, the tire force is linear to the slip angle
            //   It is weighed by the maximum tire load and normalized by the maximum
            //   slip angle.
            //   Factoring in the absolut velocity makes sure, that no forces are
            //   applied at v = 0, and only minimal forces act, when speed is low,
            //   to reduce jittering.
            const c_a = -t_i.f_z * v_abs;
            const a_slip_max_inv = 1.0 / 90.0;
            const model_lin = c_a * a_slip * a_slip_max_inv;

            // Threshold for piecewise linear combination of simplified and Pacejka model
            const th_model_0 = 2.0;
            const th_model_1 = 5.0;

            const weight = (v_abs - th_model_0) / (th_model_1 - th_model_0);
            if (v_abs <= th_model_0) {
                t_i.f_y = model_lin;
                buf_tire_model_lin.add(100.0);
                buf_tire_model_paj.add(0.0);
            } else if (v_abs >= th_model_1) {
                t_i.f_y = model_paj;
                buf_tire_model_lin.add(0.0);
                buf_tire_model_paj.add(100.0);
            }
            else {
                t_i.f_y = (1 - weight) * model_lin + weight * model_paj;
                buf_tire_model_lin.add((1 - weight) * 100.0);
                buf_tire_model_paj.add(weight * 100.0);
            }

            // Remove outliers (force peaks)
            if (t_i.f_y0 > 0.0 and @abs(t_i.f_y) - @abs(t_i.f_y0) > t_i.f_z) {
                t_i.f_y = t_i.f_y0;
            }

            t_i.f_y0 = t_i.f_y;

            // Calculate longitudinal forces
            // ---
            var power: f32 = 0.0;
            if (t_i.is_powered) {
                power = dir * t_i.f_z * tire_prm.lon.peak * @sin(tire_prm.lon.shape * std.math.atan(r_slip - tire_prm.lon.curvature * (r_slip - std.math.atan(r_slip))));
            }
            if (v_abs > 0.01) {
                const rolling_resistence = t_i.f_z * 0.01;
                t_i.f_x = -rolling_resistence + power;
            } else {
                t_i.f_x = power;
            }

            // Lateral / Longitudinal combination
            // cos(C * arctan (B * r_slip))
            const c_c = 0.5;
            const b_c = 0.2;
            const t_max = t_i.f_z;
            t_i.f_x = @min(t_i.f_x, t_max);
            if (t_i.is_powered) t_i.f_y = t_i.f_y * @max(0.0, @cos(c_c * std.math.atan(b_c * r_slip)));
            t_i.f_x = t_i.f_x * @max(0.0, @cos(c_c * std.math.atan(b_c * a_slip)));
        }

        // Drag
        // const vel = getLength2(b.vel);
        const vel = transform(-b.angle, b.vel);
        const drag = .{0.5 * 1.0 * b.cw_x * b.area_x * vel[0] * vel[0],
                       0.5 * 1.0 * b.cw_y * b.area_y * vel[1] * vel[1]};

        b.force -= drag;
        buf_drag[0].add(drag[0]);
        buf_drag[1].add(drag[1]);
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
            t_i.f_z = b.mass * gravity  * (0.5 - getLength2(t_i.pos - b.com) / sum);
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
