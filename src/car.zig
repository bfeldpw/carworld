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
    try gfx.init();
    try initCars();
    try initGfx();
}

pub fn deinit() void {
    deinitGfx();
    gfx.deinit();
}

//-----------------------------------------------------------------------------//
//   Process
//-----------------------------------------------------------------------------//

pub fn accelerate() void {
    const b = &cars.items(.body)[0];
    const a: f32 = 2.5;
    acc_ext += Vec2d{a * @cos(b.angle), a * @sin(b.angle)};
}

pub fn deccelerate() void {
    const b = &cars.items(.body)[0];
    const a: f32 = -2.5;
    acc_ext += Vec2d{a * @cos(b.angle), a * @sin(b.angle)};
}

pub fn steerLeft() void {
    cars.items(.steering)[0].target += 0.01;
    std.log.debug("target = {d:.2}", .{cars.items(.steering)[0].target});
    // var b = &cars.items(.body)[0];
    // b.angle = clampAngle(b.angle + 0.01);
}

pub fn steerRight() void {
    cars.items(.steering)[0].target -= 0.01;
    std.log.debug("target = {d:.2}", .{cars.items(.steering)[0].target});
    // var b = &cars.items(.body)[0];
    // b.angle = clampAngle(b.angle - 0.01);
}

pub fn render() !void {
    try gfx.drawBatch(&gfx_car);
    try gfx.drawBatch(&gfx_tires);
    try bfe.gfx.base.setColor(.PxyCuniF32, 0.0, 0.0, 1.0, 1.0);
    try bfe.gfx.core.setLineWidth(3);
    try gfx.drawBatch(&gfx_dbg_vec);
    try bfe.gfx.core.setLineWidth(1);
    try bfe.gfx.base.setColor(.PxyCuniF32, 1.0, 1.0, 1.0, 1.0);
}

pub fn update() !void {
    updateTireFn();
    updateForces();
    applyForces();
    try updateCarDynamics();
    clearForces();
    // bfe.gfx.cam.updateHook(cars.items(.body)[0].pos[0], cars.items(.body)[0].pos[1], 0, 0);
    try updateCarGfx();
}

//-----------------------------------------------------------------------------//
//   Internal
//-----------------------------------------------------------------------------//
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
    torque: f32 = 0.0, // Torque w.r.t COM
    angle_acc: f32 = 0.0,
    angle_vel: f32 = 0.0,
    angle: f32 = 0.0,
    com: Vec2d = .{0.0, 0.0},
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
    id: id_type.IdType() = .{},
    // Graphics
    obj: *gfx.ObjectDataType
};

var tire_prm_long: TirePrm = .{};
var tire_prm_lat: TirePrm = .{};

const TirePrm = struct {
    stiffness: f32 = 0.1,
    shape: f32 = 2.4,
    peak: f32 = 1.0,
    curvature: f32 = 0.97
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
var gfx_dbg_vec: gfx.GraphicsDataType = .{.primitive_mode = .Lines};

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

        car.tires[0].pos = .{-1.5, -0.8};
        car.tires[1].pos = .{ 1.5, -0.8};
        car.tires[2].pos = .{ 1.5,  0.8};
        car.tires[3].pos = .{-1.5,  0.8};
        car.tires[0].is_steered = false;
        car.tires[1].is_steered = true;
        car.tires[2].is_steered = true;
        car.tires[3].is_steered = false;

        car.tires[0].id.init();
        car.tires[1].id.init();
        car.tires[2].id.init();
        car.tires[3].id.init();

        car.steering.is_steering = false;
        car.steering.angle = 0.0;
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
            var v1: [8]f32 = .{tire.pos[0] + b.pos[0] - 0.12, tire.pos[1] + b.pos[1] - 0.35,
                               tire.pos[0] + b.pos[0] + 0.12, tire.pos[1] + b.pos[1] - 0.35,
                               tire.pos[0] + b.pos[0] + 0.12, tire.pos[1] + b.pos[1] + 0.35,
                               tire.pos[0] + b.pos[0] - 0.12, tire.pos[1] + b.pos[1] + 0.35,
                               };
            gfx.createObjectPolygonGfx(tire.obj, 0, &v1);

            {
                // Debug vector for forces
                try gfx.createObjectDataAddSrc(tire.obj, &gfx_dbg_vec, 2);
                var v_dbg: [4]f32 = .{tire.pos[0], tire.pos[1], tire.pos[0] + 1, tire.pos[1] + 1};
                gfx.createObjectPolygonGfx(tire.obj, 1, &v_dbg);
            }
        }
    }
    // for (cars.items(.body)) |*b| {
    // }
}

fn initGfx() !void {
    try gfx.initBatch(&gfx_car, 8 * n_cars, (4 + 1) * n_cars);
    try gfx.initBatch(&gfx_tires, 4 * 8 * n_cars, 4 * (4 + 1) * n_cars);
    try gfx.initBatch(&gfx_dbg_vec, 2 * n_cars, (2 + 1) * n_cars);
}

fn deinitGfx() void {
    gfx.deinitBatch(&gfx_car);
    gfx.deinitBatch(&gfx_tires);
    gfx.deinitBatch(&gfx_dbg_vec);
}

fn updateCarDynamics() !void {
    // Rough estimation distance to fromt axle *
    // distance to rear axle * mass
    const Inertia = 1.5 * 1.5 * 2000.0;

    const dt = 1.0 / 60.0;
    const dt2 = @as(Vec2d, @splat(dt));
    for (cars.items(.body)) |*b| {
        b.acc = acc_ext + b.force / @as(Vec2d, @splat(b.mass));
        b.vel_p = b.vel;
        b.vel += b.acc * dt2;
        b.pos += b.vel * dt2;
        b.acc += (b.vel - b.vel_p) / dt2;

        b.angle_acc = b.torque / Inertia * dt;
        b.angle_vel += b.angle_acc * dt;
        b.angle += b.angle_vel * dt;
        clampAngleInline(&b.angle);
    }
    acc_ext = .{0.0, 0.0};
    for (cars.items(.steering), cars.items(.body), cars.items(.tires)) |*s, *b, *t| {
        // if (rand.float(f32) < 0.01) {
        //     s.is_steering = !s.is_steering;
        //     s.target = rand.float(f32) * std.math.pi * 0.4 - std.math.pi * 0.2;
        //     const a: f32 = 0.1 * rand.float(f32);
        //     b.acc = Vec2d{a * @cos(b.angle), a * @sin(b.angle)};
        // }
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
}

fn updateCarGfx() !void {
    for (cars.items(.body), cars.items(.tires), cars.items(.steering)) |*b, *ts, s| {
        // const b_pos_8 =  Vec8d{b.pos[0], b.pos[1],
        //                        b.pos[0], b.pos[1],
        //                        b.pos[0], b.pos[1],
        //                        b.pos[0], b.pos[1],
        //                        };
        {
            const p0 = transformOffset(b.angle, Vec2d{-2, -1}, b.pos);
            const p1 = transformOffset(b.angle, Vec2d{ 2, -1}, b.pos);
            const p2 = transformOffset(b.angle, Vec2d{ 2,  1}, b.pos);
            const p3 = transformOffset(b.angle, Vec2d{-2,  1}, b.pos);
            var v: [8]f32 = .{p0[0], p0[1],
                              p1[0], p1[1],
                              p2[0], p2[1],
                              p3[0], p3[1],
                            };
            try gfx.modifyData(b.obj, 0, &v);
            // var p: Vec8d = transform4(b.angle, .{-1, 1, 1, -1}, .{-2, -2, 2, 2}) + b_pos_8;
            // try gfx.modifyDataSimd8(b.obj, 0, &p);
        }
        {
            for (ts) |*t| {
                if (t.is_steered) {
                    const t0 = transformOffset(s.angle, Vec2d{-0.35, -0.12}, t.pos);
                    const t1 = transformOffset(s.angle, Vec2d{ 0.35, -0.12}, t.pos);
                    const t2 = transformOffset(s.angle, Vec2d{ 0.35,  0.12}, t.pos);
                    const t3 = transformOffset(s.angle, Vec2d{-0.35,  0.12}, t.pos);
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
                    const p0 = transformOffset(b.angle, t.pos + Vec2d{-0.35, -0.12}, b.pos);
                    const p1 = transformOffset(b.angle, t.pos + Vec2d{ 0.35, -0.12}, b.pos);
                    const p2 = transformOffset(b.angle, t.pos + Vec2d{ 0.35,  0.12}, b.pos);
                    const p3 = transformOffset(b.angle, t.pos + Vec2d{-0.35,  0.12}, b.pos);
                    var v: [8]f32 = .{p0[0], p0[1],
                                    p1[0], p1[1],
                                    p2[0], p2[1],
                                    p3[0], p3[1],
                                    };
                    try gfx.modifyData(t.obj, 0, &v);
                }
                // var p: Vec8d = transform4(b.angle, .{t.pos[0] - 0.12, t.pos[0] + 0.12, t.pos[0] + 0.12, t.pos[0] - 0.12},
                //                           .{t.pos[1] - 0.35, t.pos[1] - 0.35, t.pos[1] + 0.35, t.pos[1] + 0.35}) + b_pos_8;
                // try gfx.modifyDataSimd8(t.obj, 0, &p);
            }
        }
        {
            // b.force = b.vel * @as(Vec2d, @splat(60.0 * b.mass));
            // std.log.debug("Force to stop = {d:.1}", .{b.force});
            var v: [4]f32 = .{b.pos[0], b.pos[1],
                              b.pos[0] + scaleDbg(b.mass * b.vel[0]),
                              b.pos[1] + scaleDbg(b.mass * b.vel[1])};
            try gfx.modifyData(b.obj, 1, &v);
        }
    }
}

fn updateForces() void {
    // const mu = 0.8;

    for (cars.items(.body), cars.items(.tires)) |*b, *t| {
        for (t) |*t_i| {
            // const f = mu * t_i.f_n;
            // const f = 0.25 * @sqrt(b.vel[0] * b.vel[0] + b.vel[1] * b.vel[1]) * b.mass;

            // var f_y: f32 = 0.0;
            t_i.f_y = 0.0;

            if (getLength2(b.vel) > 0.01) {

                const r = transform(b.angle, t_i.pos - b.com);
                // const r = t_i.pos - b.com;
                // const vel_r = Vec2d{-r[0], r[1]};
                const vel_r = @as(Vec2d, @splat(b.angle_vel)) * Vec2d{-r[1], r[0]};
                const vel = vel_r + b.vel;

                // t_vel = b.vel + b.angle_vel *
                // Slip angle
                const a_vel = clampAngle(std.math.atan2(vel[1], vel[0]));
                var a_slip = a_vel - t_i.angle;
                while (a_slip > 0.5 * std.math.pi) a_slip = std.math.pi - a_slip;
                while (a_slip < -0.5 * std.math.pi) a_slip = -std.math.pi - a_slip;
                a_slip = std.math.radiansToDegrees(a_slip);
                // std.log.debug("VelAngle = {d:.2}", .{std.math.radiansToDegrees(std.math.atan2(b.vel[1], b.vel[0]))});
                // std.log.debug("TireAngle = {d:.2}", .{std.math.radiansToDegrees(t_i.angle)});
                std.log.debug("SlipAngle = {d:.2}", .{a_slip});

                // Pacejka
                // F = Fz · D · sin(C · arctan(B·slip – E · (B·slip – arctan(B·slip))))
                const slip = tire_prm_lat.stiffness * a_slip;
                t_i.f_y = t_i.f_z * tire_prm_lat.peak * @sin(tire_prm_lat.shape * std.math.atan(slip - tire_prm_lat.curvature * slip - std.math.atan(slip)));
            } else {
                // b.angle_vel *= 0.1;
                // b.vel *= .{0.1, 0.1};
            }


            std.log.debug("f_y = {d:.2}", .{t_i.f_y});

            {
                const p0 = transformOffset(b.angle, t_i.pos, b.pos);

                // Direction is 90° (-sin, cos, switch coordinates and negate x)
                // but the force is oriented agains the cars movement, therefore
                // the vector has to be negated
                const p1 = transformOffset(b.angle, t_i.pos, b.pos + scaleDbg2(getVec2Lateral(t_i.f_y, t_i.angle)));
                // Debug vector for forces
                var v_dbg: [4]f32 = .{p0[0], p0[1], p1[0], p1[1]};
                try gfx.modifyData(t_i.obj, 1, &v_dbg);
            }
        }
    }
}

fn updateTireFn() void {
    for (cars.items(.body), cars.items(.tires)) |b, *t| {
        for (t) |*t_i| {
            t_i.f_z = b.mass * 0.25 * gravity;
        }
    }
}

fn applyForces() void {
    for (cars.items(.body), cars.items(.tires)) |*b, t| {
        for (t) |t_i| {
            // acc_ext -= Vec2d{t_i.f_y * @sin(t_i.angle), t_i.f_y * -@cos(t_i.angle)} / @as(Vec2d, @splat(b.mass * 100));
            // acc_ext -= getVec2Lateral(t_i.f_y, t_i.angle) / @as(Vec2d, @splat(b.mass * 100));
            const r = transform(b.angle, t_i.pos - b.com);
            const f = getVec2Lateral(t_i.f_y, t_i.angle);
            b.force += f;
            b.torque += cross2(r, f);
            std.log.debug("vec_r = {d:.2}", .{r});
            std.log.debug("vec_f = {d:.2}", .{f});
            std.log.debug("torque = {d:.2}", .{cross2(r, f)});
        }
    }
}

fn clearForces() void {
    for (cars.items(.body)) |*b| {
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
    return v * 0.001;
}

inline fn scaleDbg2 (v: Vec2d) Vec2d {
    return v * @as(Vec2d, @splat(0.001));
}

inline fn cross2(v1: Vec2d, v2: Vec2d) f32 {
    return v1[0] * v2[1] - v2[0] * v1[1];
}

inline fn getLength2(v: Vec2d) f32 {
    return @reduce(.Add, v * v);
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
    // return .{x, y};
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
