const std = @import("std");

pub const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,

    pub fn init(x: f32, y: f32, z: f32) Vec3 {
        return Vec3{ .x = x, .y = y, .z = z };
    }

    pub fn zero() Vec3 {
        return Vec3.init(0, 0, 0);
    }

    pub fn add(lhs: Vec3, rhs: Vec3) Vec3 {
        return Vec3.init(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    pub fn sub(lhs: Vec3, rhs: Vec3) Vec3 {
        return Vec3.init(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    pub fn mul(lhs: Vec3, rhs: Vec3) Vec3 {
        return Vec3.init(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
    }

    pub fn smul(self: Vec3, scalar: f32) Vec3 {
        return Vec3.init(self.x * scalar, self.y * scalar, self.z * scalar);
    }

    pub fn sdiv(self: Vec3, scalar: f32) Vec3 {
        return Vec3.init(self.x / scalar, self.y / scalar, self.z / scalar);
    }

    pub fn dot(lhs: Vec3, rhs: Vec3) f32 {
        return (lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z);
    }
};

pub const PointMass = struct {
    last_pos: Vec3,
    curr_pos: Vec3,
    acc: Vec3,
    mass: f32,
    damping: f32,
    pinned: bool,
    pin_pos: Vec3,
    links: std.ArrayList(usize),

    pub fn init(allocator: std.mem.Allocator, pos: Vec3) PointMass {
        return PointMass{
            .last_pos = pos,
            .curr_pos = pos,
            .acc = Vec3.zero(),
            .mass = 1,
            .damping = 20,
            .pinned = false,
            .pin_pos = Vec3.zero(),
            .links = std.ArrayList(usize).init(allocator),
        };
    }

    pub fn addForce(self: *PointMass, force: Vec3) void {
        self.acc = Vec3.add(self.acc, force.sdiv(self.mass));
    }

    pub fn update(self: *PointMass, time_step: f32) void {
        if (self.pinned) {
            return;
        }

        // Add some gravity acceleration
        self.addForce(.{ .x = 0, .y = self.mass * 981.0, .z = 0 });

        // Calculate velocity and dampen it
        var vel = Vec3.sub(self.curr_pos, self.last_pos);
        vel = vel.smul(1.0 - self.damping / 1000.0);

        // Calculate next position
        const time_step_sq = time_step * time_step;
        const next_pos = Vec3.add(Vec3.add(self.curr_pos, vel), Vec3.smul(self.acc, 0.5 * time_step_sq));

        self.last_pos = self.curr_pos;
        self.curr_pos = next_pos;
        self.acc = Vec3.zero();
    }

    pub fn pinAt(self: *PointMass, point: Vec3) void {
        self.pin_pos = point;
        self.pinned = true;
    }

    pub fn unpin(self: *PointMass) void {
        self.pinned = false;
    }

    pub fn attachWith(self: *PointMass, links: *std.ArrayList(Link), link: Link) !void {
        // Append to links array
        try links.append(link);
        // Store index on this object
        try self.links.append(links.items.len - 1);
    }
};

pub const Link = struct {
    resting_dis: f32,
    stiffness: f32,
    tear_sense: f32,
    p1: usize,
    p2: usize,

    pub fn solve(self: *Link, p1: *PointMass, p2: *PointMass) void {
        // Distance between the two points
        const dv = Vec3.sub(p1.curr_pos, p2.curr_pos);
        const dis = @sqrt(Vec3.dot(dv, dv));
        const dif = (self.resting_dis - dis) / dis;

        // If distance too great then tear the link
        if (dis > self.tear_sense) {
            // TODO
        }

        // Calculate how far along we should push based on stiffness and mass
        const m1 = 1.0 / p1.mass;
        const m2 = 1.0 / p2.mass;
        const s1 = (m1 / (m1 + m2)) * self.stiffness;
        const s2 = self.stiffness - s1;

        // Push the two points closer together
        p1.curr_pos = Vec3.add(p1.curr_pos, Vec3.smul(dv, dif * s1));
        p2.curr_pos = Vec3.sub(p2.curr_pos, Vec3.smul(dv, dif * s2));
    }
};

pub const Sim = struct {
    width: u32,
    height: u32,

    last_frame_time: u64,
    curr_frame_time: u64,
    left_over_time: u64,

    pointmasses: std.ArrayList(PointMass),
    links: std.ArrayList(Link),

    mouse_last_x: i32,
    mouse_last_y: i32,
    mouse_influence: u32,
    mouse_tear: u32,
    mouse_influence_scalar: f32,

    const SIM_FRAMERATE = 16;
    const CONSTRAINT_ACCURACY = 3;

    pub fn init(allocator: std.mem.Allocator, width: u32, height: u32) Sim {
        return Sim{
            .width = width,
            .height = height,
            .last_frame_time = undefined,
            .curr_frame_time = undefined,
            .left_over_time = 0,
            .pointmasses = std.ArrayList(PointMass).init(allocator),
            .links = std.ArrayList(Link).init(allocator),
            .mouse_last_x = undefined,
            .mouse_last_y = undefined,
            .mouse_influence = 80 * 80,
            .mouse_tear = 20 * 20,
            .mouse_influence_scalar = undefined,
        };
    }

    pub fn createCurtain(self: *Sim, allocator: std.mem.Allocator) !void {
        const curtain_width = 61;
        const curtain_height = 40;
        const resting_dis = 20;
        const y_start = 40;

        const mid_width = self.width / 2 - (resting_dis * curtain_width) / 2;

        var points = &self.pointmasses;
        try points.ensureTotalCapacity(curtain_width * curtain_height);
        var y: u32 = 0;
        while (y < curtain_height) {
            var x: u32 = 0;
            while (x < curtain_width) {
                var point = PointMass.init(allocator, Vec3.init(@intToFloat(f32, mid_width + x * resting_dis), @intToFloat(f32, y * resting_dis + y_start), @intToFloat(f32, y)));

                if (x != 0) {
                    const link = Link{
                        .resting_dis = resting_dis,
                        .stiffness = 0.5,
                        .tear_sense = 30,
                        .p1 = points.items.len,
                        .p2 = points.items.len - 1,
                    };
                    try point.attachWith(&self.links, link);
                }
                if (y != 0) {
                    const link = Link{
                        .resting_dis = resting_dis,
                        .stiffness = 0.5,
                        .tear_sense = 30,
                        .p1 = points.items.len,
                        .p2 = (y - 1) * curtain_width + x,
                    };
                    try point.attachWith(&self.links, link);
                }
                if (y == 0 and x % 6 == 0) {
                    point.pinAt(point.curr_pos);
                }
                try points.append(point);

                x += 1;
            }
            y += 1;
        }
    }

    pub fn start_time(self: *Sim, time: u64) void {
        self.last_frame_time = time;
    }

    fn solve_pointmass(self: *Sim, pm: *PointMass) void {
        // Link constraints (where the magic happens)
        for (pm.links.items) |i| {
            var link = self.links.items[i];
            link.solve(&self.pointmasses.items[link.p1], &self.pointmasses.items[link.p2]);
        }

        // Wall constraints
        if (pm.curr_pos.y < 0) {
            pm.curr_pos.y = 0;
        } else if (pm.curr_pos.y > @intToFloat(f32, self.height)) {
            pm.curr_pos.y = @intToFloat(f32, self.height);
        }
        if (pm.curr_pos.x < 0) {
            pm.curr_pos.x = 0;
        } else if (pm.curr_pos.x > @intToFloat(f32, self.width)) {
            pm.curr_pos.x = @intToFloat(f32, self.width);
        }

        // Set position if pinned
        if (pm.pinned) {
            pm.curr_pos = pm.pin_pos;
        }
    }

    pub fn mouse_interaction(self: *Sim, x: i32, y: i32, left_mouse: bool, right_mouse: bool) void {
        if (left_mouse or right_mouse) {
            for (self.pointmasses.items) |*pm| {
                // Distance between point and cursor
                const dx = @intToFloat(f32, x) - pm.curr_pos.x;
                const dy = @intToFloat(f32, y) - pm.curr_pos.y;
                const dis_sq = dx * dx + dy * dy;
                if (left_mouse and dis_sq < @intToFloat(f32, self.mouse_influence)) {
                    // To change the velocity we must change the last position
                    // because of how verlet integration works.
                    pm.last_pos.x = pm.curr_pos.x - @intToFloat(f32, x - self.mouse_last_x) * self.mouse_influence_scalar;
                    pm.last_pos.y = pm.curr_pos.y - @intToFloat(f32, y - self.mouse_last_y) * self.mouse_influence_scalar;
                }
                if (right_mouse and dis_sq < @intToFloat(f32, self.mouse_tear)) {
                    // TODO
                }
            }
        }
        self.mouse_last_x = x;
        self.mouse_last_y = y;
    }

    pub fn update(self: *Sim, time: u64) void {
        // Calculate amount of physics steps to take
        self.curr_frame_time = time;
        const frame_delta = self.curr_frame_time - self.last_frame_time;
        const timesteps = (frame_delta + self.left_over_time) / SIM_FRAMERATE;
        self.left_over_time = frame_delta -| timesteps * SIM_FRAMERATE;

        // Update physics
        self.mouse_influence_scalar = 2.0 / @intToFloat(f32, frame_delta);
        var iter: u32 = 0;
        while (iter < timesteps) {
            // Solve the constraints multiple times, the more it's solved the
            // more accurate it is.
            var i: u32 = 0;
            while (i < CONSTRAINT_ACCURACY) {
                for (self.pointmasses.items) |*pointmass| {
                    self.solve_pointmass(pointmass);
                }
                i += 1;
            }

            // Update the position of the points
            for (self.pointmasses.items) |*pointmass| {
                pointmass.update(@intToFloat(f32, SIM_FRAMERATE) / 1000.0);
            }

            iter += 1;
        }

        self.last_frame_time = self.curr_frame_time;
    }
};
