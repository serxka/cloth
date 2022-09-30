const std = @import("std");
const physics = @import("physics.zig");
const backend = @import("backend_sdl.zig");

const width = 1920;
const height = 1080;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();
    // defer _ = gpa.deinit();

    // var as = std.ArrayList(u32).init(gpa.allocator());
    // try as.append(2);
    // for (as.items) |a| {
    //     std.debug.print("Hello {d}\n", .{a});
    // }

    var sim = physics.Sim.init(allocator, width, height);
    try sim.createCurtain(allocator);
    return backend.execute_sim(&sim);
}
