const std = @import("std");

const physics = @import("physics.zig");
const Sim = physics.Sim;

const c = @cImport({
    @cInclude("SDL2/SDL.h");
    @cInclude("SDL2/SDL2_gfxPrimitives.h");
});

pub fn execute_sim(sim: *Sim) !void {
    const width = sim.width;
    const height = sim.height;

    // Init SDL2
    if (c.SDL_Init(c.SDL_INIT_EVERYTHING) != 0) {
        c.SDL_Log("Unable to initialize SDL: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    }
    defer c.SDL_Quit();

    // Open a new window
    const window = c.SDL_CreateWindow("Cloth", c.SDL_WINDOWPOS_UNDEFINED, c.SDL_WINDOWPOS_UNDEFINED, @intCast(c_int, width), @intCast(c_int, height), c.SDL_WINDOW_SHOWN) orelse {
        c.SDL_Log("Unable to create window: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    };
    defer c.SDL_DestroyWindow(window);

    // Hint to enable V-Sync
    if (c.SDL_SetHint(c.SDL_HINT_RENDER_VSYNC, "1") == c.SDL_FALSE) {
        c.SDL_Log("Unable to set hint: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    }

    // Create a renderer
    var render = c.SDL_CreateRenderer(window, -1, c.SDL_RENDERER_ACCELERATED) orelse {
        c.SDL_Log("Unable to create renderer: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    };
    defer c.SDL_DestroyRenderer(render);

    sim.start_time(c.SDL_GetTicks64());

    // Main IO loop
    var should_exit = false;
    while (!should_exit) {
        // Time between this frame and last
        const curr_frame_time = c.SDL_GetTicks64();

        // Poll events
        var sdl_event: c.SDL_Event = undefined;
        while (c.SDL_PollEvent(&sdl_event) > 0) {
            switch (sdl_event.type) {
                c.SDL_QUIT => {
                    should_exit = true;
                },
                c.SDL_KEYDOWN => {
                    if (sdl_event.key.keysym.sym == c.SDLK_SPACE) {
                        sim.update(curr_frame_time);
                    }
                },
                else => {},
            }
        }

        // Update mouse input
        var x: i32 = undefined;
        var y: i32 = undefined;
        const buttons = c.SDL_GetMouseState(&x, &y);
        sim.mouse_interaction(x, y, (buttons & c.SDL_BUTTON_LMASK) != 0, (buttons & c.SDL_BUTTON_RMASK) != 0);

        // Physics update
        sim.update(curr_frame_time);

        _ = c.SDL_SetRenderDrawColor(render, 0, 0, 0, c.SDL_ALPHA_OPAQUE);
        _ = c.SDL_RenderClear(render);

        // Draw update
        _ = c.SDL_SetRenderDrawColor(render, 255, 255, 255, c.SDL_ALPHA_OPAQUE);
        for (sim.links.items) |link| {
            const p1 = &sim.pointmasses.items[link.p1];
            const p2 = &sim.pointmasses.items[link.p2];
            _ = c.SDL_RenderDrawLine(render, @floatToInt(i32, p1.curr_pos.x), @floatToInt(i32, p1.curr_pos.y), @floatToInt(i32, p2.curr_pos.x), @floatToInt(i32, p2.curr_pos.y));
        }
        // for (sim.pointmasses.items) |pointmass| {
        //     _ = c.filledCircleRGBA(render, @floatToInt(i16, pointmass.curr_pos.x), @floatToInt(i16, pointmass.curr_pos.y), 3, 255, 255, 255, 255);
        // }

        c.SDL_RenderPresent(render);
    }
}
